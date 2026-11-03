// ESP32-C3-MINI1 小车板固件 (12 IO 极限精简版)
// 保留: SSD1306 OLED + I2S Simplex 声音 + 2 路差速电机
// 砍掉: BOOT 按键、TOUCH 按键、音量键、灯、SPI 屏背光、复杂情感动作
#include "wifi_board.h"
#include "codecs/no_audio_codec.h"
#include "display/oled_display.h"
#include "system_reset.h"
#include "application.h"
#include "config.h"
#include "mcp_server.h"
#include "settings.h"
#include "motor_controller.h"
#include "distance_guard.h"
#include "web_server.h"
#include "boards/common/button.h"
#include "assets/lang_config.h"   // Lang::Sounds::OGG_EXCLAMATION（前进被拦截提示音）

#include <esp_log.h>
#include <esp_system.h>          // esp_reset_reason()
#include <esp_debug_helpers.h>   // esp_backtrace_print()
#include <esp_random.h>          // esp_random()（探索模式的随机转向）
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/i2c_master.h>
#include <driver/gpio.h>
#include <esp_rom_sys.h>         // esp_rom_delay_us（开机前解总线死锁用）
#include <exception>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_netif.h>
#include <string>
#include <cstdio>   // snprintf（dance 序列拼串）

#define TAG "C3Board"

extern "C" void HandleMotorActionForApplication(int direction, int speed, int duration_ms, int priority);
extern "C" void HandleMotorActionForEmotion(const char* emotion);
extern "C" void HandleMotorActionForDance(uint8_t speed_percent);
// 带时长的舞蹈（"跳 5 秒"）：旧单参数接口保持不变，供 web_server 调用
extern "C" void HandleMotorActionForDanceFor(uint8_t speed_percent, uint32_t duration_ms);
// 语音动作忙窗口登记 + 全局速度上限（网页「🐢 速度」滑条，NVS 持久化）
extern "C" void MarkMotorBusyForApp(uint32_t duration_ms);
extern "C" int  GetGlobalMaxSpeedPct();

// ─── 全局速度上限（网页「🐢 速度」滑条）────────────────────────────────
//   ⚠️ 2026-10-20 改法：**不再在各入口逐个乘**。
//   原来有个 ApplySpeedLimit() 自由函数在摇杆/MCP/探索/表情等 8 处各乘一次，
//   但凡是绕过这些入口的路径都不受控 —— 探索模式临边脱困的 Drive(0,-40)
//   （硬编码）、原地转 Drive(±100,0)、Motor2WD::Forward/Backward/... 单动作
//   接口、以及"解析时就把限速固化进步骤"的动作序列（运行中改滑条不生效）。
//   现在统一由 Motor2WD::ApplyTarget() 这个汇流点缩放，这里只负责同步数值。
extern "C" void OnGlobalMaxSpeedChanged(int pct) {
    Motor2WD::SetSpeedLimitPct(pct);
}

// ═══ 探索模式：随机溜达 + 遇障/临边自动转向 ═══════════════════════════════
//   语音："探索两分钟" → MCP self.car.explore duration_s=120
//   平时以 speed_pct 持续前进、偶尔小幅偏向；测距保护一旦进入 obstacle/fall，
//   立刻改成原地转（跌落先短退 250ms 脱困），约 0.7s 换一次方向，直到脱离危险。
//   停止：到时自动停 / action=stop / self.motor.stop / MCP stop。
//   ⚠️ 前进本身仍被 DistanceGuard 的钩子拦（这里只是"主动绕开"，不是绕过保护）。
class ExploreRunner {
public:
    static ExploreRunner& Instance() {
        static ExploreRunner runner;
        return runner;
    }

    // 返回 true = 已在执行（含延长已有任务）；false = 任务创建失败
    bool Start(Motor2WD* motor, uint32_t duration_ms, int speed_pct) {
        if (motor == nullptr) return false;
        motor_ = motor;
        if (speed_pct < 20) speed_pct = 20;
        if (speed_pct > 100) speed_pct = 100;   // 上限放到 100 = 完全交给滑块
        speed_pct_ = speed_pct;
        int64_t now = esp_timer_get_time() / 1000;
        start_ms_ = now;
        end_ms_ = now + (int64_t)duration_ms;
        if (running_) {
            // ⚠️ 必须撤销可能还挂着的停止请求：Stop() 只置标志，任务真正退出前
            //   running_ 仍是 true。这个窗口里再 Start() 的话，任务醒来会立刻
            //   退出 —— 用户看到"说探索，车不动"（第二次探索永远起不来）。
            stop_req_ = false;
            ESP_LOGI(TAG, "explore: 已延长到 %u 秒", (unsigned)(duration_ms / 1000));
            return true;
        }
        stop_req_ = false;
        running_ = true;
        // ⚠️ 栈 3072 → 4096：排查"玩着玩着莫名重启"（已确认复位原因是 PANIC）。
        //   这个任务里会调 Drive() → ApplyTarget() → SetDutyRaw() → ledc_*，
        //   调用链不算浅，3KB 余量偏紧。加大栈是最稳妥的低成本防御。
        if (xTaskCreate(Entry, "explore", 4096, this, 5, &task_) != pdPASS) {
            running_ = false;
            ESP_LOGE(TAG, "explore: task create failed");
            return false;
        }
        ESP_LOGI(TAG, "explore: 开始 %u 秒, 速度 %d%%", (unsigned)(duration_ms / 1000), speed_pct);
        return true;
    }

    void Stop() {
        if (!running_) return;
        stop_req_ = true;
        ESP_LOGI(TAG, "explore: 收到停止请求");
    }

    bool running() const { return running_; }

    std::string StatusJson() const {
        char buf[160];
        int64_t now = esp_timer_get_time() / 1000;
        long left = (running_ && (int64_t)end_ms_ > now) ? (long)(((int64_t)end_ms_ - now) / 1000) : 0;
        snprintf(buf, sizeof(buf), "{\"running\":%s,\"left_s\":%ld,\"speed_percent\":%d}",
                 running_ ? "true" : "false", left, (int)speed_pct_);
        return std::string(buf);
    }

private:
    static void Entry(void* arg) {
        auto* self = static_cast<ExploreRunner*>(arg);
        self->Loop();
        // ⚠️ 自删前先把句柄置空：vTaskDelete 之后 task_ 指向的 TCB 已被回收，
        //   留着就是悬空句柄，将来谁拿去 vTaskSuspend/Notify 就会踩到已释放内存。
        self->task_ = nullptr;
        vTaskDelete(nullptr);
    }

    void Loop() {
        auto& guard = DistanceGuard::Instance();
        int turn_dir = 0;
        int64_t turn_until = 0;
        int64_t last_random_turn = 0;
        bool was_danger = false;
        // 速度抖动是"低频变化"的：每 2.5~4 秒换一次值（见下方说明）。
        //   原来是每 tick 重新随机，会让占空比每 50ms 变一次、PWM 疯狂重写。
        int speed_jitter = 0;
        int64_t next_jitter_ms = 0;
        // 巡航转向的"保持窗口"：转向值一旦掷出就保持一小段时间，
        //   否则下一个 tick 就被归零，轮子根本来不及偏（等于不转弯）。
        int steer_current = 0;
        int64_t steer_until = 0;

        // 外层负责"刚停又启"的重入：Start() 只清 stop_req_/延长 end_ms_，
        //   如果我们恰好在内层循环判断之后退出，就会 running_=false 但没人跑。
        for (;;) {
            while (!stop_req_) {
                int64_t now = esp_timer_get_time() / 1000;
                if (now >= end_ms_) break;

                if (guard.danger()) {
                    if (!was_danger) {
                        was_danger = true;
                        // 刚撞上障碍：随机挑一个方向（左/右各半），
                        //   并且**持续时长也随机** 0.4~1.3s —— 固定 700ms 会让
                        //   每次避障摆头幅度一模一样，看着很机械。
                        //   ⚠️ 0.3~0.8s（原 0.4~1.3s）：太长的话车会一直"磨"在障碍前，
                        //   用户体感就是"拦截之后反应好久才脱身"。短一点更容易脱离。
                        turn_dir = (esp_random() & 1) ? 100 : -100;
                        turn_until = now + 300 + (int)(esp_random() % 500);
                        if (guard.state() == DistanceGuard::kFall) {
                            // 临边：先短退脱困（后退不受前进拦截），再原地转
                            // ⚠️ 原来这里硬编码 -40，跟前进用的 speed_pct_ 脱钩：
                            //   语音说"40% 速度探索"时前进 40%、脱困后退固定 40%
                            //   （碰巧一样），但转弯仍是满舵 100% —— 用户感觉
                            //   "前进和转弯不是一个速度"。现在全部按 speed_pct_ 缩放。
                            // ⚠️ 后退同样要保底：占空比太低时电机有电流声但不转。
                            //   60% 比例 + 保底 50，滑块 50% 时还有 25%，够脱困。
                            int back = speed_pct_ * 3 / 5;
                            if (back < 50) back = 50;
                            motor_->Drive(0, -back);
                            vTaskDelay(pdMS_TO_TICKS(250));
                            // ⚠️ 这 250ms 里用户可能按了停车：不重新检查就会
                            //   在停车之后又把车拉起来（实测"按停止后车还动一下"）。
                            if (stop_req_) break;
                            now = esp_timer_get_time() / 1000;  // 睡了 250ms，重取时间
                        }
                    }
                    if (stop_req_) break;
                    if (now >= turn_until) {      // 这个方向转够时间还没脱离 → 换一个
                        // ⚠️ **重新随机**而不是简单取反：
                        //   turn_dir = -turn_dir 会让车在"左-右-左-右"之间来回摆，
                        //   路线可预测、也容易卡在角落里反复对撞。
                        //   每次重新掷一次方向 + 重新随机时长，路线才自然。
                        turn_dir = (esp_random() & 1) ? 100 : -100;
                        turn_until = now + 300 + (int)(esp_random() % 500);
                    }
                    // ⚠️ 转弯舵量必须**保底**，不能跟着 speed_pct_ 一路降下去。
                    //   原因：Expo() 会把小舵**二次压缩** ——
                    //     speed_pct_=40 → Expo(40)=26 → 滑块 50% → 13
                    //     → 被 kMinMovingDuty(15) 抬到 15。
                    //   而原地转要克服轮子侧滑摩擦，15% 的占空比只有电流声、
                    //   没有转矩，电机嗡嗡响但轮子不转（实测"激光拦截转向时
                    //   有微微电流声不转了"）。
                    //   关键是 steer 同时决定**转矩和转速**，不像 throttle 那样
                    //   可以平滑降速：压 steer 就是压转矩，压过头就堵转。
                    //   所以保底 80（Expo(80)=68）：滑块 50% 时轮速 34，转得动；
                    //   滑块拉满时更利索。想"慢点转"应该调滑块，不是压 steer。
                    int turn_mag = speed_pct_;
                    if (turn_mag < 80) turn_mag = 80;
                    if (turn_mag > 100) turn_mag = 100;
                    int turn = (turn_dir > 0) ? turn_mag : -turn_mag;
                    motor_->Drive(turn, 0);   // 原地转：throttle=0，不受前进拦截
                } else {
                    was_danger = false;
                    // ⚠️ 速度跟随网页滑块（2026-10-22）。
                    //   原来固定 speed_pct_=55，而 ApplyTarget() 里还会再乘一次滑块，
                    //   于是滑块 50% 时实际只有 55×50%=27.5% —— 双重缩放，
                    //   用户怎么拉滑块探索都提不起速度。
                    //   现在 speed_pct_ 默认 100，滑块成为唯一速度来源：
                    //   滑块 60% → 探索就是 60%，所见即所得。
                    //   （语音仍可显式指定更低值，如"用 40% 速度探索"，
                    //     那时 40 作为上限，再乘滑块符合"再慢一点"的直觉。）
                    // ── 速度抖动：**每 2.5~4 秒才换一次**，不是每个 tick 都掷骰子 ──
                    //   ⚠️ 原写法 `speed_pct_ + esp_random()%13-6` 每个 tick（50ms）
                    //   都重新随机 → 目标占空比每 50ms 变一次 → Motor2WD 里的
                    //   duty_cache_ 永不命中 → 每 tick 写 4 路 LEDC
                    //   （ledc_set_duty + ledc_update_duty 各进临界区关中断）。
                    //   这跟"吞音"是同一条根因链：关中断推迟了音频 I2S 中断。
                    //   改成低频变化后，匀速段占空比恒定 → 缓存命中 → **零 LEDC 写入**，
                    //   只有转向时才真正写 PWM。
                    if (now >= next_jitter_ms) {
                        next_jitter_ms = now + 2500 + (int)(esp_random() % 1500);
                        speed_jitter = (int)(esp_random() % 25) - 12;    // ±12%
                    }
                    int speed = speed_pct_ + speed_jitter;
                    if (speed > speed_pct_) speed = speed_pct_;              // 抖动不能超上限
                    if (speed < 1) speed = 1;

                    // ── 巡航随机转向 ──
                    //   ⚠️ 关键：转向必须**保持一段时间**，不能只生效一个 tick。
                    //   原来的写法在触发时给 steer 赋值，但下一个 tick（25ms 后）
                    //   `now - last_random_turn` 只有 25，不满足 >600 的条件 → steer
                    //   立刻归 0。等于转向是个 25ms 的脉冲，轮子还没偏过去就被拉直，
                    //   肉眼看到的就是"不转弯"或"一直朝一个方向缓慢漂"（用户反馈）。
                    //   现在用一个独立的保持窗口：触发时随机方向 + 随机保持 0.3~0.6s。
                    int steer = 0;
                    if (now < steer_until) {
                        steer = steer_current;                     // 保持当前转向
                    } else if (now - last_random_turn > 1000 &&
                               (esp_random() % 100) < 60) {
                        // 间隔 1000ms / 概率 60% / 幅度 ±55：
                        //   触发频率约 0.6 次/秒，每次保持 0.3~0.6s，
                        //   大约 25% 的时间处于转弯状态 —— 路线会自然弯曲，
                        //   又不至于一直扭来扭去走不出距离。
                        // 方向左右各半（esp_random() 是硬件真随机）。
                        last_random_turn = now;
                        steer_current = (int)(esp_random() % 111) - 55;   // -55~+55，正负都有
                        steer_until = now + 300 + (int)(esp_random() % 300);
                        steer = steer_current;
                    }
                    motor_->Drive(steer, speed);   // 持续前进（无 duration，靠下一条指令/停止收尾）
                }
                // ⚠️ 25ms（原 120ms → 50ms → 25ms）：这是**避障反应时间的上限**。
                //   每 25ms 检查一次 guard.danger()，配合 distance_guard 的 30Hz
                //   采样（33ms 一帧），实际响应延迟已经基本等于传感器本身的帧间隔，
                //   再往下压（比如 10ms）就没有意义了 —— 传感器出不了更快的数。
                //   ✅ 不会因此加重吞音：上面已经把速度抖动改成低频变化，
                //      匀速段占空比恒定 → duty_cache_ 命中 → 不写 LEDC，
                //      所以这轮循环只做"读变量 + 比较"，几乎零成本。
                //   ⚠️ 若将来又往这个循环里加"每 tick 都改占空比"的逻辑，
                //      务必同步把这里放慢，否则会重新踩回吞音那条链。
                vTaskDelay(pdMS_TO_TICKS(25));
            }

            // 退出前再确认：这期间 Start() 可能撤销了 stop_req_ 或延长了 end_ms_
            int64_t now = esp_timer_get_time() / 1000;
            if (stop_req_ || now >= end_ms_) break;
        }

        if (motor_ != nullptr) motor_->ApplyStop();
        running_ = false;
        ESP_LOGI(TAG, "explore: 结束");
    }

    Motor2WD* motor_ = nullptr;
    TaskHandle_t task_ = nullptr;
    volatile bool running_ = false;
    volatile bool stop_req_ = false;
    // ⚠️ end_ms_ 必须是 64 位：原来存成 uint32，Loop 里却和 int64 now 比，
    //   开机 49.7 天后 end_ms_ 回绕成小值 → now >= end_ms_ 恒真 →
    //   探索模式刚启动就立刻退出。
    volatile int64_t start_ms_ = 0;
    volatile int64_t end_ms_ = 0;
    // 默认 100 = 完全交给网页滑块。以前是 55，导致"滑块拉满也只有 55%"，
    //   再叠加 ApplyTarget 的滑块乘数就成了双重缩放（见 Loop 里的说明）。
    volatile int speed_pct_ = 100;
};

class CompactWifiBoard : public WifiBoard {
private:
    Button boot_button_;               // 板载 BOOT 按键（GPIO0：板载上拉 + 按下接地）
    bool boot_cfg_key_held_ = false;   // 开机瞬间 BOOT 是否按着 → 重新配网
    bool boot_click_to_consume_ = false;  // 配网抬手那次"点击"要吃掉，别切成音频自测
    i2c_master_bus_handle_t display_i2c_bus_ = nullptr;
    esp_lcd_panel_io_handle_t panel_io_ = nullptr;
    esp_lcd_panel_handle_t panel_ = nullptr;
    Display* display_ = nullptr;
    Motor2WD* motor_ = nullptr;
    bool oled_recovery_started_ = false;   // 屏幕自愈任务只起一次
    int boot_line_state_ = -1;             // 建总线后 5ms 的原始 SDA/SCL 电平
    int line_after_unstick_ = -1;          // 开机抖完 9 个时钟、总线还没建时的电平

    void InitializeDisplayI2c() {
        // 上电时先抖开总线：上次断电/干扰可能让从机卡在半个字节里，
        //   它就一直拉着 SDA/SCL 等剩下的时钟 → 后面 probe 全 timeout。
        //   必须在 i2c_new_master_bus 之前做（那时引脚还是普通 GPIO）。
        UnstickI2cBusColdBoot();
        line_after_unstick_ = I2cLineState();   // 引脚已配上拉，这次读数有效
        i2c_master_bus_config_t bus_config = {};
        bus_config.i2c_port = (i2c_port_t)0;
        bus_config.sda_io_num = DISPLAY_SDA_PIN;
        bus_config.scl_io_num = DISPLAY_SCL_PIN;
        bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
        bus_config.glitch_ignore_cnt = 7;
        bus_config.trans_queue_depth = 0;
        bus_config.flags.enable_internal_pullup = 1;
        ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &display_i2c_bus_));
        // 建总线后立刻采一次电平 —— 此刻一个事务都还没发生。
        //   若这时就 SCL=0 → 开机就被外部拉死（模块没供电 / 接线反 / 短地）；
        //   若这时是 1、运行中才变 0 → 是 I2C 外设自己卡住（另一类问题）。
        vTaskDelay(pdMS_TO_TICKS(5));
        boot_line_state_ = I2cLineState();
        ESP_LOGW(TAG, "I2C idle lines after bus init: %s", LineStateText(boot_line_state_));
    }

    // ── 物理电平诊断（只读，绝不动 I2C 状态机）─────────────────────
    //   IDF 的 i2c 驱动用 GPIO 矩阵（PIN_FUNC_GPIO + gpio_input_enable +
    //   开漏输出）配置 SDA/SCL，所以 gpio_get_level() 读到的就是真实线电平。
    //   空闲时两根线必须都是 1（模块上拉 + 这里开的内部上拉 45k）。
    //   读到 0 只有两种可能，固件都救不了，别再调速率/时序：
    //     ① 线被短到 GND；② 某个从设备没供电 —— 未上电芯片的 ESD 保护
    //        二极管会把 SDA/SCL 钳在低电平，整条总线一起死。
    //   这就是"插上屏幕后连测距也不 ACK / i2c.master: clear bus failed"的典型成因。
    int I2cLineState() {   // bit1 = SDA, bit0 = SCL
        int sda = gpio_get_level((gpio_num_t)DISPLAY_SDA_PIN);
        int scl = gpio_get_level((gpio_num_t)DISPLAY_SCL_PIN);
        return (sda << 1) | scl;
    }

    static const char* LineStateText(int st) {
        if (st < 0) return "unknown";
        switch (st) {
            case 3:  return "SDA=1 SCL=1 (idle, ok)";
            case 1:  return "SDA=0 SCL=1 (SDA held low!)";
            case 2:  return "SDA=1 SCL=0 (SCL held low!)";
            default: return "SDA=0 SCL=0 (both held low!!)";
        }
    }

    // ── 开机前解总线死锁：只用 GPIO 打 9 个 SCL，再补一个 STOP ──────────
    //   为什么不用 i2c_master_bus_reset()：见 InitializeSsd1306Display 里的警示，
    //   这个版本它会返回 ESP_ERR_INVALID_STATE 并把状态弄得更坏。
    //   死锁成因：从机（SSD1306/VL53L0X）在半个字节中间被干扰打断，它就一直
    //   等剩下的时钟、把 SDA 拉低不放 → 主机后面每个事务都 timeout，日志是
    //   `probe device timeout` + `clear bus failed`。解药就是给它 9 个时钟。
    // ⚠️⚠️ 只能在 i2c_new_master_bus() **之前**调用（此时引脚是自由 GPIO）。
    //   2026-10-19 实测：运行期把引脚从 I2C 外设手里抢过来抖时钟 → 外设收到
    //   假的边沿触发 RX FIFO 中断，ISR 里拿着过期指针写数据 →
    //   `Guru Meditation (Store access fault)` @ i2c_isr_receive_handler，
    //   19 秒一次稳定重启。所以运行期【绝不】动引脚，宁可等用户插紧线。
    void UnstickI2cBusColdBoot() {
        const auto sda = (gpio_num_t)DISPLAY_SDA_PIN;
        const auto scl = (gpio_num_t)DISPLAY_SCL_PIN;
        gpio_config_t io = {};
        io.pin_bit_mask = (1ULL << (int)sda) | (1ULL << (int)scl);
        io.mode = GPIO_MODE_INPUT_OUTPUT_OD;
        io.pull_up_en = GPIO_PULLUP_ENABLE;
        io.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io.intr_type = GPIO_INTR_DISABLE;
        gpio_config(&io);
        gpio_set_level(sda, 1);
        gpio_set_level(scl, 1);
        esp_rom_delay_us(10);
        for (int i = 0; i < 9; ++i) {          // 9 个时钟把半个字节吐完
            gpio_set_level(scl, 0); esp_rom_delay_us(5);
            gpio_set_level(scl, 1); esp_rom_delay_us(5);
        }
        gpio_set_level(sda, 0); esp_rom_delay_us(5);   // 补一个 STOP：
        gpio_set_level(scl, 0); esp_rom_delay_us(5);   //   SCL 低 → 高 → SDA 低 → 高
        gpio_set_level(scl, 1); esp_rom_delay_us(5);
        gpio_set_level(sda, 1); esp_rom_delay_us(5);
        // 引脚状态交给后面的 i2c_new_master_bus 重新配置，这里不用还原信号
    }

    // 全总线扫描（诊断用）：把 0x08~0x77 里所有 ACK 的地址打出来。
    //   0x29 = VL53L0X，0x3C/0x3D = SSD1306。
    //   如果只看到 0x29、看不到 0x3C → 屏幕那侧的线/电源/模块真的有问题，
    //   不要再怀疑固件（同一根线上 0x29 能通就证明总线与上拉是好的）。
    void ScanI2CBus() {
        ESP_LOGW(TAG, "--- I2C scan start (SDA=%d SCL=%d) ---", (int)DISPLAY_SDA_PIN,
                 (int)DISPLAY_SCL_PIN);
        int found = 0;
        for (uint8_t a = 0x08; a < 0x78; ++a) {
            if (i2c_master_probe(display_i2c_bus_, a, 50) == ESP_OK) {
                ESP_LOGW(TAG, "  ACK 0x%02X", a);
                found++;
            }
        }
        ESP_LOGW(TAG, "--- I2C scan done, %d device(s) ---", found);
    }

    void InitializeSsd1306Display() {
        // ⚠️ 2026-10-19：reflash 后 RTS 复位，OLED 自己还在内部上电复位里，
        //   这时候往 0x3C 写 init 命令会 NACK → panel_init 失败 → 走
        //   OledDisplay(nullptr,nullptr) 占位分支：渲染照跑、PushFrame 直接 return，
        //   **屏幕就永远停在上一次开机残留的画面上**（用户看到"卡在 WiFi..."、
        //   上一次是"元素乱套"，其实都是这个）。
        //   这里先等 OLED 上电稳定，再反复探测 0x3C，确认真在线才建 panel。
        vTaskDelay(pdMS_TO_TICKS(80));
        // ⚠️⚠️ 绝对不要在这里调 i2c_master_bus_reset()（2026-10-19 血的教训）：
        //   它在"总线上已有设备"时返回 ESP_ERR_INVALID_STATE，但**已经把总线状态弄坏**，
        //   之后连测距模块也 probe timeout。任何情况下都别再调这个 API。
        // 有些 SSD1306 模块的地址跳线是 0x3D，两个都探一下（之前只会试 0x3C）
        uint8_t oled_addr = 0x3C;
        bool acked = false;
        // 实测这块屏上电到能 ACK 要接近 1 秒（0x29 早就通了它还没醒），
        //   所以给到 12 次 × 80ms ≈ 1s；再晚就交给后台自愈任务接管。
        for (int i = 0; i < 12 && !acked; ++i) {
            // 区分两种失败（这是判断"软件 or 硬件"的唯一依据）：
            //   ESP_ERR_NOT_FOUND = 从机回了 NACK  → 线是好的，只是该地址没设备
            //   ESP_ERR_TIMEOUT   = SCL 被拉住不放（>20ms）→ 总线物理卡死
            esp_err_t e3c = i2c_master_probe(display_i2c_bus_, 0x3C, 20);
            if (e3c == ESP_OK) {
                oled_addr = 0x3C; acked = true; break;
            }
            esp_err_t e3d = i2c_master_probe(display_i2c_bus_, 0x3D, 20);
            if (e3d == ESP_OK) {
                oled_addr = 0x3D; acked = true; break;
            }
            int lines = I2cLineState();
            ESP_LOGW(TAG, "OLED no ACK (attempt %d/6): 0x3C=%s 0x3D=%s | %s",
                     i + 1, esp_err_to_name(e3c), esp_err_to_name(e3d), LineStateText(lines));
            if ((lines & 0x3) != 0x3) {
                // 线被钳在低电平 → 再探 5 次也只是白等 5×(2×20ms) 超时
                ESP_LOGE(TAG, "I2C 线被拉死在低电平！查 OLED/GY-530 的 VCC/GND 是否真的接上"
                              "（模块不上电会把总线钳死）、SDA(GPIO%d)/SCL(GPIO%d) 是否短地。"
                              "拔掉 OLED 再上电，0x29 若恢复 → 就是屏模块/它的接线问题",
                         (int)DISPLAY_SDA_PIN, (int)DISPLAY_SCL_PIN);
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(80));
        }
        if (acked) {
            ESP_LOGI(TAG, "OLED ACK ok at 0x%02X, creating panel", oled_addr);
        } else {
            // 探测不到也别急着放弃：可能是上拉/线长问题，仍然走正常流程试一次
            ESP_LOGE(TAG, "OLED still no ACK at 0x3C/0x3D after retries — will try anyway");
            ESP_LOGE(TAG, "  lines now: %s | after unstick: %s | after bus init: %s",
                     LineStateText(I2cLineState()), LineStateText(line_after_unstick_),
                     LineStateText(boot_line_state_));
        }

        esp_lcd_panel_io_i2c_config_t io_config = {};
        io_config.dev_addr = oled_addr;
        io_config.control_phase_bytes = 1;
        io_config.dc_bit_offset = 6;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        // 400k → 200k：同一根总线上还有 VL53L0X 每 10~28ms 一次事务，
        //   400k 下实测 OLED 整帧写（1024B）偶发 NACK（SSD1306_CMD_SET_COLUMN_RANGE
        //   failed）。降到 200k 后整帧约 46ms，150ms 周期仍够用，稳定性明显好。
        // 100k：这块屏 + 现有杜邦线/内部上拉在 200k 下仍会偶发 NACK 和花屏
        //   （花屏 = 命令成功但数据部分失败，写到了错误的光标位置）。
        //   100k 下整帧 1024B ≈ 92ms，所以刷屏周期同时放宽到 250ms。
        io_config.scl_speed_hz = 100 * 1000;
        esp_err_t err = esp_lcd_new_panel_io_i2c_v2(display_i2c_bus_, &io_config, &panel_io_);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "panel_io create failed (%s), skip OLED", esp_err_to_name(err));
            return;
        }

        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = -1;
        panel_config.bits_per_pixel = 1;
        esp_lcd_panel_ssd1306_config_t ssd1306_config = {
            .height = (uint8_t)DISPLAY_HEIGHT,
        };
        panel_config.vendor_config = &ssd1306_config;
        err = esp_lcd_new_panel_ssd1306(panel_io_, &panel_config, &panel_);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "ssd1306 create failed (%s), skip OLED", esp_err_to_name(err));
            return;
        }

        // ⚠️ 关键：panel_init 会真的往 0x3C 写命令，无 OLED 时 I2C 无 ACK
        //    不能 ESP_ERROR_CHECK，否则 abort + 看门狗重启死循环
        err = esp_lcd_panel_init(panel_);
        // 第一次 NACK 不代表屏坏（上电时序 / 总线毛刺），再给两次机会
        for (int attempt = 0; err != ESP_OK && attempt < 2; ++attempt) {
            ESP_LOGW(TAG, "OLED panel_init retry %d (%s)", attempt + 1, esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(100));
            err = esp_lcd_panel_init(panel_);
        }
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "OLED panel_init failed (%s) — no display on I2C bus. "
                          "降级为无显示模式（OledDisplay(nullptr, nullptr) 占位）",
                     esp_err_to_name(err));
            // 清理已分配的资源
            esp_lcd_panel_del(panel_);
            esp_lcd_panel_io_del(panel_io_);
            panel_ = nullptr;
            panel_io_ = nullptr;
            // ⚠️ 2026-10-19 关键修复：**不要删 I2C 总线**。
            //   OLED 偶尔 panel_init 失败（接触/上电时序），但总线本身是好的，
            //   测距模块（VL53L0X@0x29）还挂在上面。原来这里 i2c_del_master_bus +
            //   置 nullptr，导致 DistanceGuard 的 bus_ 为空 → 整个防碰撞/防跌落
            //   静默失效（日志只剩一条 "I2C bus is null"）。
            //   ESP32-C3 只有一个 I2C 控制器，删了也重建不回来。
            // display_i2c_bus_ 保留给测距用。
            // 仍创建一个 OledDisplay(nullptr, nullptr) 占位 —— 这样 application.cc
            // 调 display->SetEmotion / SetStatus / ShowNotification 不会 segfault
            // (OledDisplay 内部已支持 panel_=nullptr：PushFrame 立即返回，Lock
            //  也判 nullptr；其他方法都基于 fb_ 栈数组，panel 没参与)
            display_ = new OledDisplay(nullptr, nullptr,
                                       DISPLAY_WIDTH, DISPLAY_HEIGHT,
                                       DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
            // 后台每 5s 重探一次：线/供电恢复后自动接管，不用重启
            StartOledRecovery();
            return;
        }

        // ⚠️ 关键 bug 修复：esp_lcd_panel_ssd1306 的 init() 函数（v5.5.x）只配置了
        //    charge pump / multiplex / COM pins，但**没有发 0xAF (DISP_ON)**！
        //    结果 RAM 写满了但屏幕一直黑屏。手动补一行打开显示。
        //    参数 true = DISP_ON；false = DISP_OFF（API 名字易混淆）
        esp_err_t on_ret = esp_lcd_panel_disp_on_off(panel_, true);
        if (on_ret != ESP_OK) {
            ESP_LOGW(TAG, "esp_lcd_panel_disp_on_off returned %s", esp_err_to_name(on_ret));
        }

        display_ = new OledDisplay(panel_io_, panel_, DISPLAY_WIDTH, DISPLAY_HEIGHT,
                                   DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
    }

    // 运行期重建面板（自愈用）：io + panel + init + DISP_ON，成功返回 true
    bool TryCreateOledPanel(uint8_t addr) {
        esp_lcd_panel_io_i2c_config_t io_config = {};
        io_config.dev_addr = addr;
        io_config.control_phase_bytes = 1;
        io_config.dc_bit_offset = 6;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        io_config.scl_speed_hz = 100 * 1000;
        if (esp_lcd_new_panel_io_i2c_v2(display_i2c_bus_, &io_config, &panel_io_) != ESP_OK) {
            panel_io_ = nullptr;
            return false;
        }
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = -1;
        panel_config.bits_per_pixel = 1;
        esp_lcd_panel_ssd1306_config_t ssd1306_config = {
            .height = (uint8_t)DISPLAY_HEIGHT,
        };
        panel_config.vendor_config = &ssd1306_config;
        if (esp_lcd_new_panel_ssd1306(panel_io_, &panel_config, &panel_) != ESP_OK) {
            esp_lcd_panel_io_del(panel_io_);
            panel_io_ = nullptr;
            panel_ = nullptr;
            return false;
        }
        esp_err_t err = esp_lcd_panel_init(panel_);
        if (err != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(100));
            err = esp_lcd_panel_init(panel_);
        }
        if (err != ESP_OK) {
            esp_lcd_panel_del(panel_);
            esp_lcd_panel_io_del(panel_io_);
            panel_ = nullptr;
            panel_io_ = nullptr;
            return false;
        }
        esp_lcd_panel_disp_on_off(panel_, true);   // v5.5.x init 不发 DISP_ON
        return true;
    }

    // 屏幕自愈：开机时 OLED 不 ACK（接线/供电/上电时序）会导致整个会话都没显示。
    //   每 5s 重探 0x3C / 0x3D，一旦有 ACK 就重建面板并 AttachPanel 接管，
    //   用户重新插好线即可恢复，不用重启。
    void StartOledRecovery() {
        if (oled_recovery_started_) return;
        oled_recovery_started_ = true;
        xTaskCreate([](void* arg) {
            auto* self = static_cast<CompactWifiBoard*>(arg);
            // 前 60 秒每 2s 试一次（实测屏幕可能开机十几秒后才开始 ACK，
            //   等太久用户会以为彻底坏了）；之后每 10s 一次，长期兜底。
            int fast_left = 30;
            for (;;) {
                vTaskDelay(pdMS_TO_TICKS(fast_left > 0 ? 2000 : 10000));
                if (fast_left > 0) fast_left--;
                if (self->display_i2c_bus_ == nullptr) continue;
                auto* od = static_cast<OledDisplay*>(self->display_);
                if (!od || !od->NeedsPanelRepair()) continue;   // 屏好好的就别动
                // 先摘掉渲染侧指针，再释放底层资源，避免推帧时用已释放句柄
                od->DetachPanel();
                if (self->panel_) { esp_lcd_panel_del(self->panel_); self->panel_ = nullptr; }
                if (self->panel_io_) { esp_lcd_panel_io_del(self->panel_io_); self->panel_io_ = nullptr; }
                // ⚠️ 这里不要再调 i2c_master_bus_reset：实测返回 ESP_ERR_INVALID_STATE，
                //   而且驱动自己会打 E 级日志 —— 每 2 秒一条会刷屏。
                // 总线被钳低时 probe 只会刷 "probe device timeout" / "clear bus failed"，
                //   先读电平，不 OK 就静默跳过（30s 提示一次），别去打扰一根坏掉的总线。
                // ⚠️ 运行期【不要】去抖时钟救总线：2026-10-19 实测，
                //   把引脚从 I2C 外设手里抢过来会让外设收到假边沿 →
                //   ISR 拿过期指针写入 → Store access fault，十几秒一次重启。
                //   线被钳低时只读电平 + 10s 重试，等用户把线插好。
                int lines = self->I2cLineState();
                if ((lines & 0x3) != 0x3) {
                    static int64_t last_warn_ms = 0;
                    int64_t now_ms = esp_timer_get_time() / 1000;
                    if (now_ms - last_warn_ms > 30000) {
                        last_warn_ms = now_ms;
                        ESP_LOGW(TAG, "oled_recover: %s | after unstick: %s | after bus init: %s"
                                      " — 接线/供电问题，软件不再重试",
                                 LineStateText(lines),
                                 LineStateText(self->line_after_unstick_),
                                 LineStateText(self->boot_line_state_));
                    }
                    continue;
                }
                uint8_t addr = 0;
                if (i2c_master_probe(self->display_i2c_bus_, 0x3C, 10) == ESP_OK) addr = 0x3C;
                else if (i2c_master_probe(self->display_i2c_bus_, 0x3D, 10) == ESP_OK) addr = 0x3D;
                if (addr == 0) continue;
                ESP_LOGW(TAG, "OLED responded at 0x%02X, rebuilding panel", addr);
                if (!self->TryCreateOledPanel(addr)) continue;
                od->AttachPanel(self->panel_io_, self->panel_);
            }
        }, "oled_recover", 3072, this, 2, nullptr);
    }

    // ── 距离传感器探测（VL53L0X @ 0x29）──────────────────────────────
    //   ⚠️ ESP32-C3 只有 1 个 I2C 控制器（I2C_NUM_0），已被 OLED 占用，
    //      所以这里【必须】复用 display_i2c_bus_，绝不能新建总线 ——
    //      再调 i2c_new_master_bus(I2C_NUM_0) 会返回 ESP_ERR_INVALID_STATE。
    //      这正是"S3 板装距离传感器与屏幕共用总线冲突"最可能的成因。
    //   VL53L0X 与 SSD1306 地址不同（0x29 vs 0x3C），可共存于同一总线。
    //   本函数只验证"在不在"（读 MODEL_ID），真正的连续测距等模块到货再接。
    std::string ProbeDistanceSensor() {
        if (display_i2c_bus_ == nullptr) {
            return "{\"found\":false,\"err\":\"i2c bus not initialized\"}";
        }

        i2c_device_config_t dev_cfg = {};
        dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
        dev_cfg.device_address = 0x29;      // VL53L0X 出厂默认 7 位地址
        dev_cfg.scl_speed_hz = 400 * 1000;

        i2c_master_dev_handle_t dev = nullptr;
        esp_err_t err = i2c_master_bus_add_device(display_i2c_bus_, &dev_cfg, &dev);
        if (err != ESP_OK) {
            return std::string("{\"found\":false,\"err\":\"add_device: ") + esp_err_to_name(err) + "\"}";
        }

        // VL53L0X 寄存器为 8 位地址：0xC0 = IDENTIFICATION__MODEL_ID，正常读回 0xEE
        uint8_t reg = 0xC0;
        uint8_t model_id = 0;
        err = i2c_master_transmit_receive(dev, &reg, 1, &model_id, 1, 100);

        // 只移除本设备，不销毁总线 —— OLED 仍在正常使用同一条总线
        i2c_master_bus_rm_device(dev);

        if (err != ESP_OK) {
            return std::string("{\"found\":false,\"err\":\"no ack at 0x29: ") + esp_err_to_name(err) + "\"}";
        }

        return std::string("{\"found\":true,\"addr\":\"0x29\",\"model_id\":")
             + std::to_string((int)model_id) + "}";
    }

    void InitializeMotor() {
        motor_ = new Motor2WD(
            (gpio_num_t)MOTOR_LF_GPIO, (gpio_num_t)MOTOR_LB_GPIO,
            (gpio_num_t)MOTOR_RF_GPIO, (gpio_num_t)MOTOR_RB_GPIO);
        motor_->InitPWM();
        // ⚠️ 这里**绝对不能**调 GetGlobalMaxSpeedPct()：它走
        //   Application::GetInstance() 单例，而 board 初始化时 Application
        //   还没构造好 → 循环依赖，实测会把启动卡死在 app_init 之后
        //   （后续日志全无、屏幕不亮，就是这个症状）。
        //   开机同步改由 Application 初始化完成、从 NVS 恢复出 max_spd 之后，
        //   通过 OnGlobalMaxSpeedChanged() 回调灌进来（见 application.cc）。
    }

    // 板载 BOOT 按键：一个键走完「唤醒 / 打断 / 回时钟」
    //   空闲（时钟界面） → 按下 = 唤醒，开始聆听
    //   说话中           → 按下 = 打断（AbortSpeaking，服务端停止下发音频）
    //   聆听中           → 按下 = 关闭聆听 → 状态自动回 idle → 恢复时钟界面
    // ⚠️ 这三段逻辑 Application::HandleToggleChatEvent() 里全都已经实现好了，
    //    所以按键只负责调 ToggleChatState()，这里**不要**自己去动
    //    EnableVoiceProcessing / ShowClock —— 状态机统一管，手动改会和 wake word 抢麦。
    // 开机那一刻按着 BOOT（GPIO0 低）= 要求重新配网。
    //   ⚠️ 连采 8 次滤上电抖动；GPIO0 不是 C3 的 strapping 脚（C3 是 2/8/9），
    //      所以按住开机不会进 ROM 下载模式，可以安全拿来当"配网键"。
    static bool SampleBootKeyAtPowerOn() {
        // ⚠️ 不要重新 gpio_config()：boot_button_ 的构造函数（成员初始化列表，
        //   早于本函数执行）已经把 GPIO0 配成输入并交给 iot_button 管理。
        //   再配一次（intr_type=DISABLE）会把它的中断关掉 → 按键整个失效。
        //   GPIO0 板载有外部上拉，按下接地，直接读电平即可。
        int low = 0;
        for (int i = 0; i < 8; ++i) {
            if (gpio_get_level(BOOT_BUTTON_GPIO) == 0) ++low;
            esp_rom_delay_us(2000);
        }
        return low >= 7;
    }

    void InitializeBootButton() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            ESP_LOGI(TAG, "BOOT 按下, 当前状态=%d", (int)app.GetDeviceState());
            // 开机按住 BOOT 进配网后，抬手会产生一次"点击"：
            //   WifiConfiguring 状态下点击会切到音频自测模式（HandleToggleChatEvent），
            //   这里把这一次消费掉，避免刚配网就被切走。
            if (boot_click_to_consume_) {
                boot_click_to_consume_ = false;
                ESP_LOGI(TAG, "BOOT 开机配网抬手：忽略这次点击");
                return;
            }
            // ⚠️ 开机早期按 BOOT 的反馈（2026-10-22，用户反馈"反应很慢"）：
            //   设备此时还在 Starting/Connecting —— WiFi 连上要 7 秒，
            //   MQTT + TLS 握手再要 7 秒（实测 11.6s 发起、18.6s 才连上），
            //   这期间 ToggleChatState() 发起的会话建立注定失败/长时间等待，
            //   用户感受就是"按了没反应、过一会儿才好"。
            //   这里给一条屏幕提示（UTF8 字库，任何显示模式都能看见），
            //   然后正常返回 —— 重在消除"是不是坏了"的疑虑。
            const auto boot_state = app.GetDeviceState();
            if (boot_state == kDeviceStateStarting ||
                boot_state == kDeviceStateConnecting) {
                if (display_ != nullptr) {
                    static_cast<OledDisplay*>(display_)->ShowNotice("网络连接中，请稍候");
                }
                ESP_LOGW(TAG, "BOOT 按下但系统未就绪（state=%d，仍在连接服务），已提示等待",
                         (int)boot_state);
                return;
            }

            // 探索模式优先打断：正在乱跑时第一下按键 = 立刻停车，不切对话
            //   （否则按一下变成"开始聆听"，车还在跑，很危险）
            if (ExploreRunner::Instance().running()) {
                ExploreRunner::Instance().Stop();
                if (motor_ != nullptr) motor_->Stop();
                ESP_LOGI(TAG, "BOOT 打断探索模式");
                return;
            }
            app.ToggleChatState();
        });

        // 双击 BOOT：切换屏幕显示模式（唤醒后的文字 / 表情）
        //   ⚠️ 单击和双击不冲突：iot_button 在判定为双击时不会再发 SINGLE_CLICK 事件，
        //      所以双击不会顺带把对话状态也切一下。
        //   状态持久化到 NVS，下次开机记住。
        boot_button_.OnDoubleClick([this]() {
            auto* od = static_cast<OledDisplay*>(display_);
            if (od == nullptr) return;
            const bool next = !od->IsChatTextMode();
            od->SetChatTextMode(next);
            Settings settings("display", true);
            settings.SetBool("chat_text", next);
            ESP_LOGI(TAG, "BOOT 双击：屏幕模式 -> %s", next ? "文字" : "表情");
        });
    }

    void RegisterMcpTools() {
        auto& mcp = McpServer::GetInstance();

        // ── 距离传感器（VL53L0X @0x29，与 OLED 共用 GPIO8/9 总线）────────
        //   防碰撞 + 防跌落由 DistanceGuard 后台任务持续跑（约 30Hz），
        //   这里只读快照、不阻塞 —— 工具回调跑在 Application 主循环里。
        mcp.AddTool("self.sensor.get_distance",
            "Read the VL53L0X ToF sensor. JSON: mm (8191 = out of range), "
            "state (clear/obstacle/fall/unknown/no_sensor), blocked_forward, age_ms, fps, "
            "obstacle_mm, fall_mm. Use for 'how far is it' / 'is it safe to go forward'.",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                return DistanceGuard::Instance().StatusJson();
            });

        // ── 安全层开关 / 阈值 ────────────────────────────────────────
        mcp.AddTool("self.car.safety",
            "Anti-collision/anti-fall control. action=status|on|off|reset|set. "
            "set takes obstacle_mm (mm <= this = obstacle) and fall_mm (mm >= this = table edge). "
            "Forward is refused while danger is latched; use reset if it refuses with nothing in front.",
            PropertyList({
                Property("action", kPropertyTypeString),
                Property("obstacle_mm", kPropertyTypeInteger, 0, 0, 3000),
                Property("fall_mm", kPropertyTypeInteger, 0, 0, 8000)
            }),
            [this](const PropertyList& p) -> ReturnValue {
                std::string a = p["action"].value<std::string>();
                auto& guard = DistanceGuard::Instance();
                if (a == "on") {
                    guard.SetEnabled(true);
                } else if (a == "off") {
                    guard.SetEnabled(false);
                } else if (a == "reset") {
                    guard.ResetState();
                } else if (a == "set") {
                    auto cfg = guard.config();
                    int obs = p["obstacle_mm"].value<int>();
                    int fall = p["fall_mm"].value<int>();
                    if (obs > 0) cfg.obstacle_mm = obs;
                    if (fall > 0) cfg.fall_mm = fall;
                    guard.SetThresholds(cfg.obstacle_mm, cfg.fall_mm);
                }
                return guard.StatusJson();
            });

        // ── 探索模式：随机溜达 + 自动避障/防跌落 ──────────────────────
        mcp.AddTool("self.car.explore",
            "Random-walk exploration with automatic obstacle/edge avoidance. "
            "action=start (duration_s 5-600, default 60; speed_percent 20-100, default 100 "
            "= follow the web speed slider) | stop | status. "
            "'explore for two minutes' -> duration_s=120.",
            PropertyList({
                Property("action", kPropertyTypeString),
                Property("duration_s", kPropertyTypeInteger, 60, 5, 600),
                Property("speed_percent", kPropertyTypeInteger, 100, 20, 100)
            }),
            [this](const PropertyList& p) -> ReturnValue {
                std::string a = p["action"].value<std::string>();
                auto& ex = ExploreRunner::Instance();
                if (a == "stop") {
                    ex.Stop();
                    motor_->Stop();   // 用 Stop()：顺带中止序列，别让跳舞在"停"之后继续
                    // ⚠️ 不要 vTaskDelay 等探索任务退出：这里跑在 Application
                    //   主循环，等 60ms 会把音频采集/唤醒词/屏幕全冻住。
                    //   探索任务在每次 Drive 前都会复查 stop_req_（见 Loop），
                    //   已经不会再"复活"电机。
                    return ex.StatusJson();
                }
                if (a == "start") {
                    int sec = p["duration_s"].value<int>();
                    int spd = p["speed_percent"].value<int>();
                    if (sec < 5) sec = 5;
                    // 探索也受全局速度上限约束（原来只有摇杆/MCP 动作受控）
                    // 只有真的跑起来才标记 busy：否则任务创建失败时，
                    //   MarkMotorBusy 会把装饰动作/表情压住整整 sec 秒。
                    if (ex.Start(motor_, (uint32_t)sec * 1000u, spd)) {
                        // 探索期间挡住唤醒/说话的装饰性动作，避免刚起步就被掐断
                        MarkMotorBusyForApp((uint32_t)sec * 1000u);
                    }
                }
                return ex.StatusJson();
            });

        // （速度上限已下沉到 Motor2WD::ApplyTarget() 汇流点，此处无需再乘）

        // ── ⚠️ 这里删掉了 move_forward / move_backward / turn_left / turn_right ──
        //   2026-10-19：tools/list 的 payload 涨到 6569 字节，C3 堆碎片下
        //   esp_mqtt_client_publish 直接失败（日志 "Failed to publish message"）——
        //   服务端拿不到工具列表，语音控制等于全废。这四个工具与 self.car.act
        //   （forward/backward/turn_left/turn_right）和 self.motor.drive 完全重叠，
        //   删掉能省 ~1.2KB，行为不变。方向动作请走 self.car.act。
        mcp.AddTool("self.motor.stop", "Stop motors immediately (also stops exploration mode)",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                ExploreRunner::Instance().Stop();   // 探索模式也要停，否则它 120ms 后又把电机拉起来
                motor_->Stop();
                return "stopped";
            });

        // ── 摇杆差速：LLM 自己编排动作序列 ──
        //   steer: -100..+100 (左/右，±100 原地转)
        //   throttle: -100..+100 (前/后)
        //   duration_ms: 持续时间
        // ⚠️ throttle 默认必须是 ±100，且描述里要明说"别自己猜中间值"。
        //   实测日志：用户说"前进"时模型调 self.motor.drive 传了 throttle=40/45/50，
        //   而网页速度滑块已经设成 45% —— 两者相乘只剩 18%~22%，车几乎是在挪。
        //   同一时刻"跳舞/转向"这些内部固定 100 的动作却很正常，
        //   于是用户看到的现象就是"别的动作快、只有前进慢，不是一个速度"。
        //   根源是模型不知道有个全局滑块，自己按"中等速度"估了个 40。
        //   所以：默认值拉到 100，并在描述里点明全局缩放由滑块负责。
        mcp.AddTool("self.motor.drive",
            "Differential steering like a car wheel. "
            "steer: -100..100 (left/right, ±100 = spin in place); "
            "throttle: -100..100 (backward/forward); "
            "duration_ms: 100..10000. "
            "HOW TO TURN - two different behaviours, pick by intent: "
            "(a) spin in place: steer=100 (or -100) AND throttle=0 - the two "
            "wheels counter-rotate and the car rotates on the spot. "
            "(b) wide arc: steer=50..70 with throttle=100 - the car keeps "
            "moving while turning. "
            "For plain forward/backward use throttle=100 (or -100) with steer=0. "
            "The user's web speed slider already scales every command globally, "
            "so a mid value such as 40 makes the car crawl; only use a smaller "
            "throttle when the user explicitly asks to move slowly.",
            PropertyList({
                Property("steer", kPropertyTypeInteger, 0, -100, 100),
                // 默认 0（不是 100）：用户说"转圈"时模型常常只传 steer 不传
                //   throttle，默认 100 会让它变成一边走一边转的弧线，
                //   而用户想要的是两轮反转的原地旋转。
                Property("throttle", kPropertyTypeInteger, 0, -100, 100),
                Property("duration_ms", kPropertyTypeInteger, 500, 100, 10000)
            }),
            [this](const PropertyList& p) -> ReturnValue {
                int s = p["steer"].value<int>();
                int t = p["throttle"].value<int>();
                int d = p["duration_ms"].value<int>();
                if (t > 0 && DistanceGuard::Instance().BlocksForward()) {
                    // 打日志：不然"语音说不能前进、摇杆却能"永远查不出原因
                    ESP_LOGW(TAG, "tool drive forward REFUSED: %s",
                             DistanceGuard::Instance().StatusCStr());
                    // 返回中文、且明确"只锁前进"，否则模型会连后退/转向都不敢做
                    return std::string("已拒绝前进：前方距离触发安全阈值（只锁前进，"
                                       "后退和原地转向仍然可以）。请据此用中文简短告诉用户。"
                                       "当前状态: ") +
                           DistanceGuard::Instance().StatusJson();
                }
                // 关键修复：登记忙窗口。AI 喊"前进 4 秒"走的就是这里，
                // 原来紧随其后的 200ms 表情 twitch 会从队列抢占并把它掐断，
                // 用户看到的就是"只动一下"、"转圈只转一点点"。
                // 手动动作优先：正在探索就先停掉，否则探索任务每 120ms 重发 Drive 会把它盖掉
                ExploreRunner::Instance().Stop();
                MarkMotorBusyForApp((uint32_t)d);
                motor_->Drive(s, t, (uint32_t)d);

                // ⚠️ 2026-10-20 最终决定：**不做**行驶中拦截的语音播报。
                //   曾在这里同步等待动作结束/被拦，再用工具返回值告诉大模型，但实测无效：
                //   大模型在调用工具的同时就流式输出了"十秒后停！"（日志 540564），
                //   工具结果 4.7 秒后才回来，那时这轮对话早已结束，服务端不会再
                //   为它重新唤醒大模型 —— 时序上补不回来。
                //   代价却是实打实的：占住 AI 路径最多 8 秒，期间无法处理其他 MCP 调用。
                //   所以移除，只保留"拦停"本身（安全功能，已验证生效）。
                //   参考：真正有效的播报只有"发起时被拒"这条（见下面 t>0 的检查），
                //   因为工具是立即返回的，赶在大模型组织语言之前。
                return "driving";
            });

        // ── 动作序列：一次下发多段，固件内连续执行 ────────────────────
        //   为什么要它：LLM 连调多次 self.motor.drive 时，每次调用都要走
        //   一轮 WS 往返（服务端推理 + 传输 + 两次事件循环），段与段之间
        //   会明显停顿，车"走走停停"；而且每段都重新走一次 ramp 软启动。
        //   序列把整段压成一次调用，由固件按 10ms 精度背靠背执行。
        //   执行体在 Motor2WD::SequenceTaskEntry（独立低优先级任务），
        //   ⚠️ 绝不能在工具回调里同步循环 —— 工具是在 Application 主循环
        //   里执行的（见 mcp_server.cc:553 的 app.Schedule），阻塞会冻住
        //   音频采集/唤醒词检测/屏幕刷新。
        // ⚠️ 示例里 throttle 必须写 100：模型会直接模仿示例数值。
        //   之前示例写的是 80，实际日志里模型就照着传 40~80，
        //   再叠上网页滑块的全局缩放，车会慢到几乎不动。
        mcp.AddTool("self.motor.sequence",
            "Run a multi-step move in ONE call (no gaps). steps=\"steer,throttle,ms;...\" "
            "steer -100..100 (+=right, +/-100=spin), throttle -100..100, ms 100..5000, "
            "<=24 steps, total <=15000ms. Returns immediately; self.motor.stop aborts. "
            "PREFER over repeated self.motor.drive. "
            "IMPORTANT: use throttle=100 (or -100) in every step — the web speed "
            "slider already scales globally, mid values make the car crawl. "
            "Ex: \"0,100,800;70,100,900;0,100,800;-70,100,900\".",
            PropertyList({
                Property("steps", kPropertyTypeString)
            }),
            [this](const PropertyList& p) -> ReturnValue {
                std::string steps = p["steps"].value<std::string>();
                // 序列步骤存原始值：速度上限由 Motor2WD::ApplyTarget() 在运行时
                //   统一缩放，所以这里 limit 传 100（不预乘）—— 否则会和汇流点
                //   的缩放叠乘成 k²，滑条拉到 50% 实际只有 25%。
                uint32_t total = motor_->PlaySequence(steps.c_str(), 100);
                if (total == 0) {
                    return "rejected: bad format, over limit, or a sequence is already running";
                }
                // 忙窗口按整段总时长登记：否则序列跑一半就被 200ms 表情 twitch
                // 抢占（和之前"只动一下"是同一个机制）
                MarkMotorBusyForApp(total);
                return "sequence started (" + std::to_string(total) + " ms)";
            });

        // ── 预设积木盒：21 种动作，LLM 直接调用 ──
        //   action ∈ {forward, backward, spin_around, turn_left, turn_right,
        //              quick_forward, quick_backward, wiggle, dance, stop,
        //              wake_up, happy, sad, thinking, listening, speaking,
        //              excited, loving, angry, surprised, confused}
        mcp.AddTool("self.car.act",
            "Run a preset action: forward, backward, spin_around, turn_left, turn_right, "
            "quick_forward, quick_backward, wiggle, dance, stop, wake_up, happy, sad, thinking, "
            "listening, speaking, excited, loving, angry, surprised, confused. "
            "Pass duration_ms for 'dance 5 seconds' (built-in defaults are short: forward 1.5s, dance 1.1s).",
            PropertyList({
                Property("action", kPropertyTypeString),
                Property("duration_ms", kPropertyTypeInteger, 0, 0, 15000)
            }),
            [this](const PropertyList& p) -> ReturnValue {
                std::string a = p["action"].value<std::string>();
                int want_ms = p["duration_ms"].value<int>();   // 0 = 用动作默认时长
                ExploreRunner::Instance().Stop();              // 手动动作优先，先停探索
                auto display = Board::GetInstance().GetDisplay();

                // 离散方向走快速通道
                //   ⚠️ spin_around 复用 MOTOR_TURN_LEFT（左轮后退 + 右轮前进 = 原地转），
                //      与 turn_left 唯一区别是时长；给足 1500ms 才转得成圈。
                //   want_ms > 0 时覆盖默认时长（"前进 3 秒"靠这个）。
                auto go = [&](MotorDir dir, int pct, int ms) {
                    uint32_t d = (want_ms > 0) ? (uint32_t)want_ms : (uint32_t)ms;
                    MarkMotorBusyForApp(d);   // 登记忙窗口
                    motor_->Execute(dir, (uint8_t)pct, d);
                };
                // 前进类动作先问安全层（原来 move_forward 工具里的检查挪到这里，
                // 那四个方向工具已删除以缩小 tools/list payload）
                if ((a == "forward" || a == "quick_forward") &&
                    DistanceGuard::Instance().BlocksForward()) {
                    ESP_LOGW(TAG, "car.act %s REFUSED: %s", a.c_str(),
                             DistanceGuard::Instance().StatusCStr());
                    return std::string("已拒绝前进：前方距离触发安全阈值（只锁前进，"
                                       "后退和原地转向仍然可以）。请据此用中文简短告诉用户。"
                                       "当前状态: ") +
                           DistanceGuard::Instance().StatusJson();
                }
                if (a == "forward")        { go(MOTOR_FORWARD,    100, 1500); return "forward"; }
                if (a == "backward")       { go(MOTOR_BACKWARD,   100, 1500); return "backward"; }
                if (a == "turn_left")      { go(MOTOR_TURN_LEFT,  100,  500); return "left"; }
                if (a == "turn_right")     { go(MOTOR_TURN_RIGHT, 100,  500); return "right"; }
                if (a == "spin_around")    { go(MOTOR_TURN_LEFT,  100, 1500); return "spin"; }
                if (a == "quick_forward")  { go(MOTOR_FORWARD,    100,  300); return "q_forward"; }
                if (a == "quick_backward") { go(MOTOR_BACKWARD,   100,  300); return "q_backward"; }
                if (a == "stop")           { motor_->Stop(); return "stopped"; }

                // 情感动作：电机 + 表情 联动
                if (a == "wake_up")        { display->SetEmotion("wake");     HandleMotorActionForEmotion("wake");     return "wake_up"; }
                if (a == "happy")          { display->SetEmotion("happy");    HandleMotorActionForEmotion("happy");    return "happy"; }
                if (a == "sad")            { display->SetEmotion("sad");      HandleMotorActionForEmotion("sad");      return "sad"; }
                if (a == "thinking")       { display->SetEmotion("thinking"); HandleMotorActionForEmotion("thinking"); return "thinking"; }
                if (a == "listening")      { display->SetEmotion("listening");HandleMotorActionForEmotion("listening");return "listening"; }
                if (a == "speaking")       { display->SetEmotion("speaking"); HandleMotorActionForEmotion("speaking"); return "speaking"; }
                if (a == "excited")        { display->SetEmotion("excited");  HandleMotorActionForEmotion("excited");  return "excited"; }
                if (a == "loving")         { display->SetEmotion("loving");   HandleMotorActionForEmotion("loving");   return "loving"; }
                if (a == "angry")          { display->SetEmotion("angry");    HandleMotorActionForEmotion("angry");    return "angry"; }
                if (a == "surprised")      { display->SetEmotion("surprised");HandleMotorActionForEmotion("surprised");return "surprised"; }
                if (a == "confused")       { display->SetEmotion("confused"); HandleMotorActionForEmotion("confused"); return "confused"; }

                // 复合动作：多步序列
                if (a == "wiggle")         { display->SetEmotion("confused"); HandleMotorActionForEmotion("confused"); return "wiggle"; }
                if (a == "dance")          {
                    display->SetEmotion("excited");
                    HandleMotorActionForDanceFor((uint8_t)100,
                                                 (uint32_t)(want_ms > 0 ? want_ms : 0));
                    return "dance";
                }

                return std::string("unknown action: ") + a;
            });

        mcp.AddTool("self.network.get_ip",
            "Get current WiFi IP address",
            PropertyList(),
            [](const PropertyList&) -> ReturnValue {
                esp_netif_ip_info_t ip;
                if (esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"), &ip) == ESP_OK) {
                    char buf[16];
                    snprintf(buf, sizeof(buf), IPSTR, IP2STR(&ip.ip));
                    return std::string("IP: ") + buf;
                }
                return std::string("not connected");
            });
    }

public:
    CompactWifiBoard() : boot_button_(BOOT_BUTTON_GPIO) {
        // ⚠️ 电机必须最先初始化：LF 原先是 GPIO0，它带板载上拉电阻，
        //    在程序接管之前这颗上拉会把 MX1508 的 IN1 拉高 → 左轮持续自转
        //    （2026-10-16 已把 LF 迁到 GPIO2，但这个顺序留着仍然更保险）。
        //    Motor2WD 构造函数会把 4 路强制输出低（能压过上拉），所以要让它
        //    尽早跑；若排在 display I2C 后面，一旦 I2C 初始化变慢/总线卡住，
        //    电机脚就一直没人接管 → 开机左轮一直转。
        // 复位原因打在日志里（0=未知 1=上电 3=软复位 4=panic 5=中断看门狗
        // 6=任务看门狗 7=看门狗 9=掉电/BOD）。排查"不断重启"时先看这一行。
        ESP_LOGW(TAG, "reset reason=%d", (int)esp_reset_reason());

        // 开机瞬间按住 BOOT → 重新配网（TryWifiConnect 里会查这个标志）。
        //   必须在 WiFi 初始化之前采样完；16ms 的采样对上电时序没有影响。
        boot_cfg_key_held_ = SampleBootKeyAtPowerOn();
        boot_click_to_consume_ = boot_cfg_key_held_;
        if (boot_cfg_key_held_) {
            ESP_LOGW(TAG, "BOOT held at power-on -> will enter WiFi config mode");
        }

        // 临时调试（2026-10-19 "OBSTACLE 后 abort"）：把未捕获 C++ 异常的
        // 调用栈打出来，否则 panic 只留一行 abort 地址，没法定位 throw 点。
        std::set_terminate([]() {
            ESP_LOGE(TAG, "!!! uncaught C++ exception -> std::terminate");
            esp_backtrace_print(32);
            abort();
        });
        InitializeMotor();
        InitializeDisplayI2c();
        // ⚠️ 不要在启动时全总线扫描：112 个地址 × 50ms 超时 = 最坏 5.6 秒，
        //   而且会在 OLED 上电复位期间反复打扰总线（实测扫描后 OLED 直接不 ACK 了）。
        //   需要诊断时手动调用 ScanI2CBus()。
        InitializeSsd1306Display();
        // 测距安全层（防碰撞 + 防跌落）：复用 OLED 的 I2C 总线
        //   ⚠️ C3 只有一个 I2C 控制器，驱动内部不会再新建总线。
        //   没插模块也不会报错：任务每 2 秒重试一次初始化，期间不拦截电机。
        DistanceGuard::Instance().Start(motor_, display_i2c_bus_);
        Motor2WD::SetForwardBlockHook(&DistanceGuardBlocksForward);
        // 起一个**独立于 ramp 状态机**的常驻安全复查（50ms）。
        //   实测：语音「前进 10 秒」走 MCP 一次性指令时，RampTick 里的复查会
        //   因热路径停表/世代不匹配而整个消失 —— 日志有
        //   `OBSTACLE 68mm -> 拒绝前进` 却没有 SAFE 拦停，距离从 68mm 继续掉到
        //   47mm（车压根没停）。摇杆没事是因为它 40Hz 重发、每帧都过发起检查。
        if (motor_) motor_->StartSafetyTimer();
        // 前进被安全层拦下时的回调（2 秒节流在 Motor2WD 里）。
        //   ⚠️ 2026-10-20 最终：这里**只记一条日志**。语音反馈的三条路
        //   （服务端上报 / MCP 工具返回 / 本地「叮」声）都实测走不通或用户不需要，
        //   详见 Application::NotifyForwardBlocked() 的注释。
        //   "拦停"由 Motor2WD 自身保证，这个回调只负责留痕，删掉也不影响安全。
        Motor2WD::SetForwardBlockedCallback([]() {
            auto& app = Application::GetInstance();
            // 网页"控制模式"（遥控）下不记日志：那时对话已被整体关掉
            //   （wake word + listening + 表情音全停），没有诊断价值。
            if (app.IsControlModeActive()) return;
            app.NotifyForwardBlocked();
        });
        RegisterMcpTools();
        InitializeBootButton();
    }

    // WifiBoard 钩子：开机按着 BOOT → 跳过已保存 SSID，直接进配网 AP 模式。
    //   ⚠️ 必须**一次性**：TryWifiConnect() 在每次配网保存后都会再次调用，
    //   标志不复位就会"保存完凭据又立刻进配网"，永远连不上。
    bool IsBootConfigKeyHeld() override {
        if (!boot_cfg_key_held_) return false;
        boot_cfg_key_held_ = false;
        return true;
    }

    virtual AudioCodec* GetAudioCodec() override {
#ifdef AUDIO_I2S_METHOD_SIMPLEX
        static NoAudioCodecSimplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_SPK_GPIO_BCLK, AUDIO_I2S_SPK_GPIO_LRCK, AUDIO_I2S_SPK_GPIO_DOUT,
            AUDIO_I2S_MIC_GPIO_SCK, AUDIO_I2S_MIC_GPIO_WS, AUDIO_I2S_MIC_GPIO_DIN);
        return &audio_codec;
#else
        static NoAudioCodecDuplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN);
        return &audio_codec;
#endif
    }

    virtual Display* GetDisplay() override { return display_; }

    // 给 C 风格弱符号 HandleMotorActionForEmotion() 用
    Motor2WD* motor_for_emotion() { return motor_; }
};

DECLARE_BOARD(CompactWifiBoard);

// 弱符号：Application 在收到 emotion 字段时会调这个
// 极简映射：只对 4 种 emotion 做电机动作，其他全 STOP
extern "C" void HandleMotorActionForEmotion(const char* emotion) {
    auto board = static_cast<CompactWifiBoard*>(&Board::GetInstance());
    auto motor = board->motor_for_emotion();
    if (!emotion || !motor) return;

    std::string e(emotion);
    // 装饰动作也乘全局速度上限（原来 70/50 硬编码，网页调速对它们无效）
    const uint8_t fwd = 70;
    const uint8_t back = 50;
    if (e == "happy" || e == "loving" || e == "wake" || e == "wakeup") {
        motor->Execute(MOTOR_FORWARD, fwd, 200);
    } else if (e == "sad" || e == "crying") {
        motor->Execute(MOTOR_BACKWARD, back, 200);
    } else if (e == "angry" || e == "surprised" || e == "shocked") {
        // 两步：后撤 → 前冲。原来用 vTaskDelay(110) 同步保序，而本函数
        // 可能被 Application 主循环调用 → 每次生气都卡主循环 110ms。
        // 改成一次序列下发，立即返回（序列忙时退化为单步）。
        //   速度上限由 Motor2WD::ApplyTarget() 汇流点统一缩放，limit 传 100
        //   （不预乘，否则叠成 k²）。
        uint32_t total = motor->PlaySequence("0,-80,100;0,80,150", 100);
        if (total) {
            MarkMotorBusyForApp(total);   // 不登记会被紧随的说话/表情反馈掐断
        } else {
            motor->Execute(MOTOR_BACKWARD, back, 100);
        }
    } else {
        motor->Stop();
    }
}

// 舞蹈：四步一轮 → 前 300 / 原地左 250 / 原地右 250 / 后 300 = 1100ms
//   duration_ms = 0  → 只跳一轮（旧行为，web_server 那条路径）
//   duration_ms = 5000 → 循环拼段直到满 5 秒
//   ⚠️ 为什么必须支持时长：原来四步时长全写死，LLM 想让"跳 5 秒"却没有任何
//      参数可传，只能得到 1.1 秒 —— 用户感知就是"说 5 秒实际跳 1 秒"。
extern "C" void HandleMotorActionForDanceFor(uint8_t speed_percent, uint32_t duration_ms) {
    auto board = static_cast<CompactWifiBoard*>(&Board::GetInstance());
    auto motor = board->motor_for_emotion();
    if (!motor) return;

    int s = (int)speed_percent;
    if (s < 1) s = 1;

    // duration_ms = 0 → 跳一轮（1100ms）；否则循环拼段直到凑满
    uint32_t target = (duration_ms == 0) ? 1100u : duration_ms;
    if (target > 15000u) target = 15000u;

    // 一轮四步：{steer, throttle 符号, 时长}
    static const int kSegSteer[4]   = {   0, -100,  100,    0 };
    static const int kSegThrSign[4] = {   1,    0,    0,   -1 };
    static const int kSegMs[4]      = { 300,  250,  250,  300 };

    // 容量必须够 64 段：最坏一段 "-100,-100,5000;" = 15 字节 → 960 字节。
    //   原来 768 字节，15 秒舞（约 54 段）会在中途被 snprintf 截断，
    //   用户说"跳 15 秒"实际只跳十几秒且没有任何日志。
    char steps[1100];
    int off = 0;
    uint32_t acc = 0;
    int idx = 0;
    while (acc < target && idx < 64) {
        uint32_t remain = target - acc;
        uint32_t ms = (uint32_t)kSegMs[idx & 3];
        if (ms > remain) ms = remain;
        if (ms < 100) break;          // 剩余不足一段 → 收工，避免尾段抖动

        int n = snprintf(steps + off, sizeof(steps) - (size_t)off, "%s%d,%d,%u",
                         (idx == 0) ? "" : ";",
                         kSegSteer[idx & 3],
                         kSegThrSign[idx & 3] * s,
                         (unsigned)ms);
        if (n <= 0 || off + n >= (int)sizeof(steps)) break;
        off += n;
        acc += ms;
        ++idx;
    }
    if (idx == 0) return;

    // 序列步骤存原始值：速度上限由 Motor2WD::ApplyTarget() 在运行时统一缩放，
    //   所以这里 limit 传 100（不预乘）—— 这样滑条改动对**正在播放的序列**
    //   也能生效（原来预乘进步骤，运行中改滑条完全没反应）。
    uint32_t total = motor->PlaySequence(steps, 100);
    if (total) {
        // ⚠️ 必须登记忙窗口：dance 原来没登记，服务端随后的 emotion /
        //    说话反馈会走 QueueMotorAction → StopRealtimeMotorControl →
        //    motor->Stop() → AbortSequence()，把舞蹈当场腰斩。
        MarkMotorBusyForApp(total);
    } else {
        motor->Execute(MOTOR_FORWARD, (uint8_t)s, 300);   // 忙/解析失败兜底
    }
}

// web_server 调：极简舞蹈（保持单参数接口不变）
extern "C" void HandleMotorActionForDance(uint8_t speed_percent) {
    HandleMotorActionForDanceFor(speed_percent, 0);
}

// weak 符号：web_server 收到 steer/throttle 时调用（C3 板实现差速转向）
//   steer:    -100..+100 (左/右；+100 原地右转)
//   throttle: -100..+100 (前/后；+100 前进满速，0 停车)
// 优先级高于 application.cc 的 4 象限控制
extern "C" void HandleBoardSteerControl(int steer, int throttle) {
    auto board = static_cast<CompactWifiBoard*>(&Board::GetInstance());
    auto motor = board->motor_for_emotion();
    if (!motor) return;
    // 前端只发原始 steer/throttle（换手机 / 清 localStorage 也一致），
    //   缩放交给 Motor2WD::ApplyTarget() 汇流点（2026-10-20：原来在这里乘，
    //   结果探索模式 / 单动作接口 / 动作序列都绕过了滑条）。
    motor->Drive(steer, throttle);
}

// weak 符号：web_server 收到 {t:"interrupt"} 时调用（与板载 BOOT 键同语义）
//   空闲/待机 → 唤醒进入聆听；说话中 → 打断（AbortSpeaking）；聆听中 → 关闭回 idle
//   走 Application::ToggleChatState()，内部用 MAIN_EVENT_TOGGLE_CHAT 事件组
//   异步派发到 main 任务处理，不阻塞 httpd 任务（响应延迟 < 5ms）
extern "C" void HandleBoardToggleChat() {
    auto& app = Application::GetInstance();
    app.ToggleChatState();
}

// 弱符号：web_server 收到 action 按钮时调用
//   motor_action: 1=forward, 2=backward, 3=turn_left, 4=turn_right（motor_controller.h 枚举）
//   speed: 0..100
//   duration_ms: 0=无限（不自动停），>0=执行后自动停
//   用 motor->Execute 而不是 Application 的 HandleMotorActionForApplication，是因为
//   Execute 只用 motor 自己的 invert flag，不应用 Application 的 swap_lr/invert 配置
//   （那些配置是给 APP 内部 4 象限控制用的，会和 motor 的方向反着来，导致按钮左右颠倒）
extern "C" void HandleMotorExecuteByEnum(int motor_action, int speed, int duration_ms) {
    auto board = static_cast<CompactWifiBoard*>(&Board::GetInstance());
    auto motor = board->motor_for_emotion();
    if (!motor) return;
    if (speed <= 0) speed = 80;
    if (speed > 100) speed = 100;
    if (duration_ms < 0) duration_ms = 0;
    // 缩放已下沉到 Motor2WD::ApplyTarget() 汇流点，这里只留"最低可动"下限
    //   （真正的最小占空比保障在汇流点按等比方式做，见 kMinMovingDuty）。
    if (speed < 15) speed = 15;        // 低于死区电机根本不转，别给了电压不转
    motor->Execute(static_cast<MotorDir>(motor_action), (uint8_t)speed, (uint32_t)duration_ms);
}

// web_server / application 调：立即停车（不经过 4 象限 PWM 系统）
extern "C" void HandleMotorStop() {
    auto board = static_cast<CompactWifiBoard*>(&Board::GetInstance());
    auto motor = board->motor_for_emotion();
    if (motor) motor->Stop();
}

// 弱符号：web_server 调用来修改 motor 自身的接线修正
//   改完即生效（motor 自身 invert/swap flag），同时通过 NVS 持久化（在 application.cc）
extern "C" void SetMotorInvertLeft(bool v) {
    auto board = static_cast<CompactWifiBoard*>(&Board::GetInstance());
    auto motor = board->motor_for_emotion();
    if (motor) motor->SetInvertLeft(v);
}
extern "C" void SetMotorInvertRight(bool v) {
    auto board = static_cast<CompactWifiBoard*>(&Board::GetInstance());
    auto motor = board->motor_for_emotion();
    if (motor) motor->SetInvertRight(v);
}
extern "C" void SetMotorSwapLr(bool v) {
    auto board = static_cast<CompactWifiBoard*>(&Board::GetInstance());
    auto motor = board->motor_for_emotion();
    if (motor) motor->SetSwapLr(v);
}
extern "C" bool GetMotorInvertLeft() {
    auto board = static_cast<CompactWifiBoard*>(&Board::GetInstance());
    auto motor = board->motor_for_emotion();
    return motor ? motor->GetInvertLeft() : false;
}
extern "C" bool GetMotorInvertRight() {
    auto board = static_cast<CompactWifiBoard*>(&Board::GetInstance());
    auto motor = board->motor_for_emotion();
    return motor ? motor->GetInvertRight() : false;
}
extern "C" bool GetMotorSwapLr() {
    auto board = static_cast<CompactWifiBoard*>(&Board::GetInstance());
    auto motor = board->motor_for_emotion();
    return motor ? motor->GetSwapLr() : false;
}

// ── 网页"距离保护"面板的桥接（web_server.cc 里是弱符号）──────────────
//   {t:"dist_cfg", obstacle_mm, fall_mm} → 立即生效 + 写 NVS
extern "C" void OnDistanceGuardConfig(int obstacle_mm, int fall_mm) {
    DistanceGuard::Instance().SetThresholds(obstacle_mm, fall_mm);
}
extern "C" const char* GetDistanceGuardStatusJson() {
    return DistanceGuard::Instance().StatusCStr();
}
