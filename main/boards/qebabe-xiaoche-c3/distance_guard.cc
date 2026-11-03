#include "distance_guard.h"

// 文字模式挂起需要查屏幕当前模式（见 Loop() 里 text_mode_suspended_ 的判定）。
//   board.h 提供 Board::GetInstance().GetDisplay()，display.h 提供基类的
//   WantsChatText() 虚函数 —— OledDisplay 把它实现为"当前处于文字模式"。
//   ✅ 走基类虚接口而不是 dynamic_cast 到 OledDisplay：
//      不必把板级显示的具体类型拖进来，耦合更小。
#include "board.h"
#include "display.h"

#include "motor_controller.h"
#include "vl53l0x_c3.h"

#include "settings.h"

#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <cstdio>

#define TAG "DistGuard"

namespace {
// 连续模式 30Hz：1 帧确认 ≈ 33ms（原 2 帧 66ms、更早 3 帧 100ms）。
//   桌沿是"掉下去"级别的风险，多等 33ms 车就能多走 3~5cm。
// ⚠️ 2026-10-20 由 2 帧改成 1 帧（用户要求"更灵敏"）：响应延迟从约 120ms
//   （采样 33 + 确认 66 + ramp 复查 20）降到约 66ms。
//   代价：单帧抖动就可能进入危险。由两道保险兜住 ——
//     ① 迟滞带（进入/退出不同阈值），防读数 1~2mm 抖动反复触发；
//     ② kForceReleaseMs 强制解除，防止"基线离阈值太近"时被永久锁死。
//   ⚠️ 如果你的障碍阈值贴着安装基线（如阈值 100 / 基线 103），建议把阈值
//      降到 60~70 拉开差距，这样 1 帧确认既灵敏又不会频繁误判。
constexpr int kConfirmSamples = 1;
constexpr int kReleaseMs = 150;        // 解除危险需要的持续安全时间（原 250ms）
// 迟滞带：进入危险用阈值本身，退出必须退到阈值外「迟滞量」才算安全。
//   读数在阈值上下抖 1~2mm 时不会反复"碰撞↔解除"（实测 68/70/71mm 抖动）。
// ⚠️ 2026-10-19 改成**跟着阈值走**（原固定 40mm）：
//   固定 40 在阈值 50 时意味着"要退到 90mm 才放行"，用户设 50、实测 87,
//   明明没到阈值却一直判碰撞、前进永远被拦。现在取 阈值/3 并夹在 8~40。
// ⚠️ 2026-10-20 再收窄到 阈值/6（夹 5~25）：迟滞的本职只是"防读数 1~2mm
//   抖动"，33mm 的迟滞带太宽了 —— 阈值 100、基线 107 时退出要退到 133，
//   一次抖动进入后前进就被永久拒绝（用户实测"阈值范围内还判定拦截"）。
//   收窄后 100 → 退出 116。注意：光靠这个仍然救不回"基线离阈值太近"的
//   极端配置，真正的兜底是下面的 kForceReleaseMs（强制解除）。
constexpr int kHystMinMm = 5;
constexpr int kHystMaxMm = 25;
// 迟滞带锁死兜底：危险状态里，若读数已回到"进入阈值之外"（按进入判据并不
//   危险，只是没退够迟滞带），持续这么久就强制解除。真危险时读数会一直
//   低于进入阈值，不会误触发。
constexpr int kForceReleaseMs = 1500;
inline int HysteresisFor(int threshold_mm) {
    int h = threshold_mm / 6;
    if (h < kHystMinMm) h = kHystMinMm;
    if (h > kHystMaxMm) h = kHystMaxMm;
    return h;
}
constexpr int kSettleMs = 1500;        // 上电后静置，避免开机电还没稳就动作
}  // namespace

DistanceGuard& DistanceGuard::Instance() {
    static DistanceGuard inst;
    return inst;
}

const char* DistanceGuard::StateName(int state) {
    switch (state) {
        case kDisabled: return "disabled";
        case kNoSensor: return "no_sensor";
        case kClear:    return "clear";
        case kObstacle: return "obstacle";
        case kFall:     return "fall";
        default:        return "unknown";
    }
}

void DistanceGuard::Start(Motor2WD* motor, i2c_master_bus_handle_t bus) {
    if (started_) return;
    motor_ = motor;
    bus_ = bus;
    started_ = true;

    // 阈值从 NVS 读（网页/语音改过就沿用上次的）
    {
        // 键名带 v2：2026-10-19 把默认碰撞阈值从 130 改成 90（旧值会让车一上电
        // 就把桌面读成障碍），换键名让新默认生效，避免沿用旧值。
        Settings settings("dist_guard", true);
        // ⚠️ 键名 v2 → v3（2026-10-22）：默认障碍阈值 90 → 60。
        //   探头朝下，实测地面 86~90mm，而 NVS 沿用了 100 的旧值 →
        //   地面本身被判成障碍 → 语音"前进"每次都被拒绝
        //   （日志实证：`mm=86 state=obstacle obs<=100`，用户看到的是"车不走"）。
        //   换键名让新默认生效，不会继续读旧值。
        obstacle_mm_ = (int)settings.GetInt("obstacle_mm_v3", obstacle_mm_);
        fall_mm_ = (int)settings.GetInt("fall_mm_v3", fall_mm_);
        if (fall_mm_ <= obstacle_mm_) fall_mm_ = obstacle_mm_ + 50;
        blind_mm_ = obstacle_mm_ * 3 / 4;
    }
    // 优先级 5（原 4）：对话/控制模式下音频 AFE + Opus 会长时间占 CPU，
    //   优先级 4 的采样任务被饿住 → 读数变稀甚至停更 → 保护层拿旧值判断
    //   （用户体感："用的还是上一次碰撞的距离"、"跌落失效"）。
    //   采样任务本身只做 I2C 小事务（每帧 ~0.5ms、30Hz），优先级 5 不会拖慢音频。
    //   栈压到 2560：C3 没有 PSRAM，每 1KB 堆都很关键（tools/list 响应曾因 OOM 崩）。
    if (xTaskCreate(TaskEntry, "dist_guard", 2560, this, 5, nullptr) != pdPASS) {
        ESP_LOGE(TAG, "task create failed");
        started_ = false;
        return;
    }
    ESP_LOGI(TAG, "started (obstacle<=%dmm, fall>=%dmm, blind<=%dmm)",
             obstacle_mm_, fall_mm_, blind_mm_);
}

void DistanceGuard::TaskEntry(void* arg) {
    static_cast<DistanceGuard*>(arg)->Loop();
    vTaskDelete(nullptr);
}

void DistanceGuard::SetEnabled(bool on) {
    enabled_ = on;
    if (!on) {
        block_until_ms_ = 0;
        state_ = kDisabled;
        if (motor_ != nullptr) motor_->Stop();
    }
    ESP_LOGI(TAG, "enabled=%d", (int)on);
}

void DistanceGuard::SetThresholds(int obstacle_mm, int fall_mm) {
    if (obstacle_mm < 30) obstacle_mm = 30;
    if (obstacle_mm > 2000) obstacle_mm = 2000;
    // ⚠️ 已知地面距离时，拒绝把阈值设到地面附近：
    //   探头朝下，地面就是 90~120mm。阈值若 >= 地面，地面本身会被判成障碍，
    //   语音"前进"每次都被拒（实测 OBSTACLE 90mm -> 拒绝前进）。
    //   这里夹紧到地面的 2/3，并提示用户。NVS 里存的是夹紧后的值。
    if (baseline_seen_ && obstacle_mm + 20 >= (int)baseline_mm_) {
        int fixed = (int)baseline_mm_ * 2 / 3;
        if (fixed < 40) fixed = 40;
        ESP_LOGW(TAG, "拒绝把障碍阈值设为 %dmm（地面基线 %dmm，会把自己拦死），"
                 "已夹到 %dmm", obstacle_mm, (int)baseline_mm_, fixed);
        obstacle_mm = fixed;
    }
    if (fall_mm < obstacle_mm + 50) fall_mm = obstacle_mm + 50;
    if (fall_mm > 8000) fall_mm = 8000;
    // 盲区阈值跟着障碍阈值走：障碍越远，"贴脸"判据也相应放宽
    blind_mm_ = obstacle_mm * 3 / 4;
    obstacle_mm_ = obstacle_mm;
    fall_mm_ = fall_mm;

    // 持久化（网页面板"保存阈值"走这里）
    {
        Settings settings("dist_guard", true);
        settings.SetInt("obstacle_mm_v3", obstacle_mm_);
        settings.SetInt("fall_mm_v3", fall_mm_);
    }

    // ⚠️ 2026-10-19 实测：把阈值从 90 改成 50，读数 87 却仍然"拦截"。
    //   除了迟滞带过宽（原固定 40mm），还有一个坑：旧状态仍是 obstacle，
    //   要等"退到阈值外"才解除。改完阈值若当前读数已经在新阈值之外，
    //   直接清锁存 —— 否则用户会觉得"改了没用，必须重启"。
    if (state_ == kObstacle && (int)last_mm_ > obstacle_mm_) {
        state_ = sensor_ok_ ? kClear : kNoSensor;
        block_until_ms_ = 0;
        ESP_LOGW(TAG, "obstacle latch cleared by new threshold (last=%umm > %dmm)",
                 (unsigned)last_mm_, obstacle_mm_);
    } else if (state_ == kFall && (int)last_mm_ > 0 && (int)last_mm_ < fall_mm_) {
        state_ = sensor_ok_ ? kClear : kNoSensor;
        block_until_ms_ = 0;
        ESP_LOGW(TAG, "fall latch cleared by new threshold (last=%umm < %dmm)",
                 (unsigned)last_mm_, fall_mm_);
    }

    ESP_LOGI(TAG, "thresholds: obstacle<=%dmm (release>%dmm) fall>=%dmm blind<=%dmm",
             obstacle_mm_, obstacle_mm_ + HysteresisFor(obstacle_mm_), fall_mm_, blind_mm_);
}

DistanceGuard::Config DistanceGuard::config() const {
    Config c;
    c.obstacle_mm = obstacle_mm_;
    c.fall_mm = fall_mm_;
    c.blind_mm = blind_mm_;
    return c;
}

bool DistanceGuard::BlocksForward() const {
    if (!enabled_) return false;
    // 文字模式下雷达已按定义关闭 → 一律放行（见 Loop() 里的挂起逻辑）。
    //   ⚠️ 包括防跌落：这个模式下确实没有任何距离保护，是刻意的选择。
    if (text_mode_suspended_) return false;
    if (!sensor_ok_) return false;                 // 传感器挂了就别拦，否则车彻底动不了
    // 数据太旧 → 采样已停，不能拿旧读数当保护依据（否则会永久拦死）
    //   ⚠️ 2026-10-20：800 → 1500ms。语音对话时 OLED 在滚动播放文字，
    //     每 250ms 推一整屏（1024B @100kHz ≈ 92ms），和 VL53L0X 共用同一条 I2C，
    //     传感器采样被反复阻塞 → 超过 800ms 没新帧 → 这里直接 return false
    //     → **语音时前进拦截完全失效**（摇杆正常，因为摇杆时 OLED 不滚文字）。
    //     放宽到 1500ms 后，只要 OLED 别连续两次把总线占死就能保住防护。
    //   （代价：会用稍旧的读数判断，但 1.5s @30Hz 仍在合理范围；
    //     传感器真离线时 sensor_ok_ 会先 false，不会走到这里。）
    const uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
    if (!has_sample_ || (uint32_t)(now - last_sample_ms_) > 1500) return false;
    int st = state_;
    if (st == kObstacle || st == kFall) return true;
    if (block_until_ms_ == 0) return false;
    return (int32_t)(block_until_ms_ - now) > 0;   // 回绕安全的有符号差
}

bool DistanceGuardBlocksForward() {
    return DistanceGuard::Instance().BlocksForward();
}

void DistanceGuard::Loop() {
    auto* vl = static_cast<Vl53l0xC3*>(sensor_);
    if (vl == nullptr && bus_ != nullptr) {
        vl = new Vl53l0xC3(bus_);
        sensor_ = vl;
    }

    int obstacle_cnt = 0;
    int fall_cnt = 0;
    int i2c_fail = 0;
    int64_t clear_since = 0;
    int64_t last_valid_ms = 0;
    uint16_t last_valid_mm = 0;
    uint16_t baseline_mm = 0;   // 最近一次"正常距离"（通常是桌面这个近平面）
    int64_t baseline_ms = 0;
    int64_t unknown_since = 0;
    int64_t stuck_since = 0;    // 迟滞带锁死计时（见下面 kForceReleaseMs 的说明）
    const int64_t boot_ms = esp_timer_get_time() / 1000;

    while (true) {
        if (reset_req_) {   // MCP action=reset：清掉危险状态/配额/基准
            reset_req_ = false;
            obstacle_cnt = fall_cnt = 0;
            i2c_fail = 0;
            clear_since = 0;
            last_valid_ms = 0;
            last_valid_mm = 0;
            baseline_ms = 0;
            baseline_mm = 0;
            baseline_mm_ = 0;
            baseline_seen_ = false;
            fail_streak_ = 0;
            block_until_ms_ = 0;
            state_ = sensor_ok_ ? kClear : kNoSensor;
            ESP_LOGW(TAG, "state reset by request");
        }

        if (!enabled_) {
            obstacle_cnt = fall_cnt = 0;
            i2c_fail = 0;
            clear_since = 0;
            state_ = kDisabled;
            sensor_ok_ = false;
            sensor_ = nullptr;   // 关掉时把驱动释放掉，重新打开会重新初始化
            block_until_ms_ = 0;
            delete vl;
            vl = nullptr;
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // ── 文字模式 → 挂起测距（2026-10-22，定义："文字模式关掉雷达"）──
        //   文字模式会高频滚动推屏，与 VL53L0X 共用同一条 I2C；采样被挤掉后
        //   BlocksForward() 因"数据过期(>1500ms)"直接放行，变成时灵时不灵的
        //   假保护 —— 那比干脆关掉更糟。这里明确停采样，把总线让给屏幕。
        //   ⚠️ 代价：挂起期间**防跌落也不生效**（BlocksForward 直接放行）。
        //   ✅ 走基类虚接口 WantsChatText()，不依赖具体屏幕实现类型。
        {
            auto* disp = Board::GetInstance().GetDisplay();
            const bool want_text = (disp != nullptr) && disp->WantsChatText();
            if (want_text != text_mode_suspended_) {
                text_mode_suspended_ = want_text;
                // ⚠️ 别把 `? :` 三元表达式直接塞进 ESP_LOGW：
                //   宏展开再叠上 CRLF 换行会被切词 → 编译报
                //   "expected ')' before 'want_text'"。老老实实用 if/else。
                if (want_text) {
                    ESP_LOGW(TAG, "进入文字模式 → 挂起激光测距（I2C 让给屏幕滚动），前进不再受距离限制");
                } else {
                    ESP_LOGW(TAG, "退出文字模式 → 恢复激光测距");
                }
            }
            if (text_mode_suspended_) {
                vTaskDelay(pdMS_TO_TICKS(300));
                continue;
            }
        }

        // ── 已判定"未安装" → 彻底挂起，不碰 I2C ──────────────────────
        //   ⚠️ 必须放在 new 之前：否则每轮都会 new/delete 一次驱动对象，
        //      在内存本来就紧张的 C3 上纯属自我消耗。
        if (sensor_absent_ && !sensor_ok_) {
            const uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
            if ((int32_t)(now_ms - next_probe_ms_) < 0) {
                vTaskDelay(pdMS_TO_TICKS(500));
                continue;
            }
            // 60 秒到了，回头看一眼（用户可能开机后才把模块焊上）
            sensor_absent_ = false;
            probe_fail_ = 0;
            ESP_LOGI(TAG, "重新探测激光测距模块");
        }

        if (vl == nullptr) {
            if (bus_ == nullptr) {
                // ⚠️ 这条以前是静默的：OLED 初始化失败 → display_i2c_bus_ 为空 →
                //   保护层整个哑掉，日志里一个字都没有（2026-10-19 出现过一次，
                //   复位后拍不到任何 DistGuard 行，无从判断）。
                static int64_t last_warn = 0;
                int64_t t = esp_timer_get_time() / 1000;
                if (t - last_warn > 5000) {
                    last_warn = t;
                    ESP_LOGE(TAG, "I2C bus is null (OLED init failed?) - protection disabled");
                }
                vTaskDelay(pdMS_TO_TICKS(500));
                continue;
            }
            vl = new Vl53l0xC3(bus_);
            sensor_ = vl;
        }

        // ── 初始化（没装模块时不要无限重试）─────────────────────────
        //   ⚠️ 用户反馈："有的人不装激光雷达"。原逻辑无条件每 2 秒重试，
        //      没装模块的板子会永久刷 `no ACK at 0x29`，并且每次重试都占用
        //      I2C 总线（与 OLED 共用）→ 白白拖慢刷屏和测距。
        //   现在：连续失败 3 次（约 6 秒）判定"未安装" → 停采样、放总线，
        //      仅 60 秒后回头看一次。整个过程不拦截电机（sensor_ok_=false 时
        //      BlocksForward() 直接返回 false，车照常能走）。
        if (!sensor_ok_) {
            state_ = kNoSensor;
            if (!vl->Init()) {
                if (++probe_fail_ >= 3) {
                    sensor_absent_ = true;
                    next_probe_ms_ = (uint32_t)(esp_timer_get_time() / 1000) + 60000;
                    // 释放驱动：未安装时留着它没有任何意义（也避免下轮反复 new）
                    delete vl;
                    vl = nullptr;
                    sensor_ = nullptr;
                    ESP_LOGW(TAG, "未检测到激光测距模块（I2C 0x29 无响应）→ 已关闭距离保护，"
                                  "前进不再受距离限制。装好模块后重启即可启用（不重启则 60 秒后自动重试）。");
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    continue;
                }
                vTaskDelay(pdMS_TO_TICKS(2000));
                continue;
            }
            probe_fail_ = 0;
            if (!vl->StartContinuous()) {   // back-to-back，传感器自己跑 ~30Hz
                vTaskDelay(pdMS_TO_TICKS(2000));
                continue;
            }
            sensor_ok_ = true;
            state_ = kClear;
            baseline_seen_ = false;     // 重新初始化 = 重新认识环境
            baseline_mm_ = 0;
            last_sample_ms_ = (uint32_t)(esp_timer_get_time() / 1000);
            has_sample_ = true;
            stat_frames_ = 0;
            stat_since_ms_ = last_sample_ms_;
            ESP_LOGI(TAG, "sensor ready (continuous back-to-back, consume-new-frame-only)");
        }

        // ── 车停着就别采样，把 I2C 总线整个让给 OLED ──────────────────
        //   用户提议"文字模式下禁用传感器"，方向对（确实该在不需要时让出总线），
        //   但按**显示模式**开关不安全 —— 语音说"前进"时既是文字模式、车又在动。
        //   改成按**车有没有在动**判：
        //     · 车停着 → 不可能撞/掉下去，暂停采样，I2C 100% 让给 OLED
        //       （对话滚文字时每 250ms 要推 92ms，两者共用 C3 唯一那条 I2C）
        //     · 车一动 → 立刻恢复采样。IsMoving() 是纯内存查询，恢复是即时的。
        //   ⚠️ 不需要 XSHUT 硬件引脚：软件跳过 I2C 读效果完全一样，而硬件关断
        //      后重启还要重新初始化（几十 ms），反而更慢更麻烦。
        //   暂停期间 last_sample_ms_ 不再更新 → BlocksForward() 会因"数据太旧"
        //   放行，这正是我们要的（车没动就不需要拦）。
        if (motor_ != nullptr && !motor_->IsMoving()) {
            // ⚠️ 不是"完全不采样"，而是降到 500ms 一次：
            //   完全停掉的实测副作用是待机时面板读数彻底不刷新（DistGuard 日志
            //   也消失），看起来就像传感器掉了。低频采样既保住了面板可读性，
            //   又把 I2C 事务从 ~100 txn/s 压到 2 txn/s，争用基本归零。
            static int64_t idle_last_sample_ms = 0;
            const int64_t tnow = esp_timer_get_time() / 1000;
            if (tnow - idle_last_sample_ms < 500) {
                vTaskDelay(pdMS_TO_TICKS(20));   // 20ms 一问，车一动立刻恢复
                continue;
            }
            idle_last_sample_ms = tnow;
            // 走到下面正常采这一帧
        }

        // ── 取一帧：**只消费新帧** ──────────────────────────────────
        //   ① 不再自己触发测量：传感器在连续模式里自己 30Hz 跑，我们只读结果。
        //      单次测量的毛病是"触发→等 30ms→读"，CPU 被音频抢或 I2C 被 OLED
        //      占用时一帧要拖到 150ms 超时，采样变稀 → 车已经冲到桌沿了还在用
        //      100ms 前的读数判断（用户体感："用的还是上一次碰撞的距离"）。
        //   ② 中断没置位就**什么都不读**直接返回（ReadLatest 返回 0）→ 永远不会
        //      把上一帧当新数据（连续模式"读数卡住"的正确解法）。
        uint16_t mm = 0;
        uint8_t st = 0;
        int rr = vl->ReadLatest(&mm, &st);
        if (rr == 0) {
            // 还没有新帧（正常）。原来 2ms 轮询 = 每秒 500 次 I2C 事务，
            // 在单核 C3 上和音频任务抢总线/调度，会让语音断断续续。
            //   ⚠️ 2026-10-20 回退：曾尝试放宽到 30ms 想减少 I2C 争用，方向错了。
            //   争用的源头是 OLED 那 92ms 的整帧写，不是这里的空读；放宽反而让
            //   被 OLED 阻塞之后的采样更补不上，采样间隔更容易超过新鲜度窗口，
            //   结果是语音时前进拦截失效。改回 10ms，保证一有新帧就立刻取到。
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        if (rr < 0) {                        // I2C 真失败
            fail_streak_ = ++i2c_fail;
            if (i2c_fail >= 20) {
                ESP_LOGW(TAG, "sensor lost (i2c x%d), will re-init", i2c_fail);
                sensor_ok_ = false;
                state_ = kNoSensor;
                block_until_ms_ = 0;
                delete vl;
                vl = nullptr;
                sensor_ = nullptr;
                i2c_fail = 0;
            }
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        i2c_fail = 0;
        fail_streak_ = 0;

        // I2C 读返回全 0xFF（mm=65535 + st=15）说明这次传输是脏的，不是真实测量：
        //   直接丢帧，不参与 any 判定（否则会被当成"超量程"误判跌落）。
        //   2026-10-19 实测出现过一次，紧跟着就打出 "升级为 EDGE/FALL 8191mm"。
        if (mm == 0xFFFF && st == 15) {
            vTaskDelay(pdMS_TO_TICKS(6));
            continue;
        }

        int64_t now = esp_timer_get_time() / 1000;
        last_sample_ms_ = (uint32_t)now;
        has_sample_ = true;
        last_mm_ = mm;
        last_status_ = st;

        // ── 采样帧率统计 + 诊断 ──────────────────────────────────────
        //   fps 是**最近 1 秒真正拿到的新帧数**（正常 25~30）。对话/控制模式下如果
        //   掉到个位数甚至 0，说明音频任务把采样饿死了 —— 这正是"读数不实时、
        //   跌落失效"的可直接观测证据，不用再猜。
        stat_frames_++;
        if ((uint32_t)((uint32_t)now - stat_since_ms_) >= 1000) {
            fps_ = (int)stat_frames_;
            stat_frames_ = 0;
            stat_since_ms_ = (uint32_t)now;
        }
        {
            static int64_t last_diag_ms = 0;
            int age = (last_diag_ms == 0) ? 0 : (int)(now - last_diag_ms);
            // 10s 一条（原 3s）：串口打印走 USB-Serial-JTAG，本身也要 CPU；
            //   正常跑车时不需要 3s 一条，出问题再看 `EDGE/FALL` / 拒绝日志即可。
            if (last_diag_ms == 0 || age >= 10000) {
                // ⚠️ 带上阈值（进/出/跌落）：原来只有 mm/st/state/base，
                //   排查"读数到阈值了却没拦"时根本看不出阈值设的多少，
                //   只能靠猜。有了 obs/exit 一眼就能判断是否"阈值贴着基线"。
                ESP_LOGI(TAG, "mm=%u st=%u fps=%d state=%s base=%umm obs<=%d exit>%d fall>=%d",
                         (unsigned)mm, (unsigned)st, (int)fps_, StateName(state_),
                         (unsigned)baseline_mm_, obstacle_mm_,
                         obstacle_mm_ + HysteresisFor(obstacle_mm_), fall_mm_);
                last_diag_ms = now;
            }
        }

        // ── 分类 ───────────────────────────────────────────────────
        //   ⚠️ 防跌落不能只看"读数很大"：车头正对空房间、或传感器装得偏高时，
        //      正常读数本来就可能 >450mm —— 直接判跌落会让车永远不敢前进。
        //      "桌沿"的真正特征是「**先测到桌面这个近平面，然后忽然跳远/失效**」，
        //      所以跌落必须有 baseline（最近一次正常距离）作参照。
        // 8190/8191/65535 都是"没有可信目标"（超量程或脏数据），一律不算有效距离。
        //   注意：不算有效 ≠ 不拦 —— 下面会走超量程分支判跌落。
        const bool raw_range_ok = (mm > 0) && ((int)mm < 8000);
        // ⚠️ 实测这颗 GY-530 正常读数时 range_status 是 11（原始 0x5B），
        //   与 ST 文档「0 = Range Valid」不一致。只认 st==0 的话 baseline 永远
        //   建不起来 → 防跌落彻底失效（车会一路开下桌子）。所以 0 和 11 都算"无误码"。
        const bool status_ok = (st == 0) || (st == 11);
        const bool valid = status_ok && raw_range_ok;
        // ⚠️ 2026-10-19：这里原来要求 baseline 在 2s 内新鲜。
        //   用户把车举起来朝上（读数 8190、status=6）时，2s 后 baseline 过期 →
        //   判成 unknown → **不拦**。可"见过桌面、现在什么都测不到"本身就是桌沿/悬空，
        //   必须拦。改成"本次开机内见过近平面"就一直算数（reset/重新初始化才清）。
        const bool has_baseline = baseline_seen_ && ((int)baseline_mm_ < fall_mm_);
        (void)baseline_ms;

        // ⚠️ 这里**曾经**放过一段"每帧自动降阈值"的逻辑，已删除（2026-10-22）。
        //   它是个正反馈死循环，实测把阈值一路打到下限 40mm：
        //     74205  障碍阈值 68mm 太接近地面 81mm → 降到 54mm
        //     74705  障碍阈值 54mm 太接近地面 60mm → 降到 40mm
        //   成因：阈值一降，更多读数被判"近距离"，而基线 baseline_mm_
        //   会跟着近距离读数往下漂 → 阈值再降 → 收敛到下限。
        //   40mm 的障碍阈值等于把防碰撞彻底关掉（用户反馈"拦截失效"）。
        //   教训：**不要用自己被测对象当基准去调自己的阈值**。
        //   现在只保留 `!baseline_seen_` 里那次一次性修正（只在首次建基线时执行）。
        // ── 迟滞（2026-10-19 实测补）────────────────────────────────
        //   原来"进入/退出用同一个阈值"，读数在阈值上下抖 1~2mm（实测 68/70/71mm）
        //   就会: 碰撞 → 165ms 后解除 → 又碰撞 …… 车表现为"一顿一顿、走走停停"，
        //   体感就像"一直在用上一次碰撞的判断"。
        //   现在进入用 enter 阈值、退出要退到更远（+kHysteresisMm）才算安全。
        const int obs_enter = obstacle_mm_;
        const int hyst = HysteresisFor(obstacle_mm_);
        const int obs_exit = obstacle_mm_ + hyst;
        const int obs_th = (state_ == kObstacle) ? obs_exit : obs_enter;
        const int fall_th = (state_ == kFall) ? (fall_mm_ - hyst) : fall_mm_;

        int cand;
        if (raw_range_ok && (int)mm <= obs_th) {
            // ★ 关键：只要"距离数字"很小就按障碍算，**不看 range_status**。
            //   贴脸/近距时 VL53L0X 经常给出 status=1/2/4（sigma/相位失败），
            //   但距离值其实已经算出来了（实测手挡在 68mm，status 非 0）。
            //   以前要求 status==0 才算有效 → 状态一直是 unknown → 前进没被拦（漏判）。
            //   安全优先：宁可误拦，不可漏拦。
            cand = kObstacle;
            // ⚠️ 2026-10-22 补（防跌落失效的根因）：
            //   本传感器朝下测地面，地面读数（实测 ~95mm）本来就会落进"障碍"
            //   阈值内 —— 于是代码**永远走这个分支**，而 baseline_seen_ 只在
            //   下面第 423 行的"正常距离"分支里才置位，结果 baseline 一次都
            //   建不起来。后果：车一旦走到桌沿，读数变超量程(8191)，
            //   走 else 分支时 has_baseline==false → 判 kUnknown → **不拦** →
            //   车直接掉下桌子（用户实测"前进 10 秒防跌落失效"）。
            //   这里补上：首次见到任何近距离读数就把它当作近处参照。
            //   安全上没问题 —— 真正的桌沿是"从近突然变超量程"，
            //   而 baseline 只提供"我见过地面"这一个前提。
            if (!baseline_seen_) {
                baseline_mm = mm;
                baseline_ms = now;
                baseline_mm_ = mm;
                baseline_seen_ = true;
                // ⚠️ 阈值合理性自检（2026-10-22）：
                //   障碍阈值必须**明显小于**地面距离，否则地面本身就被判成障碍，
                //   语音"前进"每次都被拒 —— 日志实证：
                //     `mm=91 state=obstacle obs<=90` + `OBSTACLE 90mm -> 拒绝前进`
                //   用户可能在网页把阈值留在 90（NVS 里持久化的），而地面正好 90~95。
                //   NVS 是用户配置、不该擅自改写；但运行时必须修正，
                //   否则车一步都走不了。取地面距离的 2/3。
                if (obstacle_mm_ + 20 >= (int)mm) {
                    int fixed = (int)mm * 2 / 3;
                    if (fixed < 40) fixed = 40;    // 再小就没意义了（那叫贴脸）
                    if (fixed > 200) fixed = 200;
                    ESP_LOGW(TAG, "障碍阈值 %dmm 太接近地面 %dmm（地面会被误判成障碍），"
                             "运行时自动降到 %dmm；建议在网页把阈值改到 %d 以下",
                             obstacle_mm_, (int)mm, fixed, fixed);
                    obstacle_mm_ = fixed;
                    blind_mm_ = fixed * 3 / 4;
                }
            }
            last_valid_ms = now;
            last_valid_mm = mm;
        } else if (valid && (int)mm >= fall_th) {
            cand = has_baseline ? kFall : kClear;      // 没有近处参照 → 当作"前方空"，不拦
            last_valid_ms = now;
            last_valid_mm = mm;
        } else if (valid) {
            cand = kClear;
            baseline_mm = mm;                          // 正常距离带 → 刷新 baseline
            baseline_ms = now;
            baseline_mm_ = mm;
            baseline_seen_ = true;                     // 见过近平面 → 之后超量程一律算桌沿
            last_valid_ms = now;
            last_valid_mm = mm;
        } else if (st == 3) {
            // 最小量程失败：目标比传感器能测的最近距离还近 → 贴脸
            cand = kObstacle;
        } else if (st == 1 || st == 2) {
            // sigma / signal 不足且距离数字也不小 → 前方没有可靠目标。
            //   见过桌面 → 当桌沿处理（拦）；没见过 → 判不了。
            cand = has_baseline ? kFall : kUnknown;
        } else {
            // st==4（相位失败）或 8191：超量程，但也可能是近距盲区。
            //   ① 紧接在"很近"的有效读数之后 → 贴脸障碍（否则会后退掉下桌子）
            //   ② 刚测过正常距离、现在突然失效 → 这就是跨过桌沿
            //   ③ 都没有 → 判不了，不动作
            bool near_recent = (last_valid_ms > 0) && (now - last_valid_ms < 500) &&
                               ((int)last_valid_mm <= blind_mm_);
            if (near_recent) {
                cand = kObstacle;
            } else if (has_baseline) {
                cand = kFall;
            } else {
                cand = kUnknown;
            }
        }

        obstacle_cnt = (cand == kObstacle) ? obstacle_cnt + 1 : 0;
        fall_cnt = (cand == kFall) ? fall_cnt + 1 : 0;

        const bool danger = (state_ == kObstacle || state_ == kFall);

        // 开机静置 1.5s：刚上电时电机、电源、传感器都还没稳，先别动
        const bool settled = (now - boot_ms) > kSettleMs;

        if (!danger) {
            if (settled && obstacle_cnt >= kConfirmSamples) {
                EnterObstacle();
                clear_since = 0;
            } else if (settled && fall_cnt >= kConfirmSamples) {
                EnterFall();
                clear_since = 0;
            } else if (cand == kClear) {
                state_ = kClear;
                unknown_since = 0;
            } else {
                // 单帧"读数无效"不立刻改状态，避免面板 安全/读数无效 来回闪
                if (unknown_since == 0) unknown_since = now;
                if (now - unknown_since > 300) state_ = kUnknown;
            }
        } else {
            // 危险中：解除条件 =「不再出现危险读数」持续 kReleaseMs。
            //   ⚠️ 以前要求 cand 严格连续 250ms == kClear。实测桌面反光下 valid 帧和
            //   unknown 帧会交替出现（30Hz），clear_since 每帧被清零 → 状态永久卡在
            //   obstacle，前进永远被拒（用户实测：阈值 30mm、读数 106mm 仍显示
            //   「碰撞 / 拦截 是」）。现在 unknown 也算"不危险"。
            if (cand == kObstacle || cand == kFall) {
                // 另一类危险升级：障碍中又出现跌落 → 升级成 fall（封锁时间更长）
                if (cand == kFall && state_ == kObstacle && fall_cnt >= kConfirmSamples) {
                    state_ = kFall;
                    block_until_ms_ = (uint32_t)(now + 2000);
                    ESP_LOGW(TAG, "升级为 EDGE/FALL %umm", (unsigned)last_mm_);
                }
                clear_since = 0;
            } else {
                if (clear_since == 0) clear_since = now;
                if (now - clear_since >= kReleaseMs) {
                    state_ = kClear;
                    block_until_ms_ = 0;
                    obstacle_cnt = fall_cnt = 0;
                    ESP_LOGI(TAG, "danger cleared (mm=%u st=%u)", (unsigned)last_mm_,
                             (unsigned)last_status_);
                }
            }

            // ── 迟滞带锁死兜底（2026-10-20）─────────────────────────────
            //   症状：用户设障碍阈值 100mm，而安装基线就贴在 107mm。一次读数
            //   抖动到 ≤100 就进入 kObstacle，之后因为迟滞带要求退到 116mm 才
            //   放行，而实际读数一直是 107 → cand 永远是 kObstacle →
            //   clear_since 被无限清零 → **前进被永久拒绝**。
            //   判据：状态是 kObstacle，但当前读数已经**不满足进入条件**
            //   （mm > obs_enter）→ 说明只是"没退够迟滞带"，不是真危险。
            //   持续 kForceReleaseMs 就强制解除。
            //   ⚠️ 只对 kObstacle 生效，kFall（桌沿）不做兜底 —— 那是掉下去级
            //      的风险，宁可锁死也不能自动放行。
            if (state_ == kObstacle && raw_range_ok && (int)mm > obs_enter) {
                if (stuck_since == 0) {
                    stuck_since = now;
                } else if (now - stuck_since >= kForceReleaseMs) {
                    ESP_LOGW(TAG, "迟滞带锁死兜底：强制解除 (mm=%u > enter=%d 已持续 %dms)",
                             (unsigned)mm, obs_enter, (int)(now - stuck_since));
                    state_ = kClear;
                    // ⚠️ 解除后**继续拦 kForceReleaseMs**：原来这里直接清零，
                    //   于是"解除 → 摇杆/动作还在持续请求前进 → 立刻又走 → 又进入
                    //   障碍 → 又停"，车变成走走停停的抖动。
                    //   实测日志（阈值70 / 基线72~76）：
                    //     SAFE(ramp) 拦停 → 读数回到73 → 兜底解除 → net=30 又走
                    //     → SAFE(ramp) 又停 …… 用户看到的"拦截不停"其实是这个。
                    //   留一段缓冲，给操作者时间松手/把障碍物移开。
                    block_until_ms_ = now + kForceReleaseMs;
                    obstacle_cnt = fall_cnt = 0;
                    clear_since = 0;
                    stuck_since = 0;
                }
            } else {
                stuck_since = 0;
            }

            // ⚠️ 2026-10-19 血泪：这里原来"危险持续期间每 600ms 再退一次"，
            //    结果是墙一直在 13cm 内时 → 每 600ms 抖一下、无限后退，
            //    电机反复冷启动把电池拉到 BOD 复位 → 表现为「电机间断动 + 不断重启」。
            //    现在一次危险只做一次动作，之后只保持"停止 + 禁止前进"。
        }
    }
}

// 动作配额：10 秒内最多 3 次，超了就只停不动（防止无限后退 / 反复冷启动拉崩电池）
void DistanceGuard::EnterObstacle() {
    int64_t now = esp_timer_get_time() / 1000;
    state_ = kObstacle;
    block_until_ms_ = (uint32_t)(now + 800);
    ESP_LOGW(TAG, "OBSTACLE %umm -> 拒绝前进", (unsigned)last_mm_);
    // ⚠️ 这里**绝不碰电机 API**（2026-10-19 两次实机崩溃的根因）：
    //   从 dist_guard 任务里调 ApplyStop()/Drive() 会在启动竞态下
    //   触发未捕获 C++ 异常 → std::terminate → abort → 无限重启。
    //   停车改由 Motor2WD::RampTick() 里的同一个钩子完成（电机自己的定时器任务）。
}

void DistanceGuard::EnterFall() {
    int64_t now = esp_timer_get_time() / 1000;
    state_ = kFall;
    block_until_ms_ = (uint32_t)(now + 2000);
    // 带上判据：只有 mm/st/阈值都打出来，才分得清"真到桌沿"还是
    //   "一帧脏数据（8191/st=6）被当成桌沿"—— 后者是"语音拒绝前进但
    //   距离看着很正常"的常见来源。
    ESP_LOGW(TAG, "EDGE/FALL %umm st=%u -> 拒绝前进 (fall_mm>=%d, base=%umm)",
             (unsigned)last_mm_, (unsigned)last_status_, fall_mm_, (unsigned)baseline_mm_);
    // 同上：只置状态，不动电机。跌落只停不退，脱困靠用户/LLM 明确下"后退"。
}

std::string DistanceGuard::StatusJson() const {
    return std::string(StatusCStr());
}

const char* DistanceGuard::StatusCStr() const {
    // thread_local：这个函数会被 httpd（网页 250ms 轮询）和 Application 主循环
    //   （MCP 工具日志）并发调用，共用一块 static buffer 会互相覆盖 →
    //   前端偶发收到半新半旧的 JSON。
    thread_local char buf[420];
    const uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
    long age = has_sample_ ? (long)(uint32_t)(now - last_sample_ms_) : -1;
    snprintf(buf, sizeof(buf),
             "{\"sensor_ok\":%s,\"enabled\":%s,\"mm\":%u,\"range_status\":%u,"
             "\"state\":\"%s\",\"blocked_forward\":%s,\"age_ms\":%ld,"
             "\"obstacle_mm\":%d,\"obstacle_exit_mm\":%d,\"fall_mm\":%d,"
             "\"baseline_mm\":%u,\"i2c_fail\":%d,\"fps\":%d}",
             sensor_ok_ ? "true" : "false",
             enabled_ ? "true" : "false",
             (unsigned)last_mm_, (unsigned)last_status_,
             StateName(state_),
             BlocksForward() ? "true" : "false",
             age, obstacle_mm_, obstacle_mm_ + HysteresisFor(obstacle_mm_), fall_mm_,
             (unsigned)baseline_mm_, fail_streak_, (int)fps_);
    return buf;
}
