// ── 单颗 VL53L0X 同时做「防碰撞 + 防跌落」────────────────────────────────
// 装法：屏幕正上方，微微朝下（h≈8cm、俯角≈20° → 正常斜距≈23cm）。
// 判据（一颗传感器两个方向）：
//   · 读数变小   → 前方有障碍
//   · 读数变大 / 超量程 8191 → 前方悬空（桌沿、跌落）
//   · 正常值夹在中间 → 安全
//
// ⚠️ 最大的坑：近距盲区（约 3~5cm）也会返回 8191。只按"大=跌落"会把
//    贴在脸前的障碍误判成悬崖 → 车会往后退（更危险）。所以用"上一帧有效
//    距离很近 + 时间上紧邻"来把盲区归到障碍一侧。
#pragma once

#include <driver/i2c_master.h>

#include <string>

class Motor2WD;

class DistanceGuard {
public:
    enum State : int {
        kDisabled = 0,   // 用户关了
        kNoSensor = 1,   // 传感器没接 / 初始化失败（不拦截电机）
        kClear = 2,      // 安全
        kObstacle = 3,   // 前方障碍
        kFall = 4,       // 前方悬空（防跌落）
        kUnknown = 5,    // 读数无效且无法判断
    };

    struct Config {
        // 默认按"屏幕上方略朝下"的装法：桌面正常读数约 115~130mm，
        //   所以碰撞阈值取 90mm（正常值掉 30% 才算贴脸），不会一上电就自我触发。
        int obstacle_mm = 90;    // ≤ 这个距离 = 障碍
        int fall_mm = 450;       // ≥ 这个距离 = 悬空/跌落
        int blind_mm = 67;       // 上一帧有效距离 ≤ 这个值 → 无效帧按"贴脸"算
    };

    static DistanceGuard& Instance();

    // 惰性启动（内部建任务）。bus 直接用 OLED 那条 I2C 总线，不要新建。
    void Start(Motor2WD* motor, i2c_master_bus_handle_t bus);

    void SetEnabled(bool on);
    bool enabled() const { return enabled_; }
    void ResetState() { reset_req_ = true; }

    void SetThresholds(int obstacle_mm, int fall_mm);
    Config config() const;

    // 前进是否被拦截（Motor2WD::Drive 的前进钩子调用这个）
    bool BlocksForward() const;

    // 探索模式等内部逻辑用的快速查询（不拼 JSON）
    int state() const { return state_; }
    uint16_t last_mm() const { return last_mm_; }
    bool sensor_ready() const { return sensor_ok_; }
    bool danger() const { return state_ == kObstacle || state_ == kFall; }

    // 给 MCP 工具用的状态 JSON
    std::string StatusJson() const;
    // 给网页（弱符号桥接）用的 C 字符串版本：内部固定缓冲区，不要长期保存指针
    const char* StatusCStr() const;

    static const char* StateName(int state);

private:
    DistanceGuard() = default;
    static void TaskEntry(void* arg);
    void Loop();
    void EnterObstacle();
    void EnterFall();

    Motor2WD* motor_ = nullptr;
    i2c_master_bus_handle_t bus_ = nullptr;
    void* sensor_ = nullptr;   // Vl53l0xC3*，避免头文件里暴露驱动实现

    volatile bool enabled_ = true;
    volatile bool sensor_ok_ = false;
    volatile int state_ = kNoSensor;
    volatile uint16_t last_mm_ = 0;
    volatile uint8_t last_status_ = 0;
    volatile uint16_t baseline_mm_ = 0;   // 最近一次正常距离（诊断用）
    // 本次开机内是否见过"近处平面"（桌面）。
    //   见过的之后，任何超量程/相位失败都按"桌沿"处理 —— 不再要求 baseline 新鲜
    //   （用户把车举起来朝上，读数 8190，2s 后 baseline 过期就漏拦了）。
    volatile bool baseline_seen_ = false;
    volatile int fail_streak_ = 0;        // 连续 I2C 失败帧数（诊断用）
    volatile int fps_ = 0;                // 最近 1 秒真正取到的新帧数（诊断用）
    // ⚠️ 用 32 位毫秒（开机计）而不是 volatile int64_t：
    //   BlocksForward() 由 esp_timer 任务和采样任务并发读，RISC-V 上 64 位读
    //   会被拆成两次 LDR —— 可能读到"新低 32 位 + 旧高 32 位"的垃圾，
    //   age 变成几百万 ms 或负数，某一 tick 就误判"数据太旧 → 不拦"。
    //   32 位单字读写天然原子，49.7 天回绕用无符号差值比较即可。
    volatile uint32_t block_until_ms_ = 0;
    volatile uint32_t last_sample_ms_ = 0;
    // "有没有采过样"必须单独存：把 last_sample_ms_==0 当哨兵的话，
    //   开机 49.7 天时 now 恰好等于 0 的那 1ms 会被误判成"数据陈旧 → 不拦"。
    volatile bool has_sample_ = false;
    uint32_t stat_since_ms_ = 0;          // fps 统计窗口起点（仅采样任务内用）
    int stat_frames_ = 0;                 // 统计窗口内帧数（仅采样任务内用）

    volatile bool reset_req_ = false;

    // 阈值（MCP 可改，整型读写在本平台是原子的，够用）
    //   ⚠️ 2026-10-22 障碍阈值 90 → 60：这颗传感器**朝下测地面**，
    //      实测地面就在 86~90mm。阈值 90/100 时地面本身就被判成"障碍"
    //      （日志：`mm=86 state=obstacle obs<=100`），后果是语音"前进"
    //      被 BlocksForward() 直接拒绝 —— 用户看到的就是"车不走"。
    //   朝下的探头其实不存在"前方障碍物"，真正要防的是**地面消失**
    //      （桌沿，走 fall_mm 那条判断）。所以这条阈值只保留很小的值，
    //      用来拦"贴地/凸起"（地面突然近于 60mm 说明磕到东西了）。
    volatile int obstacle_mm_ = 60;
    volatile int fall_mm_ = 450;
    volatile int blind_mm_ = 45;

    bool started_ = false;

    // ── 传感器存在性探测（2026-10-22）─────────────────────────────
    //   用户反馈："有的人不装激光雷达"。原逻辑每 2 秒重试一次 Init()，
    //   没装模块的板子会**永久**刷 `no ACK at 0x29`，而且每次重试都要
    //   占用 I2C 总线 —— 这条总线跟 OLED 共用，白白拖慢刷屏和测距。
    //   现在：连续失败 3 次判"未安装"，之后停止访问总线，仅 60 秒回头看一次
    //   （万一后来焊上了）。装好模块后重启可立即启用。
    uint8_t probe_fail_ = 0;         // Init 连续失败次数
    bool sensor_absent_ = false;     // true = 已判定未安装，采样任务挂起
    uint32_t next_probe_ms_ = 0;     // 下次重新探测的时间点（ms）

    // ── 文字模式挂起（2026-10-22，用户决定："文字模式关掉雷达"）────
    //   屏幕进入对话文字模式后会高频滚动推屏，和 VL53L0X 共用同一条 I2C；
    //   采样被挤掉后 BlocksForward() 会因"数据过期"直接放行，
    //   变成一种时灵时不灵的假保护 —— 比没有更糟。
    //   按定义：文字模式里干脆明确停采样，把总线让给屏幕。
    //   ⚠️ 副作用：挂起期间**防跌落也不生效**。
    bool text_mode_suspended_ = false;
};

// 注册给 Motor2WD 的前进拦截钩子
bool DistanceGuardBlocksForward();
