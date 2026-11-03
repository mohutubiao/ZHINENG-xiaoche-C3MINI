// ── VL53L0X（GY-530）精简驱动 / ESP-IDF i2c_master ────────────────────────
// 初始化序列照搬 Pololu VL53L0X Arduino 库（即 ST 官方 VL53L0X_DataInit /
// StaticInit / PerformRefCalibration 流程 + DefaultTuningSettings 表），
// 只把 Wire 换成本项目的 i2c_master，并去掉本用途用不到的
// VCSEL 周期调节 / 时序预算重算等接口。
//
// 使用前提：I2C 总线已由别处创建（小车板上是 OLED 用的 I2C_NUM_0，
// SDA=GPIO9 / SCL=GPIO8）。ESP32-C3 只有一个 I2C 控制器，**不能**再新建总线。
#pragma once

#include <cstddef>
#include <cstdint>
#include <driver/i2c_master.h>

class Vl53l0xC3 {
public:
    // VL53L0X 超出量程 / 无效时返回的原始值
    static constexpr uint16_t kRangeInvalid = 8191;

    Vl53l0xC3(i2c_master_bus_handle_t bus, uint8_t addr = 0x29);
    ~Vl53l0xC3();

    Vl53l0xC3(const Vl53l0xC3&) = delete;
    Vl53l0xC3& operator=(const Vl53l0xC3&) = delete;

    // 完整上电初始化（含 SPAD 参考校准 + VHV/相位校准，耗时约 100~150ms）
    bool Init();

    // 连续测量（back-to-back），之后每次 Read() 等一帧，约 30~35ms 一帧
    bool StartContinuous();
    void StopContinuous();

    // 读一帧。返回 false = I2C 失败/超时（传感器掉线）。
    // 返回 true 时：*mm 为原始距离，*range_status 为 0..5
    //   0 = 有效；1 = sigma fail；2 = signal fail；
    //   3 = 最小量程失败（太近）；4 = 相位失败（超量程/太远）；5 = 硬件失败
    //   距离无效时 *mm == 8191
    bool Read(uint16_t* mm, uint8_t* range_status, uint32_t timeout_ms = 80);

    // 单次测量：每次自己触发测量 + 清中断，不依赖连续模式的中断状态。
    //   缺点：I2C 忙 / CPU 被音频抢占时，一帧要等满超时（150ms），采样会变稀。
    bool ReadSingle(uint16_t* mm, uint8_t* range_status, uint32_t timeout_ms = 150);

    // 连续模式专用：只取"新完成的一帧"，由保护层循环调用（推荐）。
    //   返回  1 = 取到新帧（*mm / *range_status 有效）
    //   返回  0 = 还没有新帧（中断未置位），调用方等一会儿再来
    //   返回 -1 = I2C 通信失败
    //   ⚠️ 关键是 0 这个分支：中断未置位时**绝不读结果寄存器**，所以永远不会重复
    //      消费上一帧（这正是之前"读数卡住 / 一直用上一次碰撞距离"的根因）。
    //      而采样节拍由传感器自己驱动（back-to-back），本任务被高优先级音频任务
    //      饿住时，读回来也总是最新一帧，不是旧值。
    int ReadLatest(uint16_t* mm, uint8_t* range_status);

    bool ready() const { return ready_; }

private:
    enum : uint8_t {
        SYSRANGE_START = 0x00,
        SYSTEM_SEQUENCE_CONFIG = 0x01,
        SYSTEM_INTERMEASUREMENT_PERIOD = 0x04,
        SYSTEM_INTERRUPT_CONFIG_GPIO = 0x0A,
        SYSTEM_INTERRUPT_CLEAR = 0x0B,
        RESULT_INTERRUPT_STATUS = 0x13,
        RESULT_RANGE_STATUS = 0x14,
        MSRC_CONFIG_CONTROL = 0x60,
        FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44,
        GPIO_HV_MUX_ACTIVE_HIGH = 0x84,
        GLOBAL_CONFIG_SPAD_ENABLES_REF_0 = 0xB0,
        GLOBAL_CONFIG_REF_EN_START_SELECT = 0xB6,
        DYNAMIC_SPAD_REF_EN_START_OFFSET = 0x4F,
        DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD = 0x4E,
        IDENTIFICATION_MODEL_ID = 0xC0,
        OSC_CALIBRATE_VAL = 0xF8,
        POWER_MANAGEMENT_GO1_POWER_FORCE = 0x80,
        VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV = 0x89,
    };

    bool Write8(uint8_t reg, uint8_t value);
    bool Write16(uint8_t reg, uint16_t value);
    bool Write32(uint8_t reg, uint32_t value);
    bool Read8(uint8_t reg, uint8_t* value);
    bool Read16(uint8_t reg, uint16_t* value);
    bool ReadMulti(uint8_t reg, uint8_t* dst, size_t count);

    bool SetSignalRateLimit(float mcps);
    bool GetSpadInfo(uint8_t* count, bool* type_is_aperture);
    bool PerformSingleRefCalibration(uint8_t vhv_init_byte);
    bool WaitInterrupt(uint32_t timeout_ms);

    i2c_master_bus_handle_t bus_ = nullptr;
    i2c_master_dev_handle_t dev_ = nullptr;
    uint8_t addr_ = 0x29;
    uint8_t stop_variable_ = 0;
    bool ready_ = false;
};
