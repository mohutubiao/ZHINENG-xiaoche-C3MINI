#include "vl53l0x_c3.h"

#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <cstring>

#define TAG "VL53L0X"

// ST 官方 DefaultTuningSettings（vl53l0x_tuning.h），Pololu 库里是内联写死的，
// 这里压成表；顺序不能改，尤其是 0xFF 翻页。
static const uint8_t kTuning[][2] = {
    {0xFF, 0x01}, {0x00, 0x00},
    {0xFF, 0x00}, {0x09, 0x00}, {0x10, 0x00}, {0x11, 0x00},
    {0x24, 0x01}, {0x25, 0xFF}, {0x75, 0x00},
    {0xFF, 0x01}, {0x4E, 0x2C}, {0x48, 0x00}, {0x30, 0x20},
    {0xFF, 0x00}, {0x30, 0x09}, {0x54, 0x00}, {0x31, 0x04},
    {0x32, 0x03}, {0x40, 0x83}, {0x46, 0x25}, {0x60, 0x00},
    {0x27, 0x00}, {0x50, 0x06}, {0x51, 0x00}, {0x52, 0x96},
    {0x56, 0x08}, {0x57, 0x30}, {0x61, 0x00}, {0x62, 0x00},
    {0x64, 0x00}, {0x65, 0x00}, {0x66, 0xA0},
    {0xFF, 0x01}, {0x22, 0x32}, {0x47, 0x14}, {0x49, 0xFF}, {0x4A, 0x00},
    {0xFF, 0x00}, {0x7A, 0x0A}, {0x7B, 0x00}, {0x78, 0x21},
    {0xFF, 0x01}, {0x23, 0x34}, {0x42, 0x00}, {0x44, 0xFF},
    {0x45, 0x26}, {0x46, 0x05}, {0x40, 0x40}, {0x0E, 0x06},
    {0x20, 0x1A}, {0x43, 0x40},
    {0xFF, 0x00}, {0x34, 0x03}, {0x35, 0x44},
    {0xFF, 0x01}, {0x31, 0x04}, {0x4B, 0x09}, {0x4C, 0x05}, {0x4D, 0x04},
    {0xFF, 0x00}, {0x44, 0x00}, {0x45, 0x20}, {0x47, 0x08},
    {0x48, 0x28}, {0x67, 0x00}, {0x70, 0x04}, {0x71, 0x01},
    {0x72, 0xFE}, {0x76, 0x00}, {0x77, 0x00},
    {0xFF, 0x01}, {0x0D, 0x01},
    {0xFF, 0x00}, {0x80, 0x01}, {0x01, 0xF8},
    {0xFF, 0x01}, {0x8E, 0x01}, {0x00, 0x01},
    {0xFF, 0x00}, {0x80, 0x00},
};

Vl53l0xC3::Vl53l0xC3(i2c_master_bus_handle_t bus, uint8_t addr) : bus_(bus), addr_(addr) {
    if (bus_ == nullptr) {
        return;
    }
    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = addr_;
    dev_cfg.scl_speed_hz = 400 * 1000;
    if (i2c_master_bus_add_device(bus_, &dev_cfg, &dev_) != ESP_OK) {
        ESP_LOGE(TAG, "add device 0x%02X failed", addr_);
        dev_ = nullptr;
    }
}

Vl53l0xC3::~Vl53l0xC3() {
    if (dev_ != nullptr) {
        i2c_master_bus_rm_device(dev_);
        dev_ = nullptr;
    }
}

bool Vl53l0xC3::Write8(uint8_t reg, uint8_t value) {
    if (dev_ == nullptr) return false;
    uint8_t buf[2] = {reg, value};
    return i2c_master_transmit(dev_, buf, sizeof(buf), 50) == ESP_OK;
}

bool Vl53l0xC3::Write16(uint8_t reg, uint16_t value) {
    if (dev_ == nullptr) return false;
    uint8_t buf[3] = {reg, (uint8_t)(value >> 8), (uint8_t)value};
    return i2c_master_transmit(dev_, buf, sizeof(buf), 50) == ESP_OK;
}

bool Vl53l0xC3::Write32(uint8_t reg, uint32_t value) {
    if (dev_ == nullptr) return false;
    uint8_t buf[5] = {reg, (uint8_t)(value >> 24), (uint8_t)(value >> 16),
                      (uint8_t)(value >> 8), (uint8_t)value};
    return i2c_master_transmit(dev_, buf, sizeof(buf), 50) == ESP_OK;
}

bool Vl53l0xC3::Read8(uint8_t reg, uint8_t* value) {
    if (dev_ == nullptr) return false;
    return i2c_master_transmit_receive(dev_, &reg, 1, value, 1, 50) == ESP_OK;
}

bool Vl53l0xC3::Read16(uint8_t reg, uint16_t* value) {
    uint8_t buf[2] = {};
    if (!ReadMulti(reg, buf, 2)) return false;
    *value = ((uint16_t)buf[0] << 8) | buf[1];
    return true;
}

bool Vl53l0xC3::ReadMulti(uint8_t reg, uint8_t* dst, size_t count) {
    if (dev_ == nullptr) return false;
    return i2c_master_transmit_receive(dev_, &reg, 1, dst, count, 50) == ESP_OK;
}

bool Vl53l0xC3::SetSignalRateLimit(float mcps) {
    if (mcps < 0 || mcps > 511.99f) return false;
    // Q9.7 定点
    return Write16(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, (uint16_t)(mcps * (1 << 7)));
}

bool Vl53l0xC3::WaitInterrupt(uint32_t timeout_ms) {
    int64_t deadline = esp_timer_get_time() / 1000 + (int64_t)timeout_ms;
    while (true) {
        uint8_t st = 0;
        if (!Read8(RESULT_INTERRUPT_STATUS, &st)) return false;
        if ((st & 0x07) != 0) return true;
        if (esp_timer_get_time() / 1000 > deadline) return false;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

bool Vl53l0xC3::GetSpadInfo(uint8_t* count, bool* type_is_aperture) {
    uint8_t tmp = 0;
    if (!Write8(0x80, 0x01)) return false;
    if (!Write8(0xFF, 0x01)) return false;
    if (!Write8(0x00, 0x00)) return false;
    if (!Write8(0xFF, 0x06)) return false;

    if (!Read8(0x83, &tmp)) return false;
    if (!Write8(0x83, tmp | 0x04)) return false;
    if (!Write8(0xFF, 0x07)) return false;
    if (!Write8(0x81, 0x01)) return false;
    if (!Write8(0x80, 0x01)) return false;
    if (!Write8(0x94, 0x6b)) return false;
    if (!Write8(0x83, 0x00)) return false;

    // 等 0x83 被芯片清零（SPAD 信息就绪）
    int64_t deadline = esp_timer_get_time() / 1000 + 500;
    while (true) {
        if (!Read8(0x83, &tmp)) return false;
        if (tmp != 0x00) break;
        if (esp_timer_get_time() / 1000 > deadline) {
            ESP_LOGE(TAG, "SPAD info timeout");
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    if (!Write8(0x83, 0x01)) return false;
    if (!Read8(0x92, &tmp)) return false;

    *count = tmp & 0x7f;
    *type_is_aperture = (tmp >> 7) & 0x01;

    if (!Write8(0x81, 0x00)) return false;
    if (!Write8(0xFF, 0x06)) return false;
    uint8_t v83 = 0;
    if (!Read8(0x83, &v83)) return false;
    if (!Write8(0x83, v83 & ~0x04)) return false;
    if (!Write8(0xFF, 0x01)) return false;
    if (!Write8(0x00, 0x01)) return false;
    if (!Write8(0xFF, 0x00)) return false;
    if (!Write8(0x80, 0x00)) return false;
    return true;
}

bool Vl53l0xC3::PerformSingleRefCalibration(uint8_t vhv_init_byte) {
    if (!Write8(SYSRANGE_START, 0x01 | vhv_init_byte)) return false;
    if (!WaitInterrupt(500)) {
        ESP_LOGE(TAG, "ref calibration timeout (vhv=0x%02X)", vhv_init_byte);
        return false;
    }
    if (!Write8(SYSTEM_INTERRUPT_CLEAR, 0x01)) return false;
    if (!Write8(SYSRANGE_START, 0x00)) return false;
    return true;
}

bool Vl53l0xC3::Init() {
    ready_ = false;
    if (dev_ == nullptr) return false;

    uint8_t model = 0;
    if (!Read8(IDENTIFICATION_MODEL_ID, &model)) {
        // 没装模块时保护层每 2s 重试一次初始化，日志按 10s 节流，避免刷屏。
        //   没装模块不影响小车其它功能：sensor_ok_ 一直 false → BlocksForward()
        //   直接返回 false（不拦截），语音/遥控/表情全部照常。
        static int64_t last_warn_ms = 0;
        int64_t now = esp_timer_get_time() / 1000;
        if (now - last_warn_ms > 10000) {
            last_warn_ms = now;
            ESP_LOGW(TAG, "no ACK at 0x%02X (module not installed / SDA-SCL swapped)", addr_);
        }
        return false;
    }
    if (model != 0xEE) {
        ESP_LOGE(TAG, "unexpected model id 0x%02X (expected 0xEE)", model);
        return false;
    }

    // ── DataInit ──────────────────────────────────────────────
    uint8_t vhv = 0;
    if (!Read8(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, &vhv)) return false;
    if (!Write8(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, vhv | 0x01)) return false;  // 2V8 模式

    if (!Write8(0x88, 0x00)) return false;
    if (!Write8(0x80, 0x01)) return false;
    if (!Write8(0xFF, 0x01)) return false;
    if (!Write8(0x00, 0x00)) return false;
    if (!Read8(0x91, &stop_variable_)) return false;
    if (!Write8(0x00, 0x01)) return false;
    if (!Write8(0xFF, 0x00)) return false;
    if (!Write8(0x80, 0x00)) return false;

    uint8_t msrc = 0;
    if (!Read8(MSRC_CONFIG_CONTROL, &msrc)) return false;
    if (!Write8(MSRC_CONFIG_CONTROL, msrc | 0x12)) return false;

    if (!SetSignalRateLimit(0.25f)) return false;
    if (!Write8(SYSTEM_SEQUENCE_CONFIG, 0xFF)) return false;

    // ── StaticInit：SPAD 参考 ─────────────────────────────────
    uint8_t spad_count = 0;
    bool spad_is_aperture = false;
    if (!GetSpadInfo(&spad_count, &spad_is_aperture)) return false;

    uint8_t ref_spad_map[6] = {};
    if (!ReadMulti(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6)) return false;

    if (!Write8(0xFF, 0x01)) return false;
    if (!Write8(DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00)) return false;
    if (!Write8(DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C)) return false;
    if (!Write8(0xFF, 0x00)) return false;
    if (!Write8(GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4)) return false;

    uint8_t first_spad = spad_is_aperture ? 12 : 0;
    uint8_t enabled = 0;
    for (uint8_t i = 0; i < 48; i++) {
        if (i < first_spad || enabled == spad_count) {
            ref_spad_map[i / 8] &= ~(1 << (i % 8));
        } else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x01) {
            enabled++;
        }
    }
    {
        uint8_t buf[7] = {GLOBAL_CONFIG_SPAD_ENABLES_REF_0};
        memcpy(&buf[1], ref_spad_map, 6);
        if (i2c_master_transmit(dev_, buf, sizeof(buf), 50) != ESP_OK) return false;
    }

    // ── tuning settings ──────────────────────────────────────
    for (auto& kv : kTuning) {
        if (!Write8(kv[0], kv[1])) return false;
    }

    // ── GPIO 中断配置（新样本就绪，低有效）────────────────────
    if (!Write8(SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04)) return false;
    uint8_t hv = 0;
    if (!Read8(GPIO_HV_MUX_ACTIVE_HIGH, &hv)) return false;
    if (!Write8(GPIO_HV_MUX_ACTIVE_HIGH, hv & ~0x10)) return false;
    if (!Write8(SYSTEM_INTERRUPT_CLEAR, 0x01)) return false;

    // 关掉 MSRC / TCC（默认），与 ST API 一致
    if (!Write8(SYSTEM_SEQUENCE_CONFIG, 0xE8)) return false;

    // ── 参考校准（VHV + 相位）────────────────────────────────
    if (!Write8(SYSTEM_SEQUENCE_CONFIG, 0x01)) return false;
    if (!PerformSingleRefCalibration(0x40)) return false;
    if (!Write8(SYSTEM_SEQUENCE_CONFIG, 0x02)) return false;
    if (!PerformSingleRefCalibration(0x00)) return false;
    if (!Write8(SYSTEM_SEQUENCE_CONFIG, 0xE8)) return false;

    ready_ = true;
    ESP_LOGI(TAG, "init ok (spad_count=%u, aperture=%d)", spad_count, (int)spad_is_aperture);
    return true;
}

bool Vl53l0xC3::StartContinuous() {
    if (!ready_) return false;
    // 先清掉校准阶段残留的中断，否则 ReadLatest 第一次会把旧结果当新帧
    Write8(SYSTEM_INTERRUPT_CLEAR, 0x01);
    if (!Write8(0x80, 0x01)) return false;
    if (!Write8(0xFF, 0x01)) return false;
    if (!Write8(0x00, 0x00)) return false;
    if (!Write8(0x91, stop_variable_)) return false;
    if (!Write8(0x00, 0x01)) return false;
    if (!Write8(0xFF, 0x00)) return false;
    if (!Write8(0x80, 0x00)) return false;
    // 0x02 = back-to-back（不等间隔，测得越快越好）
    return Write8(SYSRANGE_START, 0x02);
}

void Vl53l0xC3::StopContinuous() {
    if (!ready_) return;
    Write8(SYSRANGE_START, 0x01);
    Write8(0xFF, 0x01);
    Write8(0x00, 0x00);
    Write8(0x91, 0x00);
    Write8(0x00, 0x01);
    Write8(0xFF, 0x00);
}

bool Vl53l0xC3::ReadSingle(uint16_t* mm, uint8_t* range_status, uint32_t timeout_ms) {
    if (!ready_ || mm == nullptr || range_status == nullptr) return false;

    Write8(SYSTEM_INTERRUPT_CLEAR, 0x01);   // 清掉可能残留的中断

    // 单次测量启动序列（ST 手册要求带 stop_variable 的这套翻页写）
    if (!Write8(0x80, 0x01)) return false;
    if (!Write8(0xFF, 0x01)) return false;
    if (!Write8(0x00, 0x00)) return false;
    if (!Write8(0x91, stop_variable_)) return false;
    if (!Write8(0x00, 0x01)) return false;
    if (!Write8(0xFF, 0x00)) return false;
    if (!Write8(0x80, 0x00)) return false;
    if (!Write8(SYSRANGE_START, 0x01)) return false;

    if (!WaitInterrupt(timeout_ms)) return false;

    uint8_t buf[12] = {};
    if (!ReadMulti(RESULT_RANGE_STATUS, buf, sizeof(buf))) return false;

    *range_status = (buf[0] & 0x78) >> 3;
    *mm = ((uint16_t)buf[10] << 8) | buf[11];

    Write8(SYSTEM_INTERRUPT_CLEAR, 0x01);
    return true;
}

int Vl53l0xC3::ReadLatest(uint16_t* mm, uint8_t* range_status) {
    if (!ready_ || mm == nullptr || range_status == nullptr) return -1;

    // ① 0x13（中断状态）+ 0x14..0x1F（12 字节结果）一次读 13 字节。
    //    原来分两次事务（先读中断、再读结果），在 C3 上和音频/OLED 抢 I2C 时
    //    额外多一次总线往返 —— 合并后每帧只占一次事务，语音断断续续的问题会好一些。
    uint8_t buf[13] = {};
    if (!ReadMulti(RESULT_INTERRUPT_STATUS, buf, sizeof(buf))) return -1;

    // ② 没有新帧就立刻返回 0 —— **不消费结果寄存器**，
    //    这样哪怕上层每秒只被调度到 1 次，也不会把上一帧当成新数据。
    if ((buf[0] & 0x07) == 0) return 0;

    *range_status = (buf[1] & 0x78) >> 3;              // buf[1] = 0x14
    *mm = ((uint16_t)buf[11] << 8) | buf[12];          // 0x1E / 0x1F

    // ③ 清中断 → back-to-back 模式下传感器自动开始下一次测量
    if (!Write8(SYSTEM_INTERRUPT_CLEAR, 0x01)) return -1;
    return 1;
}

bool Vl53l0xC3::Read(uint16_t* mm, uint8_t* range_status, uint32_t timeout_ms) {
    if (!ready_ || mm == nullptr || range_status == nullptr) return false;
    if (!WaitInterrupt(timeout_ms)) return false;

    // 0x14 起 12 字节：buf[0] 的 bit3..6 是 range status，buf[10..11] 是距离（mm）
    uint8_t buf[12] = {};
    if (!ReadMulti(RESULT_RANGE_STATUS, buf, sizeof(buf))) return false;

    *range_status = (buf[0] & 0x78) >> 3;
    *mm = ((uint16_t)buf[10] << 8) | buf[11];

    // 清中断，允许下一帧
    if (!Write8(SYSTEM_INTERRUPT_CLEAR, 0x01)) return false;
    return true;
}
