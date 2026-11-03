#ifndef _AUDIO_CODEC_H
#define _AUDIO_CODEC_H

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <driver/i2s_std.h>

#include <vector>
#include <string>
#include <functional>

#include "board.h"

// I2S DMA 缓冲深度（2026-10-22 从 6×240 加大到 8×320）
//   ⚠️ 这是"吞音"最直接的物理上限：
//     输出 24000Hz 时，6×240=1440 帧只有 **60ms** 余量。
//     C3 是单核，Wi-Fi 收发 / Opus 解码 / AFE 任何一环卡住 60ms，
//     DMA 就喂空 → 播放断点 → 听感就是吞掉半个字。
//   改成 8×320=2560 帧 ≈ **107ms**，抗抖动能力接近翻倍。
//   代价：多占 (2560-1440)×4 ≈ 4.5KB 内部 RAM（可用堆约 73KB，够）；
//        播放延迟增加 ~47ms（对话场景完全无感）。
//   顺带好处：DMA 中断从每 10ms 一次变成每 13ms 一次，CPU 负载更低。
#define AUDIO_CODEC_DMA_DESC_NUM 8
#define AUDIO_CODEC_DMA_FRAME_NUM 320

// 麦克风侧（rx）单独用小缓冲：6×240（原值）
//   ⚠️ 加大缓冲只对**播放**有意义 —— 输出是 DMA 自己在喂，缓冲大才扛得住
//      CPU 卡顿；采集侧是我们主动读，缓冲大只是白占内存。
//   而 C3 内存很紧（实测空闲 36~50KB、最低 17.5KB），能省 4.5KB 就省。
//   仅 Simplex（tx/rx 两个独立通道）生效；Duplex 共用一个 config，不受影响。
#define AUDIO_CODEC_RX_DMA_DESC_NUM 6
#define AUDIO_CODEC_RX_DMA_FRAME_NUM 240

class AudioCodec {
public:
    AudioCodec();
    virtual ~AudioCodec();
    
    virtual void SetOutputVolume(int volume);
    virtual void SetInputGain(float gain);
    virtual void EnableInput(bool enable);
    virtual void EnableOutput(bool enable);

    virtual void OutputData(std::vector<int16_t>& data);
    virtual bool InputData(std::vector<int16_t>& data);
    virtual void Start();

    inline bool duplex() const { return duplex_; }
    inline bool input_reference() const { return input_reference_; }
    inline int input_sample_rate() const { return input_sample_rate_; }
    inline int output_sample_rate() const { return output_sample_rate_; }
    inline int input_channels() const { return input_channels_; }
    inline int output_channels() const { return output_channels_; }
    inline int output_volume() const { return output_volume_; }
    inline float input_gain() const { return input_gain_; }
    inline bool input_enabled() const { return input_enabled_; }
    inline bool output_enabled() const { return output_enabled_; }

protected:
    i2s_chan_handle_t tx_handle_ = nullptr;
    i2s_chan_handle_t rx_handle_ = nullptr;

    bool duplex_ = false;
    bool input_reference_ = false;
    bool input_enabled_ = false;
    bool output_enabled_ = false;
    int input_sample_rate_ = 0;
    int output_sample_rate_ = 0;
    int input_channels_ = 1;
    int output_channels_ = 1;
    int output_volume_ = 70;
    float input_gain_ = 0.0;

    virtual int Read(int16_t* dest, int samples) = 0;
    virtual int Write(const int16_t* data, int samples) = 0;
};

#endif // _AUDIO_CODEC_H
