#ifndef OLED_DISPLAY_H
#define OLED_DISPLAY_H

// C3 精简版 OledDisplay —— 不依赖 LVGL 子模块
// - 用 esp_lcd_panel_draw_bitmap 直接写 1KB 1bpp framebuffer
// - 待机显示 NTP 时钟 + IP（参考 yyxc.ino 的 drawClock 布局）
// - 唤醒后显示 RoboEyes 风格眼睛（6 种情绪，静态图）
// - 50ms 定时器推一帧

#include "display.h"

#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <atomic>
#include <cstdint>
#include <string>

// RoboEyes 前向声明（完整定义放在 oled_display.cc，头文件不必依赖那个 GPL 库）
class OledGfx;                          // RoboEyes 用的极简 Adafruit-GFX 兼容后端
template <typename T> class RoboEyes;   // FluxGarage RoboEyes 模板，前向声明即可持有指针

class OledDisplay : public Display {
private:
    friend class OledGfx;      // RoboEyes 的图形后端要能调这里的绘图函数

    esp_lcd_panel_io_handle_t panel_io_ = nullptr;
    esp_lcd_panel_handle_t panel_ = nullptr;

    // 1bpp framebuffer: 128 * 64 / 8 = 1024 bytes
    static constexpr int kWidth  = 128;
    static constexpr int kHeight = 64;
    static constexpr int kFbSize = kWidth * kHeight / 8;
    uint8_t fb_[kFbSize] = {0};
    // ── 脏矩形局部刷新（2026-10-20）──────────────────────────────────
    //   100kHz 下推整屏 1024B 要 92ms，这才是刷新率上不去的根因（400k/200k
    //   实测都会花屏，见 compact_wifi_board.cc 里 io_config.scl_speed_hz 的注释）。
    //   fb_prev_ 保存"已经推到屏上的内容"，每帧逐 page 比对，只推变化的那几
    //   个 page —— SSD1306 1bpp 按 8 行一个 page 组织，一个 page = kWidth 字节。
    //   表情动画（眼睛/眨眼）通常只变 1~2 个 page → 128~256B → 推送 12~23ms，
    //   于是刷新周期能从 250ms 压到 100ms，而总线占用反而从 37% 降到 ~23%。
    uint8_t fb_prev_[kFbSize] = {0};
    uint8_t push_buf_[kFbSize] = {0};
    uint32_t push_bytes_acc_ = 0;    // 统计用：累计推送字节数（算平均帧大小）
    uint32_t push_frames_acc_ = 0;
    // ⚠️ 首帧必须整屏推：fb_prev_ 初值全 0，但它**不代表屏幕上真实的内容** ——
    //   上电时 SSD1306 的 GDDRAM 是随机值。若按脏矩形比较，那些"fb_ 刚好也是
    //   空白"的 page 会被判成无变化而跳过 → 屏幕上残留随机像素 = 花条。
    //   实测症状：开机 WiFi 界面上下、待机时钟/唤醒界面底部出现花条。
    //   panel 被重建（自愈 AttachPanel）后同样要置 true —— 那时屏幕内容也未知。
    bool need_full_push_ = true;

    SemaphoreHandle_t lock_ = nullptr;
    esp_timer_handle_t refresh_timer_ = nullptr;
    // 刷屏任务：esp_timer 回调只负责 notify，真正的 I2C 传输（~23ms 阻塞）在
    //   低优先级任务里做，避免高优先级 timer 任务把音频/唤醒词按在地上摩擦。
    TaskHandle_t refresh_task_ = nullptr;
    static void RefreshTaskEntry(void* arg);

    // 显示状态
    std::atomic<bool> dirty_{true};           // framebuffer 有改动需要推
    std::atomic<bool> animated_mode_{false};   // true=表情模式, false=时钟模式
    std::atomic<int>  emotion_idx_{0};         // 当前表情 0-5
    std::atomic<uint32_t> last_change_ms_{0};  // 上次状态改变时间
    std::atomic<bool> clock_synced_{false};    // NTP 同步后置 true
    std::atomic<bool> wifi_connected_{false};
    std::atomic<bool> music_mode_{false};      // true=播放音乐（整屏 ♫）
    std::atomic<bool> speaking_{false};        // true=TTS 正在说话（嘴跟着轻微开合）
    bool mirror_x_ = false;                    // 屏幕方向（AttachPanel 补挂时要重设）
    bool mirror_y_ = false;
    std::atomic<int> push_fail_streak_{0};     // 连续推送失败次数（>20 判定屏掉了/花屏）
    char status_text_[32] = {0};              // 状态文字（表情下方一行）
    char song_name_[32] = {0};                // 歌名（音乐界面顶部显示）
    int  song_w_ = 0;                         // 歌名绘制宽度（SetSongName 时算好）

    // ── 两个"整屏信息页"（优先级高于音乐/表情/时钟） ──
    //   ① 配网热点：SSID + 配网页地址（进入 AP 模式时显示，连上网自动消失）
    //   ② 激活验证码：未绑定账号时服务端下发，用户要抄到 APP/控制台
    //   ⚠️ 这两页内容不变时不重绘（special_drawn_），否则每 150ms 推 1KB I2C 纯属浪费
    bool cfg_info_visible_ = false;
    bool act_code_visible_ = false;
    bool special_drawn_ = false;
    char cfg_ssid_[40] = {0};
    char cfg_url_[32] = {0};
    char act_code_[16] = {0};
    // ── 对话文字模式（网页「🖥 屏幕：表情 / 对话文字」，NVS 记忆） ──
    //   放到这里的理由：屏幕 Play Sequence 与"对话不停滚动"天然冲突，
    //   所以必须和音乐/表情/时钟一样进同一个优先级链（RenderTick）。
    bool chat_text_mode_ = false;     // true = 文字模式（有对话时滚动播放，播完让位给时钟/表情）
    // 文字模式占位界面当前画的状态（-1 = 还没画过 / 0 = 聆听中 / 1 = 说话中）。
    //   用它避免每 tick 重绘整屏 —— 一帧 I2C 要 23~36ms。
    int text_idle_state_ = -1;

    // 临时提示覆盖层（见 ShowNotice）：有内容且未过期时遮住整屏。
    //   覆盖层只画一次（notice_drawn_），避免 20Hz 反复推整屏挤占 I2C。
    char notice_text_[48] = {};
    int64_t notice_until_ms_ = 0;
    bool notice_drawn_ = false;
    bool chat_playing_ = false;       // 正在滚动播放某条消息
    // ── 最短显示时间 + 排队 ──
    //   服务端 STT（你说的话）和 LLM（助手回复）文本常常几十毫秒内先后到达，
    //   不设保底的话 user 刚上屏就被 assistant 覆盖，用户根本看不到自己说了什么。
    //   所以每条消息有 kChatMinShowMs 保底，期间新消息进队列（只留最新一条），
    //   到期自动顶上来 —— 仍然是"可打断"，只是不再一闪而过。
    static constexpr int64_t kChatMinShowMs = 700;
    bool    chat_pending_ = false;
    char    chat_pending_role_[16] = {0};
    char    chat_pending_text_[384] = {0};
    int64_t chat_lock_until_ms_ = 0;  // 当前这条的最早可被打断时刻
    void CheckPendingChat();          // 到点了把排队的消息顶上来（RenderTick 里每 tick 查）
    char chat_role_[16] = {0};
    char chat_text_[384] = {0};       // 当前播放的消息（超长截断，够放 4~5 屏）
    int  chat_line_off_[32] = {0};    // 每行首字节偏移
    int  chat_line_len_[32] = {0};    // 每行字节数
    int  chat_line_count_ = 0;
    int  chat_content_h_ = 0;         // 排版后的总高度（px）
    int  chat_scroll_px_ = 0;         // 当前滚动偏移
    int64_t chat_last_step_ms_ = 0;   // 上次按时间推进滚动的时刻
    int64_t chat_hold_until_ms_ = 0;  // 滚到底后的停留截止时刻
    static constexpr int kChatLineH = 16;   // 14px 字体 + 2px 行距 → 64px 屏刚好 4 行
    void LayoutChatLines();           // UTF-8 感知的自动换行（只在收到新消息时跑一次）
    void RenderChatPlaying();         // 画当前滚动窗口
    bool AdvanceChatScroll();         // 按时间推进滚动；返回 true = 需要重绘

    void RenderWifiConfigInfo();
    void RenderActivationCode();
    int  MeasureUtf8String(const char* s);    // 量 UTF-8 行宽（居中用，DrawUtf8String 只能边画边得宽）
    // 单个 UTF-8 字符的像素宽度（换行排版用）
    int  MeasureUtf8Char(const char* s, int* out_len);

    // ── RoboEyes：直接复用 yyxc 那套 FluxGarage RoboEyes ──
    //   眼睛的大小/形状/情绪/动画 100% 照搬，本项目只补一个极简的图形后端。
    //   RoboEyes 全程只用到 4 个方法：clearDisplay / display / fillRoundRect / fillTriangle
    OledGfx* gfx_ = nullptr;
    RoboEyes<OledGfx>* eyes_ = nullptr;
    void InitRoboEyes();                               // 构造时创建 RoboEyes 实例
    void ApplyRoboEmotion(int idx);                    // emotion idx → mood / position / 动画

    // ── RoboEyes 动画状态机 ──
    int   gaze_x_      = 0;        // 视线水平偏移 -3..+3
    int   gaze_y_      = 0;        // 视线垂直偏移 -2..+2
    int   blink_phase_ = 0;        // 0=睁, 1=上半闭, 2=全闭, 3=下半闭
    int   blink_cnt_   = 0;        // 当前 blink_phase 剩余帧数
    int   next_blink_ms_ = 3000;   // 下次眨眼时间 (相对启动)
    uint32_t boot_ms_  = 0;        // 启动时间基准 (esp_timer_get_time()/1000)
    int      last_clock_minute_ = -1;   // 上次渲染的分钟（0..1439，-1=未渲染）。分钟变化才重绘，~1次/分钟

    // 6 个表情（每只眼睛 24x32 竖向圆角矩形，bit 排列横向 MSB-first 1bpp）
    // 表情索引: 0=neutral 1=happy 2=sad 3=thinking 4=listening 5=speaking
    // 当前所有情绪共用同一张位图
    // 眼睛已改为程序化绘制（DrawEyeShape），不再需要 6 张各自为政的眼型位图

    // ---------- 基础画图 ----------
    void Clear();
    void SetPixel(int x, int y, bool on);
    void DrawRect(int x, int y, int w, int h, bool on);
    void DrawFillRect(int x, int y, int w, int h, bool on);
    void DrawLine(int x0, int y0, int x1, int y1, bool on);
    void DrawHLine(int x, int y, int w, bool on);  // 横向水平线（快）
    void DrawBitmap(int x, int y, int w, int h, const uint8_t* data);
    void DrawChar6x8(int x, int y, char c, bool on = true);
    int  DrawString6x8(int x, int y, const char* s, bool on = true);
    void DrawChar3x5(int x, int y, char c, bool on = true);
    int  DrawString3x5(int x, int y, const char* s, bool on = true);
    // 缩放字体：scale=2 → 10x14 字 / cell 12x16；用于大号时间
    void DrawCharScaled(int x, int y, char c, int scale, bool on = true);
    int  DrawStringScaled(int x, int y, const char* s, int scale, bool on = true);

    // ---------- 界面 ----------
    void RenderClock();      // 待机时钟: 日期/时间/IP/秒进度条
    void RenderFace();       // 表情: 两只眼睛
    void RenderStatusBar();  // 顶部一行: 日期 + 屏幕模式（文字/表情）
    void RenderTextIdle();   // 文字模式下没有对话内容时的占位（聆听中.../说话中...）

public:
    // 临时提示（覆盖整屏，duration_ms 后自动消失，任何显示模式下都能看到）。
    //   ⚠️ 不用基类的 ShowNotification：它只写 status_text_，而那个字段在
    //      表情模式下早已不渲染（中文用 6x8 字体会画成实心方块，2026-10-19 移除）。
    //   这里用 DrawUtf8String（font_puhui_14_1）才画得出中文。
    //   典型用途：开机早期按 BOOT 提示"网络连接中"，别让用户以为死机了。
    void ShowNotice(const char* text, int duration_ms = 3000);

private:
    // 下面这些仍是内部绘图函数，别因为上面的 public 段落漏出来。
    void DrawMouth(int cx, int y, int idx, uint32_t rel_ms, bool talking);
    int  DrawUtf8String(int x, int y, const char* s);         // 中文/ASCII 混合（14px 普惠字体）
    int  DrawUtf8Char(int x, int y, uint32_t uni);            // 单个字符（UTF-8 已解码）
    void DrawEllipseFill(int cx, int cy, int a, int b);       // 实心椭圆
    void DrawEyeShape(int cx, int cy, int w, int h);           // 眼睛：统一椭圆（形状固定）
    void DrawRoundRectFill(int x, int y, int w, int h, int r, bool on = true);  // 实心圆角矩形
    void DrawTriangleFill(int x0,int y0,int x1,int y1,int x2,int y2, bool on = true);  // 实心三角形（眼皮）
    void DrawMusicNote(int x, int y);                         // ♫ 双音符 48x40
    void PushFrame();        // framebuffer → SSD1306

    static void RefreshTimerCb(void* arg);

    // Display 基类
    virtual bool Lock(int timeout_ms = 0) override;
    virtual void Unlock() override;

public:
    OledDisplay(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_handle_t panel,
                int width, int height, bool mirror_x, bool mirror_y);
    ~OledDisplay();

    // 主动刷新（外部也可以调，比如 NTP 同步后）
    void ForceRefresh() { dirty_ = true; }

    // 通知（基类 Display 也有同名虚函数，这里加 override 并接收 synced 参数）
    void OnClockSynced(bool synced) override {
        clock_synced_ = synced;
        // 从未同步→已同步：强制立即渲染（不等分钟变化）
        if (synced) last_clock_minute_ = -1;
        dirty_ = true;
    }
    void OnWifiState(bool connected) override {
        wifi_connected_ = connected;
        // 从断网→联网：强制立即渲染（不等分钟变化）
        if (connected) last_clock_minute_ = -1;
        dirty_ = true;
    }

    // Display 基类接口
    virtual void SetStatus(const char* status) override;
    virtual void ShowNotification(const char* notification, int duration_ms = 3000) override;
    virtual void ShowNotification(const std::string& notification, int duration_ms = 3000) override;
    virtual void SetEmotion(const char* emotion) override;
    virtual void SetChatMessage(const char* role, const char* content) override;
    virtual void SetAnimatedEmotionMode(bool enable) override;
    virtual bool IsAnimatedEmotionMode() const override { return animated_mode_; }
    virtual void UpdateAnimatedEmotion() override;   // 实际是 timer 回调里推

    // 整屏信息页（配网引导 / 激活验证码）
    virtual void SetWifiConfigInfo(const char* ssid, const char* url) override;
    virtual void SetActivationCode(const char* code) override;

    // 屏幕显示模式：true = 对话文字（滚动播放）/ false = 表情·时钟
    virtual void SetChatTextMode(bool text_mode) override;
    bool IsChatTextMode() const { return chat_text_mode_; }
    // 告诉 Application：文字模式下别用 display_mode_ 把 user 消息拦掉
    bool WantsChatText() const override { return chat_text_mode_; }

    // 音乐模式：整屏居中大音符（服务端 AI 歌曲会带 "% play_music..." 标记）
    virtual void SetMusicMode(bool on) override;
    virtual void SetSongName(const char* name) override;  // 歌名（音乐界面顶部）
    virtual void SetSpeaking(bool on) override;           // 说话中 → 嘴跟着动
    void RenderMusic();                             // 整屏 ♫，替代表情/时钟
    void RenderTick();                              // 统一渲染入口（timer + 主循环）

    // 运行期补挂屏幕：开机时 0x3C 没 ACK → 先用 nullptr 占位，之后线/供电恢复
    //   （或用户重新插好）由板级 recovery 任务调这个函数接管，不需要重启。
    void AttachPanel(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_handle_t panel);
    // 换屏/重建前先摘掉：避免渲染线程继续往已释放的 panel 写
    void DetachPanel();
    // 面板是否需要用重建：panel 为空（没挂上）或连续推送失败过多（花屏/掉线）
    bool NeedsPanelRepair() const;
};

#endif // OLED_DISPLAY_H
