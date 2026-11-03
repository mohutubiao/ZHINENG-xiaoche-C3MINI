// C3 精简版 OledDisplay 实现 —— 不依赖 LVGL
// Framebuffer: 128*64 / 8 = 1024 字节
// 每个字节 8 个像素垂直方向（bit 0 = top, bit 7 = bottom）
// SSD1306 页面模式按页写，这里按列组织 1bpp 像素

#include "oled_display.h"

#include <esp_log.h>
#include <esp_random.h>
#include <esp_timer.h>
#include <esp_netif.h>
#include <ctime>
#include <cstring>
#include <cstdio>
#include <cmath>
#include <algorithm>

// 借用工程里现成的普惠中文字库（14px / 1bpp）来画中文歌名。
// OLED 自带的 6x8 字体只有 ASCII，画不了汉字；这个字库本来就是 lvgl 显示用的、
// 已经编进固件了，这里引用它只是让链接器别把它裁掉，不额外增加 flash 占用。
#include "lvgl.h"
#include "font/lv_font_fmt_txt.h"   // 直接读字库数据表（不用 lvgl 运行时）

#define TAG "OledDisplay"

// 6x8 ASCII 字体（0x20~0x7E 完整可打印字符）
// 每个字符 6 字节，bit 1=亮
// 简化版：内嵌 96 字符表（参考 Roboto 6x8 公共域字体）
// 实际使用时，文件太大；这里用一个 6x8 工具子集（数字/字母/标点）
// 为了减少体积，我们用 5x7 字体 + 留 1px 间距的 6x8 网格

namespace {

// 6x8 基础 ASCII 字体子集（数字 0-9 + 大写 A-Z + 部分符号 + 小写 a-z）
// 数据按行排列：8 行 × 1 字节 = 8 字节/字符；这里为了紧凑用 6 字节/字符
// 标准 6x8 字体（5x7 + 1px 间距）
// 仅含我们用得到的字符
constexpr uint8_t kFont5x7[][5] = {
    // '0' (0x30)
    {0x3E, 0x51, 0x49, 0x45, 0x3E},
    // '1' (0x31)
    {0x00, 0x42, 0x7F, 0x40, 0x00},
    // '2' (0x32)
    {0x42, 0x61, 0x51, 0x49, 0x46},
    // '3' (0x33)
    {0x21, 0x41, 0x45, 0x4B, 0x31},
    // '4' (0x34)
    {0x18, 0x14, 0x12, 0x7F, 0x10},
    // '5' (0x35)
    {0x27, 0x45, 0x45, 0x45, 0x39},
    // '6' (0x36)
    {0x3C, 0x4A, 0x49, 0x49, 0x30},
    // '7' (0x37)
    {0x01, 0x71, 0x09, 0x05, 0x03},
    // '8' (0x38)
    {0x36, 0x49, 0x49, 0x49, 0x36},
    // '9' (0x39)
    {0x06, 0x49, 0x49, 0x29, 0x1E},
    // ':' (0x3A)
    {0x00, 0x36, 0x36, 0x00, 0x00},
    // '-' (0x2D)
    {0x08, 0x08, 0x08, 0x08, 0x08},
    // '.' (0x2E)
    {0x00, 0x00, 0x60, 0x60, 0x00},
    // ' ' (0x20)
    {0x00, 0x00, 0x00, 0x00, 0x00},
    // 'A' (0x41)
    {0x7E, 0x11, 0x11, 0x11, 0x7E},
    // 'B' (0x42)
    {0x7F, 0x49, 0x49, 0x49, 0x36},
    // 'C' (0x43)
    {0x3E, 0x41, 0x41, 0x41, 0x22},
    // 'D' (0x44)
    {0x7F, 0x41, 0x41, 0x22, 0x1C},
    // 'E' (0x45)
    {0x7F, 0x49, 0x49, 0x49, 0x41},
    // 'F' (0x46)
    {0x7F, 0x09, 0x09, 0x09, 0x01},
    // 'G' (0x47)
    {0x3E, 0x41, 0x49, 0x49, 0x7A},
    // 'H' (0x48)
    {0x7F, 0x08, 0x08, 0x08, 0x7F},
    // 'I' (0x49)
    {0x00, 0x41, 0x7F, 0x41, 0x00},
    // 'J' (0x4A)
    {0x20, 0x40, 0x41, 0x3F, 0x01},
    // 'K' (0x4B)
    {0x7F, 0x08, 0x14, 0x22, 0x41},
    // 'L' (0x4C)
    {0x7F, 0x40, 0x40, 0x40, 0x40},
    // 'M' (0x4D)
    {0x7F, 0x02, 0x04, 0x02, 0x7F},
    // 'N' (0x4E)
    {0x7F, 0x04, 0x08, 0x10, 0x7F},
    // 'O' (0x4F)
    {0x3E, 0x41, 0x41, 0x41, 0x3E},
    // 'P' (0x50)
    {0x7F, 0x09, 0x09, 0x09, 0x06},
    // 'Q' (0x51)
    {0x3E, 0x41, 0x51, 0x21, 0x5E},
    // 'R' (0x52)
    {0x7F, 0x09, 0x19, 0x29, 0x46},
    // 'S' (0x53)
    {0x46, 0x49, 0x49, 0x49, 0x31},
    // 'T' (0x54)
    {0x01, 0x01, 0x7F, 0x01, 0x01},
    // 'U' (0x55)
    {0x3F, 0x40, 0x40, 0x40, 0x3F},
    // 'V' (0x56)
    {0x1F, 0x20, 0x40, 0x20, 0x1F},
    // 'W' (0x57)
    {0x3F, 0x40, 0x38, 0x40, 0x3F},
    // 'X' (0x58)
    {0x63, 0x14, 0x08, 0x14, 0x63},
    // 'Y' (0x59)
    {0x07, 0x08, 0x70, 0x08, 0x07},
    // 'Z' (0x5A)
    {0x61, 0x51, 0x49, 0x45, 0x43},
    // 'a' (0x61)
    {0x20, 0x54, 0x54, 0x54, 0x78},
    // 'b' (0x62)
    {0x7F, 0x48, 0x44, 0x44, 0x38},
    // 'c' (0x63)
    {0x38, 0x44, 0x44, 0x44, 0x20},
    // 'd' (0x64)
    {0x38, 0x44, 0x44, 0x48, 0x7F},
    // 'e' (0x65)
    {0x38, 0x54, 0x54, 0x54, 0x18},
    // 'f' (0x66)
    {0x08, 0x7E, 0x09, 0x01, 0x02},
    // 'g' (0x67)
    {0x0C, 0x52, 0x52, 0x52, 0x3E},
    // 'h' (0x68)
    {0x7F, 0x08, 0x04, 0x04, 0x78},
    // 'i' (0x69)
    {0x00, 0x44, 0x7D, 0x40, 0x00},
    // 'j' (0x6A)
    {0x20, 0x40, 0x44, 0x3D, 0x00},
    // 'k' (0x6B)
    {0x7F, 0x10, 0x28, 0x44, 0x00},
    // 'l' (0x6C)
    {0x00, 0x41, 0x7F, 0x40, 0x00},
    // 'm' (0x6D)
    {0x7C, 0x04, 0x18, 0x04, 0x78},
    // 'n' (0x6E)
    {0x7C, 0x08, 0x04, 0x04, 0x78},
    // 'o' (0x6F)
    {0x38, 0x44, 0x44, 0x44, 0x38},
    // 'p' (0x70)
    {0x7C, 0x14, 0x14, 0x14, 0x08},
    // 'q' (0x71)
    {0x08, 0x14, 0x14, 0x18, 0x7C},
    // 'r' (0x72)
    {0x7C, 0x08, 0x04, 0x04, 0x08},
    // 's' (0x73)
    {0x48, 0x54, 0x54, 0x54, 0x20},
    // 't' (0x74)
    {0x04, 0x3F, 0x44, 0x40, 0x20},
    // 'u' (0x75)
    {0x3C, 0x40, 0x40, 0x20, 0x7C},
    // 'v' (0x76)
    {0x1C, 0x20, 0x40, 0x20, 0x1C},
    // 'w' (0x77)
    {0x3C, 0x40, 0x30, 0x40, 0x3C},
    // 'x' (0x78)
    {0x44, 0x28, 0x10, 0x28, 0x44},
    // 'y' (0x79)
    {0x0C, 0x50, 0x50, 0x50, 0x3C},
    // 'z' (0x7A)
    {0x44, 0x64, 0x54, 0x4C, 0x44},
};

// 字符 → 字体索引
constexpr int kFontIdxSpace = 13;
constexpr int kFontIdxDash  = 11;
constexpr int kFontIdxDot   = 12;
constexpr int kFontIdxColon = 10;

int FontIdxForChar(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'Z') return 14 + (c - 'A');
    if (c >= 'a' && c <= 'z') return 40 + (c - 'a');
    if (c == ':') return kFontIdxColon;
    if (c == '-') return kFontIdxDash;
    if (c == '.') return kFontIdxDot;
    return kFontIdxSpace;
}

} // namespace

// ----------------------- 基础画图 -----------------------

void OledDisplay::Clear() {
    memset(fb_, 0, kFbSize);
    dirty_ = true;
}

void OledDisplay::SetPixel(int x, int y, bool on) {
    if (x < 0 || x >= kWidth || y < 0 || y >= kHeight) return;
    int byte = (y / 8) * kWidth + x;
    int bit  = y % 8;
    if (on) fb_[byte] |= (1 << bit);
    else    fb_[byte] &= ~(1 << bit);
}

void OledDisplay::DrawHLine(int x, int y, int w, bool on) {
    for (int i = 0; i < w; i++) SetPixel(x + i, y, on);
}

void OledDisplay::DrawRect(int x, int y, int w, int h, bool on) {
    DrawHLine(x, y, w, on);
    DrawHLine(x, y + h - 1, w, on);
    for (int i = 1; i < h - 1; i++) {
        SetPixel(x, y + i, on);
        SetPixel(x + w - 1, y + i, on);
    }
}

void OledDisplay::DrawFillRect(int x, int y, int w, int h, bool on) {
    for (int j = 0; j < h; j++)
        for (int i = 0; i < w; i++)
            SetPixel(x + i, y + j, on);
}

void OledDisplay::DrawLine(int x0, int y0, int x1, int y1, bool on) {
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    while (true) {
        SetPixel(x0, y0, on);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

void OledDisplay::DrawBitmap(int x, int y, int w, int h, const uint8_t* data) {
    // 数据格式：行优先，每行 (w/8) 字节，每像素 1 bit，**MSB-first**（bit7 = 每字节最左像素）
    // ⚠️ 关键 bug 修复：原代码用 (data[byte] >> bit) & 1（LSB-first）读，
    //    而 kEye* 位图是 MSB-first 编码的 —— 每位字节内左右翻转，圆形被打散成
    //    "鱼/蝴蝶结"碎片。用 Python 解码验证：MSB-first 下 kEyeNeutral 是标准圆形，
    //    LSB-first 下就是屏幕上看到的碎片图案。
    const int bytes_per_row = w / 8;
    for (int j = 0; j < h; j++) {
        for (int i = 0; i < w; i++) {
            int byte = j * bytes_per_row + (i / 8);
            int bit  = 7 - (i % 8);
            bool on = (data[byte] >> bit) & 1;
            SetPixel(x + i, y + j, on);
        }
    }
}

void OledDisplay::DrawChar6x8(int x, int y, char c, bool on) {
    int idx = FontIdxForChar(c);
    const uint8_t* glyph = kFont5x7[idx];
    // 5x7 字体画在 (x, y) 处，底部对齐 y+7
    for (int col = 0; col < 5; col++) {
        uint8_t bits = glyph[col];
        for (int row = 0; row < 7; row++) {
            if ((bits >> row) & 1) SetPixel(x + col, y + row, on);
        }
    }
}

int OledDisplay::DrawString6x8(int x, int y, const char* s, bool on) {
    int cx = x;
    while (*s) {
        DrawChar6x8(cx, y, *s, on);
        cx += 6;  // 5px 宽 + 1px 间距
        s++;
    }
    return cx - x;
}

// 缩放版 5x7 字体（每个像素 = scale x scale 块）
// scale=2 → 10x14 字，cell 12x16；scale=3 → 15x21 字，cell 18x24
// 用于待机时钟的大时间显示
void OledDisplay::DrawCharScaled(int x, int y, char c, int scale, bool on) {
    int idx = FontIdxForChar(c);
    const uint8_t* glyph = kFont5x7[idx];
    for (int col = 0; col < 5; col++) {
        uint8_t bits = glyph[col];
        for (int row = 0; row < 7; row++) {
            if ((bits >> row) & 1) {
                for (int dy = 0; dy < scale; dy++) {
                    for (int dx = 0; dx < scale; dx++) {
                        SetPixel(x + col * scale + dx, y + row * scale + dy, on);
                    }
                }
            }
        }
    }
}

int OledDisplay::DrawStringScaled(int x, int y, const char* s, int scale, bool on) {
    int cx = x;
    while (*s) {
        DrawCharScaled(cx, y, *s, scale, on);
        cx += 6 * scale;  // 5px 字 + 1px 间距，整体缩放
        s++;
    }
    return cx - x;
}

// 3x5 数字小字体（紧凑型，仅数字和冒号）
namespace {
const uint8_t kFont3x5_digits[10][3] = {
    {0x7, 0x5, 0x7},  // 0
    {0x2, 0x7, 0x2},  // 1
    {0x5, 0x7, 0x5},  // 2 (实际 0x5, 0x5, 0x5 = 全亮不好)
    {0x5, 0x7, 0x5},  // 3
    {0x7, 0x2, 0x7},  // 4
    {0x7, 0x5, 0x5},  // 5
    {0x7, 0x5, 0x5},  // 6
    {0x1, 0x1, 0x7},  // 7
    {0x7, 0x5, 0x7},  // 8
    {0x7, 0x5, 0x7},  // 9
};
const uint8_t kColon[3] = {0x0, 0x2, 0x0};
} // namespace

void OledDisplay::DrawChar3x5(int x, int y, char c, bool on) {
    const uint8_t* g = nullptr;
    if (c >= '0' && c <= '9') g = kFont3x5_digits[c - '0'];
    else if (c == ':')        g = kColon;
    if (!g) return;
    for (int col = 0; col < 3; col++) {
        uint8_t bits = g[col];
        for (int row = 0; row < 3; row++) {  // 5 行但只用前 3 行
            if ((bits >> row) & 1) SetPixel(x + col, y + row, on);
        }
    }
}

int OledDisplay::DrawString3x5(int x, int y, const char* s, bool on) {
    int cx = x;
    while (*s) { DrawChar3x5(cx, y, *s, on); cx += 4; s++; }
    return cx - x;
}

// ----------------------- 推送 framebuffer -----------------------

void OledDisplay::PushFrame() {
    if (!panel_) return;
    // SSD1306 1bpp 模式：esp_lcd_panel_draw_bitmap 每字节 8 个垂直像素
    // ⚠️ 2026-10-19 关键修复：原来不管返回值都 `dirty_ = false`。
    //   实测 I2C 偶发 NACK（"SSD1306_CMD_SET_COLUMN_RANGE failed"），
    //   失败那一帧被标成"已推送"，而时钟界面只在**分钟变化**时重绘 →
    //   屏幕就永远停在最后一张成功的画面上（用户看到"卡在 WiFi..."）。
    //   现在失败重试 3 次，仍失败就保留 dirty_，下个 tick 再试。
    // ── 脏矩形：只推变化的 page（SSD1306 1bpp，8 行 = 1 page = kWidth 字节）──
    constexpr int kPages = kHeight / 8;
    int pmin = -1, pmax = -1;
    if (need_full_push_) {
        // 整屏推，把屏幕上的随机残留一次性覆盖掉（见 .h 里 need_full_push_ 说明）
        pmin = 0;
        pmax = kPages - 1;
    } else {
        for (int p = 0; p < kPages; ++p) {
            if (memcmp(&fb_[p * kWidth], &fb_prev_[p * kWidth], kWidth) != 0) {
                if (pmin < 0) pmin = p;
                pmax = p;
            }
        }
        if (pmin < 0) {          // 画面没变化，一个字节都不用推
            dirty_ = false;
            return;
        }
    }

    const int y0 = pmin * 8;
    const int y1 = (pmax + 1) * 8;
    const int nbytes = (pmax - pmin + 1) * kWidth;
    memcpy(push_buf_, &fb_[pmin * kWidth], nbytes);

    for (int attempt = 0; attempt < 3; ++attempt) {
        esp_err_t err = esp_lcd_panel_draw_bitmap(panel_, 0, y0, kWidth, y1, push_buf_);
        if (err == ESP_OK) {
            // 只有确认推上去了，才把这一块记为"已同步"
            if (need_full_push_) {
                memcpy(fb_prev_, fb_, kFbSize);
                need_full_push_ = false;
            } else {
                memcpy(&fb_prev_[pmin * kWidth], &fb_[pmin * kWidth], nbytes);
            }
            push_bytes_acc_ += (uint32_t)nbytes;
            push_frames_acc_++;
            dirty_ = false;
            push_fail_streak_ = 0;
            return;
        }
        ESP_LOGW(TAG, "PushFrame failed (%s), retry %d/3", esp_err_to_name(err), attempt + 1);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    dirty_ = true;   // 没推上去，下个 tick 继续推
    push_fail_streak_++;   // 连续失败 → 板级 recovery 会重建 panel（花屏/掉线自愈）
}

void OledDisplay::DetachPanel() {
    if (!Lock(200)) return;
    panel_ = nullptr;
    panel_io_ = nullptr;
    Unlock();
}

bool OledDisplay::NeedsPanelRepair() const {
    return panel_ == nullptr || push_fail_streak_.load() > 20;
}

// 运行期补挂屏幕（开机 0x3C 没 ACK 时由板级 recovery 任务调用）
void OledDisplay::AttachPanel(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_handle_t panel) {
    if (!panel) return;
    if (!Lock(200)) return;
    panel_io_ = panel_io;
    panel_ = panel;
    esp_lcd_panel_mirror(panel_, mirror_x_, mirror_y_);
    esp_lcd_panel_disp_on_off(panel_, true);   // v5.5.x 的 ssd1306 init 不发 DISP_ON
    last_clock_minute_ = -1;                   // 强制重绘当前界面
    special_drawn_ = false;                    // 信息页（配网/验证码）也要重画
    // 面板是重建的 → 屏幕上的内容未知（可能是残留/随机像素），
    //   下一帧必须整屏推一次，否则脏矩形会把残留留下变成花条。
    need_full_push_ = true;
    push_fail_streak_ = 0;
    dirty_ = true;
    Unlock();
    ESP_LOGI(TAG, "panel attached at runtime, display recovered");
}

bool OledDisplay::Lock(int timeout_ms) {
    if (!lock_) return true;
    return xSemaphoreTakeRecursive(lock_, timeout_ms == 0 ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

void OledDisplay::Unlock() {
    if (lock_) xSemaphoreGiveRecursive(lock_);
}

// 统一的渲染入口：20Hz timer 和主循环(UpdateAnimatedEmotion)都必须走这里。
// ⚠️ 之前有两条渲染路径且逻辑不一致：主循环那条漏了 music_mode_ 分支，
//    直接用 animated_mode_ 判断 → 放音乐时主循环每 tick 把表情画回去，
//    和 timer 画的音符交替出现，看起来就是"隐约能看到表情"。
void OledDisplay::RenderTick() {
    // ── 临时提示覆盖层（ShowNotice）──────────────────────────────
    //   优先级：低于验证码/配网引导（那两个必须停屏等用户抄/连），
    //   高于音乐/文字/表情/时钟 —— 提示就是要让人一眼看见。
    //   只在进入时画一次（notice_drawn_），不随 20Hz tick 反复推整屏。
    if (!act_code_visible_ && !cfg_info_visible_ && notice_until_ms_ != 0) {
        const int64_t now_ms = esp_timer_get_time() / 1000;
        if (now_ms < notice_until_ms_) {
            if (!notice_drawn_) {
                notice_drawn_ = true;
                Clear();
                DrawHLine(0, 12, kWidth, true);
                int w = MeasureUtf8String(notice_text_);
                int x = (kWidth - w) / 2;
                if (x < 0) x = 0;
                DrawUtf8String(x, 26, notice_text_);
            }
            return;    // 提示期间不画其他内容
        }
        notice_until_ms_ = 0;
        notice_drawn_ = false;
        dirty_ = true; // 提示结束 → 下一帧恢复常规界面
    }

    // 整屏信息页优先级最高：配网引导必须能看见（否则"按住 BOOT 重新配网"后
    //   屏上只有 WiFi...，用户不知道连哪个热点）；验证码更是必须停在屏上等抄。
    if (act_code_visible_ || cfg_info_visible_) {
        if (!special_drawn_) {
            special_drawn_ = true;
            if (act_code_visible_) {
                RenderActivationCode();
            } else {
                RenderWifiConfigInfo();
            }
        }
    } else if (music_mode_) {
        // 🎵 音乐：整屏大音符，替代表情/时钟（音乐结束自动恢复）
        RenderMusic();
    } else if (chat_text_mode_) {
        // 💬 文字模式（双击 BOOT 或网页切换）：
        //   ⚠️ 整个模式**独占屏幕，任何情况下都不画表情**。
        //      原来的分支是 `chat_text_mode_ && chat_playing_` —— 没有对话在播
        //      时就掉到下面的 animated_mode_ 去画眼睛，于是"文字模式里还有表情"。
        //      现在没对话时画占位界面（聆听中.../说话中...）。
        CheckPendingChat();          // 排队的消息到点了就顶上来
        if (chat_playing_) {
            if (AdvanceChatScroll()) RenderChatPlaying();
        } else if (animated_mode_) {
            // 活跃（listening / speaking）但还没有文字内容 → 显示状态提示。
            //   animated_mode_ 就是"活跃"标志：Application::HandleStateChangedEvent
            //   里按 `is_active = (listening || speaking)` 置位。
            //   只在"聆听/说话"切换时重绘一次 —— 20Hz 推整屏会挤占音频和测距的 I2C。
            const int st = speaking_.load() ? 1 : 0;
            if (st != text_idle_state_) {
                text_idle_state_ = st;
                RenderTextIdle();
            }
        } else {
            // 待机（idle）：显示时钟，同样**不画表情**。
            text_idle_state_ = -1;
            time_t now = time(nullptr);
            struct tm tm;
            localtime_r(&now, &tm);
            int min = tm.tm_hour * 60 + tm.tm_min;
            if (min != last_clock_minute_) {
                last_clock_minute_ = min;
                RenderClock();
            }
        }
    } else if (animated_mode_) {
        // 表情模式：保持 20Hz（眨眼动画需要 4 帧@50ms = 200ms 过渡）
        RenderFace();
    } else {
        special_drawn_ = false;   // 离开信息页 → 下次进来必须重画
        // 时钟模式：分钟变化才重绘（~1次/分钟）
        // 大幅省掉 fb 重绘 + I2C 1024 字节传输，腾出 CPU 给音频/网络
        // NTP 还没同步时 time=0 → min=0，跟 last_clock_minute_=-1 比起来仍触发
        // 一次渲染（显示 NTP...），之后保持不刷直到 NTP 同步
        time_t now = time(nullptr);
        struct tm tm;
        localtime_r(&now, &tm);
        int min = tm.tm_hour * 60 + tm.tm_min;
        if (min != last_clock_minute_) {
            last_clock_minute_ = min;
            RenderClock();
        }
    }
    if (dirty_) PushFrame();
}

// 到点了就把排队的消息顶上来（每 tick 查一次，见 RenderTick 的 chat 分支）
void OledDisplay::CheckPendingChat() {
    if (!chat_pending_) return;
    const int64_t now = esp_timer_get_time() / 1000;
    if (now < chat_lock_until_ms_) return;

    chat_pending_ = false;
    strncpy(chat_role_, chat_pending_role_, sizeof(chat_role_) - 1);
    chat_role_[sizeof(chat_role_) - 1] = 0;
    strncpy(chat_text_, chat_pending_text_, sizeof(chat_text_) - 1);
    chat_text_[sizeof(chat_text_) - 1] = 0;
    LayoutChatLines();
    chat_scroll_px_ = 0;
    chat_last_step_ms_ = now;
    chat_hold_until_ms_ = 0;
    chat_lock_until_ms_ = now + kChatMinShowMs;
    chat_playing_ = true;
    dirty_ = true;
    ESP_LOGI(TAG, "chat show queued role=%s lines=%d", chat_role_, chat_line_count_);
}

// esp_timer 回调（优先级 22）只置通知，绝不做 I2C —— 见 RefreshTaskEntry 注释
void OledDisplay::RefreshTimerCb(void* arg) {
    auto* self = static_cast<OledDisplay*>(arg);
    if (self->refresh_task_) xTaskNotifyGive(self->refresh_task_);
}

// 刷屏任务（优先级 3，低于音频/唤醒词）：
//   RenderTick 里 dirty_ 时会 PushFrame → I2C 写 1024B ≈ 23ms 阻塞。
//   原来这段阻塞跑在 esp_timer 任务里（优先级 22），每帧把 audio_input(8) 和
//   唤醒词按停 23ms，占用率 23%~46%，体感就是"唤醒慢、语音断续"。
//   挪到低优先级任务后，音频随时可以抢占这 23ms，波形不会再被切碎。
void OledDisplay::RefreshTaskEntry(void* arg) {
    auto* self = static_cast<OledDisplay*>(arg);
    // 优先级写 1：音频链路 input(8)/output(7)/codec(6) 都要能立刻抢占刷屏。
    //   日志里原来是写死的 "prio 3"，跟实际值对不上，排查时会误导。
    ESP_LOGI(TAG, "refresh task started (150ms tick, prio 1, dirty-rect push)");
    int frames = 0;
    int64_t last_report = esp_timer_get_time() / 1000;
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (self->Lock(100)) {
            self->RenderTick();
            self->Unlock();
            frames++;
        }
        // 每 30s 报一次刷屏帧数：<10 说明任务被饿死/卡住（屏幕会停住不动）
        int64_t now = esp_timer_get_time() / 1000;
        if (now - last_report >= 30000) {
            // panel=0x0 表示屏没初始化成功（PushFrame 直接 return，画面永远不变）
            // avg bytes/frame 用来验证脏矩形有没有真的生效：
            //   ~1024 = 每帧都在推整屏（等于没优化，表情多半每帧全屏重绘）
            //   ~256  以下 = 局部刷新正常，刷新率可以放心往上提
            int avg = 0;
            if (self->push_frames_acc_ > 0) {
                avg = (int)(self->push_bytes_acc_ / self->push_frames_acc_);
            }
            ESP_LOGI(TAG, "refresh alive: %d frames / 30s, avg %d B/frame, panel=%p",
                     frames, avg, self->panel_);
            frames = 0;
            self->push_bytes_acc_ = 0;
            self->push_frames_acc_ = 0;
            last_report = now;
        }
    }
}

// ----------------------- 待机时钟 -----------------------

// ── 顶部状态栏：左边日期，右边屏幕模式 ──
//   ⚠️ 原来这里是 "2026-10-22 THU" 居中，星期没有任何实际用途；
//      腾出来的位置改成显示当前模式（文字/表情），配合双击 BOOT 切换，
//      一眼就能看出现在是哪种。
//   时钟界面和文字模式的占位界面共用这一行，保证视觉一致。
void OledDisplay::RenderStatusBar() {
    time_t now = time(nullptr);
    struct tm tm;
    localtime_r(&now, &tm);

    char buf[32];
    snprintf(buf, sizeof(buf), "%04d-%02d-%02d",
             tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday);
    DrawString6x8(2, 2, buf, true);

    // 右侧模式指示用中文（font_puhui_14_1 含汉字）。
    //   y 取 0 而不是 2：14px 字高比 6x8 的 ASCII 更占纵向空间，
    //   从 y=0 起画才能让字形墨迹落在分隔线(y=12)之上。
    const char* mode_txt = chat_text_mode_ ? "文字" : "表情";
    const int mode_w = MeasureUtf8String(mode_txt);
    DrawUtf8String(kWidth - 2 - mode_w, 0, mode_txt);
}

// ── 文字模式下的占位界面（当前没有对话内容在播）──
//   ⚠️ 这个函数存在的意义就是"**绝不画表情**"：
//      原来 RenderTick 的文字分支只在 chat_playing_ 为真时才占屏，
//      没有对话时就掉进 animated_mode_ 去画眼睛 —— 用户反馈
//      "文字模式下还是能看到表情"。现在文字模式独占整块屏。
void OledDisplay::RenderTextIdle() {
    Clear();
    RenderStatusBar();
    DrawHLine(0, 12, kWidth, true);

    // 中间一行状态提示
    const char* msg = speaking_.load() ? "说话中..." : "聆听中...";
    const int w = MeasureUtf8String(msg);
    int x = (kWidth - w) / 2;
    if (x < 0) x = 0;
    DrawUtf8String(x, 26, msg);
}

void OledDisplay::RenderClock() {
    Clear();

    // 开机/联网阶段：文字 2x 放大并居中（原来 6x8 小字挤在左上角，几乎看不清）
    if (!wifi_connected_) {
        const char* t = "WiFi...";
        int tw = (int)strlen(t) * 6 * 2;                  // 2x 缩放后的宽度
        DrawStringScaled((kWidth - tw) / 2, 24, t, 2, true);
        return;
    }

    if (!clock_synced_) {
        const char* t = "NTP...";
        int tw = (int)strlen(t) * 6 * 2;
        DrawStringScaled((kWidth - tw) / 2, 24, t, 2, true);
        return;
    }

    time_t now = time(nullptr);
    struct tm tm;
    localtime_r(&now, &tm);

    char buf[32];

    RenderStatusBar();

    // 顶部分隔线 — y=12
    DrawHLine(0, 12, kWidth, true);

    // 中部: 大号 HH:MM (2x 缩放 → 10x14 字 / cell 12x16) — y=22
    // 5 字符 × 12 cell = 60 px 宽，居中 x=34
    // ⚠️ 不显示秒，秒要去掉 — 只刷一次/分钟
    snprintf(buf, sizeof(buf), "%02d:%02d", tm.tm_hour, tm.tm_min);
    int time_w = 5 * 6 * 2;  // 5 字符 × 6 cell × 2 scale
    DrawStringScaled((kWidth - time_w) / 2, 22, buf, 2, true);

    // IP 地址 — y=46
    esp_netif_ip_info_t ip;
    if (esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"), &ip) == ESP_OK) {
        snprintf(buf, sizeof(buf), "IP " IPSTR, IP2STR(&ip.ip));
    } else {
        snprintf(buf, sizeof(buf), "IP ---");
    }
    int w = (int)strlen(buf) * 6;
    DrawString6x8((kWidth - w) / 2, 46, buf, true);
}

// ----------------------- 整屏信息页 -----------------------

// 配网引导：热点名 + 配网页地址（进入 AP 模式时由板级 SetWifiConfigInfo 显示）
void OledDisplay::RenderWifiConfigInfo() {
    Clear();

    const char* l1 = "请连接热点";
    DrawUtf8String((kWidth - MeasureUtf8String(l1)) / 2, 2, l1);

    int w = (int)strlen(cfg_ssid_) * 6;
    int x = (kWidth - w) / 2;
    DrawString6x8(x < 0 ? 0 : x, 20, cfg_ssid_, true);

    const char* l2 = "打开浏览器";
    DrawUtf8String((kWidth - MeasureUtf8String(l2)) / 2, 34, l2);

    w = (int)strlen(cfg_url_) * 6;
    x = (kWidth - w) / 2;
    DrawString6x8(x < 0 ? 0 : x, 52, cfg_url_, true);
}

// 激活验证码：6 位用 3x 放大（108px 宽），更长的自动降到 2x
void OledDisplay::RenderActivationCode() {
    Clear();

    const char* title = "验证码";
    DrawUtf8String((kWidth - MeasureUtf8String(title)) / 2, 2, title);

    int n = (int)strlen(act_code_);
    int scale = (n * 6 * 3 <= kWidth) ? 3 : 2;
    int w = n * 6 * scale;
    int x = (kWidth - w) / 2;
    DrawStringScaled(x < 0 ? 0 : x, 22, act_code_, scale, true);

    // ⚠️ 提示语用纯中文：14px 普惠字库的 ASCII 覆盖不可靠，混排可能出空白
    const char* hint = "输入到控制台";
    DrawUtf8String((kWidth - MeasureUtf8String(hint)) / 2, 46, hint);
}

// ----------------------- 表情 -----------------------

// ══════════════════════════════════════════════════════════════════════════════
//  RoboEyes 适配层 —— 眼睛直接复用 yyxc 那套 FluxGarage RoboEyes
//
//  那个库全程只用到 4 个绘图方法 + millis()/random() 两个 Arduino 运行时函数，
//  所以补齐这两样和一个极简后端，就能原封不动用它的眼睛：
//  大小(28x28) / 形状(圆角矩形) / 情绪(mood + position + 各种动画) 跟 yyxc 完全一致。
//
//  ⚠️ 这两个宏必须留到本文件末尾：RoboEyes 是模板，成员函数在被调用处才实例化，
//     提前 #undef 会编译失败。esp_random() 不受影响（标识符不同）。
//  ⚠️ random(n) 里必须判 0：库里会对可能为 0 的变化量取模，%0 在 ESP32 上直接异常
//
//  头文件用项目自带的 vendored 版 third_party/roboeyes/ —— display/roboeyes_adapter.cc
//  （给 LVGL 用的）用的就是这一份，还比 Arduino 库那版新（多了 LOVING/SURPRISED/SLEEPY）。
//  ⚠️ FluxGarage RoboEyes 是 GPL-3.0。
//  ⚠️ 它的成员初始化顺序有问题（spaceBetweenDefault 声明在使用它的 eyeLxDefault 之后）
//     → 构造时是 UB，所以 main/CMakeLists.txt 里对 oled_display.cc 也加了
//     -Wno-maybe-uninitialized -Wno-reorder，跟官方对 adapter 的处理一致。
//     UB 造成的"默认坐标算错"由 InitRoboEyes() 手动重设坐标覆盖掉。
// ══════════════════════════════════════════════════════════════════════════════
typedef uint8_t byte;
#define millis()   ((unsigned long)(esp_timer_get_time() / 1000ULL))
#define random(n)  ((unsigned long)((n) ? (esp_random() % (unsigned long)(n)) : 0UL))
#include "third_party/roboeyes/src/FluxGarage_RoboEyes.h"

// RoboEyes 的图形后端。清屏/推屏由 RenderTick 统一管（本项目只有一条渲染入口），
// 所以 clearDisplay / display 故意留空
class OledGfx {
public:
    OledGfx(OledDisplay* d) : disp_(d) {}
    void clearDisplay() {}
    void display()      {}
    void fillRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color) {
        disp_->DrawRoundRectFill(x, y, w, h, r, color != 0);
    }
    void fillTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color) {
        disp_->DrawTriangleFill(x0, y0, x1, y1, x2, y2, color != 0);
    }
private:
    OledDisplay* disp_;
};

void OledDisplay::InitRoboEyes() {
    delete eyes_;  eyes_ = nullptr;
    delete gfx_;   gfx_  = nullptr;
    gfx_  = new OledGfx(this);
    eyes_ = new RoboEyes<OledGfx>(*gfx_);
    // 128x40 + 50fps。⚠️ 高度故意传 40 而不是 64：
    //   RoboEyes 的 screenHeight 只用来算 idle 随机张望的范围
    //   （getScreenConstraint_Y = screenHeight - 眼睛高），传 64 会让眼睛跑到
    //   y=28 直接压到嘴上；传 40 就把它的活动范围锁死在上半屏。
    eyes_->begin(128, 40, 50);
    eyes_->setWidth(28, 28);
    // ⚠️ begin() 和 setWidth() 都**不重算**默认坐标：那些默认值在构造函数里
    //    已经按初始的 36x36 算死了。不手动摆位置会出两个问题：
    //      ① 变 28 宽后整体偏左 8px   ② 垂直按 64 行居中(y=14)，会盖住嘴
    //    所以这里显式摆正：宽 28 + 间距 10 → 居中起点 x=31；顶部 y=4
    eyes_->eyeLxDefault = eyes_->eyeLx = eyes_->eyeLxNext = 31;
    eyes_->eyeLyDefault = eyes_->eyeLy = eyes_->eyeLyNext = 4;
    eyes_->eyeRxDefault = eyes_->eyeRx = eyes_->eyeRxNext = 69;   // 31 + 28 + 10
    eyes_->eyeRyDefault = eyes_->eyeRy = eyes_->eyeRyNext = 4;
    eyes_->setAutoblinker(ON);
    eyes_->setMood(DEFAULT);
    ApplyRoboEmotion(emotion_idx_.load());
}

// emotion idx → RoboEyes 的 mood / 视线 / 动画
// ⚠️ 每个分支都全量重设所有开关（照 yyxc 的 drawFace 写法）：
//    否则上一个情绪的 flicker / curiosity / sweat 会残留到下一个情绪上
void OledDisplay::ApplyRoboEmotion(int idx) {
    if (!eyes_) return;
    RoboEyes<OledGfx>* e = eyes_;
    e->setWidth(28, 28);
    e->setSweat(false);
    e->setCuriosity(false);
    e->setHFlicker(false, 0);
    e->setVFlicker(false, 0);
    // ⚠️ idle 默认一律关闭：idle 会每隔几秒把眼睛目标位置设成
    //    random(getScreenConstraint_X()) = random(0..62)，也就是眼睛自己跑到屏幕
    //    左边或右边"乱瞟"——用户反馈"默认眼睛向右看"就是它干的。
    //    用户要的默认状态是「在中间、看着你」，所以这里统一关掉。
    //    以后想要"发呆乱看"的效果，再单独对某个情绪 setIdleMode(true, ...)。
    e->setIdleMode(false);
    switch (idx) {
        case 1:  // happy：笑眼 + 笑动画
            e->setMood(HAPPY);    e->setPosition(DEFAULT); e->setIdleMode(false);
            e->anim_laugh();      break;
        case 2:  // sad：眼皮下垂 + 往下看
            e->setMood(TIRED);    e->setPosition(S);       e->setIdleMode(false); break;
        case 3:  // thinking：往上看 + 好奇（外侧那只眼睛放大）
            e->setMood(DEFAULT);  e->setPosition(N);       e->setCuriosity(true); e->setIdleMode(false); break;
        case 4:  // listening：正视中间 + 外侧眼睛微微放大（专注听你说话，而不是看别处）
            e->setMood(DEFAULT);  e->setPosition(DEFAULT); e->setCuriosity(true); e->setIdleMode(false); break;
        case 5:  // speaking：轻微垂直抖动（像在讲话）
            e->setMood(DEFAULT);  e->setPosition(DEFAULT); e->setVFlicker(true, 3); e->setIdleMode(false); break;
        case 6:  // surprised：眼睛放大到 34 + 好奇
            e->setMood(DEFAULT);  e->setPosition(DEFAULT); e->setCuriosity(true);
            e->setWidth(34, 34);
            e->eyeLxNext = 25;    // 变宽后要重算居中：setPosition() 是按"当前"宽度算的，
                                  // 此时宽度还是 28 → 不加这行会偏右 6px
            break;
        case 7:  // angry：怒眉 + 横向抖动
            e->setMood(ANGRY);    e->setPosition(DEFAULT); e->setHFlicker(true, 2); e->setIdleMode(false); break;
        default: // neutral：默认圆眼，正视中间（不开 idle，见上方说明）
            e->setMood(DEFAULT);  e->setPosition(DEFAULT); e->setIdleMode(false); break;
    }
}

void OledDisplay::RenderFace() {
    Clear();

    int idx = emotion_idx_.load();
    if (idx < 0 || idx >= 8) idx = 0;      // 情绪从 6 种扩展到 8 种（新增 惊讶 / 生气）

    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    if (boot_ms_ == 0) boot_ms_ = now_ms;
    uint32_t rel = now_ms - boot_ms_;

    // ── 眼睛：整套复用 yyxc 的 FluxGarage RoboEyes ──
    //   眨眼 / idle 随机张望 / 情绪动画全部在库里自己跑，这里什么都不用管，
    //   大小形状情绪表达和 yyxc 完全一致（begin/setWidth/setMood... 照搬）
    if (!eyes_) InitRoboEyes();
    if (eyes_) eyes_->update();

    // ── 嘴：还是我们自己的实心小方块 ──
    //   RoboEyes 本身不带嘴（只有眼睛+眼皮+汗珠），所以嘴这块自己画，
    //   照用户给的参考图来的形状 + 说话时轻微开合
    DrawMouth(kWidth / 2, 47, idx, rel, speaking_.load());

    // ⚠️ 2026-10-19：这里原来会在 y=56 画一行 status_text_（6x8 ASCII 字体）。
    //   status_text_ 里是中文（"正在聆听"之类），6x8 字体没有中文字形，
    //   每个 UTF-8 字节都会被当成一个字符画成 6x8 的**实心方块** ——
    //   结果表情下面多出一排大色块，看起来就是"眼睛/嘴错位"。已删除：
    //   表情模式只保留 眼睛 + 嘴（状态信息在网页和日志里都有）。
    (void)status_text_;
}

// 嘴：按情绪索引画不同形状，让表情真正有区别
//   idx: 0 neutral 1 happy 2 sad 3 thinking 4 listening 5 speaking
// 嘴：所有情绪共用同一个「实心圆角矩形」—— 照用户给的参考图定形状。
//   ⚠️ 之前的 U 形笑弧 / ∩ 垂嘴 / 波浪线 / 圆圈 / 椭圆全部删除：
//      用户明确要求「所有的嘴都用这个，其他的删除，情绪用眼睛表达」。
//   只有说话时做轻微开合（否则说话时整张脸是静止的），形状本身不变。
void OledDisplay::DrawMouth(int cx, int y, int idx, uint32_t rel_ms, bool talking) {
    (void)idx;                        // 情绪不再影响嘴形
    int mw = 11;                      // 嘴宽：接近正方形的比例（参考图）
    int mh = 9;                       // 嘴高
    if (talking) {
        // 说话：小幅开合（周期约 330ms）——张开时变高变窄，像真的在张嘴
        //   ⚠️ 高度上限 11：嘴顶不要碰到眼睛底(40) —— y=47 时最大只到 42..52
        static const int kH[3] = {6, 9, 11};
        static const int kW[3] = {13, 11, 10};
        int t = (int)((rel_ms / 110) % 3);
        mh = kH[t];
        mw = kW[t];
    }
    DrawRoundRectFill(cx - mw / 2, y - mh / 2, mw, mh, 3);
}

// 实心圆角矩形（眼睛/嘴都用；角半径 r，自动收敛到合法范围）
void OledDisplay::DrawRoundRectFill(int x, int y, int w, int h, int r, bool on) {
    if (w <= 0 || h <= 0) return;
    if (r < 1) { DrawFillRect(x, y, w, h, on); return; }
    if (r > w / 2) r = w / 2;
    if (r > h / 2) r = h / 2;
    DrawFillRect(x, y + r, w, h - 2 * r, on);             // 中间主体
    for (int j = 0; j < r; j++) {                          // 上下各 r 行圆角
        int dy = r - 1 - j;
        int dx = (int)(r - sqrtf((float)(r * r - dy * dy)) + 0.5f);
        if (dx < 0) dx = 0;
        if (2 * dx >= w) continue;
        DrawHLine(x + dx, y + j, w - 2 * dx, on);
        DrawHLine(x + dx, y + h - 1 - j, w - 2 * dx, on);
    }
}

// 实心三角形（扫描线填充）。RoboEyes 用它画上下眼睑——
// ⚠️ 传 BGCOLOR(0) 时是"把眼睛切掉一块"，所以必须支持 on=false，不能只会画白
void OledDisplay::DrawTriangleFill(int x0, int y0, int x1, int y1, int x2, int y2, bool on) {
    const int vx[3] = {x0, x1, x2};
    const int vy[3] = {y0, y1, y2};
    int minY = vy[0], maxY = vy[0];
    for (int k = 1; k < 3; k++) {
        if (vy[k] < minY) minY = vy[k];
        if (vy[k] > maxY) maxY = vy[k];
    }
    for (int y = minY; y <= maxY; y++) {
        int lo = 100000, hi = -100000;
        for (int k = 0; k < 3; k++) {
            int ay = vy[k], by = vy[(k + 1) % 3];
            int ax = vx[k], bx = vx[(k + 1) % 3];
            if (ay == by) continue;                        // 水平边：与该行无交点
            if (y < (ay < by ? ay : by) || y > (ay > by ? ay : by)) continue;
            int xi = ax + (y - ay) * (bx - ax) / (by - ay);
            if (xi < lo) lo = xi;
            if (xi > hi) hi = xi;
        }
        if (lo <= hi) DrawHLine(lo, y, hi - lo + 1, on);
    }
}

// 实心椭圆（逐行扫描填充）
void OledDisplay::DrawEllipseFill(int cx, int cy, int a, int b) {
    if (a <= 0 || b <= 0) return;
    for (int dy = -b; dy <= b; dy++) {
        float t = 1.0f - (float)(dy * dy) / (float)(b * b);
        if (t < 0) t = 0;
        int w = (int)(a * sqrtf(t));
        DrawHLine(cx - w, cy + dy, 2 * w + 1, true);
    }
}

// 眼睛：统一形状（饱满的圆角方块——指数 0.42 让它介于椭圆和矩形之间，更有神）
//   ⚠️ 故意没有 bend 弯曲参数：弯成线条会让眼睛形状跳变，看着不像同一张脸。
//   情绪和眨眼全部靠"这颗方块缩放到多高"来表达。
void OledDisplay::DrawEyeShape(int cx, int cy, int w, int h) {
    if (w < 2 || h < 1) return;
    const int half = w / 2;
    const float hh = h / 2.0f;
    for (int dx = -half; dx <= half; dx++) {
        float t = (float)dx / half;           // -1 .. 1
        float k = 1.0f - t * t;               // 中间 1，两端 0
        if (k < 0) k = 0;
        int col_h = (int)(hh * 2.0f * powf(k, 0.42f));
        if (col_h < 1) col_h = 1;
        DrawFillRect(cx + dx, cy - col_h / 2, 1, col_h, true);
    }
}

// ⚠️ 必须用 font_puhui_14_1（6650 字形，含汉字）！
//    font_puhui_basic_14_1 是"基础拉丁"字库，**一个汉字都没有**（只有 ASCII/希腊/西里尔）
//    → 中文歌名查不到字形、宽度算出来是 0，屏幕上什么都画不出来。
//    这才是"音乐播放没有中文名称"的真正原因（不是抓取逻辑的问题）。
//    注意 1688KB 是它的 C 源码文本体积，编译进固件的二进制数据只有 ~220KB，装得下。
extern const lv_font_t font_puhui_14_1;

// ⚠️ 绝不能调 lvgl 的 lv_font_get_glyph_bitmap()：它在 lv_font.c:55 会把
//    req_raw_bitmap 强制清 0 走 A8 转换，需要一个 draw_buf，而 C3 上 lvgl
//    根本没初始化（本文件是非 LVGL 的纯 framebuffer 实现）→ 传 NULL 直接崩。
//    所以这里只读字库那几张 const 表，零运行时依赖。
// unicode → glyph id（照 lvgl 的 get_glyph_dsc_id，覆盖字库实际用到的格式）
static uint32_t LookupGlyphId(const lv_font_fmt_txt_dsc_t* fd, uint32_t uni) {
    for (uint16_t i = 0; i < fd->cmap_num; i++) {
        const lv_font_fmt_txt_cmap_t* cm = &fd->cmaps[i];
        if (uni < cm->range_start) continue;
        uint32_t rcp = uni - cm->range_start;                  // 相对码点
        if (rcp >= cm->range_length) continue;

        if (cm->type == LV_FONT_FMT_TXT_CMAP_FORMAT0_TINY) {
            return cm->glyph_id_start + rcp;
        }
        if (cm->type == LV_FONT_FMT_TXT_CMAP_FORMAT0_FULL) {
            // ⚠️ FORMAT0 系列的 glyph_id_ofs_list 是 uint8_t*（SPARSE 系列才是 uint16_t*），
            //    弄错会把字形查串（见 lv_font_fmt_txt.h:99-102 的规范）
            const uint8_t* ol = (const uint8_t*)cm->glyph_id_ofs_list;
            // 偏移为 0 且不是区间首字符 → 这个字在字库里是缺的（lvgl 的判定方式）
            if (ol[rcp] == 0 && uni != cm->range_start) continue;
            return cm->glyph_id_start + ol[rcp];
        }
        if (cm->type == LV_FONT_FMT_TXT_CMAP_SPARSE_TINY ||
            cm->type == LV_FONT_FMT_TXT_CMAP_SPARSE_FULL) {
            // ⚠️ SPARSE 必须二分：中文那张 cmap 有 8000+ 项，
            //    线性扫一遍每次要几千次比较，20Hz 重绘歌名会明显吃 CPU
            //    （lvgl 自己也是用 bsearch）
            const uint16_t* ul = cm->unicode_list;
            int lo = 0, hi = (int)cm->list_length - 1, idx = -1;
            while (lo <= hi) {
                int mid = (lo + hi) >> 1;
                uint16_t v = ul[mid];
                if (v == rcp) { idx = mid; break; }
                if (v < rcp) lo = mid + 1; else hi = mid - 1;
            }
            if (idx < 0) continue;
            if (cm->type == LV_FONT_FMT_TXT_CMAP_SPARSE_TINY) {
                return cm->glyph_id_start + idx;
            }
            const uint16_t* ol = (const uint16_t*)cm->glyph_id_ofs_list;
            return cm->glyph_id_start + ol[idx];
        }
    }
    return 0;      // 0 = 没这个字（字库保留项，无墨迹）
}

// 字形的纵向相对偏移。只取相对量，就不用纠结 lvgl 各版本 ofs_y 的语义差异
static inline int GlyphTop(const lv_font_fmt_txt_glyph_dsc_t* gd) {
    return -(int)gd->box_h - (int)gd->ofs_y;
}

// 画一个字符：top = 字形顶部（由 DrawUtf8String 对齐后传进来）
int OledDisplay::DrawUtf8Char(int x, int top, uint32_t uni) {
    const lv_font_t* f = &font_puhui_14_1;
    const lv_font_fmt_txt_dsc_t* fd = (const lv_font_fmt_txt_dsc_t*)f->dsc;
    uint32_t gid = LookupGlyphId(fd, uni);
    if (!gid) return 0;
    const lv_font_fmt_txt_glyph_dsc_t* gd = &fd->glyph_dsc[gid];
    if (gd->box_w == 0 || gd->box_h == 0) return gd->adv_w >> 4;   // 空格等
    if (gd->box_w > 32 || gd->box_h > 32) return gd->adv_w >> 4;   // 异常数据兜底，防读越界

    const uint8_t* bmp = &fd->glyph_bitmap[gd->bitmap_index];
    // ⚠️ lv_font_conv 生成的 1bpp 位图是**连续位流**：像素 (i,j) 在 bit = j*box_w + i，
    //    行与行之间还会共用同一个字节（**不是**每行按字节对齐！）。
    //    证据：'!' 是 2x10 = 20 bit = 3 字节，而不是按行对齐的 10 字节。
    //    按"每行字节跨度"去解会得到一片随机点（字形彻底不可辨）。
    for (int j = 0; j < gd->box_h; j++) {
        for (int i = 0; i < gd->box_w; i++) {
            uint32_t bit = (uint32_t)j * gd->box_w + i;
            if ((bmp[bit >> 3] >> (7 - (bit & 7))) & 1) {
                SetPixel(x + gd->ofs_x + i, top + j, true);
            }
        }
    }
    return gd->adv_w >> 4;                            // adv_w 是 28.4 定点
}

// 量一行 UTF-8（中文/ASCII 混合）的像素宽度。
//   DrawUtf8String 是"边画边得到宽度"，要先把整行居中就必须能单独量。
int OledDisplay::MeasureUtf8String(const char* s) {
    if (!s) return 0;
    const lv_font_t* f = &font_puhui_14_1;
    const lv_font_fmt_txt_dsc_t* fd = (const lv_font_fmt_txt_dsc_t*)f->dsc;
    if (fd->bpp != 1) return (int)strlen(s) * 7;

    int w = 0;
    const uint8_t* p = (const uint8_t*)s;
    while (*p) {
        uint32_t uni = 0;
        int len = 1;
        if      ((*p & 0x80) == 0)    { uni = *p; len = 1; }
        else if ((*p & 0xE0) == 0xC0) { uni = ((uint32_t)(p[0] & 0x1F) <<  6) | (p[1] & 0x3F); len = 2; }
        else if ((*p & 0xF0) == 0xE0) { uni = ((uint32_t)(p[0] & 0x0F) << 12) | ((uint32_t)(p[1] & 0x3F) << 6) | (p[2] & 0x3F); len = 3; }
        else if ((*p & 0xF8) == 0xF0) { uni = ((uint32_t)(p[0] & 0x07) << 18) | ((uint32_t)(p[1] & 0x3F) << 12) | ((uint32_t)(p[2] & 0x3F) << 6) | (p[3] & 0x3F); len = 4; }
        else { p++; continue; }

        uint32_t gid = LookupGlyphId(fd, uni);
        if (gid) {
            const lv_font_fmt_txt_glyph_dsc_t* gd = &fd->glyph_dsc[gid];
            w += gd->adv_w >> 4;              // adv_w 是 28.4 定点
        } else {
            w += 7;                            // 缺字：按半角估，别把行挤歪
        }
        p += len;
    }
    return w;
}

// UTF-8 字符串（中文/ASCII 混合），返回实际占用宽度
int OledDisplay::DrawUtf8String(int x, int y, const char* s) {
    if (!s) return 0;
    const lv_font_t* f = &font_puhui_14_1;
    const lv_font_fmt_txt_dsc_t* fd = (const lv_font_fmt_txt_dsc_t*)f->dsc;
    if (fd->bpp != 1) return 0;            // 只处理 1bpp（本字库就是 1bpp）

    // 1) 解码 UTF-8（一次最多 32 字，够一行 128px 了）
    uint32_t unis[32];
    int n = 0;
    const uint8_t* p = (const uint8_t*)s;
    while (*p && n < 32) {
        uint32_t uni = 0;
        int len = 1;
        if      ((*p & 0x80) == 0) { uni = *p; len = 1; }
        else if ((*p & 0xE0) == 0xC0) { uni = ((uint32_t)(p[0] & 0x1F) <<  6) | (p[1] & 0x3F); len = 2; }
        else if ((*p & 0xF0) == 0xE0) { uni = ((uint32_t)(p[0] & 0x0F) << 12) | ((uint32_t)(p[1] & 0x3F) << 6) | (p[2] & 0x3F); len = 3; }
        else if ((*p & 0xF8) == 0xF0) { uni = ((uint32_t)(p[0] & 0x07) << 18) | ((uint32_t)(p[1] & 0x3F) << 12) | ((uint32_t)(p[2] & 0x3F) << 6) | (p[3] & 0x3F); len = 4; }
        else { p++; continue; }                       // 非法字节，跳过
        unis[n++] = uni;
        p += len;
    }

    // 2) 取所有字顶部偏移的最小值，画的时候减掉 → 整行紧贴 y（不受 ofs_y 语义影响）
    int min_top = 0;
    bool have = false;
    for (int k = 0; k < n; k++) {
        uint32_t gid = LookupGlyphId(fd, unis[k]);
        if (!gid) continue;
        const lv_font_fmt_txt_glyph_dsc_t* gd = &fd->glyph_dsc[gid];
        if (gd->box_h == 0) continue;
        int t = GlyphTop(gd);
        if (!have || t < min_top) { min_top = t; have = true; }
    }

    // 3) 逐个画
    int pen = x;
    for (int k = 0; k < n; k++) {
        uint32_t gid = LookupGlyphId(fd, unis[k]);
        if (!gid) { pen += 7; continue; }             // 缺字：补个空格宽度
        const lv_font_fmt_txt_glyph_dsc_t* gd = &fd->glyph_dsc[gid];
        int top = (gd->box_h == 0) ? 0 : (GlyphTop(gd) - min_top);
        pen += DrawUtf8Char(pen, y + top, unis[k]);
    }
    return pen - x;
}

// 🎵 ♫ 双音符（48x40）：两个符头 + 两根符干 + 顶部横梁
//   之前画单个 ♪：符头椭圆孤零零在底部，很容易被看成「一张嘴」。
//   改成 ♫ 后两个符头 + 横梁的辨识度高得多，一眼就是音乐符号。
void OledDisplay::DrawMusicNote(int x, int y) {
    DrawEllipseFill(x + 13, y + 31, 9, 6);      // 左符头
    DrawEllipseFill(x + 35, y + 31, 9, 6);      // 右符头
    DrawFillRect(x + 20, y + 12, 5, 20, true);  // 左符干
    DrawFillRect(x + 42, y + 12, 5, 20, true);  // 右符干
    DrawFillRect(x + 20, y + 12, 27, 6, true);  // 顶部横梁（连两根符干）
}

// ----------------------- 接口 -----------------------

OledDisplay::OledDisplay(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_handle_t panel,
                         int width, int height, bool mirror_x, bool mirror_y) {
    panel_io_ = panel_io;
    panel_ = panel;
    width_ = width;
    height_ = height;
    mirror_x_ = mirror_x;
    mirror_y_ = mirror_y;

    lock_ = xSemaphoreCreateRecursiveMutex();

    // 启动 10Hz 刷新定时器（100ms）
    //   ⚠️ 原为 20Hz / 50ms，改慢的原因（2026-10-17）：
    //      RefreshTimerCb 里会 RenderTick() → dirty_ 时 PushFrame()，
    //      而 PushFrame 走 I2C 400kHz 写 1024 字节 framebuffer，
    //      耗时 1024×9bit / 400000 ≈ 23ms，且是阻塞传输。
    //      这个回调跑在 esp_timer 任务里（优先级 22），远高于
    //      audio_input 任务（优先级 8）—— 于是每 50ms 就有约 23ms
    //      让唤醒词检测和音频采集完全停摆，I2C 占用率高达 46%。
    //      体感就是"唤醒反应慢"（唤醒瞬间若正在刷表情动画尤其明显）。
    //      改成 100ms 后占用率降到约 23%，唤醒最多快 23ms；
    //      表情动画仍有 10fps，肉眼可接受。
    //   ⚠️ 遗留：ESP-IDF 不建议在 esp_timer 回调里做长时间阻塞操作。
    //      这里只是降频缓解；彻底解法是把 I2C 传输挪到独立的低优先级任务，
    //      由 timer 只置标志位，任务侧再真正刷屏。
    // 真正的刷屏在低优先级任务里做（见 RefreshTaskEntry）。
    //   周期 150ms（约 6.7fps）：眨眼/表情肉眼依然连贯，I2C 占用从 23% 降到约 15%。
    //   如果想更省，可以继续加到 200ms；代价是眨眼起始会有一点顿。
    // ⚠️⚠️ 栈大小单位踩坑史（2026-10-19）：ESP-IDF 的 `xTaskCreateStatic` 的
    //   ulStackDepth 是**字节数**（不是 FreeRTOS 原版的那个"字数"）。
    //   我按字数传了 896 → 实际只有 896 字节栈 → RenderFace（字体解码 + RoboEyes
    //   + snprintf）直接栈溢出：开机停在 "WiFi..." 不再刷新，一唤醒进表情模式
    //   就重启。现象是"屏幕卡住 + 唤醒像重启"，非常像业务 bug，其实是栈。
    //   现在改回动态 xTaskCreate（单位明确是字节），给 3584 字节，和 IDF
    //   定时器任务默认栈一样；堆这边已经把 tools/list 的多余拷贝砍掉了，够用。
    // ⚠️ 优先级 3 → 1（2026-10-22）：屏幕是全系统实时性要求**最低**的东西。
    //   原来这里 3，比 opus_codec(2) 还高 —— 解码被刷屏抢 CPU 正是"吞音"根因之一。
    //   音频侧提到 6/7 之后，这里顺势压到 1：任何时候音频有活都能立刻抢占刷屏。
    //   代价是刷屏可能被推迟几帧，但本任务已是脏矩形推送（~400B/帧、6.7fps），
    //   延迟几十毫秒肉眼看不出来。
    if (xTaskCreate(RefreshTaskEntry, "oled_refresh", 3584, this, 1, &refresh_task_) != pdPASS) {
        ESP_LOGE(TAG, "refresh task create failed, fallback to timer-only");
        refresh_task_ = nullptr;
    }
    esp_timer_create_args_t tcfg = {};
    tcfg.callback = &OledDisplay::RefreshTimerCb;
    tcfg.arg = this;
    tcfg.name = "oled_tick";
    ESP_ERROR_CHECK(esp_timer_create(&tcfg, &refresh_timer_));
    // ⚠️ 2026-10-20：250ms(4fps) → 150ms(6.7fps)。
    //   脏矩形实测 avg ≈ 400 B/frame（原 1024，省 ~60%），单帧推送 92ms → ~36ms。
    //   ⚠️ 试过 100ms(10fps)，实测会翻车：日志出现
    //     "i2c.master: clear bus failed" + "SSD1306_CMD_SET_COLUMN_RANGE failed"
    //     → PushFrame 报 ESP_ERR_INVALID_STATE。原因不是占用率（24% vs 原来 37%
    //     反而更低），而是**事务次数**从 4 次/秒涨到 10 次/秒，和测距事务撞车
    //     概率显著变高 —— 同一根总线上还有 VL53L0X。
    //   150ms 是个平衡点：比原来 4fps 快 67%，事务 6.7 次/秒，占用降到 ~24%。
    ESP_ERROR_CHECK(esp_timer_start_periodic(refresh_timer_, 150 * 1000));

    // ⚠️ 关键 bug 修复：原代码完全忽略了 mirror_x/mirror_y 参数，config.h 里的
    //    DISPLAY_MIRROR_X/Y 是死代码。手动调 esp_lcd_panel_mirror() 让方向设置生效。
    //    注意：必须在 RenderFace/PushFrame 之前调，否则第一帧会按错误方向画。
    if (panel_) {
        esp_err_t mret = esp_lcd_panel_mirror(panel_, mirror_x, mirror_y);
        if (mret != ESP_OK) {
            ESP_LOGW(TAG, "esp_lcd_panel_mirror returned %s", esp_err_to_name(mret));
        } else {
            ESP_LOGI(TAG, "OLED mirror set: x=%d y=%d", mirror_x ? 1 : 0, mirror_y ? 1 : 0);
        }
    }

    // 首次点亮：默认显示待机时钟（WiFi/NTP 还没就绪时显示占位文字）
    // ⚠️ 改：原先直接 RenderFace() 显示眼睛，违反"开机应显示时钟"的设计。
    //    构造时 animated_mode_ = false（默认值），refresh timer 会走 RenderClock()。
    Clear();
    RenderClock();
    PushFrame();

    // 建好 RoboEyes（眼睛复用 yyxc 那套）。放在这里是因为构造函数结束时它的
    // 定义已经可见；RenderFace 里也有一道兜底判空
    InitRoboEyes();

    ESP_LOGI(TAG, "OledDisplay ready (128x64, 1bpp, no LVGL) - boot with clock");
}

OledDisplay::~OledDisplay() {
    if (refresh_timer_) {
        esp_timer_stop(refresh_timer_);
        esp_timer_delete(refresh_timer_);
        refresh_timer_ = nullptr;
    }
    if (refresh_task_) {
        refresh_task_ = nullptr;   // 通知句柄失效后任务收到 notify 会失败并继续等待，不崩
    }
    delete eyes_;  eyes_ = nullptr;      // RoboEyes（含内部的 OledGfx 后端）
    delete gfx_;   gfx_  = nullptr;
    if (lock_) vSemaphoreDelete(lock_);
}

void OledDisplay::SetStatus(const char* status) {
    Lock(100);
    if (status) {
        strncpy(status_text_, status, sizeof(status_text_) - 1);
        status_text_[sizeof(status_text_) - 1] = 0;
    } else {
        status_text_[0] = 0;
    }
    dirty_ = true;
    Unlock();
}

// 临时提示：置覆盖层状态，真正的绘制在 RenderTick 顶部做。
//   ⚠️ 不用 status_text_ 那套：它在表情模式下不渲染（中文 6x8 会画成实心块）。
void OledDisplay::ShowNotice(const char* text, int duration_ms) {
    Lock(100);
    if (text != nullptr) {
        strncpy(notice_text_, text, sizeof(notice_text_) - 1);
        notice_text_[sizeof(notice_text_) - 1] = 0;
    } else {
        notice_text_[0] = 0;
    }
    notice_until_ms_ = (duration_ms > 0)
        ? (esp_timer_get_time() / 1000 + duration_ms) : 0;
    notice_drawn_ = false;      // 强制下一帧重绘
    dirty_ = true;
    Unlock();
}

void OledDisplay::ShowNotification(const char* notification, int duration_ms) {
    SetStatus(notification);
}

void OledDisplay::ShowNotification(const std::string& notification, int duration_ms) {
    SetStatus(notification.c_str());
}

void OledDisplay::SetChatMessage(const char* role, const char* content) {
    Lock(100);
    if (content && *content) {
        const int64_t now = esp_timer_get_time() / 1000;
        if (chat_playing_ && now < chat_lock_until_ms_) {
            // 上一条还没显示够 → 排队（只留最新一条），到期自动顶上来。
            //   "可打断"仍然成立，只是不再是毫秒级的闪一下。
            strncpy(chat_pending_role_, role ? role : "", sizeof(chat_pending_role_) - 1);
            chat_pending_role_[sizeof(chat_pending_role_) - 1] = 0;
            strncpy(chat_pending_text_, content, sizeof(chat_pending_text_) - 1);
            chat_pending_text_[sizeof(chat_pending_text_) - 1] = 0;
            chat_pending_ = true;
            ESP_LOGI(TAG, "chat queued role=%s (wait %dms)", role,
                     (int)(chat_lock_until_ms_ - now));
            Unlock();
            return;
        }
        strncpy(chat_role_, role ? role : "", sizeof(chat_role_) - 1);
        chat_role_[sizeof(chat_role_) - 1] = 0;
        strncpy(chat_text_, content, sizeof(chat_text_) - 1);
        chat_text_[sizeof(chat_text_) - 1] = 0;
        LayoutChatLines();
        // ⚠️ 新消息直接打断上一次播放（用户要求的"下一个任务可以打断"）：
        //   不排队、不拼接，从头开始滚。SetChatMessage 本来就是"一条新消息"的语义，
        //   Application 里 ASR 实时字幕也会高频调用，这里必须是最新的赢。
        chat_scroll_px_ = 0;
        chat_last_step_ms_ = esp_timer_get_time() / 1000;
        chat_hold_until_ms_ = 0;
        chat_playing_ = true;
        chat_lock_until_ms_ = now + kChatMinShowMs;   // 保底显示时间
        chat_pending_ = false;
        dirty_ = true;
        ESP_LOGI(TAG, "chat msg role=%s len=%u lines=%d content_h=%d mode=%d",
                 chat_role_, (unsigned)strlen(chat_text_), chat_line_count_,
                 chat_content_h_, (int)chat_text_mode_);
    } else {
        // 空串 = 清屏。这正是"退出唤醒"的信号（Application 在 kDeviceStateIdle /
        //   音频通道关闭 / 状态复位都会调），所以这里收掉播放 → RenderTick 下一帧
        //   就会掉到 clock 分支，屏幕回到待机时钟。
        const bool was_playing = chat_playing_;
        chat_playing_ = false;
        chat_text_[0] = 0;
        chat_role_[0] = 0;
        chat_line_count_ = 0;
        chat_hold_until_ms_ = 0;
        chat_pending_ = false;        // 退出唤醒时把排队的也丢掉，免得回到时钟后又冒出来
        chat_lock_until_ms_ = 0;
        SetStatus("");
        dirty_ = true;
        if (was_playing) last_clock_minute_ = -1;   // 强制重绘时钟（否则屏上残留最后一句）
    }
    Unlock();
}

// 屏幕显示模式（网页「🖥 屏幕」切换）
void OledDisplay::SetChatTextMode(bool text_mode) {
    Lock(100);
    if (chat_text_mode_ != text_mode) {
        chat_text_mode_ = text_mode;
        ESP_LOGI(TAG, "display mode -> %s (wants text=%d)",
                 text_mode ? "chat text" : "emotion/clock", (int)WantsChatText());
        last_clock_minute_ = -1;      // 换模式必须重画（否则屏上还留着上一模式的内容）
        text_idle_state_ = -1;        // 占位界面同理：重置后下一帧必定重绘
        dirty_ = true;
    }
    Unlock();
}

// 单个 UTF-8 字符的像素宽度；out_len 回写该字符占用字节数（1~4）
int OledDisplay::MeasureUtf8Char(const char* s, int* out_len) {
    const lv_font_t* f = &font_puhui_14_1;
    const lv_font_fmt_txt_dsc_t* fd = (const lv_font_fmt_txt_dsc_t*)f->dsc;
    if (fd->bpp != 1) { *out_len = 1; return 7; }

    const uint8_t* p = (const uint8_t*)s;
    uint32_t uni = 0;
    int len = 1;
    if      ((*p & 0x80) == 0)    { uni = *p; len = 1; }
    else if ((*p & 0xE0) == 0xC0) { if (!p[1]) { *out_len = 1; return 3; }
                                    uni = ((uint32_t)(p[0] & 0x1F) << 6) | (p[1] & 0x3F); len = 2; }
    else if ((*p & 0xF0) == 0xE0) { if (!p[1] || !p[2]) { *out_len = 1; return 3; }
                                    uni = ((uint32_t)(p[0] & 0x0F) << 12) | ((uint32_t)(p[1] & 0x3F) << 6) | (p[2] & 0x3F); len = 3; }
    else if ((*p & 0xF8) == 0xF0) { if (!p[1] || !p[2] || !p[3]) { *out_len = 1; return 3; }
                                    uni = ((uint32_t)(p[0] & 0x07) << 18) | ((uint32_t)(p[1] & 0x3F) << 12) | ((uint32_t)(p[2] & 0x3F) << 6) | (p[3] & 0x3F); len = 4; }
    else { *out_len = 1; return 3; }

    *out_len = len;
    uint32_t gid = LookupGlyphId(fd, uni);
    if (!gid) return 7;                       // 缺字：按半角估，别把行挤歪
    return fd->glyph_dsc[gid].adv_w >> 4;     // adv_w 是 28.4 定点
}

// 自动换行：UTF-8 感知（中文 14px / ASCII 7px 混排），按字节偏移记录每行。
//   ⚠️ 只在收到新消息时跑一次，滚动渲染是"取子串直接描点"，不再重新排版。
void OledDisplay::LayoutChatLines() {
    chat_line_count_ = 0;
    const int max_w = kWidth - 4;                 // 左右各留 2px
    const int len = (int)strlen(chat_text_);
    int i = 0, line_start = 0, w = 0;
    while (i < len && chat_line_count_ < 32) {
        if (chat_text_[i] == '\n') {              // 消息里的硬换行
            chat_line_off_[chat_line_count_] = line_start;
            chat_line_len_[chat_line_count_] = i - line_start;
            chat_line_count_++;
            line_start = i + 1;
            w = 0;
            i++;
            continue;
        }
        int clen = 0;
        int cw = MeasureUtf8Char(chat_text_ + i, &clen);
        if (clen <= 0 || i + clen > len) break;
        if (w + cw > max_w && i > line_start) {
            chat_line_off_[chat_line_count_] = line_start;
            chat_line_len_[chat_line_count_] = i - line_start;
            chat_line_count_++;
            line_start = i;
            w = 0;
        }
        w += cw;
        i += clen;
    }
    if (i > line_start && chat_line_count_ < 32) {
        chat_line_off_[chat_line_count_] = line_start;
        chat_line_len_[chat_line_count_] = i - line_start;
        chat_line_count_++;
    }
    chat_content_h_ = chat_line_count_ * kChatLineH;
}

// 画当前滚动窗口（每行按字节偏移临时取子串）
void OledDisplay::RenderChatPlaying() {
    Clear();
    const int first = chat_scroll_px_ / kChatLineH;
    const int off   = chat_scroll_px_ % kChatLineH;
    for (int i = 0; i < kHeight / kChatLineH + 2; ++i) {
        const int li = first + i;
        if (li >= chat_line_count_) break;
        int n = chat_line_len_[li];
        if (n <= 0) continue;
        if (n > 80) n = 80;                       // 单行上限（约 11 个汉字）
        char line[88];
        memcpy(line, chat_text_ + chat_line_off_[li], (size_t)n);
        line[n] = 0;
        DrawUtf8String(2, i * kChatLineH - off + 1, line);
    }
    dirty_ = true;    // 滚动中：下一帧继续推
}

// 按时间推进滚动。返回 true = 需要重绘
bool OledDisplay::AdvanceChatScroll() {
    const int64_t now = esp_timer_get_time() / 1000;
    const int64_t dt  = now - chat_last_step_ms_;
    if (dt < 100) return false;                   // 至少 100ms 一步，别跟 92ms 的 I2C 互相追尾
    chat_last_step_ms_ = now;

    const int max_scroll = chat_content_h_ > kHeight ? chat_content_h_ - kHeight : 0;
    if (chat_scroll_px_ < max_scroll) {
        // ~32px/s：每 250ms 的 tick 推进 8px，一行看 0.5s，读得清
        chat_scroll_px_ += (int)(32 * dt / 1000);
        if (chat_scroll_px_ > max_scroll) chat_scroll_px_ = max_scroll;
        return true;
    }
    // 已到底：停留 2.5s 让人读完最后一行，然后自动收尾
    if (chat_hold_until_ms_ == 0) { chat_hold_until_ms_ = now + 2500; return true; }
    if (now >= chat_hold_until_ms_) {
        chat_playing_ = false;
        last_clock_minute_ = -1;      // 播完 → 立刻把时钟画回来（等分钟变化太慢了）
        return true;
    }
    return false;    // 停留期间不重绘：屏自然停住，I2C 0 占用
}

// 配网热点信息：ssid 传空 = 退出配网界面（连上网 / 退出配网模式时由 WifiBoard 调）
void OledDisplay::SetWifiConfigInfo(const char* ssid, const char* url) {
    Lock(100);
    const bool visible = (ssid && *ssid);
    if (visible) {
        strncpy(cfg_ssid_, ssid, sizeof(cfg_ssid_) - 1);
        cfg_ssid_[sizeof(cfg_ssid_) - 1] = 0;
        strncpy(cfg_url_, url ? url : "", sizeof(cfg_url_) - 1);
        cfg_url_[sizeof(cfg_url_) - 1] = 0;
    } else {
        cfg_ssid_[0] = 0;
        cfg_url_[0] = 0;
    }
    cfg_info_visible_ = visible;
    special_drawn_ = false;      // 内容变了 → 下一帧重画
    dirty_ = true;
    Unlock();
}

// 激活验证码：非空 = 整屏显示（用户抄到 APP/控制台）；传空 = 清除
void OledDisplay::SetActivationCode(const char* code) {
    Lock(100);
    const bool visible = (code && *code);
    if (visible) {
        strncpy(act_code_, code, sizeof(act_code_) - 1);
        act_code_[sizeof(act_code_) - 1] = 0;
    } else {
        act_code_[0] = 0;
    }
    act_code_visible_ = visible;
    special_drawn_ = false;
    dirty_ = true;
    if (!visible) last_clock_minute_ = -1;   // 清掉后立刻恢复时钟/表情
    Unlock();
}

void OledDisplay::SetEmotion(const char* emotion) {
    Lock(100);
    int idx = 0;  // default = neutral
    if (emotion) {
        std::string e(emotion);
        // ⚠️ 原表只认 6 个英文词，服务端实际会下发 20+ 种情绪名和 emoji
        //    （laughing / winking / confused / 😆 / 🤔 ...），全部落到 neutral
        //    → 用户看到"只有眨眼、没有别的表情"。这里做全覆盖映射。
        if      (e == "neutral" || e == "😶" || e == "sleepy" || e == "😴") {
            idx = 0;   // 平静 / 困
        }
        else if (e == "happy"  || e == "🙂" || e == "😊" || e == "smile"   ||
                 e == "loving" || e == "😍" || e == "kissy"   || e == "😘"  ||
                 e == "cool"   || e == "😎" || e == "relaxed" || e == "😌"  ||
                 e == "confident" || e == "😏" || e == "delicious" || e == "🤤" ||
                 e == "wink"   || e == "winking" || e == "😉") {
            idx = 1;   // 开心 / 喜欢 / 得意
        }
        else if (e == "sad"    || e == "😔" || e == "crying" || e == "😭" ||
                 e == "cry"    || e == "embarrassed" || e == "😳") {
            idx = 2;   // 难过 / 哭 / 害羞
        }
        else if (e == "angry"  || e == "😠" || e == "mad" || e == "😡" ||
                 e == "annoyed" || e == "🙄") {
            idx = 7;   // 生气（横椭圆眼 + 视线下压 + 眨得勤）
        }
        else if (e == "thinking" || e == "🤔" || e == "confused" || e == "🙄" ||
                 e == "silly"    || e == "😜" || e == "thinking_face") {
            idx = 3;   // 思考 / 困惑（视线往右）
        }
        else if (e == "listening" || e == "curious") {
            idx = 4;   // 聆听（视线往左、睁大）
        }
        else if (e == "surprised" || e == "😲" || e == "shocked" || e == "😱") {
            idx = 6;   // 惊讶（眼睛放到最大最圆，且很少眨眼 = 盯着看）
        }
        else if (e == "speaking" || e == "talking" || e == "wake" ||
                 e == "laughing" || e == "😆" || e == "funny" || e == "😂" ||
                 e == "laugh") {
            idx = 5;   // 说话 / 大笑（嘴会动）
        }
        // 其余未知情绪 → 保持 0 neutral
    }
    emotion_idx_ = idx;
    ApplyRoboEmotion(idx);      // 让 RoboEyes 立刻切到这套 mood / 视线 / 动画
    dirty_ = true;
    Unlock();
}

void OledDisplay::SetAnimatedEmotionMode(bool enable) {
    animated_mode_ = enable;
    // 进入时钟模式时把 last_clock_minute_ 设为 -1，让下一帧立即渲染
    if (!enable) {
        last_clock_minute_ = -1;
    }
    dirty_ = true;
}

void OledDisplay::SetMusicMode(bool on) {
    if (music_mode_.load() == on) return;
    music_mode_ = on;
    dirty_ = true;   // 时钟模式也要立刻重画（去掉/加上音符）
}

// 🎵 音乐界面：整屏居中 2 个 32x32 大音符，替代表情/时钟显示
void OledDisplay::RenderMusic() {
    Clear();

    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    if (boot_ms_ == 0) boot_ms_ = now_ms;
    uint32_t rel = now_ms - boot_ms_;

    // 歌名（顶部居中，14px 中文字库 —— font_puhui_14_1 含 12814 个汉字）
    //   宽度在 SetSongName 里已经量好了，这里直接用
    if (song_name_[0]) {
        int x = (kWidth - song_w_) / 2;
        if (x < 0) x = 0;                  // 歌名太长就左对齐，别两边都被裁掉
        DrawUtf8String(x, 0, song_name_);
    }

    // 单个大 ♫（48x40）：比并排两个小音符更大更清楚，
    //   也不会像单音符 ♪ 那样把符头椭圆看成「一张嘴」
    static constexpr int kNoteW = 48;
    static constexpr int kNoteH = 40;
    int x0     = (kWidth - kNoteW) / 2;
    int base_y = 18;                           // 顶部歌名约占 0..16，音符从 18 起

    int bob = (int)((rel / 300) % 3) - 1;      // -1 / 0 / 1，轻微跳动
    DrawMusicNote(x0, base_y + bob);
    // 音乐界面不画状态文字：竖向空间都给了歌名和音符

    dirty_ = true;   // 持续动画：下一帧继续推屏
}

void OledDisplay::UpdateAnimatedEmotion() {
    // 主循环每 tick 也会调这里 → 必须走 RenderTick()，不能自己判断
    // （否则会和 timer 路径逻辑不一致，见 RenderTick 的注释）
    if (Lock(0)) {
        RenderTick();
        Unlock();
    }
}

void OledDisplay::SetSpeaking(bool on) {
    if (speaking_.load() == on) return;
    speaking_ = on;
    dirty_ = true;
}

void OledDisplay::SetSongName(const char* name) {
    Lock(100);
    if (!name || !*name) {
        song_name_[0] = 0;
        song_w_ = 0;
    } else {
        snprintf(song_name_, sizeof(song_name_), "%s", name);   // char[32]：最多 10 个汉字，够屏宽
        // 宽度只在这里算一次（用离屏坐标测宽，越界像素会被 SetPixel 裁掉）。
        // 否则 RenderMusic 每帧都要重跑一遍 UTF-8 解码 + 字形查找，白烧 CPU
        song_w_ = DrawUtf8String(-200, 0, song_name_);
    }
    Unlock();
    dirty_ = true;
}
