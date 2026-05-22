/**
 * @file  oled_driver.cpp
 * @brief OLED UI driver implementation
 */

#include "oled_driver.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <gpiod.h>
#include <iomanip>
#include <linux/i2c-dev.h>
#include <snprintf.h>
#include <sstream>
#include <stdexcept>
#include <sys/ioctl.h>
#include <thread>
#include <unistd.h>

// Avoid pulling <cstdio> via sntprintf.h on some systems
#include <cstdio>

namespace oled {

// ─────────────────────────────────────────────────────────────
//  Helpers
// ─────────────────────────────────────────────────────────────
static float lerp(float a, float b, float t) { return a + (b - a) * t; }

/// Cubic ease-out
static float easeOut(float t) {
    t = std::clamp(t, 0.f, 1.f);
    return 1.f - (1.f - t) * (1.f - t) * (1.f - t);
}

static int64_t nowMs() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}

/// Format float with fixed decimals; max width chars
static std::string fmtF(float v, int prec = 2, int total_w = 0) {
    char buf[32];
    if (total_w > 0) snprintf(buf, sizeof(buf), "%*.*f", total_w, prec, v);
    else              snprintf(buf, sizeof(buf), "%.*f",        prec, v);
    return buf;
}

// ─────────────────────────────────────────────────────────────
//  5×7 ASCII font  (columns, bit-0 = top row)
//  Characters 0x20 – 0x7E  (95 glyphs)
// ─────────────────────────────────────────────────────────────
static const uint8_t kFont5x7[][5] = {
    {0x00,0x00,0x00,0x00,0x00}, // 0x20 space
    {0x00,0x00,0x5F,0x00,0x00}, // 0x21 !
    {0x00,0x07,0x00,0x07,0x00}, // 0x22 "
    {0x14,0x7F,0x14,0x7F,0x14}, // 0x23 #
    {0x24,0x2A,0x7F,0x2A,0x12}, // 0x24 $
    {0x23,0x13,0x08,0x64,0x62}, // 0x25 %
    {0x36,0x49,0x55,0x22,0x50}, // 0x26 &
    {0x00,0x05,0x03,0x00,0x00}, // 0x27 '
    {0x00,0x1C,0x22,0x41,0x00}, // 0x28 (
    {0x00,0x41,0x22,0x1C,0x00}, // 0x29 )
    {0x14,0x08,0x3E,0x08,0x14}, // 0x2A *
    {0x08,0x08,0x3E,0x08,0x08}, // 0x2B +
    {0x00,0x50,0x30,0x00,0x00}, // 0x2C ,
    {0x08,0x08,0x08,0x08,0x08}, // 0x2D -
    {0x00,0x60,0x60,0x00,0x00}, // 0x2E .
    {0x20,0x10,0x08,0x04,0x02}, // 0x2F /
    {0x3E,0x51,0x49,0x45,0x3E}, // 0x30 0
    {0x00,0x42,0x7F,0x40,0x00}, // 0x31 1
    {0x42,0x61,0x51,0x49,0x46}, // 0x32 2
    {0x21,0x41,0x45,0x4B,0x31}, // 0x33 3
    {0x18,0x14,0x12,0x7F,0x10}, // 0x34 4
    {0x27,0x45,0x45,0x45,0x39}, // 0x35 5
    {0x3C,0x4A,0x49,0x49,0x30}, // 0x36 6
    {0x01,0x71,0x09,0x05,0x03}, // 0x37 7
    {0x36,0x49,0x49,0x49,0x36}, // 0x38 8
    {0x06,0x49,0x49,0x29,0x1E}, // 0x39 9
    {0x00,0x36,0x36,0x00,0x00}, // 0x3A :
    {0x00,0x56,0x36,0x00,0x00}, // 0x3B ;
    {0x08,0x14,0x22,0x41,0x00}, // 0x3C <
    {0x14,0x14,0x14,0x14,0x14}, // 0x3D =
    {0x00,0x41,0x22,0x14,0x08}, // 0x3E >
    {0x02,0x01,0x51,0x09,0x06}, // 0x3F ?
    {0x32,0x49,0x79,0x41,0x3E}, // 0x40 @
    {0x7E,0x11,0x11,0x11,0x7E}, // 0x41 A
    {0x7F,0x49,0x49,0x49,0x36}, // 0x42 B
    {0x3E,0x41,0x41,0x41,0x22}, // 0x43 C
    {0x7F,0x41,0x41,0x22,0x1C}, // 0x44 D
    {0x7F,0x49,0x49,0x49,0x41}, // 0x45 E
    {0x7F,0x09,0x09,0x09,0x01}, // 0x46 F
    {0x3E,0x41,0x49,0x49,0x7A}, // 0x47 G
    {0x7F,0x08,0x08,0x08,0x7F}, // 0x48 H
    {0x00,0x41,0x7F,0x41,0x00}, // 0x49 I
    {0x20,0x40,0x41,0x3F,0x01}, // 0x4A J
    {0x7F,0x08,0x14,0x22,0x41}, // 0x4B K
    {0x7F,0x40,0x40,0x40,0x40}, // 0x4C L
    {0x7F,0x02,0x0C,0x02,0x7F}, // 0x4D M
    {0x7F,0x04,0x08,0x10,0x7F}, // 0x4E N
    {0x3E,0x41,0x41,0x41,0x3E}, // 0x4F O
    {0x7F,0x09,0x09,0x09,0x06}, // 0x50 P
    {0x3E,0x41,0x51,0x21,0x5E}, // 0x51 Q
    {0x7F,0x09,0x19,0x29,0x46}, // 0x52 R
    {0x46,0x49,0x49,0x49,0x31}, // 0x53 S
    {0x01,0x01,0x7F,0x01,0x01}, // 0x54 T
    {0x3F,0x40,0x40,0x40,0x3F}, // 0x55 U
    {0x1F,0x20,0x40,0x20,0x1F}, // 0x56 V
    {0x3F,0x40,0x38,0x40,0x3F}, // 0x57 W
    {0x63,0x14,0x08,0x14,0x63}, // 0x58 X
    {0x07,0x08,0x70,0x08,0x07}, // 0x59 Y
    {0x61,0x51,0x49,0x45,0x43}, // 0x5A Z
    {0x00,0x7F,0x41,0x41,0x00}, // 0x5B [
    {0x02,0x04,0x08,0x10,0x20}, // 0x5C backslash
    {0x00,0x41,0x41,0x7F,0x00}, // 0x5D ]
    {0x04,0x02,0x01,0x02,0x04}, // 0x5E ^
    {0x40,0x40,0x40,0x40,0x40}, // 0x5F _
    {0x00,0x01,0x02,0x04,0x00}, // 0x60 `
    {0x20,0x54,0x54,0x54,0x78}, // 0x61 a
    {0x7F,0x48,0x44,0x44,0x38}, // 0x62 b
    {0x38,0x44,0x44,0x44,0x20}, // 0x63 c
    {0x38,0x44,0x44,0x48,0x7F}, // 0x64 d
    {0x38,0x54,0x54,0x54,0x18}, // 0x65 e
    {0x08,0x7E,0x09,0x01,0x02}, // 0x66 f
    {0x0C,0x52,0x52,0x52,0x3E}, // 0x67 g
    {0x7F,0x08,0x04,0x04,0x78}, // 0x68 h
    {0x00,0x44,0x7D,0x40,0x00}, // 0x69 i
    {0x20,0x40,0x44,0x3D,0x00}, // 0x6A j
    {0x7F,0x10,0x28,0x44,0x00}, // 0x6B k
    {0x00,0x41,0x7F,0x40,0x00}, // 0x6C l
    {0x7C,0x04,0x18,0x04,0x78}, // 0x6D m
    {0x7C,0x08,0x04,0x04,0x78}, // 0x6E n
    {0x38,0x44,0x44,0x44,0x38}, // 0x6F o
    {0x7C,0x14,0x14,0x14,0x08}, // 0x70 p
    {0x08,0x14,0x14,0x18,0x7C}, // 0x71 q
    {0x7C,0x08,0x04,0x04,0x08}, // 0x72 r
    {0x48,0x54,0x54,0x54,0x20}, // 0x73 s
    {0x04,0x3F,0x44,0x40,0x20}, // 0x74 t
    {0x3C,0x40,0x40,0x20,0x7C}, // 0x75 u
    {0x1C,0x20,0x40,0x20,0x1C}, // 0x76 v
    {0x3C,0x40,0x30,0x40,0x3C}, // 0x77 w
    {0x44,0x28,0x10,0x28,0x44}, // 0x78 x
    {0x0C,0x50,0x50,0x50,0x3C}, // 0x79 y
    {0x44,0x64,0x54,0x4C,0x44}, // 0x7A z
    {0x00,0x08,0x36,0x41,0x00}, // 0x7B {
    {0x00,0x00,0x7F,0x00,0x00}, // 0x7C |
    {0x00,0x41,0x36,0x08,0x00}, // 0x7D }
    {0x10,0x08,0x08,0x10,0x08}, // 0x7E ~
};

// ═════════════════════════════════════════════════════════════
//  FrameBuffer
// ═════════════════════════════════════════════════════════════
FrameBuffer::FrameBuffer() { clear(); }

void FrameBuffer::clear() { buf_.fill(0); }

void FrameBuffer::setPixel(int x, int y, bool on) {
    if (x < 0 || x >= kW || y < 0 || y >= kH) return;
    int idx = (y >> 3) * kW + x;
    if (on) buf_[idx] |=  (uint8_t)(1u << (y & 7));
    else    buf_[idx] &= ~(uint8_t)(1u << (y & 7));
}

bool FrameBuffer::getPixel(int x, int y) const {
    if (x < 0 || x >= kW || y < 0 || y >= kH) return false;
    return (buf_[(y >> 3) * kW + x] >> (y & 7)) & 1;
}

void FrameBuffer::drawHLine(int x, int y, int w) {
    for (int i = 0; i < w; ++i) setPixel(x + i, y);
}

void FrameBuffer::drawVLine(int x, int y, int h) {
    for (int i = 0; i < h; ++i) setPixel(x, y + i);
}

void FrameBuffer::drawLine(int x0, int y0, int x1, int y1) {
    int dx = std::abs(x1 - x0), dy = -std::abs(y1 - y0);
    int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    for (;;) {
        setPixel(x0, y0);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

void FrameBuffer::drawRect(int x, int y, int w, int h) {
    drawHLine(x, y, w);
    drawHLine(x, y + h - 1, w);
    drawVLine(x, y, h);
    drawVLine(x + w - 1, y, h);
}

void FrameBuffer::fillRect(int x, int y, int w, int h) {
    for (int row = y; row < y + h; ++row)
        drawHLine(x, row, w);
}

void FrameBuffer::drawCircle(int cx, int cy, int r) {
    int x = 0, y = r, d = 3 - 2 * r;
    auto plot8 = [&](int xi, int yi) {
        setPixel(cx+xi, cy+yi); setPixel(cx-xi, cy+yi);
        setPixel(cx+xi, cy-yi); setPixel(cx-xi, cy-yi);
        setPixel(cx+yi, cy+xi); setPixel(cx-yi, cy+xi);
        setPixel(cx+yi, cy-xi); setPixel(cx-yi, cy-xi);
    };
    while (y >= x) { plot8(x, y); if (d < 0) d += 4*x+6; else { d += 4*(x-y)+10; --y; } ++x; }
}

void FrameBuffer::fillCircle(int cx, int cy, int r) {
    for (int y = -r; y <= r; ++y) {
        int hw = (int)std::sqrt((float)(r*r - y*y));
        drawHLine(cx - hw, cy + y, 2*hw + 1);
    }
}

void FrameBuffer::invertRect(int x, int y, int w, int h) {
    for (int row = y; row < y + h && row < kH; ++row)
        for (int col = x; col < x + w && col < kW; ++col) {
            int idx = (row >> 3) * kW + col;
            buf_[idx] ^= (uint8_t)(1u << (row & 7));
        }
}

// ── Text ──────────────────────────────────────────────────────
int FrameBuffer::drawChar(int x, int y, char c, int scale, bool invert) {
    if (c < 0x20 || c > 0x7E) c = '?';
    const uint8_t* glyph = kFont5x7[c - 0x20];
    for (int col = 0; col < 5; ++col) {
        uint8_t bits = glyph[col];
        for (int row = 0; row < 7; ++row) {
            bool on = (bits >> row) & 1;
            if (invert) on = !on;
            for (int sy = 0; sy < scale; ++sy)
                for (int sx = 0; sx < scale; ++sx)
                    setPixel(x + col*scale + sx, y + row*scale + sy, on);
        }
    }
    // background column gap
    if (invert) {
        for (int row = 0; row < 7*scale; ++row)
            for (int sx = 0; sx < scale; ++sx)
                setPixel(x + 5*scale + sx, y + row, true);
    }
    return 6 * scale;
}

int FrameBuffer::drawString(int x, int y, const std::string& s, int scale, bool invert) {
    int cx = x;
    for (char c : s) cx += drawChar(cx, y, c, scale, invert);
    return cx - x;
}

void FrameBuffer::drawStringCenter(int y, const std::string& s, int scale, bool invert) {
    int tw = (int)s.size() * charW(scale);
    drawString((kW - tw) / 2, y, s, scale, invert);
}

void FrameBuffer::drawStringRight(int xr, int y, const std::string& s, int scale) {
    int tw = (int)s.size() * charW(scale);
    drawString(xr - tw, y, s, scale);
}

// ── Widgets ───────────────────────────────────────────────────
void FrameBuffer::drawProgressBar(int x, int y, int w, int h, float pct) {
    pct = std::clamp(pct, 0.f, 1.f);
    drawRect(x, y, w, h);
    int filled = (int)((w - 2) * pct);
    if (filled > 0) fillRect(x + 1, y + 1, filled, h - 2);
}

void FrameBuffer::drawScrollbar(int x, int y, int h, float pos, float vis) {
    vis = std::clamp(vis, 0.1f, 1.f);
    int bar_h = (int)(h * vis);
    int bar_y = y + (int)((h - bar_h) * pos);
    drawVLine(x, y, h);
    fillRect(x, bar_y, 2, bar_h);
}

// ── Transition blit ───────────────────────────────────────────
void FrameBuffer::blitShifted(const FrameBuffer& src, int dx) {
    // Operate page-by-page: each page row is kW bytes
    for (int page = 0; page < kPages; ++page) {
        uint8_t*       dst = buf_.data()     + page * kW;
        const uint8_t* sr  = src.buf_.data() + page * kW;
        if (dx == 0) {
            std::memcpy(dst, sr, kW);
        } else if (dx > 0 && dx < kW) {
            // src shifted right by dx
            std::memset(dst, 0, dx);
            std::memcpy(dst + dx, sr, kW - dx);
        } else if (dx < 0 && dx > -kW) {
            int shift = -dx;
            std::memcpy(dst, sr + shift, kW - shift);
            std::memset(dst + (kW - shift), 0, shift);
        } else {
            std::memset(dst, 0, kW);
        }
    }
}

// ═════════════════════════════════════════════════════════════
//  OledDisplay  (SH1106 I²C)
// ═════════════════════════════════════════════════════════════
OledDisplay::OledDisplay(const char* dev, uint8_t addr)
    : addr_(addr), dev_(dev) {}

OledDisplay::~OledDisplay() {
    if (fd_ >= 0) close(fd_);
}

bool OledDisplay::cmd(uint8_t c) {
    uint8_t buf[2] = {0x00, c};
    return write(fd_, buf, 2) == 2;
}

bool OledDisplay::cmds(std::initializer_list<uint8_t> cs) {
    for (uint8_t c : cs) if (!cmd(c)) return false;
    return true;
}

bool OledDisplay::init() {
    fd_ = open(dev_, O_RDWR);
    if (fd_ < 0) return false;
    if (ioctl(fd_, I2C_SLAVE, addr_) < 0) return false;

    // SH1106 init sequence
    static const uint8_t seq[] = {
        0xAE,        // display off
        0xD5, 0x80,  // set display clock div / oscillator
        0xA8, 0x3F,  // multiplex ratio (64MUX)
        0xD3, 0x00,  // display offset = 0
        0x40,        // start line = 0
        0xAD, 0x8B,  // internal charge pump ON
        0xA1,        // segment remap (flip x)
        0xC8,        // COM scan direction (flip y)
        0xDA, 0x12,  // COM pins hardware config
        0x81, 0xCF,  // contrast
        0xD9, 0x1F,  // pre-charge period
        0xDB, 0x40,  // VCOM deselect level
        0xA4,        // entire display ON (follow RAM)
        0xA6,        // normal display (not inverted)
        0xAF,        // display ON
    };
    for (uint8_t c : seq)
        if (!cmd(c)) return false;

    // Clear display RAM
    FrameBuffer blank;
    flush(blank);
    return true;
}

bool OledDisplay::writePage(int page, const uint8_t* data128) {
    // SH1106 has 2-column offset
    cmd((uint8_t)(0xB0 | page));   // set page
    cmd(0x02);                      // low  column = 2 (offset)
    cmd(0x10);                      // high column = 0

    uint8_t buf[129];
    buf[0] = 0x40; // data mode
    std::memcpy(buf + 1, data128, 128);
    return write(fd_, buf, 129) == 129;
}

void OledDisplay::flush(const FrameBuffer& fb) {
    const uint8_t* raw = fb.raw();
    for (int p = 0; p < kPages; ++p)
        writePage(p, raw + p * kW);
}

void OledDisplay::setContrast(uint8_t c) { cmds({0x81, c}); }

void OledDisplay::setOn(bool on) { cmd(on ? 0xAF : 0xAE); }

// ═════════════════════════════════════════════════════════════
//  InputHandler
// ═════════════════════════════════════════════════════════════
struct InputHandler::GpioHandle {
    gpiod_chip* chip = nullptr;
    gpiod_line* line = nullptr;

    ~GpioHandle() {
        if (line) gpiod_line_release(line);
        if (chip) gpiod_chip_close(chip);
    }
};

static bool openGpio(const GpioPin& pin, InputHandler::GpioHandle& h,
                     const char* consumer) {
    h.chip = gpiod_chip_open(pin.chip);
    if (!h.chip) return false;
    h.line = gpiod_chip_get_line(h.chip, pin.line);
    if (!h.line) return false;
    // Request with pull-up if kernel API supports it (libgpiod >= 1.5)
    // Fallback to plain input otherwise
    int rc = gpiod_line_request_input_flags(
        h.line, consumer, GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP);
    if (rc < 0) {
        rc = gpiod_line_request_input(h.line, consumer);
    }
    return rc >= 0;
}

// Encoder CW/CCW lookup table
// index = (prev_AB << 2) | curr_AB
static const int8_t kEncTable[16] = {
    0, +1, -1,  0,
   -1,  0,  0, +1,
   +1,  0,  0, -1,
    0, -1, +1,  0,
};

InputHandler::InputHandler(Config cfg) : cfg_(std::move(cfg)) {}

InputHandler::~InputHandler() { stop(); }

bool InputHandler::init() {
    static const char* consumer = "oled_ui";
    const GpioPin pins[] = {
        cfg_.enc_a, cfg_.enc_b, cfg_.enc_btn,
        cfg_.confirm, cfg_.back
    };
    gpios_.clear();
    for (auto& pin : pins) {
        auto h = std::make_unique<GpioHandle>();
        if (!openGpio(pin, *h, consumer)) return false;
        gpios_.push_back(std::move(h));
    }
    // Read initial encoder state
    int a = gpiod_line_get_value(gpios_[0]->line);
    int b = gpiod_line_get_value(gpios_[1]->line);
    enc_prev_ = (uint8_t)((a << 1) | b);
    return true;
}

void InputHandler::start() {
    if (running_.exchange(true)) return;
    thread_ = std::thread(&InputHandler::loop, this);
}

void InputHandler::stop() {
    running_.store(false);
    if (thread_.joinable()) thread_.join();
}

Ev InputHandler::pop() {
    std::lock_guard<std::mutex> lk(q_mtx_);
    if (queue_.empty()) return Ev::None;
    Ev ev = queue_.front();
    queue_.pop_front();
    return ev;
}

void InputHandler::push(Ev ev) {
    {
        std::lock_guard<std::mutex> lk(q_mtx_);
        if (queue_.size() < 16) queue_.push_back(ev);
    }
    if (cb_) cb_(ev);
}

int InputHandler::readLine(int idx) const {
    return gpiod_line_get_value(gpios_[idx]->line);
}

void InputHandler::loop() {
    using namespace std::chrono;
    while (running_.load()) {
        // ── Encoder ─────────────────────────────────────────
        int a = readLine(0), b = readLine(1);
        uint8_t curr = (uint8_t)((a << 1) | b);
        if (curr != enc_prev_) {
            int8_t dir = kEncTable[(enc_prev_ << 2) | curr];
            if (dir > 0) push(Ev::EncCW);
            else if (dir < 0) push(Ev::EncCCW);
            enc_prev_ = curr;
        }

        // ── Button helper ────────────────────────────────────
        int64_t now = nowMs();
        auto handleBtn = [&](BtnState& bs, int gpio_idx,
                              Ev short_ev, Ev long_ev) {
            bool raw = (readLine(gpio_idx) == 0); // active-low
            // Debounce
            if (raw != bs.last) {
                bs.last = raw;
                // immediate debounce via stable flag reset
                bs.stable = false;
            }
            if (!bs.stable) {
                // Check again after ~30ms – track press start on first stable press
                bs.stable = true;
                if (raw) {
                    bs.press_ms   = now;
                    bs.long_fired = false;
                } else if (bs.press_ms > 0 && !bs.long_fired) {
                    push(short_ev);
                    bs.press_ms = 0;
                }
            }
            // Long press detection
            if (raw && bs.press_ms > 0 && !bs.long_fired &&
                (now - bs.press_ms) >= cfg_.long_ms) {
                push(long_ev);
                bs.long_fired = true;
            }
        };

        handleBtn(bs_enc_,     2, Ev::EncPress,    Ev::EncPress);
        handleBtn(bs_confirm_, 3, Ev::Confirm,      Ev::ConfirmLong);
        handleBtn(bs_back_,    4, Ev::Back,         Ev::BackLong);

        std::this_thread::sleep_for(milliseconds(cfg_.poll_ms));
    }
}

// ═════════════════════════════════════════════════════════════
//  UIScreen base – common title bar
// ═════════════════════════════════════════════════════════════
void UIScreen::drawTitleBar(FrameBuffer& fb, const char* label,
                            int cur_idx, int total) {
    // Invert top 9 rows for title background
    fb.fillRect(0, 0, kW, 9);
    // Title label (white text on black → draw inverted)
    fb.drawString(2, 1, label, 1, true);
    // Screen index indicator on the right, e.g. "2/5"
    if (total > 1) {
        char buf[8];
        snprintf(buf, sizeof(buf), "%d/%d", cur_idx, total);
        fb.drawStringRight(kW - 1, 1, buf, 1);
        // invert those chars too
        int tw = (int)std::strlen(buf) * FrameBuffer::charW(1);
        fb.invertRect(kW - 1 - tw, 0, tw, 9);
    }
    // separator line
    fb.drawHLine(0, 9, kW);
}

// ═════════════════════════════════════════════════════════════
//  Screen: Menu
// ═════════════════════════════════════════════════════════════
const MenuScreen::Item MenuScreen::kItems[kCount] = {
    {ScreenID::Joints,   "Joints",     "[J]"},
    {ScreenID::IMU,      "IMU",        "[I]"},
    {ScreenID::Joystick, "Joystick",   "[G]"},
    {ScreenID::LogInfo,  "AimRT Log",  "[L]"},
    {ScreenID::SBC,      "SBC Status", "[S]"},
};

MenuScreen::MenuScreen(NavCb nav) : nav_(std::move(nav)) {}

void MenuScreen::onEnter() {
    pulse_ = 0.f;
}

void MenuScreen::onInput(Ev ev) {
    switch (ev) {
    case Ev::EncCW:
        sel_ = (sel_ + 1) % kCount;
        tsy_ = (float)(sel_ * 12);
        pulse_ = 0.f;
        break;
    case Ev::EncCCW:
        sel_ = (sel_ - 1 + kCount) % kCount;
        tsy_ = (float)(sel_ * 12);
        pulse_ = 0.f;
        break;
    case Ev::EncPress:
    case Ev::Confirm:
        if (nav_) nav_(kItems[sel_].sid, +1);
        break;
    default: break;
    }
}

void MenuScreen::update(float dt) {
    sy_    = lerp(sy_,    tsy_,   std::min(1.f, dt * 20.f));
    pulse_ = std::fmod(pulse_ + dt * 2.f, 2.f * 3.14159f);
}

void MenuScreen::render(FrameBuffer& fb) const {
    fb.clear();

    // ── IP bar (top 9 px, black bg inverted) ─────────────
    fb.fillRect(0, 0, kW, 9);
    std::string ip_str = "IP: " + (ip_.empty() ? "---" : ip_);
    fb.drawString(2, 1, ip_str, 1, /*invert=*/true);
    fb.drawHLine(0, 9, kW);

    // ── Menu items starting at y=11 ───────────────────────
    constexpr int kItemH = 12;
    constexpr int kContentY = 11;

    for (int i = 0; i < kCount; ++i) {
        int iy = kContentY + i * kItemH - (int)sy_;
        if (iy < kContentY - kItemH || iy > kH) continue;

        bool selected = (i == sel_);
        if (selected) {
            // Pulsing selection rectangle
            float alpha = (std::sin(pulse_) + 1.f) * 0.5f;
            int px = (int)(alpha * 1.f); // 0 or 1 border pulse
            fb.fillRect(1, iy - 1, kW - 2, kItemH - 1);
            (void)px;
        }

        // Icon (2 chars)
        fb.drawString(2, iy + 2, kItems[i].icon, 1, selected);
        // Label
        fb.drawString(16, iy + 2, kItems[i].label, 1, selected);

        // Arrow for selected item
        if (selected) {
            fb.drawString(kW - 8, iy + 2, ">", 1, /*invert=*/true);
        }
    }

    // Scrollbar
    if (kCount > 4) {
        float pos = (float)sel_ / (float)(kCount - 1);
        fb.drawScrollbar(kW - 2, kContentY, kH - kContentY, pos, 4.f/(float)kCount);
    }
}

// ═════════════════════════════════════════════════════════════
//  Screen: Joints
// ═════════════════════════════════════════════════════════════
JointsScreen::JointsScreen(NavCb nav) : nav_(std::move(nav)) {}

void JointsScreen::onEnter() { top_ = 0; soff_ = tsoff_ = 0.f; }

void JointsScreen::setData(const JointState& d) {
    std::lock_guard<std::mutex> lk(mtx_);
    data_ = d;
}

void JointsScreen::onInput(Ev ev) {
    std::lock_guard<std::mutex> lk(mtx_);
    int n = (int)data_.joints.size();
    switch (ev) {
    case Ev::EncCW:
        if (top_ < std::max(0, n - 4)) {
            ++top_;
            tsoff_ = (float)(top_ * kRowH);
        }
        break;
    case Ev::EncCCW:
        if (top_ > 0) {
            --top_;
            tsoff_ = (float)(top_ * kRowH);
        }
        break;
    case Ev::Back:
        if (nav_) nav_(ScreenID::Menu, -1);
        break;
    default: break;
    }
}

void JointsScreen::update(float dt) {
    soff_ = lerp(soff_, tsoff_, std::min(1.f, dt * 18.f));
}

void JointsScreen::render(FrameBuffer& fb) const {
    fb.clear();
    std::lock_guard<std::mutex> lk(mtx_);

    drawTitleBar(fb, "JOINTS", 1, 5);

    int n = (int)data_.joints.size();
    if (n == 0) {
        fb.drawStringCenter(28, "No data", 1);
        return;
    }

    // Content area starts at y=11
    constexpr int kY0   = 11;
    constexpr int kBarH = 3;

    int y_offset = (int)soff_;

    for (int i = 0; i < n; ++i) {
        int iy = kY0 + i * kRowH - y_offset;
        if (iy < kY0 - kRowH || iy > kH) continue;

        const auto& j = data_.joints[i];

        // Joint name (truncated to 3 chars)
        std::string nm = j.name.substr(0, 3);
        fb.drawString(0, iy, nm, 1);

        // Position value
        char posbuf[12];
        snprintf(posbuf, sizeof(posbuf), "%+.2f", j.pos_rad);
        fb.drawString(20, iy, posbuf, 1);

        // Temperature + small bar
        char tmpbuf[10];
        snprintf(tmpbuf, sizeof(tmpbuf), "%.1f", j.temp_c);
        fb.drawString(68, iy, tmpbuf, 1);
        fb.drawString(100, iy, "\xC2", 1); // degree-like char -> use 'o'
        fb.drawChar  (100, iy, 'o', 1);
        fb.drawChar  (106, iy, 'C', 1);

        // Temperature bar (full row width in bottom 3px of row)
        float tpct = std::clamp((j.temp_c - 20.f) / 60.f, 0.f, 1.f);
        fb.drawProgressBar(0, iy + kRowH - kBarH - 1, kW - 4, kBarH, tpct);
    }

    // Scrollbar
    if (n > 4) {
        float pos = (float)top_ / (float)(n - 1);
        fb.drawScrollbar(kW - 2, kY0, kH - kY0, pos, 4.f / (float)n);
    }
}

// ═════════════════════════════════════════════════════════════
//  Screen: IMU
// ═════════════════════════════════════════════════════════════
IMUScreen::IMUScreen(NavCb nav) : nav_(std::move(nav)) {}

void IMUScreen::setData(const IMUState& d) {
    std::lock_guard<std::mutex> lk(mtx_);
    data_ = d;
}

void IMUScreen::onInput(Ev ev) {
    if (ev == Ev::Back && nav_) nav_(ScreenID::Menu, -1);
}

void IMUScreen::update(float dt) {
    float tx, ty;
    {
        std::lock_guard<std::mutex> lk(mtx_);
        // Map roll/pitch to bubble offset (max ±20 deg → ±14px radius)
        tx = std::clamp(data_.roll  / 30.f, -1.f, 1.f) * 14.f;
        ty = std::clamp(data_.pitch / 30.f, -1.f, 1.f) * 14.f;
    }
    float k = std::min(1.f, dt * 10.f);
    sdot_x_ = lerp(sdot_x_, tx, k);
    sdot_y_ = lerp(sdot_y_, ty, k);

    // Trail – record every ~50ms (simple counter)
    static float tr_t = 0.f;
    tr_t += dt;
    if (tr_t >= 0.05f) {
        tr_t = 0.f;
        trail_.push_back({(int8_t)sdot_x_, (int8_t)sdot_y_});
        if (trail_.size() > 12) trail_.pop_front();
    }
}

void IMUScreen::render(FrameBuffer& fb) const {
    fb.clear();
    drawTitleBar(fb, "IMU", 2, 5);

    IMUState d;
    {
        std::lock_guard<std::mutex> lk(mtx_);
        d = data_;
    }

    // ── Left: values ─────────────────────────────────────
    // Gyro row 1
    char buf[24];
    snprintf(buf, sizeof(buf), "Gx%+5.1f", d.gx);
    fb.drawString(0, 11, buf, 1);
    snprintf(buf, sizeof(buf), "Gy%+5.1f", d.gy);
    fb.drawString(0, 20, buf, 1);
    snprintf(buf, sizeof(buf), "Gz%+5.1f", d.gz);
    fb.drawString(0, 29, buf, 1);

    // Euler
    snprintf(buf, sizeof(buf), "R%+5.1f", d.roll);
    fb.drawString(0, 40, buf, 1);
    snprintf(buf, sizeof(buf), "P%+5.1f", d.pitch);
    fb.drawString(0, 49, buf, 1);
    snprintf(buf, sizeof(buf), "Y%+5.1f", d.yaw);
    fb.drawString(0, 58, buf, 1);

    // ── Right: bubble level ───────────────────────────────
    // Center of bubble indicator
    constexpr int cx = 100, cy = 38, r = 20;
    fb.drawCircle(cx, cy, r);
    // Crosshair
    fb.drawHLine(cx - 4, cy, 9);
    fb.drawVLine(cx, cy - 4, 9);

    // Trail dots (fading = earlier = smaller)
    int tidx = 0;
    for (auto& [tx, ty] : trail_) {
        bool big = (tidx > (int)trail_.size() - 4);
        int px = cx + (int)tx, py = cy + (int)ty;
        if (big) fb.drawCircle(px, py, 2);
        else     fb.setPixel(px, py);
        ++tidx;
    }

    // Current dot
    int dx = cx + (int)sdot_x_, dy = cy + (int)sdot_y_;
    fb.fillCircle(dx, dy, 3);

    // Separator
    fb.drawVLine(70, 11, kH - 11);
}

// ═════════════════════════════════════════════════════════════
//  Screen: Joystick
// ═════════════════════════════════════════════════════════════
JoystickScreen::JoystickScreen(NavCb nav) : nav_(std::move(nav)) {}

void JoystickScreen::setData(const JoystickState& d) {
    std::lock_guard<std::mutex> lk(mtx_);
    data_ = d;
}

void JoystickScreen::onInput(Ev ev) {
    if (ev == Ev::Back && nav_) nav_(ScreenID::Menu, -1);
}

void JoystickScreen::update(float dt) {
    JoystickState d;
    { std::lock_guard<std::mutex> lk(mtx_); d = data_; }
    float k = std::min(1.f, dt * 15.f);
    slx_ = lerp(slx_, d.lx, k);
    sly_ = lerp(sly_, d.ly, k);
    srx_ = lerp(srx_, d.rx, k);
    sry_ = lerp(sry_, d.ry, k);
}

void JoystickScreen::render(FrameBuffer& fb) const {
    fb.clear();
    drawTitleBar(fb, "JOYSTICK", 3, 5);

    JoystickState d;
    { std::lock_guard<std::mutex> lk(mtx_); d = data_; }

    // ── Left stick (center 23,38, r=18) ─────────────────
    constexpr int lx = 23, ly = 38, sr = 17;
    fb.drawCircle(lx, ly, sr);
    fb.drawHLine(lx - 3, ly, 7);
    fb.drawVLine(lx, ly - 3, 7);
    int ldx = lx + (int)(slx_ * (sr - 3));
    int ldy = ly + (int)(sly_ * (sr - 3));
    fb.fillCircle(ldx, ldy, 3);

    // ── Right stick (center 78,38, r=17) ─────────────────
    constexpr int rx = 78, ry = 38;
    fb.drawCircle(rx, ry, sr);
    fb.drawHLine(rx - 3, ry, 7);
    fb.drawVLine(rx, ry - 3, 7);
    int rdx = rx + (int)(srx_ * (sr - 3));
    int rdy = ry + (int)(sry_ * (sr - 3));
    fb.fillCircle(rdx, rdy, 3);

    // ── Buttons (right side, x=110) ──────────────────────
    // Y
    fb.drawString(110, 13, "Y", 1);
    if (d.btn_y) fb.invertRect(109, 12, 7, 9);
    // B (right)
    fb.drawString(119, 20, "B", 1);
    if (d.btn_b) fb.invertRect(118, 19, 7, 9);
    // X (left)
    fb.drawString(101, 20, "X", 1);
    if (d.btn_x) fb.invertRect(100, 19, 7, 9);
    // A (bottom)
    fb.drawString(110, 27, "A", 1);
    if (d.btn_a) fb.invertRect(109, 26, 7, 9);

    // LB/RB
    std::string lb = d.btn_lb ? "[LB]" : " LB ";
    std::string rb = d.btn_rb ? "[RB]" : " RB ";
    fb.drawString(0, 11, lb, 1);
    fb.drawString(50, 11, rb, 1);

    // Start / Select
    fb.drawString(43, 56, d.btn_start  ? "[ST]" : " ST ", 1);
    fb.drawString(80, 56, d.btn_select ? "[SE]" : " SE ", 1);
}

// ═════════════════════════════════════════════════════════════
//  Screen: AimRT Log
// ═════════════════════════════════════════════════════════════
LogScreen::LogScreen(NavCb nav) : nav_(std::move(nav)) {}

void LogScreen::onEnter() { auto_scroll_ = true; }

void LogScreen::addLog(const LogEntry& e) {
    std::lock_guard<std::mutex> lk(mtx_);
    logs_.push_back(e);
    if ((int)logs_.size() > kMaxLogs) logs_.pop_front();
    if (auto_scroll_) {
        int n = (int)logs_.size();
        top_   = std::max(0, n - 4);
        tsoff_ = (float)(top_ * kRowH);
        soff_  = tsoff_;
    }
}

void LogScreen::clear() {
    std::lock_guard<std::mutex> lk(mtx_);
    logs_.clear();
    top_ = 0; soff_ = tsoff_ = 0.f;
}

void LogScreen::onInput(Ev ev) {
    std::lock_guard<std::mutex> lk(mtx_);
    int n = (int)logs_.size();
    switch (ev) {
    case Ev::EncCW:
        if (top_ < std::max(0, n - 4)) {
            ++top_;
            tsoff_      = (float)(top_ * kRowH);
            auto_scroll_= (top_ >= n - 4);
        }
        break;
    case Ev::EncCCW:
        if (top_ > 0) {
            --top_;
            tsoff_      = (float)(top_ * kRowH);
            auto_scroll_= false;
        }
        break;
    case Ev::Back:
        if (nav_) nav_(ScreenID::Menu, -1);
        break;
    default: break;
    }
}

void LogScreen::update(float dt) {
    soff_   = lerp(soff_, tsoff_, std::min(1.f, dt * 18.f));
    blink_t_= std::fmod(blink_t_ + dt * 2.f, 2.f * 3.14159f);
}

void LogScreen::render(FrameBuffer& fb) const {
    fb.clear();

    std::lock_guard<std::mutex> lk(mtx_);
    int n = (int)logs_.size();

    // Title bar with live blink dot
    drawTitleBar(fb, "AIMRT LOG", 4, 5);
    if (n > 0 && auto_scroll_) {
        // blinking dot: ● when live
        bool blink_on = (std::sin(blink_t_) > 0.f);
        if (blink_on) fb.fillCircle(kW - 4, 4, 2);
    }

    if (n == 0) {
        fb.drawStringCenter(28, "No logs", 1);
        return;
    }

    constexpr int kY0 = 11;
    int y_offset = (int)soff_;

    for (int i = 0; i < n; ++i) {
        int iy = kY0 + i * kRowH - y_offset;
        if (iy < kY0 - kRowH || iy > kH) continue;

        const auto& e = logs_[i];

        // Level indicator
        const char* lvl = (e.level == LogEntry::WARN) ? "W" :
                          (e.level == LogEntry::ERR)  ? "E" : "I";
        bool is_err  = (e.level == LogEntry::ERR);
        bool is_warn = (e.level == LogEntry::WARN);

        fb.drawChar(0, iy + 1, lvl[0], 1, is_err || is_warn);

        // Module (max 4 chars)
        std::string mod = e.module.substr(0, 4);
        fb.drawString(8, iy + 1, mod, 1);

        // Message (remaining width)
        int msg_x = 8 + (int)mod.size() * 6 + 2;
        int max_chars = (kW - 4 - msg_x) / 6;
        std::string msg = e.msg.substr(0, std::min((int)e.msg.size(), max_chars));
        fb.drawString(msg_x, iy + 1, msg, 1);

        // Underline separator
        fb.drawHLine(0, iy + kRowH - 1, kW - 4);
    }

    // Scrollbar
    if (n > 4) {
        float pos = (float)top_ / (float)std::max(1, n - 4);
        fb.drawScrollbar(kW - 2, kY0, kH - kY0, pos, 4.f / (float)n);
    }
}

// ═════════════════════════════════════════════════════════════
//  Screen: SBC Status
// ═════════════════════════════════════════════════════════════
SBCScreen::SBCScreen(NavCb nav) : nav_(std::move(nav)) {}

void SBCScreen::setData(const SBCStatus& d) {
    std::lock_guard<std::mutex> lk(mtx_);
    data_ = d;
    anim_core_.resize(d.core_pct.size(), 0.f);
}

void SBCScreen::onInput(Ev ev) {
    if (ev == Ev::Back && nav_) nav_(ScreenID::Menu, -1);
}

void SBCScreen::update(float dt) {
    SBCStatus d;
    { std::lock_guard<std::mutex> lk(mtx_); d = data_; }
    anim_t_ += dt;

    float k = std::min(1.f, dt * 8.f);
    anim_core_.resize(d.core_pct.size(), 0.f);
    for (size_t i = 0; i < d.core_pct.size(); ++i)
        anim_core_[i] = lerp(anim_core_[i], d.core_pct[i] / 100.f, k);

    float rp = (d.ram_total_mb > 0.f) ? d.ram_used_mb / d.ram_total_mb : 0.f;
    anim_ram_ = lerp(anim_ram_, rp, k);
}

void SBCScreen::render(FrameBuffer& fb) const {
    fb.clear();

    SBCStatus d;
    { std::lock_guard<std::mutex> lk(mtx_); d = data_; }

    drawTitleBar(fb, "SBC STATUS", 5, 5);

    // CPU temp on right of title bar (overwrite)
    char tbuf[12];
    snprintf(tbuf, sizeof(tbuf), "%.1foC", d.cpu_temp);
    fb.drawStringRight(kW - 1, 1, tbuf, 1);
    int tw = (int)std::strlen(tbuf) * FrameBuffer::charW(1);
    fb.invertRect(kW - 1 - tw, 0, tw, 9);

    // ── CPU cores (2 columns) ────────────────────────────
    // Content y=11, each bar row = 12px
    // Left  col: cores 0,1,2,3  x=0
    // Right col: cores 4,5,6,7  x=66
    int ncores = (int)std::min((int)anim_core_.size(), 8);
    constexpr int kBarW = 54, kBarH = 5, kLabelW = 12;

    for (int i = 0; i < ncores; ++i) {
        int col   = i >= 4 ? 1 : 0;
        int row   = i >= 4 ? i - 4 : i;
        int bx    = col * 64 + kLabelW;
        int by    = 11 + row * 13;
        int lx    = col * 64;

        // Core label
        char lbuf[4];
        snprintf(lbuf, sizeof(lbuf), "C%d", i);
        fb.drawString(lx, by + 1, lbuf, 1);

        // Animated bar
        float p = (i < (int)anim_core_.size()) ? anim_core_[i] : 0.f;
        fb.drawProgressBar(bx, by, kBarW, kBarH + 2, p);

        // Percentage text under bar
        int pct = (int)(p * 100.f);
        char pbuf[5]; snprintf(pbuf, sizeof(pbuf), "%3d%%", pct);
        fb.drawString(bx + kBarW - 24, by + kBarH + 3, pbuf, 1);
    }

    // ── RAM bar ──────────────────────────────────────────
    int ram_y = 11 + 4 * 13 + 2;  // below 4 rows
    char rbuf[24];
    snprintf(rbuf, sizeof(rbuf), "RAM %.0f/%.0fMB",
             d.ram_used_mb, d.ram_total_mb);
    fb.drawString(0, ram_y, rbuf, 1);
    fb.drawProgressBar(0, ram_y + 9, kW - 1, 5, anim_ram_);
}

// ═════════════════════════════════════════════════════════════
//  UIManager
// ═════════════════════════════════════════════════════════════
UIManager::UIManager(Config cfg) : cfg_(std::move(cfg)) {
    auto nav = [this](ScreenID id, int dir) { navigate(id, dir); };

    menu_     = std::make_unique<MenuScreen>    (nav);
    joints_   = std::make_unique<JointsScreen>  (nav);
    imu_      = std::make_unique<IMUScreen>      (nav);
    joystick_ = std::make_unique<JoystickScreen>(nav);
    log_      = std::make_unique<LogScreen>     (nav);
    sbc_      = std::make_unique<SBCScreen>     (nav);

    screens_[static_cast<int>(ScreenID::Menu)]     = menu_.get();
    screens_[static_cast<int>(ScreenID::Joints)]   = joints_.get();
    screens_[static_cast<int>(ScreenID::IMU)]      = imu_.get();
    screens_[static_cast<int>(ScreenID::Joystick)] = joystick_.get();
    screens_[static_cast<int>(ScreenID::LogInfo)]  = log_.get();
    screens_[static_cast<int>(ScreenID::SBC)]      = sbc_.get();

    cur_ = menu_.get();
}

UIManager::~UIManager() { stop(); }

bool UIManager::init() {
    disp_  = std::make_unique<OledDisplay>(cfg_.i2c_dev, cfg_.oled_addr);
    input_ = std::make_unique<InputHandler>(cfg_.input);

    if (!disp_->init())  return false;
    if (!input_->init()) return false;

    input_->setCallback([this](Ev ev) {
        if (cur_) cur_->onInput(ev);
    });
    input_->start();

    cur_->onEnter();
    return true;
}

void UIManager::navigate(ScreenID id, int dir) {
    UIScreen* next = screens_[static_cast<int>(id)];
    if (!next || next == cur_) return;

    // Capture current frame as "previous" for transition
    if (cur_) cur_->render(fb_prev_);

    if (cur_) cur_->onExit();
    prev_       = cur_;
    cur_        = next;
    in_trans_   = true;
    trans_t_    = 0.f;
    trans_dir_  = dir;
    cur_->onEnter();
}

void UIManager::tick(float dt) {
    if (cur_) cur_->update(dt);

    if (in_trans_) {
        trans_t_ += dt;
        float t = std::min(trans_t_ / trans_dur_, 1.f);
        float et = easeOut(t);

        // Current screen renders to fb_cur_
        if (cur_) cur_->render(fb_cur_);

        // Offsets
        int new_dx, old_dx;
        if (trans_dir_ > 0) {  // forward: new comes from right
            new_dx = (int)((1.f - et) * kW);
            old_dx = -(int)(et * kW);
        } else {               // backward: new comes from left
            new_dx = -(int)((1.f - et) * kW);
            old_dx = (int)(et * kW);
        }

        // Composite: old shifted + new shifted (OR)
        fb_final_.clear();
        FrameBuffer tmp;
        tmp.clear();
        tmp.blitShifted(fb_prev_, old_dx);
        fb_final_.blitShifted(fb_cur_, new_dx);
        // Merge by OR-ing raw bytes
        const uint8_t* a = fb_final_.raw();
        const uint8_t* b = tmp.raw();
        // We need writable access – use blitShifted result already in fb_final_
        // Re-do: blit old into fb_final first, then OR new
        fb_final_.blitShifted(fb_prev_, old_dx);
        tmp.blitShifted(fb_cur_, new_dx);
        {
            auto* fd = const_cast<uint8_t*>(fb_final_.raw());
            const uint8_t* ts = tmp.raw();
            for (int i = 0; i < kBufLen; ++i) fd[i] |= ts[i];
        }

        disp_->flush(fb_final_);

        if (t >= 1.f) in_trans_ = false;
    } else {
        if (cur_) {
            cur_->render(fb_cur_);
            disp_->flush(fb_cur_);
        }
    }
}

void UIManager::run() {
    using namespace std::chrono;
    running_.store(true);
    auto frame_dur = microseconds(1'000'000 / cfg_.fps);
    auto last = steady_clock::now();

    while (running_.load()) {
        auto now  = steady_clock::now();
        float dt  = duration_cast<microseconds>(now - last).count() / 1'000'000.f;
        last      = now;
        dt        = std::clamp(dt, 0.001f, 0.1f);

        tick(dt);

        auto elapsed = steady_clock::now() - now;
        if (elapsed < frame_dur)
            std::this_thread::sleep_for(frame_dur - elapsed);
    }
}

void UIManager::stop() {
    running_.store(false);
    if (input_) input_->stop();
}

// ── Thread-safe data setters ──────────────────────────────────
void UIManager::setRobotIP      (const std::string& ip) { menu_->setIP(ip); }
void UIManager::setJointState   (const JointState& d)   { joints_->setData(d); }
void UIManager::setIMUState     (const IMUState& d)      { imu_->setData(d); }
void UIManager::setJoystickState(const JoystickState& d) { joystick_->setData(d); }
void UIManager::addLog          (const LogEntry& e)      { log_->addLog(e); }
void UIManager::setSBCStatus    (const SBCStatus& d)     { sbc_->setData(d); }

} // namespace oled