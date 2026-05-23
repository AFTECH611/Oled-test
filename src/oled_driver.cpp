/**
 * @file  oled_driver.cpp
 * @brief OLED UI driver implementation – C++20 / GCC 13
 */

#include "oled_driver.hpp"

#include <algorithm>
#include <bit>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <stdexcept>

#include <fcntl.h>
#include <gpiod.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace oled {

// ─────────────────────────────────────────────────────────────
//  Internal helpers
// ─────────────────────────────────────────────────────────────
namespace detail {

[[nodiscard]] constexpr float lerp(float a, float b, float t) noexcept {
    return a + t * (b - a);
}

[[nodiscard]] constexpr float easeOut(float t) noexcept {
    t = std::clamp(t, 0.f, 1.f);
    return 1.f - (1.f - t) * (1.f - t) * (1.f - t);
}

[[nodiscard]] int64_t nowMs() noexcept {
    using namespace std::chrono;
    return duration_cast<milliseconds>(
        steady_clock::now().time_since_epoch()).count();
}

/// printf-style formatting into a std::string (C++17 replacement for std::format)
[[nodiscard]] std::string sfmt(const char* fmt, ...) {
    char buf[128];
    va_list ap;
    va_start(ap, fmt);
    std::vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    return buf;
}

/// Format a float: width + precision (C++17 replacement for std::format float)
[[nodiscard]] std::string fmtF(float v, int prec = 2, int total_w = 0) {
    char buf[64];
    if (total_w > 0)
        std::snprintf(buf, sizeof(buf), "%*.*f", total_w, prec, v);
    else
        std::snprintf(buf, sizeof(buf), "%.*f", prec, v);
    return buf;
}

} // namespace detail

// ─────────────────────────────────────────────────────────────
//  5×7 ASCII font  (columns, bit-0 = top row)
//  Characters 0x20 – 0x7E  (95 glyphs)
// ─────────────────────────────────────────────────────────────
static constexpr std::array<std::array<uint8_t, 5>, 95> kFont5x7 {{
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
}};

// ═════════════════════════════════════════════════════════════
//  FrameBuffer
// ═════════════════════════════════════════════════════════════
FrameBuffer::FrameBuffer() { buf_.fill(0); }

void FrameBuffer::clear() { buf_.fill(0); }

void FrameBuffer::setPixel(int x, int y, bool on) {
    if (x < 0 || x >= kW || y < 0 || y >= kH) return;
    const int idx = (y >> 3) * kW + x;
    const auto bit = static_cast<uint8_t>(1u << (y & 7));
    if (on) buf_[idx] |=  bit;
    else    buf_[idx] &= ~bit;
}

bool FrameBuffer::getPixel(int x, int y) const {
    if (x < 0 || x >= kW || y < 0 || y >= kH) return false;
    return (buf_[(y >> 3) * kW + x] >> (y & 7)) & 1u;
}

void FrameBuffer::drawHLine(int x, int y, int w) {
    for (int i = 0; i < w; ++i) setPixel(x + i, y);
}

void FrameBuffer::drawVLine(int x, int y, int h) {
    for (int i = 0; i < h; ++i) setPixel(x, y + i);
}

void FrameBuffer::drawLine(int x0, int y0, int x1, int y1) {
    int dx  =  std::abs(x1 - x0);
    int dy  = -std::abs(y1 - y0);
    int sx  = x0 < x1 ? 1 : -1;
    int sy  = y0 < y1 ? 1 : -1;
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
    drawHLine(x, y,         w);
    drawHLine(x, y + h - 1, w);
    drawVLine(x,         y, h);
    drawVLine(x + w - 1, y, h);
}

void FrameBuffer::fillRect(int x, int y, int w, int h) {
    for (int row = y; row < y + h; ++row)
        drawHLine(x, row, w);
}

void FrameBuffer::drawCircle(int cx, int cy, int r) {
    int xi = 0, yi = r, d = 3 - 2 * r;
    auto plot8 = [&](int a, int b) {
        setPixel(cx + a, cy + b); setPixel(cx - a, cy + b);
        setPixel(cx + a, cy - b); setPixel(cx - a, cy - b);
        setPixel(cx + b, cy + a); setPixel(cx - b, cy + a);
        setPixel(cx + b, cy - a); setPixel(cx - b, cy - a);
    };
    while (yi >= xi) {
        plot8(xi, yi);
        if (d < 0) d += 4 * xi + 6;
        else { d += 4 * (xi - yi) + 10; --yi; }
        ++xi;
    }
}

void FrameBuffer::fillCircle(int cx, int cy, int r) {
    for (int yi = -r; yi <= r; ++yi) {
        int hw = static_cast<int>(std::sqrt(static_cast<float>(r * r - yi * yi)));
        drawHLine(cx - hw, cy + yi, 2 * hw + 1);
    }
}

void FrameBuffer::invertRect(int x, int y, int w, int h) {
    for (int row = y; row < y + h && row < kH; ++row)
        for (int col = x; col < x + w && col < kW; ++col) {
            const int idx = (row >> 3) * kW + col;
            buf_[idx] ^= static_cast<uint8_t>(1u << (row & 7));
        }
}

// ── Text ──────────────────────────────────────────────────────
int FrameBuffer::drawChar(int x, int y, char c, int scale, bool invert) {
    if (c < 0x20 || c > 0x7E) c = '?';
    const auto& glyph = kFont5x7[static_cast<uint8_t>(c) - 0x20];

    for (int col = 0; col < 5; ++col) {
        const uint8_t bits = glyph[col];
        for (int row = 0; row < 7; ++row) {
            bool on = static_cast<bool>((bits >> row) & 1u);
            if (invert) on = !on;
            for (int sy = 0; sy < scale; ++sy)
                for (int sx = 0; sx < scale; ++sx)
                    setPixel(x + col * scale + sx, y + row * scale + sy, on);
        }
    }
    if (invert) {
        for (int row = 0; row < 7 * scale; ++row)
            for (int sx = 0; sx < scale; ++sx)
                setPixel(x + 5 * scale + sx, y + row, true);
    }
    return 6 * scale;
}

int FrameBuffer::drawString(int x, int y, std::string_view s,
                            int scale, bool invert) {
    int cx = x;
    for (char c : s) cx += drawChar(cx, y, c, scale, invert);
    return cx - x;
}

void FrameBuffer::drawStringCenter(int y, std::string_view s,
                                   int scale, bool invert) {
    const int tw = static_cast<int>(s.size()) * charW(scale);
    drawString((kW - tw) / 2, y, s, scale, invert);
}

void FrameBuffer::drawStringRight(int xr, int y, std::string_view s,
                                  int scale) {
    const int tw = static_cast<int>(s.size()) * charW(scale);
    drawString(xr - tw, y, s, scale);
}

// ── Widgets ───────────────────────────────────────────────────
void FrameBuffer::drawProgressBar(int x, int y, int w, int h, float pct) {
    pct = std::clamp(pct, 0.f, 1.f);
    drawRect(x, y, w, h);
    if (const int filled = static_cast<int>((w - 2) * pct); filled > 0)
        fillRect(x + 1, y + 1, filled, h - 2);
}

void FrameBuffer::drawScrollbar(int x, int y, int h,
                                float pos, float vis) {
    vis = std::clamp(vis, 0.1f, 1.f);
    const int bar_h = static_cast<int>(h * vis);
    const int bar_y = y + static_cast<int>((h - bar_h) * pos);
    drawVLine(x, y, h);
    fillRect(x, bar_y, 2, bar_h);
}

// ── Transition blit ───────────────────────────────────────────
void FrameBuffer::blitShifted(const FrameBuffer& src, int dx) {
    // [FIX-7] Must take a reference to the actual buffer, not a value-copy.
    //         The original "auto dst_span = rawMut()" copied the array by value,
    //         so all writes went into the copy and were discarded – the transition
    //         animation produced a black screen instead of sliding panels.
    auto& dst_span = rawMut();
    const auto& src_span = src.raw();

    for (int page = 0; page < kPages; ++page) {
        uint8_t*       dst = dst_span.data() + page * kW;
        const uint8_t* sr  = src_span.data() + page * kW;

        if (dx == 0) {
            std::memcpy(dst, sr, kW);
        } else if (dx > 0 && dx < kW) {
            std::memset(dst, 0, dx);
            std::memcpy(dst + dx, sr, kW - dx);
        } else if (dx < 0 && dx > -kW) {
            const int shift = -dx;
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
OledDisplay::OledDisplay(std::string_view dev, uint8_t addr)
    : addr_(addr), dev_(dev) {}

OledDisplay::~OledDisplay() {
    if (fd_ >= 0) ::close(fd_);
}

bool OledDisplay::cmd(uint8_t c) {
    const std::array<uint8_t, 2> buf = {0x00, c};
    return ::write(fd_, buf.data(), buf.size()) == static_cast<ssize_t>(buf.size());
}

bool OledDisplay::cmds(std::initializer_list<uint8_t> cs) {
    for (uint8_t c : cs)
        if (!cmd(c)) return false;
    return true;
}

bool OledDisplay::init() {
    fd_ = ::open(dev_.c_str(), O_RDWR);
    if (fd_ < 0) return false;
    if (::ioctl(fd_, I2C_SLAVE, addr_) < 0) return false;

    // SSD1306 init sequence
    static constexpr std::array<uint8_t, 25> kInitSeq = {{
        0xAE,        // display off
        0xD5, 0x80,  // clock divide ratio / oscillator frequency
        0xA8, 0x3F,  // multiplex ratio (64MUX)
        0xD3, 0x00,  // display offset = 0
        0x40,        // start line = 0
        0x8D, 0x14,  // charge pump: enable
        0xA1,        // segment remap (flip x)
        0xC8,        // COM scan direction (flip y)
        0xDA, 0x12,  // COM pins hardware config
        0x81, 0xCF,  // contrast
        0xD9, 0xF1,  // pre-charge period
        0xDB, 0x40,  // VCOM deselect level
        0xA4,        // entire display ON (follow RAM)
        0xA6,        // normal display (not inverted)
        0x20, 0x00,  // horizontal addressing mode
        0xAF,        // display ON
    }};
    for (uint8_t c : kInitSeq) if (!cmd(c)) return false;

    FrameBuffer blank;
    flush(blank);
    return true;
}

bool OledDisplay::writePage(int page, const uint8_t* data) {
    // [FIX-6] SH1106 has a built-in 2-pixel column offset (unlike SSD1306).
    //         Without this correction the display shifts 2 pixels to the left
    //         and the rightmost 2 columns wrap around to appear at the left.
    //         Column 2 (0x02 low nibble + 0x10 high nibble) is the true origin.
    cmd(static_cast<uint8_t>(0xB0 | page));  // set page address
    cmd(0x02);                                // low  nibble of col start = 2
    cmd(0x10);                                // high nibble of col start = 0

    std::array<uint8_t, 129> buf;
    buf[0] = 0x40;  // data mode
    std::copy(data, data + 128, buf.begin() + 1);
    return ::write(fd_, buf.data(), buf.size()) == static_cast<ssize_t>(buf.size());
}

void OledDisplay::flush(const FrameBuffer& fb) {
    const auto& raw = fb.raw();
    for (int p = 0; p < kPages; ++p) {
        writePage(p, raw.data() + p * kW);
    }
}

void OledDisplay::setContrast(uint8_t c) { cmds({0x81, c}); }
void OledDisplay::setOn(bool on)         { cmd(on ? 0xAF : 0xAE); }

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
    // Non-copyable
    GpioHandle()                             = default;
    GpioHandle(const GpioHandle&)            = delete;
    GpioHandle& operator=(const GpioHandle&) = delete;
};

static bool openGpio(const GpioPin& pin, InputHandler::GpioHandle& h,
                     const char* consumer) {
    h.chip = gpiod_chip_open(std::string{pin.chip}.c_str());
    if (!h.chip) return false;
    h.line = gpiod_chip_get_line(h.chip, pin.line);
    if (!h.line) return false;
    // Try pull-up first (libgpiod >= 1.5), fallback to plain input
    int rc = gpiod_line_request_input_flags(
        h.line, consumer, GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP);
    if (rc < 0) rc = gpiod_line_request_input(h.line, consumer);
    return rc >= 0;
}

// [FIX-2] Encoder CW/CCW lookup table – index = (prev_AB << 2) | curr_AB
// Gray-code quadrature:  CW  = 00->01->11->10->00
//                        CCW = 00->10->11->01->00
// +1 = CW step, -1 = CCW step, 0 = invalid/no-change
// The original table had swapped entries that produced wrong direction
// detection and missed detents on most encoders.
static constexpr std::array<int8_t, 16> kEncTable = {{
//  curr: 00   01   10   11    prev
          0,  -1,  +1,   0,  // 00
         +1,   0,   0,  -1,  // 01
         -1,   0,   0,  +1,  // 10
          0,  +1,  -1,   0,  // 11
}};

InputHandler::InputHandler(Config cfg) : cfg_(std::move(cfg)) {}
InputHandler::~InputHandler() { stop(); }

bool InputHandler::init() {
    static constexpr const char* kConsumer = "oled_ui";
    const std::array<GpioPin, 5> pins = {{
        cfg_.enc_a, cfg_.enc_b, cfg_.enc_btn, cfg_.confirm, cfg_.back
    }};
    gpios_.clear();
    for (const auto& pin : pins) {
        auto h = std::make_unique<GpioHandle>();
        if (!openGpio(pin, *h, kConsumer)) return false;
        gpios_.push_back(std::move(h));
    }
    const int a = gpiod_line_get_value(gpios_[0]->line);
    const int b = gpiod_line_get_value(gpios_[1]->line);
    enc_prev_ = static_cast<uint8_t>((a << 1) | b);
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
    std::lock_guard lk{q_mtx_};
    if (queue_.empty()) return Ev::None;
    Ev ev = queue_.front();
    queue_.pop_front();
    return ev;
}

void InputHandler::push(Ev ev) {
    // [FIX-1] Acquire lock, enqueue, then RELEASE before calling cb_.
    //         Original code held the lock while calling cb_(), which ran
    //         onInput() on the render thread – potential deadlock and jank.
    // [FIX-5] Removed printf/fflush from inside the lock (and entirely).
    //         They were serialising the 1 ms poll loop, causing visible lag.
    {
        std::lock_guard lk{q_mtx_};
        if (queue_.size() < 16) queue_.push_back(ev);
    }
    // cb_ is safe to call without the lock: it was set before start() [FIX-4]
    // and is only read/called from this thread from that point on.
    if (cb_) cb_(ev);
}

int InputHandler::readLine(std::size_t idx) const {
    return gpiod_line_get_value(gpios_[idx]->line);
}

void InputHandler::loop() {
    using namespace std::chrono;

    while (running_.load(std::memory_order_relaxed)) {

        // ====================================================
        // Rotary Encoder (Quadrature Decoder)
        // ====================================================

        const int a = readLine(0);
        const int b = readLine(1);

        // If direction reversed:
        // swap a/b below
        const uint8_t curr =
            static_cast<uint8_t>((a << 1) | b);

        if (curr != enc_prev_) {
            const int8_t dir = kEncTable[(enc_prev_ << 2) | curr];
            if (dir != 0) {
                enc_accum_ += dir;

                // [FIX-3] Threshold = 2, not 4.
                // Most encoders produce 2 transitions per detent (A then B or
                // vice-versa). With threshold=4 you need TWO full detents before
                // an event fires, making the encoder feel sluggish/non-responsive.
                if (enc_accum_ >= 2) {
                    push(Ev::EncCW);
                    enc_accum_ = 0;
                } else if (enc_accum_ <= -2) {
                    push(Ev::EncCCW);
                    enc_accum_ = 0;
                }
            }
            enc_prev_ = curr;
        }

        // ====================================================
        // Buttons
        // ====================================================

        const int64_t now = detail::nowMs();

        auto handleBtn =
            [&](BtnState& bs,
                std::size_t gpio_idx,
                Ev short_ev,
                Ev long_ev)
        {
            const bool raw =
                (readLine(gpio_idx) == 0);

            // State changed
            if (raw != bs.last) {

                bs.last = raw;
                bs.change_ms = now;
            }

            // Debounce window
            if ((now - bs.change_ms)
                < cfg_.debounce_ms)
            {
                return;
            }

            // Stable pressed
            if (raw && !bs.stable) {

                bs.stable = true;
                bs.press_ms = now;
                bs.long_fired = false;
            }

            // Stable released
            else if (!raw && bs.stable) {
                bs.stable = false;
                if (!bs.long_fired && short_ev != Ev::None) {
                    push(short_ev);
                }
                bs.press_ms = 0;
            }

            // Long press
            if (raw && bs.stable && !bs.long_fired &&
                long_ev != Ev::None &&
                (now - bs.press_ms) >= cfg_.long_ms)
            {
                push(long_ev);
                bs.long_fired = true;
            }
        };

        // Encoder push button
        handleBtn(
            bs_enc_,
            2,
            Ev::EncPress,
            Ev::None
        );

        // Confirm button
        handleBtn(
            bs_confirm_,
            3,
            Ev::Confirm,
            Ev::ConfirmLong
        );

        // Back button
        handleBtn(
            bs_back_,
            4,
            Ev::Back,
            Ev::BackLong
        );

        std::this_thread::sleep_for(
            milliseconds(cfg_.poll_ms));
    }
}

// ═════════════════════════════════════════════════════════════
//  UIScreen base – common title bar
// ═════════════════════════════════════════════════════════════
void UIScreen::drawTitleBar(FrameBuffer& fb, std::string_view label,
                            int cur_idx, int total) {
    fb.fillRect(0, 0, kW, 9);
    fb.drawString(2, 1, label, 1, /*invert=*/true);

    if (total > 1) {
        const auto idxStr = detail::sfmt("%d/%d", cur_idx, total);
        const int  tw     = static_cast<int>(idxStr.size()) * FrameBuffer::charW();
        fb.drawStringRight(kW - 1, 1, idxStr, 1);
        fb.invertRect(kW - 1 - tw, 0, tw, 9);
    }
    fb.drawHLine(0, 9, kW);
}

// ═════════════════════════════════════════════════════════════
//  Screen: Menu
// ═════════════════════════════════════════════════════════════
const std::array<MenuScreen::Item, MenuScreen::kCount> MenuScreen::kItems = {{
    {ScreenID::Joints,   "Joints",     "[J]"},
    {ScreenID::IMU,      "IMU",        "[I]"},
    {ScreenID::Joystick, "Joystick",   "[G]"},
    {ScreenID::LogInfo,  "AimRT Log",  "[L]"},
    {ScreenID::SBC,      "SBC Status", "[S]"},
}};

MenuScreen::MenuScreen(NavCb nav) : nav_(std::move(nav)) {}

void MenuScreen::onEnter() { pulse_ = 0.f; }

void MenuScreen::onInput(Ev ev) {
    switch (ev) {
    case Ev::EncCW:
        sel_ = (sel_ + 1) % kCount;
        tsy_ = static_cast<float>(sel_ * 12);
        pulse_ = 0.f;
        break;
    case Ev::EncCCW:
        sel_ = (sel_ - 1 + kCount) % kCount;
        tsy_ = static_cast<float>(sel_ * 12);
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
    sy_    = detail::lerp(sy_,  tsy_, std::min(1.f, dt * 20.f));
    pulse_ = std::fmod(pulse_ + dt * 2.f, 2.f * 3.14159265f);
}

void MenuScreen::render(FrameBuffer& fb) const {
    fb.clear();

    // IP bar
    fb.fillRect(0, 0, kW, 9);
    fb.drawString(2, 1, detail::sfmt("IP: %s", ip_.empty() ? "---" : ip_.c_str()),
                  1, /*invert=*/true);
    fb.drawHLine(0, 9, kW);

    constexpr int kItemH    = 12;
    constexpr int kContentY = 11;

    for (int i = 0; i < kCount; ++i) {
        const int iy = kContentY + i * kItemH - static_cast<int>(sy_);
        if (iy < kContentY - kItemH || iy > kH) continue;

        const bool selected = (i == sel_);
        if (selected) {
            fb.fillRect(1, iy - 1, kW - 2, kItemH - 1);
        }
        fb.drawString(2,      iy + 2, kItems[i].icon,  1, selected);
        fb.drawString(16,     iy + 2, kItems[i].label, 1, selected);
        if (selected)
            fb.drawString(kW - 8, iy + 2, ">", 1, /*invert=*/true);
    }

    if (kCount > 4) {
        const float pos = static_cast<float>(sel_) / static_cast<float>(kCount - 1);
        fb.drawScrollbar(kW - 2, kContentY, kH - kContentY,
                         pos, 4.f / static_cast<float>(kCount));
    }
}

// ═════════════════════════════════════════════════════════════
//  Screen: Joints
// ═════════════════════════════════════════════════════════════
JointsScreen::JointsScreen(NavCb nav) : nav_(std::move(nav)) {}

void JointsScreen::onEnter() { top_ = 0; soff_ = tsoff_ = 0.f; }

void JointsScreen::setData(JointState d) {
    std::lock_guard lk{mtx_};
    data_ = std::move(d);
}

void JointsScreen::onInput(Ev ev) {
    std::lock_guard lk{mtx_};
    const int n = static_cast<int>(data_.joints.size());
    switch (ev) {
    case Ev::EncCW:
        if (top_ < std::max(0, n - 4)) {
            ++top_;
            tsoff_ = static_cast<float>(top_ * kRowH);
        }
        break;
    case Ev::EncCCW:
        if (top_ > 0) {
            --top_;
            tsoff_ = static_cast<float>(top_ * kRowH);
        }
        break;
    case Ev::Back:
        if (nav_) nav_(ScreenID::Menu, -1);
        break;
    default: break;
    }
}

void JointsScreen::update(float dt) {
    soff_ = detail::lerp(soff_, tsoff_, std::min(1.f, dt * 18.f));
}

void JointsScreen::render(FrameBuffer& fb) const {
    fb.clear();
    std::lock_guard lk{mtx_};

    drawTitleBar(fb, "JOINTS", 1, 5);

    const int n = static_cast<int>(data_.joints.size());
    if (n == 0) { fb.drawStringCenter(28, "No data"); return; }

    constexpr int kY0   = 11;
    constexpr int kBarH = 3;
    const int     y_off = static_cast<int>(soff_);

    for (int i = 0; i < n; ++i) {
        const int iy = kY0 + i * kRowH - y_off;
        if (iy < kY0 - kRowH || iy > kH) continue;

        const auto& j = data_.joints[i];
        fb.drawString(0,   iy, j.name.substr(0, 3));
        fb.drawString(20,  iy, detail::sfmt("%+.2f", j.pos_rad));
        fb.drawString(68,  iy, detail::sfmt("%.1f", j.temp_c));
        fb.drawChar  (100, iy, 'o');
        fb.drawChar  (106, iy, 'C');

        const float tpct = std::clamp((j.temp_c - 20.f) / 60.f, 0.f, 1.f);
        fb.drawProgressBar(0, iy + kRowH - kBarH - 1, kW - 4, kBarH, tpct);
    }

    if (n > 4) {
        const float pos = static_cast<float>(top_) / static_cast<float>(n - 1);
        fb.drawScrollbar(kW - 2, kY0, kH - kY0, pos, 4.f / static_cast<float>(n));
    }
}

// ═════════════════════════════════════════════════════════════
//  Screen: IMU
// ═════════════════════════════════════════════════════════════
IMUScreen::IMUScreen(NavCb nav) : nav_(std::move(nav)) {}

void IMUScreen::setData(IMUState d) {
    std::lock_guard lk{mtx_};
    data_ = std::move(d);
}

void IMUScreen::onInput(Ev ev) {
    if (ev == Ev::Back && nav_) nav_(ScreenID::Menu, -1);
}

void IMUScreen::update(float dt) {
    float tx, ty;
    {
        std::lock_guard lk{mtx_};
        tx = std::clamp(data_.roll  / 30.f, -1.f, 1.f) * 14.f;
        ty = std::clamp(data_.pitch / 30.f, -1.f, 1.f) * 14.f;
    }
    const float k = std::min(1.f, dt * 10.f);
    sdot_x_ = detail::lerp(sdot_x_, tx, k);
    sdot_y_ = detail::lerp(sdot_y_, ty, k);

    // Trail – record at ~50 ms intervals
    static float tr_t = 0.f;
    tr_t += dt;
    if (tr_t >= 0.05f) {
        tr_t = 0.f;
        trail_.push_back({static_cast<int8_t>(sdot_x_),
                          static_cast<int8_t>(sdot_y_)});
        if (trail_.size() > 12) trail_.pop_front();
    }
}

void IMUScreen::render(FrameBuffer& fb) const {
    fb.clear();
    drawTitleBar(fb, "IMU", 2, 5);

    IMUState d;
    { std::lock_guard lk{mtx_}; d = data_; }

    // Gyro + Euler values (left column)
    fb.drawString(0,  11, detail::sfmt("Gx%+5.1f", d.gx));
    fb.drawString(0,  20, detail::sfmt("Gy%+5.1f", d.gy));
    fb.drawString(0,  29, detail::sfmt("Gz%+5.1f", d.gz));
    fb.drawString(0,  40, detail::sfmt("R%+5.1f",  d.roll));
    fb.drawString(0,  49, detail::sfmt("P%+5.1f",  d.pitch));
    fb.drawString(0,  58, detail::sfmt("Y%+5.1f",  d.yaw));

    // Bubble level (right)
    constexpr int cx = 100, cy = 38, r = 20;
    fb.drawCircle(cx, cy, r);
    fb.drawHLine(cx - 4, cy,     9);
    fb.drawVLine(cx,     cy - 4, 9);

    int tidx = 0;
    for (auto [tx, ty] : trail_) {
        const int px = cx + tx, py = cy + ty;
        if (tidx > static_cast<int>(trail_.size()) - 4)
            fb.drawCircle(px, py, 2);
        else
            fb.setPixel(px, py);
        ++tidx;
    }
    fb.fillCircle(cx + static_cast<int>(sdot_x_),
                  cy + static_cast<int>(sdot_y_), 3);

    fb.drawVLine(70, 11, kH - 11);
}

// ═════════════════════════════════════════════════════════════
//  Screen: Joystick
// ═════════════════════════════════════════════════════════════
JoystickScreen::JoystickScreen(NavCb nav) : nav_(std::move(nav)) {}

void JoystickScreen::setData(JoystickState d) {
    std::lock_guard lk{mtx_};
    data_ = std::move(d);
}

void JoystickScreen::onInput(Ev ev) {
    if (ev == Ev::Back && nav_) nav_(ScreenID::Menu, -1);
}

void JoystickScreen::update(float dt) {
    JoystickState d;
    { std::lock_guard lk{mtx_}; d = data_; }
    const float k = std::min(1.f, dt * 15.f);
    slx_ = detail::lerp(slx_, d.lx, k);
    sly_ = detail::lerp(sly_, d.ly, k);
    srx_ = detail::lerp(srx_, d.rx, k);
    sry_ = detail::lerp(sry_, d.ry, k);
}

void JoystickScreen::render(FrameBuffer& fb) const {
    fb.clear();
    drawTitleBar(fb, "JOYSTICK", 3, 5);

    JoystickState d;
    { std::lock_guard lk{mtx_}; d = data_; }

    // Left stick
    constexpr int lx = 23, ly = 38, sr = 17;
    fb.drawCircle(lx, ly, sr);
    fb.drawHLine(lx - 3, ly, 7);
    fb.drawVLine(lx, ly - 3, 7);
    fb.fillCircle(lx + static_cast<int>(slx_ * (sr - 3)),
                  ly + static_cast<int>(sly_ * (sr - 3)), 3);

    // Right stick
    constexpr int rx = 78, ry = 38;
    fb.drawCircle(rx, ry, sr);
    fb.drawHLine(rx - 3, ry, 7);
    fb.drawVLine(rx, ry - 3, 7);
    fb.fillCircle(rx + static_cast<int>(srx_ * (sr - 3)),
                  ry + static_cast<int>(sry_ * (sr - 3)), 3);

    // Face buttons (Y / B / X / A cross)
    auto drawBtn = [&](int bx, int by, char label, bool pressed) {
        fb.drawChar(bx, by, label);
        if (pressed) fb.invertRect(bx - 1, by - 1, 7, 9);
    };
    drawBtn(110, 13, 'Y', d.btn_y);
    drawBtn(119, 20, 'B', d.btn_b);
    drawBtn(101, 20, 'X', d.btn_x);
    drawBtn(110, 27, 'A', d.btn_a);

    // Shoulder buttons
    fb.drawString(0,  11, d.btn_lb ? "[LB]" : " LB ");
    fb.drawString(50, 11, d.btn_rb ? "[RB]" : " RB ");

    // Start / Select
    fb.drawString(43, 56, d.btn_start  ? "[ST]" : " ST ");
    fb.drawString(80, 56, d.btn_select ? "[SE]" : " SE ");
}

// ═════════════════════════════════════════════════════════════
//  Screen: AimRT Log
// ═════════════════════════════════════════════════════════════
LogScreen::LogScreen(NavCb nav) : nav_(std::move(nav)) {}

void LogScreen::onEnter() { auto_scroll_ = true; }

void LogScreen::addLog(LogEntry e) {
    std::lock_guard lk{mtx_};
    logs_.push_back(std::move(e));
    if (static_cast<int>(logs_.size()) > kMaxLogs) logs_.pop_front();
    if (auto_scroll_) {
        const int n = static_cast<int>(logs_.size());
        top_   = std::max(0, n - 4);
        tsoff_ = static_cast<float>(top_ * kRowH);
        soff_  = tsoff_;
    }
}

void LogScreen::clear() {
    std::lock_guard lk{mtx_};
    logs_.clear();
    top_ = 0; soff_ = tsoff_ = 0.f;
}

void LogScreen::onInput(Ev ev) {
    std::lock_guard lk{mtx_};
    const int n = static_cast<int>(logs_.size());
    switch (ev) {
    case Ev::EncCW:
        if (top_ < std::max(0, n - 4)) {
            ++top_;
            tsoff_      = static_cast<float>(top_ * kRowH);
            auto_scroll_= (top_ >= n - 4);
        }
        break;
    case Ev::EncCCW:
        if (top_ > 0) {
            --top_;
            tsoff_      = static_cast<float>(top_ * kRowH);
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
    soff_    = detail::lerp(soff_, tsoff_, std::min(1.f, dt * 18.f));
    blink_t_ = std::fmod(blink_t_ + dt * 2.f, 2.f * 3.14159265f);
}

void LogScreen::render(FrameBuffer& fb) const {
    fb.clear();
    std::lock_guard lk{mtx_};
    const int n = static_cast<int>(logs_.size());

    drawTitleBar(fb, "AIMRT LOG", 4, 5);
    if (n > 0 && auto_scroll_ && std::sin(blink_t_) > 0.f)
        fb.fillCircle(kW - 4, 4, 2);  // live blink dot

    if (n == 0) { fb.drawStringCenter(28, "No logs"); return; }

    constexpr int kY0    = 11;
    const int     y_off  = static_cast<int>(soff_);

    for (int i = 0; i < n; ++i) {
        const int iy = kY0 + i * kRowH - y_off;
        if (iy < kY0 - kRowH || iy > kH) continue;

        const auto& e = logs_[i];
        using Level = LogEntry::Level;

        const char lvl_ch = (e.level == Level::Warn) ? 'W' :
                            (e.level == Level::Err)  ? 'E' : 'I';
        const bool hi = (e.level == Level::Warn || e.level == Level::Err);
        fb.drawChar(0, iy + 1, lvl_ch, 1, hi);

        const auto  mod     = std::string_view{e.module}.substr(0, 4);
        const int   msg_x   = 8 + static_cast<int>(mod.size()) * 6 + 2;
        const int   max_ch  = (kW - 4 - msg_x) / 6;
        fb.drawString(8,     iy + 1, mod);
        fb.drawString(msg_x, iy + 1,
                      std::string_view{e.msg}.substr(
                          0, std::min(static_cast<int>(e.msg.size()), max_ch)));
        fb.drawHLine(0, iy + kRowH - 1, kW - 4);
    }

    if (n > 4) {
        const float pos = static_cast<float>(top_) /
                          static_cast<float>(std::max(1, n - 4));
        fb.drawScrollbar(kW - 2, kY0, kH - kY0, pos,
                         4.f / static_cast<float>(n));
    }
}

// ═════════════════════════════════════════════════════════════
//  Screen: SBC Status
// ═════════════════════════════════════════════════════════════
SBCScreen::SBCScreen(NavCb nav) : nav_(std::move(nav)) {}

void SBCScreen::setData(SBCStatus d) {
    std::lock_guard lk{mtx_};
    anim_core_.resize(d.core_pct.size(), 0.f);
    data_ = std::move(d);
}

void SBCScreen::onInput(Ev ev) {
    if (ev == Ev::Back && nav_) nav_(ScreenID::Menu, -1);
}

void SBCScreen::update(float dt) {
    SBCStatus d;
    { std::lock_guard lk{mtx_}; d = data_; }
    anim_t_ += dt;

    const float k = std::min(1.f, dt * 8.f);
    anim_core_.resize(d.core_pct.size(), 0.f);
    for (std::size_t i = 0; i < anim_core_.size(); ++i)
        anim_core_[i] = detail::lerp(anim_core_[i], d.core_pct[i] / 100.f, k);

    const float rp = (d.ram_total_mb > 0.f)
                     ? d.ram_used_mb / d.ram_total_mb : 0.f;
    anim_ram_ = detail::lerp(anim_ram_, rp, k);
}

void SBCScreen::render(FrameBuffer& fb) const {
    fb.clear();
    SBCStatus d;
    { std::lock_guard lk{mtx_}; d = data_; }

    drawTitleBar(fb, "SBC STATUS", 5, 5);

    // CPU temp in title bar
    const auto tbuf = detail::sfmt("%.1foC", d.cpu_temp);
    const int  tw   = static_cast<int>(tbuf.size()) * FrameBuffer::charW();
    fb.drawStringRight(kW - 1, 1, tbuf, 1);
    fb.invertRect(kW - 1 - tw, 0, tw, 9);

    // Core bars (2-column layout, max 8 cores)
    constexpr int kBarW   = 54;
    constexpr int kBarH   = 5;
    constexpr int kLabelW = 12;
    const int     ncores  = static_cast<int>(
        std::min(anim_core_.size(), std::size_t{8}));

    for (int i = 0; i < ncores; ++i) {
        const int col = i >= 4 ? 1 : 0;
        const int row = i >= 4 ? i - 4 : i;
        const int bx  = col * 64 + kLabelW;
        const int by  = 11 + row * 13;
        const int lx  = col * 64;

        fb.drawString(lx, by + 1, detail::sfmt("C%d", i));
        fb.drawProgressBar(bx, by, kBarW, kBarH + 2, anim_core_[i]);
        fb.drawString(bx + kBarW - 24, by + kBarH + 3,
                      detail::sfmt("%3d%%",
                                  static_cast<int>(anim_core_[i] * 100.f)));
    }

    // RAM bar
    const int ram_y = 11 + 4 * 13 + 2;
    fb.drawString(0, ram_y,
                  detail::sfmt("RAM %.0f/%.0fMB",
                              d.ram_used_mb, d.ram_total_mb));
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

    screens_[static_cast<std::size_t>(ScreenID::Menu)]     = menu_.get();
    screens_[static_cast<std::size_t>(ScreenID::Joints)]   = joints_.get();
    screens_[static_cast<std::size_t>(ScreenID::IMU)]      = imu_.get();
    screens_[static_cast<std::size_t>(ScreenID::Joystick)] = joystick_.get();
    screens_[static_cast<std::size_t>(ScreenID::LogInfo)]  = log_.get();
    screens_[static_cast<std::size_t>(ScreenID::SBC)]      = sbc_.get();

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
    UIScreen* next = screens_[static_cast<std::size_t>(id)];
    if (!next || next == cur_) return;

    if (cur_) cur_->render(fb_prev_);
    if (cur_) cur_->onExit();

    prev_      = cur_;
    cur_       = next;
    in_trans_  = true;
    trans_t_   = 0.f;
    trans_dir_ = dir;
    cur_->onEnter();
}

void UIManager::tick(float dt) {
    if (cur_) cur_->update(dt);

    if (in_trans_) {
        trans_t_ += dt;
        const float t  = std::min(trans_t_ / trans_dur_, 1.f);
        const float et = detail::easeOut(t);

        if (cur_) cur_->render(fb_cur_);

        int new_dx, old_dx;
        if (trans_dir_ > 0) {
            new_dx =  static_cast<int>((1.f - et) * kW);
            old_dx = -static_cast<int>(et * kW);
        } else {
            new_dx = -static_cast<int>((1.f - et) * kW);
            old_dx =  static_cast<int>(et * kW);
        }

        // Composite: OR-merge old shifted + new shifted
        fb_final_.blitShifted(fb_prev_, old_dx);
        fb_tmp_.blitShifted(fb_cur_, new_dx);

        auto dst = fb_final_.rawMut();
        auto src = fb_tmp_.raw();
        for (int i = 0; i < kBufLen; ++i) dst[i] |= src[i];

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
    const auto frame_dur = microseconds(1'000'000 / cfg_.fps);
    auto last = steady_clock::now();

    while (running_.load(std::memory_order_relaxed)) {
        const auto now = steady_clock::now();
        const float dt = std::clamp(
            duration_cast<microseconds>(now - last).count() / 1'000'000.f,
            0.001f, 0.1f);
        last = now;

        tick(dt);

        if (const auto elapsed = steady_clock::now() - now; elapsed < frame_dur)
            std::this_thread::sleep_for(frame_dur - elapsed);
    }
}

void UIManager::stop() {
    running_.store(false);
    if (input_) input_->stop();
}

// ── Thread-safe data setters ──────────────────────────────────
void UIManager::setRobotIP      (std::string ip)       { menu_->setIP(std::move(ip)); }
void UIManager::setJointState   (JointState d)         { joints_->setData(std::move(d)); }
void UIManager::setIMUState     (IMUState d)           { imu_->setData(std::move(d)); }
void UIManager::setJoystickState(JoystickState d)      { joystick_->setData(std::move(d)); }
void UIManager::addLog          (LogEntry e)           { log_->addLog(std::move(e)); }
void UIManager::setSBCStatus    (SBCStatus d)          { sbc_->setData(std::move(d)); }

} // namespace oled