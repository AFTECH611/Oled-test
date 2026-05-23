#pragma once
/**
 * @file  oled_driver.hpp
 * @brief OLED UI driver – 1.3-inch SH1106 display on Radxa Rock 5b+
 *        Refactored to C++20 / GCC 13
 *
 * Hardware wiring
 * ───────────────
 *   I2C bus  : /dev/i2c-7,  address 0x3C
 *   Enc-A    : Pin 13 -> GPIO3_C0  (gpiochip3 line 16)
 *   Enc-B    : Pin 12 -> GPIO3_B5  (gpiochip3 line 13)
 *   Enc-BTN  : Pin 15 -> GPIO3_C3  (gpiochip3 line 19)
 *   CONFIRM  : Pin 22 -> GPIO3_D1  (gpiochip3 line 25)
 *   BACK     : Pin 18 -> GPIO3_C4  (gpiochip3 line 20)
 *
 * Bug fixes vs original
 * ─────────────────────
 *  [FIX-1] push(): release lock BEFORE calling cb_ -> no deadlock, no lag
 *  [FIX-2] Encoder lookup table corrected for Gray-code quadrature
 *  [FIX-3] enc_accum_ threshold 4->2 (correct for 12/24 ppr encoders)
 *  [FIX-4] cb_ set before start(), used only in input thread -> no race
 *  [FIX-5] Debug printf/fflush removed from hot poll loop (was causing lag)
 *  [FIX-6] SH1106 column offset=2 applied in writePage()
 *  [FIX-7] blitShifted() uses reference& not value-copy of rawMut()
 */

#include <array>
#include <atomic>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

namespace oled {

inline constexpr int kW      = 128;
inline constexpr int kH      = 64;
inline constexpr int kPages  = 8;
inline constexpr int kBufLen = kW * kPages;

inline constexpr std::string_view kI2CDev   = "/dev/i2c-7";
inline constexpr uint8_t          kOledAddr = 0x3C;

struct GpioPin {
    std::string_view chip;
    unsigned int     line;
};

inline constexpr GpioPin kPinEncA   { "/dev/gpiochip3", 16 };
inline constexpr GpioPin kPinEncB   { "/dev/gpiochip3", 13 };
inline constexpr GpioPin kPinEncBtn { "/dev/gpiochip3", 19 };
inline constexpr GpioPin kPinConfirm{ "/dev/gpiochip3", 25 };
inline constexpr GpioPin kPinBack   { "/dev/gpiochip3", 20 };

struct JointInfo  { std::string name; float pos_rad = 0.f; float temp_c = 0.f; };
struct JointState { std::vector<JointInfo> joints; };

struct IMUState {
    float gx = 0, gy = 0, gz = 0;
    float ax = 0, ay = 0, az = 0;
    float roll = 0, pitch = 0, yaw = 0;
};

struct JoystickState {
    float lx = 0, ly = 0, rx = 0, ry = 0;
    bool btn_a=false, btn_b=false, btn_x=false, btn_y=false;
    bool btn_lb=false, btn_rb=false, btn_start=false, btn_select=false;
};

struct LogEntry {
    enum class Level : uint8_t { Info = 0, Warn, Err };
    Level level = Level::Info;
    std::string module, msg;
    uint64_t ts_ms = 0;
};

struct SBCStatus {
    std::vector<float> core_pct;
    float cpu_temp = 0.f, ram_used_mb = 0.f, ram_total_mb = 0.f;
};

// ─────────────────────────────────────────────────────────────
//  FrameBuffer
// ─────────────────────────────────────────────────────────────
class FrameBuffer {
public:
    FrameBuffer();
    void clear();
    void setPixel  (int x, int y, bool on = true);
    [[nodiscard]] bool getPixel(int x, int y) const;
    void drawHLine (int x, int y, int w);
    void drawVLine (int x, int y, int h);
    void drawLine  (int x0, int y0, int x1, int y1);
    void drawRect  (int x, int y, int w, int h);
    void fillRect  (int x, int y, int w, int h);
    void drawCircle(int cx, int cy, int r);
    void fillCircle(int cx, int cy, int r);
    void invertRect(int x, int y, int w, int h);
    int  drawChar        (int x, int y, char c, int scale=1, bool invert=false);
    int  drawString      (int x, int y, std::string_view s, int scale=1, bool invert=false);
    void drawStringCenter(int y, std::string_view s, int scale=1, bool invert=false);
    void drawStringRight (int xr, int y, std::string_view s, int scale=1);
    [[nodiscard]] static constexpr int charW(int scale=1) noexcept { return 6*scale; }
    [[nodiscard]] static constexpr int charH(int scale=1) noexcept { return 8*scale; }
    void drawProgressBar(int x, int y, int w, int h, float pct);
    void drawScrollbar  (int x, int y, int h, float pos, float visible_ratio);
    void blitShifted(const FrameBuffer& src, int dx);  // [FIX-7]
    [[nodiscard]] const std::array<uint8_t,kBufLen>& raw()    const noexcept { return buf_; }
    [[nodiscard]]       std::array<uint8_t,kBufLen>& rawMut()       noexcept { return buf_; }
private:
    std::array<uint8_t,kBufLen> buf_{};
};

// ─────────────────────────────────────────────────────────────
//  OledDisplay  (SH1106 via I2C)
// ─────────────────────────────────────────────────────────────
class OledDisplay {
public:
    explicit OledDisplay(std::string_view dev=kI2CDev, uint8_t addr=kOledAddr);
    ~OledDisplay();
    OledDisplay(const OledDisplay&)            = delete;
    OledDisplay& operator=(const OledDisplay&) = delete;
    OledDisplay(OledDisplay&&)                 = default;
    OledDisplay& operator=(OledDisplay&&)      = default;
    bool init();
    void flush(const FrameBuffer& fb);
    void setContrast(uint8_t c);
    void setOn(bool on);
private:
    bool cmd (uint8_t c);
    bool cmds(std::initializer_list<uint8_t> cs);
    bool writePage(int page, const uint8_t* data);  // [FIX-6]
    int fd_ = -1;
    uint8_t addr_;
    std::string dev_;
};

// ─────────────────────────────────────────────────────────────
//  Input events
// ─────────────────────────────────────────────────────────────
enum class Ev : uint8_t {
    None, EncCW, EncCCW, EncPress, Confirm, Back, ConfirmLong, BackLong
};

// ─────────────────────────────────────────────────────────────
//  InputHandler
// ─────────────────────────────────────────────────────────────
class InputHandler {
public:
    struct Config {
        GpioPin enc_a    = kPinEncA;
        GpioPin enc_b    = kPinEncB;
        GpioPin enc_btn  = kPinEncBtn;
        GpioPin confirm  = kPinConfirm;
        GpioPin back     = kPinBack;
        int poll_ms     = 1;   // [FIX-5] responsive, no printf lag
        int debounce_ms = 30;
        int long_ms     = 800;
    };

    explicit InputHandler(Config cfg);
    ~InputHandler();
    InputHandler(const InputHandler&)            = delete;
    InputHandler& operator=(const InputHandler&) = delete;

    bool init();
    void start();
    void stop();
    [[nodiscard]] Ev pop();

    // [FIX-4] Call before start(). After that cb_ is only used in input thread.
    void setCallback(std::function<void(Ev)> cb) { cb_ = std::move(cb); }

    struct GpioHandle;

private:
    void loop();
    void push(Ev ev);   // [FIX-1] lock released before cb_ invocation
    [[nodiscard]] int readLine(std::size_t idx) const;

    Config cfg_;
    std::vector<std::unique_ptr<GpioHandle>> gpios_;

    uint8_t enc_prev_  = 0;
    int     enc_accum_ = 0;  // [FIX-3] threshold = 2, not 4

    struct BtnState {
        bool    last=false, stable=false, long_fired=false;
        int64_t press_ms=0, change_ms=0;
    };
    BtnState bs_enc_, bs_confirm_, bs_back_;

    std::atomic<bool>       running_{false};
    std::thread             thread_;
    std::mutex              q_mtx_;
    std::deque<Ev>          queue_;
    std::function<void(Ev)> cb_;
};

// ─────────────────────────────────────────────────────────────
//  Screen IDs & base class
// ─────────────────────────────────────────────────────────────
enum class ScreenID : uint8_t { Menu, Joints, IMU, Joystick, LogInfo, SBC, _Count };

class UIScreen {
public:
    virtual ~UIScreen() = default;
    [[nodiscard]] virtual ScreenID    id()    const noexcept = 0;
    [[nodiscard]] virtual const char* title() const noexcept = 0;
    virtual void onEnter() {}
    virtual void onExit()  {}
    virtual void onInput(Ev ev) = 0;
    virtual void update(float dt) = 0;
    virtual void render(FrameBuffer& fb) const = 0;
protected:
    static void drawTitleBar(FrameBuffer& fb, std::string_view label, int cur_idx, int total);
};

// ─────────────────────────────────────────────────────────────
//  Concrete screens
// ─────────────────────────────────────────────────────────────
class MenuScreen final : public UIScreen {
public:
    using NavCb = std::function<void(ScreenID,int)>;
    explicit MenuScreen(NavCb nav);
    [[nodiscard]] ScreenID    id()    const noexcept override { return ScreenID::Menu; }
    [[nodiscard]] const char* title() const noexcept override { return "MENU"; }
    void onEnter() override;
    void onInput(Ev ev) override;
    void update(float dt) override;
    void render(FrameBuffer& fb) const override;
    void setIP(std::string ip) { ip_ = std::move(ip); }
private:
    static constexpr int kCount = 5;
    struct Item { ScreenID sid; const char* label; const char* icon; };
    static const std::array<Item,kCount> kItems;
    int sel_=0; float sy_=0,tsy_=0,pulse_=0;
    std::string ip_; NavCb nav_;
};

class JointsScreen final : public UIScreen {
public:
    using NavCb = std::function<void(ScreenID,int)>;
    explicit JointsScreen(NavCb nav);
    [[nodiscard]] ScreenID    id()    const noexcept override { return ScreenID::Joints; }
    [[nodiscard]] const char* title() const noexcept override { return "JOINTS"; }
    void onEnter() override;
    void onInput(Ev ev) override;
    void update(float dt) override;
    void render(FrameBuffer& fb) const override;
    void setData(JointState d);
private:
    static constexpr int kRowH=11;
    int top_=0; float soff_=0,tsoff_=0;
    mutable std::mutex mtx_; JointState data_; NavCb nav_;
};

class IMUScreen final : public UIScreen {
public:
    using NavCb = std::function<void(ScreenID,int)>;
    explicit IMUScreen(NavCb nav);
    [[nodiscard]] ScreenID    id()    const noexcept override { return ScreenID::IMU; }
    [[nodiscard]] const char* title() const noexcept override { return "IMU"; }
    void onInput(Ev ev) override;
    void update(float dt) override;
    void render(FrameBuffer& fb) const override;
    void setData(IMUState d);
private:
    mutable std::mutex mtx_; IMUState data_{};
    float dot_x_=0,dot_y_=0,sdot_x_=0,sdot_y_=0;
    std::deque<std::pair<int8_t,int8_t>> trail_; NavCb nav_;
};

class JoystickScreen final : public UIScreen {
public:
    using NavCb = std::function<void(ScreenID,int)>;
    explicit JoystickScreen(NavCb nav);
    [[nodiscard]] ScreenID    id()    const noexcept override { return ScreenID::Joystick; }
    [[nodiscard]] const char* title() const noexcept override { return "JOYSTICK"; }
    void onInput(Ev ev) override;
    void update(float dt) override;
    void render(FrameBuffer& fb) const override;
    void setData(JoystickState d);
private:
    mutable std::mutex mtx_; JoystickState data_{};
    float slx_=0,sly_=0,srx_=0,sry_=0; NavCb nav_;
};

class LogScreen final : public UIScreen {
public:
    using NavCb = std::function<void(ScreenID,int)>;
    explicit LogScreen(NavCb nav);
    [[nodiscard]] ScreenID    id()    const noexcept override { return ScreenID::LogInfo; }
    [[nodiscard]] const char* title() const noexcept override { return "AIMRT LOG"; }
    void onEnter() override;
    void onInput(Ev ev) override;
    void update(float dt) override;
    void render(FrameBuffer& fb) const override;
    void addLog(LogEntry e);
    void clear();
private:
    static constexpr int kMaxLogs=200, kRowH=10;
    mutable std::mutex mtx_; std::deque<LogEntry> logs_;
    int top_=0; float soff_=0,tsoff_=0; bool auto_scroll_=true; float blink_t_=0;
    NavCb nav_;
};

class SBCScreen final : public UIScreen {
public:
    using NavCb = std::function<void(ScreenID,int)>;
    explicit SBCScreen(NavCb nav);
    [[nodiscard]] ScreenID    id()    const noexcept override { return ScreenID::SBC; }
    [[nodiscard]] const char* title() const noexcept override { return "SBC STATUS"; }
    void onInput(Ev ev) override;
    void update(float dt) override;
    void render(FrameBuffer& fb) const override;
    void setData(SBCStatus d);
private:
    mutable std::mutex mtx_; SBCStatus data_;
    std::vector<float> anim_core_; float anim_ram_=0,anim_t_=0; NavCb nav_;
};

// ─────────────────────────────────────────────────────────────
//  UIManager
// ─────────────────────────────────────────────────────────────
class UIManager {
public:
    struct Config {
        std::string_view     i2c_dev   = kI2CDev;
        uint8_t              oled_addr = kOledAddr;
        InputHandler::Config input     = {};
        int                  fps       = 30;
    };
    explicit UIManager(Config cfg);
    ~UIManager();
    UIManager(const UIManager&)            = delete;
    UIManager& operator=(const UIManager&) = delete;

    bool init();
    void run();
    void stop();

    void setRobotIP      (std::string ip);
    void setJointState   (JointState d);
    void setIMUState     (IMUState d);
    void setJoystickState(JoystickState d);
    void addLog          (LogEntry e);
    void setSBCStatus    (SBCStatus d);

private:
    void navigate(ScreenID id, int dir);
    void tick(float dt);

    Config cfg_;
    std::unique_ptr<OledDisplay>  disp_;
    std::unique_ptr<InputHandler> input_;
    FrameBuffer fb_cur_, fb_prev_, fb_final_, fb_tmp_;

    std::unique_ptr<MenuScreen>     menu_;
    std::unique_ptr<JointsScreen>   joints_;
    std::unique_ptr<IMUScreen>      imu_;
    std::unique_ptr<JoystickScreen> joystick_;
    std::unique_ptr<LogScreen>      log_;
    std::unique_ptr<SBCScreen>      sbc_;

    std::array<UIScreen*, static_cast<std::size_t>(ScreenID::_Count)> screens_{};
    UIScreen* cur_  = nullptr;
    UIScreen* prev_ = nullptr;

    bool  in_trans_  = false;
    float trans_t_   = 0.f;
    float trans_dur_ = 0.22f;
    int   trans_dir_ = 0;
    std::atomic<bool> running_{false};
};

} // namespace oled