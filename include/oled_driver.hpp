#pragma once
/**
 * @file  oled_driver.hpp
 * @brief OLED UI driver – SH1106 128x64, Radxa Rock 5b+, C++17
 *
 * Hardware wiring (giữ nguyên pin gốc)
 * ──────────────────────────────────────
 *   I2C  : /dev/i2c-7   addr 0x3C
 *   Enc-A  : gpiochip3 line 16  (Pin 13)
 *   Enc-B  : gpiochip3 line 13  (Pin 12)
 *   Enc-BTN: gpiochip3 line 19  (Pin 15)
 *   CONFIRM: gpiochip3 line 25  (Pin 22)
 *   BACK   : gpiochip3 line 20  (Pin 18)
 *
 * Root-cause analysis của các bug gốc
 * ─────────────────────────────────────
 *  BUG-A  cb_ được gọi trực tiếp từ input thread → onInput() → navigate()
 *         → cur_->render() chạy song song với render thread đang dùng cur_
 *         → race condition, crash, màn hình corrupt.
 *         FIX: input thread chỉ enqueue vào ev_queue_; render thread drain.
 *
 *  BUG-B  Encoder lookup table sai chiều (Gray-code bị đảo).
 *         FIX: bảng đúng theo chuẩn quadrature Gray-code.
 *
 *  BUG-C  enc_accum_ threshold = 4 → cần xoay 2 detent mới ra 1 event.
 *         FIX: threshold = 2 (phù hợp encoder 12/24 ppr phổ biến).
 *         Nếu encoder của bạn ra event quá nhanh, tăng lên 4.
 *
 *  BUG-D  printf/fflush trong hot loop 1ms → thêm ~5-10ms latency/iter.
 *         FIX: bỏ toàn bộ.
 *
 *  BUG-E  SH1106 column start = 0; đúng phải là 2 (hardware offset).
 *         FIX: cmd(0x02) thay vì cmd(0x00).
 *
 *  BUG-F  blitShifted(): "auto dst = rawMut()" → copy array → writes discarded.
 *         FIX: "auto& dst = rawMut()".
 *
 *  BUG-G  tick() composite: "auto dst = fb_final_.rawMut()" → copy lại.
 *         FIX: lấy raw pointer trực tiếp.
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

struct GpioPin { std::string_view chip; unsigned int line; };

// ── Pin definitions (giữ nguyên) ────────────────────────────────
inline constexpr GpioPin kPinEncA   { "/dev/gpiochip3", 16 };
inline constexpr GpioPin kPinEncB   { "/dev/gpiochip3", 13 };
inline constexpr GpioPin kPinEncBtn { "/dev/gpiochip3", 19 };
inline constexpr GpioPin kPinConfirm{ "/dev/gpiochip3", 25 };
inline constexpr GpioPin kPinBack   { "/dev/gpiochip3", 20 };

// ── Data structs ─────────────────────────────────────────────────
struct JointInfo  { std::string name; float pos_rad=0, temp_c=0; };
struct JointState { std::vector<JointInfo> joints; };
struct IMUState   { float gx=0,gy=0,gz=0,ax=0,ay=0,az=0,roll=0,pitch=0,yaw=0; };
struct JoystickState {
    float lx=0,ly=0,rx=0,ry=0;
    bool btn_a=0,btn_b=0,btn_x=0,btn_y=0,btn_lb=0,btn_rb=0,btn_start=0,btn_select=0;
};
struct LogEntry {
    enum class Level : uint8_t { Info=0, Warn, Err };
    Level level=Level::Info; std::string module,msg; uint64_t ts_ms=0;
};
struct SBCStatus {
    std::vector<float> core_pct;
    float cpu_temp=0, ram_used_mb=0, ram_total_mb=0;
};

// ── FrameBuffer ──────────────────────────────────────────────────
class FrameBuffer {
public:
    FrameBuffer() { buf_.fill(0); }
    void clear() { buf_.fill(0); }

    void setPixel  (int x, int y, bool on=true);
    bool getPixel  (int x, int y) const;
    void drawHLine (int x, int y, int w);
    void drawVLine (int x, int y, int h);
    void drawLine  (int x0,int y0,int x1,int y1);
    void drawRect  (int x,int y,int w,int h);
    void fillRect  (int x,int y,int w,int h);
    void drawCircle(int cx,int cy,int r);
    void fillCircle(int cx,int cy,int r);
    void invertRect(int x,int y,int w,int h);

    int  drawChar        (int x,int y,char c,int scale=1,bool inv=false);
    int  drawString      (int x,int y,std::string_view s,int scale=1,bool inv=false);
    void drawStringCenter(int y,std::string_view s,int scale=1,bool inv=false);
    void drawStringRight (int xr,int y,std::string_view s,int scale=1);

    static constexpr int charW(int sc=1) noexcept { return 6*sc; }
    static constexpr int charH(int sc=1) noexcept { return 8*sc; }

    void drawProgressBar(int x,int y,int w,int h,float pct);
    void drawScrollbar  (int x,int y,int h,float pos,float vis_ratio);
    void blitShifted    (const FrameBuffer& src, int dx);  // [FIX-F]

    const std::array<uint8_t,kBufLen>& raw()    const noexcept { return buf_; }
          std::array<uint8_t,kBufLen>& rawMut()       noexcept { return buf_; }
private:
    std::array<uint8_t,kBufLen> buf_{};
};

// ── OledDisplay (SH1106) ─────────────────────────────────────────
class OledDisplay {
public:
    explicit OledDisplay(std::string_view dev=kI2CDev, uint8_t addr=kOledAddr);
    ~OledDisplay();
    OledDisplay(const OledDisplay&)=delete;
    OledDisplay& operator=(const OledDisplay&)=delete;

    bool init();
    void flush(const FrameBuffer& fb);
    void setContrast(uint8_t c);
    void setOn(bool on);
private:
    bool cmd(uint8_t c);
    bool cmds(std::initializer_list<uint8_t> cs);
    bool writePage(int page, const uint8_t* data); // [FIX-E] col offset=2
    int fd_=-1; uint8_t addr_; std::string dev_;
};

// ── Input events ─────────────────────────────────────────────────
enum class Ev : uint8_t { None,EncCW,EncCCW,EncPress,Confirm,Back,ConfirmLong,BackLong };

// ── InputHandler ─────────────────────────────────────────────────
// Hanya mengisi antrian; tidak memanggil callback langsung.
// Render thread menguras antrian di awal setiap frame. [FIX-A]
class InputHandler {
public:
    struct Config {
        GpioPin enc_a   =kPinEncA,  enc_b  =kPinEncB;
        GpioPin enc_btn =kPinEncBtn,confirm=kPinConfirm,back=kPinBack;
        int poll_ms=1, debounce_ms=30, long_ms=800;
    };

    explicit InputHandler(Config cfg);
    ~InputHandler();
    InputHandler(const InputHandler&)=delete;
    InputHandler& operator=(const InputHandler&)=delete;

    bool init();
    void start();
    void stop();

    // Drain tất cả events đang pending – gọi từ render thread [FIX-A]
    std::vector<Ev> drainEvents();

    struct GpioHandle;
private:
    void loop();
    void enqueue(Ev ev);
    int  readLine(std::size_t idx) const;

    Config cfg_;
    std::vector<std::unique_ptr<GpioHandle>> gpios_;

    uint8_t enc_prev_=0;
    int     enc_accum_=0;   // [FIX-C] threshold=2

    struct BtnState {
        bool last=false,stable=false,long_fired=false;
        int64_t press_ms=0,change_ms=0;
    };
    BtnState bs_enc_,bs_confirm_,bs_back_;

    std::atomic<bool> running_{false};
    std::thread       thread_;
    std::mutex        q_mtx_;
    std::deque<Ev>    queue_;
};

// ── Screen IDs ───────────────────────────────────────────────────
enum class ScreenID : uint8_t { Menu,Joints,IMU,Joystick,LogInfo,SBC,_Count };

// ── UIScreen base ────────────────────────────────────────────────
class UIScreen {
public:
    virtual ~UIScreen()=default;
    virtual ScreenID    id()    const noexcept=0;
    virtual const char* title() const noexcept=0;
    virtual void onEnter(){}
    virtual void onExit() {}
    virtual void onInput(Ev ev)=0;
    virtual void update(float dt)=0;
    virtual void render(FrameBuffer& fb) const=0;
protected:
    static void drawTitleBar(FrameBuffer& fb,std::string_view label,int cur,int total);
};

// ── Concrete screens ─────────────────────────────────────────────
class MenuScreen final : public UIScreen {
public:
    using NavCb=std::function<void(ScreenID,int)>;
    explicit MenuScreen(NavCb nav);
    ScreenID    id()    const noexcept override { return ScreenID::Menu; }
    const char* title() const noexcept override { return "MENU"; }
    void onEnter() override;
    void onInput(Ev ev) override;
    void update(float dt) override;
    void render(FrameBuffer& fb) const override;
    void setIP(std::string ip){ ip_=std::move(ip); }
private:
    static constexpr int kCount=5;
    struct Item{ ScreenID sid; const char* label; const char* icon; };
    static const std::array<Item,kCount> kItems;
    int sel_=0; float sy_=0,tsy_=0,pulse_=0;
    std::string ip_; NavCb nav_;
};

class JointsScreen final : public UIScreen {
public:
    using NavCb=std::function<void(ScreenID,int)>;
    explicit JointsScreen(NavCb nav);
    ScreenID    id()    const noexcept override { return ScreenID::Joints; }
    const char* title() const noexcept override { return "JOINTS"; }
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
    using NavCb=std::function<void(ScreenID,int)>;
    explicit IMUScreen(NavCb nav);
    ScreenID    id()    const noexcept override { return ScreenID::IMU; }
    const char* title() const noexcept override { return "IMU"; }
    void onInput(Ev ev) override;
    void update(float dt) override;
    void render(FrameBuffer& fb) const override;
    void setData(IMUState d);
private:
    mutable std::mutex mtx_; IMUState data_{};
    float sdot_x_=0,sdot_y_=0;
    std::deque<std::pair<int8_t,int8_t>> trail_;
    NavCb nav_;
};

class JoystickScreen final : public UIScreen {
public:
    using NavCb=std::function<void(ScreenID,int)>;
    explicit JoystickScreen(NavCb nav);
    ScreenID    id()    const noexcept override { return ScreenID::Joystick; }
    const char* title() const noexcept override { return "JOYSTICK"; }
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
    using NavCb=std::function<void(ScreenID,int)>;
    explicit LogScreen(NavCb nav);
    ScreenID    id()    const noexcept override { return ScreenID::LogInfo; }
    const char* title() const noexcept override { return "AIMRT LOG"; }
    void onEnter() override;
    void onInput(Ev ev) override;
    void update(float dt) override;
    void render(FrameBuffer& fb) const override;
    void addLog(LogEntry e);
    void clear();
private:
    static constexpr int kMaxLogs=200,kRowH=10;
    mutable std::mutex mtx_; std::deque<LogEntry> logs_;
    int top_=0; float soff_=0,tsoff_=0; bool auto_scroll_=true; float blink_t_=0;
    NavCb nav_;
};

class SBCScreen final : public UIScreen {
public:
    using NavCb=std::function<void(ScreenID,int)>;
    explicit SBCScreen(NavCb nav);
    ScreenID    id()    const noexcept override { return ScreenID::SBC; }
    const char* title() const noexcept override { return "SBC STATUS"; }
    void onInput(Ev ev) override;
    void update(float dt) override;
    void render(FrameBuffer& fb) const override;
    void setData(SBCStatus d);
private:
    mutable std::mutex mtx_; SBCStatus data_;
    std::vector<float> anim_core_; float anim_ram_=0,anim_t_=0; NavCb nav_;
};

// ── UIManager ────────────────────────────────────────────────────
class UIManager {
public:
    struct Config {
        std::string_view     i2c_dev  =kI2CDev;
        uint8_t              oled_addr=kOledAddr;
        InputHandler::Config input    ={};
        int                  fps      =30;
    };
    explicit UIManager(Config cfg);
    ~UIManager();
    UIManager(const UIManager&)=delete;
    UIManager& operator=(const UIManager&)=delete;

    bool init();
    void run();   // blocking render loop
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

    FrameBuffer fb_cur_,fb_prev_,fb_final_,fb_tmp_;

    std::unique_ptr<MenuScreen>     menu_;
    std::unique_ptr<JointsScreen>   joints_;
    std::unique_ptr<IMUScreen>      imu_;
    std::unique_ptr<JoystickScreen> joystick_;
    std::unique_ptr<LogScreen>      log_;
    std::unique_ptr<SBCScreen>      sbc_;

    std::array<UIScreen*, static_cast<std::size_t>(ScreenID::_Count)> screens_{};
    UIScreen* cur_=nullptr;

    bool  in_trans_ =false;
    float trans_t_  =0,trans_dur_=0.22f;
    int   trans_dir_=0;

    std::atomic<bool> running_{false};
};

} // namespace oled