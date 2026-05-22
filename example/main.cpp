/**
 * @file  main.cpp
 * @brief Demo entry point for the OLED UI driver.
 *
**/
#include "include/oled_driver.hpp"

#include <chrono>
#include <csignal>
#include <cstdio>
#include <string>
#include <thread>
#include <vector>

// ─────────────────────────────────────────────────────────────
//  Signal handling  (Ctrl-C graceful exit)
// ─────────────────────────────────────────────────────────────
static oled::UIManager* g_ui = nullptr;

static void onSignal(int) {
    if (g_ui) g_ui->stop();
}

// ─────────────────────────────────────────────────────────────
//  Static demo data
// ─────────────────────────────────────────────────────────────

// --- Joint State ------------------------------------------------
static oled::JointState makeJointState() {
    oled::JointState js;
    // 12-DOF humanoid leg / arm example joints
    const struct { const char* name; float pos; float temp; } kJoints[] = {
        { "L_HIP_YAW",    0.00f,  38.5f },
        { "L_HIP_ROLL",  -0.12f,  37.2f },
        { "L_HIP_PITCH",  0.45f,  41.0f },
        { "L_KNEE",       0.87f,  43.8f },
        { "L_ANKLE_P",   -0.33f,  39.1f },
        { "L_ANKLE_R",    0.05f,  36.4f },
        { "R_HIP_YAW",    0.00f,  38.7f },
        { "R_HIP_ROLL",   0.11f,  37.5f },
        { "R_HIP_PITCH",  0.44f,  40.8f },
        { "R_KNEE",       0.88f,  44.1f },
        { "R_ANKLE_P",   -0.32f,  39.3f },
        { "R_ANKLE_R",   -0.04f,  36.6f },
    };
    for (auto& j : kJoints) {
        oled::JointInfo ji;
        ji.name    = j.name;
        ji.pos_rad = j.pos;
        ji.temp_c  = j.temp;
        js.joints.push_back(ji);
    }
    return js;
}

// --- IMU State --------------------------------------------------
static oled::IMUState makeIMUState() {
    oled::IMUState imu;
    imu.gx    =  0.012f;   // rad/s
    imu.gy    = -0.007f;
    imu.gz    =  0.003f;
    imu.ax    =  0.08f;    // m/s²
    imu.ay    = -0.03f;
    imu.az    =  9.81f;
    imu.roll  =  1.2f;     // degrees
    imu.pitch = -0.4f;
    imu.yaw   =  87.3f;
    return imu;
}

// --- Joystick State ---------------------------------------------
static oled::JoystickState makeJoystickState() {
    oled::JoystickState joy;
    joy.lx       =  0.55f;
    joy.ly       = -0.30f;
    joy.rx       = -0.10f;
    joy.ry       =  0.80f;
    joy.btn_a    = true;
    joy.btn_b    = false;
    joy.btn_x    = true;
    joy.btn_y    = false;
    joy.btn_lb   = false;
    joy.btn_rb   = true;
    joy.btn_start  = false;
    joy.btn_select = false;
    return joy;
}

// --- AimRT Log entries ------------------------------------------
static std::vector<oled::LogEntry> makeLogEntries() {
    using L = oled::LogEntry;
    std::vector<L> logs;

    auto add = [&](L::Level lvl, const char* mod, const char* msg, uint64_t ts) {
        L e;  e.level = lvl; e.module = mod; e.msg = msg; e.ts_ms = ts;
        logs.push_back(e);
    };

    uint64_t t = 1'000; // ms offset from start
    add(L::INFO, "core",     "AimRT runtime started",          t += 120);
    add(L::INFO, "executor", "Thread pool ready (8 workers)",  t += 340);
    add(L::INFO, "plugin",   "IMU plugin loaded OK",           t += 80 );
    add(L::INFO, "plugin",   "Joystick plugin loaded OK",      t += 60 );
    add(L::WARN, "joint",    "Joint L_KNEE temp > 43 degC",    t += 500);
    add(L::INFO, "ctrl",     "Balance controller active",      t += 200);
    add(L::INFO, "gait",     "Walking gait engaged",           t += 150);
    add(L::WARN, "imu",      "Gyro drift detected 0.01 r/s",  t += 700);
    add(L::ERR,  "comms",    "CAN frame timeout ch2",          t += 300);
    add(L::INFO, "comms",    "CAN ch2 recovered",              t += 120);
    add(L::INFO, "gait",     "Step cycle: 0.48 s",             t += 480);
    add(L::WARN, "ctrl",     "ZMP margin 12% (low)",           t += 900);
    add(L::INFO, "ctrl",     "ZMP margin restored 28%",        t += 400);
    add(L::INFO, "sbc",      "CPU temp 61 degC – nominal",     t += 200);
    add(L::ERR,  "joint",    "R_ANKLE_R enc read fail",        t += 100);
    add(L::INFO, "joint",    "R_ANKLE_R enc OK after retry",   t += 50 );
    add(L::INFO, "gait",     "Terrain adapt: slope 3 deg",     t += 600);

    return logs;
}

// --- SBC Status -------------------------------------------------
static oled::SBCStatus makeSBCStatus() {
    oled::SBCStatus s;
    // Radxa Rock 5b+ has 8 cores (4× A55 + 4× A76)
    s.core_pct    = { 42.f, 38.f, 91.f, 85.f, 17.f, 22.f, 11.f, 9.f };
    s.cpu_temp    = 61.5f;
    s.ram_used_mb = 3412.f;
    s.ram_total_mb= 8192.f;
    return s;
}

// ─────────────────────────────────────────────────────────────
//  main
// ─────────────────────────────────────────────────────────────
int main() {
    // ── UI manager config ────────────────────────────────────
    oled::UIManager::Config cfg;
    cfg.i2c_dev   = oled::kI2CDev;
    cfg.oled_addr = oled::kOledAddr;
    cfg.fps       = 30;

    // GPIO pin config (default Radxa Rock 5b+ wiring)
    cfg.input.enc_a   = oled::kPinEncA;
    cfg.input.enc_b   = oled::kPinEncB;
    cfg.input.enc_btn = oled::kPinEncBtn;
    cfg.input.confirm = oled::kPinConfirm;
    cfg.input.back    = oled::kPinBack;
    cfg.input.poll_ms    = 5;
    cfg.input.debounce_ms= 30;
    cfg.input.long_ms    = 800;

    oled::UIManager ui(cfg);
    g_ui = &ui;

    std::signal(SIGINT,  onSignal);
    std::signal(SIGTERM, onSignal);

    if (!ui.init()) {
        std::fprintf(stderr, "[main] UIManager::init() failed – check I²C/GPIO.\n");
        return 1;
    }

    // ── Push static data into every screen ──────────────────

    // Robot IP
    ui.setRobotIP("192.168.1.42");

    // Joint state (12 joints)
    ui.setJointState(makeJointState());

    // IMU
    ui.setIMUState(makeIMUState());

    // Joystick
    ui.setJoystickState(makeJoystickState());

    // AimRT log – push entries one by one so timestamps differ
    for (const auto& e : makeLogEntries())
        ui.addLog(e);

    // SBC
    ui.setSBCStatus(makeSBCStatus());

    std::printf("[main] OLED UI running.\n"
                "  Encoder CW/CCW : scroll / change menu selection\n"
                "  Encoder press  : confirm / enter screen\n"
                "  CONFIRM button : same as encoder press\n"
                "  BACK button    : return to menu\n"
                "  Ctrl-C         : exit\n\n");

    // ── Optional: simulate slowly changing data in background ──
    // Uncomment the block below to watch values drift on screen.
    /*
    std::thread updater([&ui] {
        float t = 0.f;
        while (true) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            t += 0.05f;

            // Gently animate IMU
            oled::IMUState imu;
            imu.gx    = 0.01f * std::sin(t);
            imu.gy    = 0.01f * std::cos(t * 0.7f);
            imu.gz    = 0.005f;
            imu.ax    = 0.1f  * std::sin(t * 0.4f);
            imu.ay    = 0.05f * std::cos(t * 0.6f);
            imu.az    = 9.81f;
            imu.roll  = 2.f   * std::sin(t * 0.3f);
            imu.pitch = 1.5f  * std::cos(t * 0.5f);
            imu.yaw   = std::fmod(t * 5.f, 360.f);
            ui.setIMUState(imu);

            // Animate joystick
            oled::JoystickState joy;
            joy.lx = std::sin(t);
            joy.ly = std::cos(t * 0.8f);
            joy.rx = std::cos(t * 1.2f);
            joy.ry = std::sin(t * 0.6f);
            joy.btn_a = (static_cast<int>(t * 2) & 1) == 0;
            joy.btn_b = !joy.btn_a;
            ui.setJoystickState(joy);

            // Animate SBC cores
            oled::SBCStatus sbc;
            sbc.core_pct.resize(8);
            for (int i = 0; i < 8; ++i)
                sbc.core_pct[i] = 50.f + 40.f * std::sin(t + i * 0.8f);
            sbc.cpu_temp     = 58.f + 6.f * std::sin(t * 0.2f);
            sbc.ram_used_mb  = 3200.f + 400.f * std::sin(t * 0.1f);
            sbc.ram_total_mb = 8192.f;
            ui.setSBCStatus(sbc);
        }
    });
    updater.detach();
    */

    // ── Blocking render loop ─────────────────────────────────
    ui.run();   // returns when stop() is called

    g_ui = nullptr;
    std::printf("[main] Shutdown complete.\n");
    return 0;
}