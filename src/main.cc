#include <Wire.h>
#include <Pololu3piPlus32U4.h>
#include "robot/robot.hh"
#include "robot/subsystems/drivetrain/config.hh"
#include "robot/subsystems/line_follower/line_follower.hh"
#include "robot/subsystems/line_follower/line_follower_config.hh"

using namespace Pololu3piPlus32U4;

static pll::robot        robot;
static pll::subsystems::line_follower follower(pll::config::LF_FOLLOW_WHITE);

static OLED    oled;
static ButtonA button_a;
static ButtonB button_b;

auto setup() -> void {
    Wire.begin();
    robot.init();  // robot must be held still during gyro calibration

    robot.drivetrain().left_motor_ctrl().set_gains({
        pll::config::MC_KFF,
        pll::config::MC_KP,
        pll::config::MC_KI,
        pll::config::MC_KD,
    });
    robot.drivetrain().right_motor_ctrl().set_gains({
        pll::config::MC_KFF + 15.0f,
        pll::config::MC_KP,
        pll::config::MC_KI,
        pll::config::MC_KD,
    });

    oled.clear();
    oled.print(F("A:Cal  B:Run"));
}

auto loop() -> void {
    static uint32_t last_us = micros();
    const  uint32_t now_us  = micros();
    const  float    dt_s    = (float)(now_us - last_us) * 1e-6f;
    last_us = now_us;

    if (button_a.getSingleDebouncedPress()) {
        follower.stop();
        robot.drivetrain().stop();

        oled.clear();
        oled.print(F("Calibrating"));
        follower.calibrate();

        oled.clear();
        oled.print(F("Cal OK"));
        oled.gotoXY(0, 1);
        oled.print(F("B: start"));
    }

    if (button_b.getSingleDebouncedPress()) {
        if (follower.state() == pll::subsystems::follower_state::running) {
            follower.stop();
            oled.clear();
            oled.print(F("Stopped"));
        } else if (follower.is_calibrated()) {
            follower.start();
            oled.clear();
            oled.print(F("Following"));
        } else {
            oled.clear();
            oled.print(F("A: calibrate"));
        }
    }

    // Line follower and drivetrain both drive motors; only one runs at a time.
    if (follower.state() == pll::subsystems::follower_state::running) {
        follower.update();
    } else {
        robot.update(dt_s);
    }
}
