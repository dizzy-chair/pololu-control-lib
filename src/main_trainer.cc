#include <Pololu3piPlus32U4.h>
#include "robot/subsystems/line_follower/line_follower.hh"
#include "robot/subsystems/line_follower/line_follower_config.hh"
#include "robot/subsystems/line_follower/pid_trainer.hh"
#include "robot/subsystems/line_follower/pid_trainer_config.hh"

using namespace Pololu3piPlus32U4;

static pll::subsystems::line_follower follower(pll::config::LF_FOLLOW_WHITE);
static OLED    oled;
static ButtonA button_a;
static ButtonB button_b;

static pll::trainer::pid_trainer trainer(follower, oled);
static bool training_started = false;
static bool training_done    = false;

auto setup() -> void {
    oled.clear();
    oled.setLayout21x8();
    oled.print(F("A:Cal  B:Run"));
}

static void show_final_result() {
    pll::trainer::trainer_result_t best{};
    if (!trainer.best_result(best)) {
        oled.clear();
        oled.print(F("No valid lvl"));
        return;
    }

    // Display gains × 10000 so they are readable on the 11×4 display.
    // To recover: value = display_number / 10000
    oled.clear();
    oled.gotoXY(0, 0);
    oled.print(F("S:"));
    oled.print(best.base_speed);

    oled.gotoXY(0, 1);
    oled.print(F("Ks:"));
    oled.print((int16_t)(best.gains.kp_straight * 10000.0f));
    oled.print(F(" Ds:"));
    oled.print((int16_t)(best.gains.kd_straight * 10000.0f));

    oled.gotoXY(0, 2);
    oled.print(F("Kc:"));
    oled.print((int16_t)(best.gains.kp_curve * 10000.0f));
    oled.print(F(" Dc:"));
    oled.print((int16_t)(best.gains.kd_curve * 10000.0f));

    oled.gotoXY(0, 3);
    oled.print(F("SR:"));
    oled.print((int16_t)(best.gains.speed_reduction * 1000.0f));
    oled.print(F(" M:"));
    oled.print((uint16_t)(best.verify_mse < 65535 ? best.verify_mse : 65535));
}

auto loop() -> void {
    if (training_done) return;

    if (button_a.getSingleDebouncedPress() && !training_started) {
        oled.clear();
        oled.print(F("Calibrating"));
        follower.calibrate();
        oled.clear();
        oled.print(F("Cal OK"));
        oled.gotoXY(0, 1);
        oled.print(F("B: start train"));
    }

    if (button_b.getSingleDebouncedPress() && follower.is_calibrated() && !training_started) {
        training_started = true;
        trainer.start();
    }

    if (training_started) {
        const bool running = trainer.update();
        if (!running) {
            training_started = false;
            training_done    = true;
            show_final_result();
        }
        delay(pll::config::TR_LOOP_PERIOD_MS);
    }
}
