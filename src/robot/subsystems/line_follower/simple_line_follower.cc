#include <Arduino.h>

#include "simple_line_follower.hh"
#include "simple_line_follower_config.hh"

namespace pll::subsystems {

using namespace pll::config;

simple_line_follower::simple_line_follower(pll::subsystems::drivetrain& dt, bool follow_white)
    : _drivetrain(dt), _follow_white(follow_white) {}

void simple_line_follower::calibrate() {
    _state = simple_follower_state::idle;
    _sensors.resetCalibration();

    for (uint16_t i = 0; i < SLF_CALIBRATION_SAMPLES; i++) {
        int16_t l, r;
        if (i > SLF_CALIBRATION_SAMPLES / 4 && i <= 3 * SLF_CALIBRATION_SAMPLES / 4) {
            l = -SLF_CALIBRATION_SPEED;
            r =  SLF_CALIBRATION_SPEED;
        } else {
            l =  SLF_CALIBRATION_SPEED;
            r = -SLF_CALIBRATION_SPEED;
        }
        _drivetrain.set_motor_cmds(l, r);
        _sensors.calibrate();
    }

    _drivetrain.stop();
    _calibrated = true;
}

void simple_line_follower::start() {
    if (_calibrated)
        _state = simple_follower_state::running;
}

void simple_line_follower::stop() {
    _state      = simple_follower_state::idle;
    _prev_error = 0;
    _drivetrain.stop();
}

void simple_line_follower::update() {
    uint16_t sensor_values[5];
    const uint16_t position = _follow_white
        ? _sensors.readLineWhite(sensor_values)
        : _sensors.readLineBlack(sensor_values);

    const int16_t error   = (int16_t)position - 2000;
    const int16_t error_d = error - _prev_error;
    const float   ctl     = SLF_KP * error + SLF_KD * error_d;

    const int16_t left  = constrain((int16_t)(SLF_BASE_SPEED + ctl), -SLF_MOTOR_MAX_CMD, SLF_MOTOR_MAX_CMD);
    const int16_t right = constrain((int16_t)(SLF_BASE_SPEED - ctl), -SLF_MOTOR_MAX_CMD, SLF_MOTOR_MAX_CMD);

    _drivetrain.set_motor_cmds(left, right);
    _prev_error = error;
}

simple_follower_state simple_line_follower::state()         const { return _state; }
bool                  simple_line_follower::is_calibrated() const { return _calibrated; }

}  // namespace pll::subsystems
