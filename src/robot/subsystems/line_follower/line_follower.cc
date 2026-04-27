#include "line_follower.hh"
#include "line_follower_config.hh"
#include <Pololu3piPlus32U4Motors.h>
#include <PololuBuzzer.h>

namespace pll::subsystems {

using namespace pll::config;
using Mot = Pololu3piPlus32U4::Motors;

static int16_t clamp16(int16_t v, int16_t lo, int16_t hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

line_follower::line_follower(bool follow_white)
    : _gains{LF_KP_STRAIGHT, LF_KD_STRAIGHT, LF_KP_CURVE, LF_KD_CURVE, LF_SPEED_REDUCTION},
      _base_speed(LF_BASE_SPEED),
      _min_speed(LF_MIN_SPEED),
      _max_speed(LF_MAX_SPEED),
      _curve_threshold(LF_CURVE_THRESHOLD),
      _follow_white(follow_white),
      _calibrated(false),
      _state(follower_state::idle),
      _sensor_values{0, 0, 0, 0, 0},
      _prev_error(0),
      _on_curve(false) {}

void line_follower::calibrate() {
    _state = follower_state::calibrating;
    _sensors.resetCalibration();
    _prev_error = 0;

    for (uint16_t i = 0; i < LF_CALIBRATION_SAMPLES; i++) {
        if (i > LF_CALIBRATION_SAMPLES / 4 && i <= 3 * LF_CALIBRATION_SAMPLES / 4) {
            Mot::setSpeeds(-LF_CALIBRATION_SPEED, LF_CALIBRATION_SPEED);
        } else {
            Mot::setSpeeds(LF_CALIBRATION_SPEED, -LF_CALIBRATION_SPEED);
        }
        _sensors.calibrate();
    }

    Mot::setSpeeds(0, 0);
    _calibrated = true;
    _state = follower_state::idle;
}

void line_follower::start() {
    if (!_calibrated) return;
    _prev_error = 0;
    _on_curve   = false;
    _state      = follower_state::running;
}

void line_follower::stop() {
    _state = follower_state::idle;
    Mot::setSpeeds(0, 0);
}

void line_follower::set_gains(line_follower_gains_t gains) { _gains = gains; }

void line_follower::set_speed_profile(int16_t base, int16_t min_spd, int16_t max_spd) {
    _base_speed = base;
    _min_speed  = min_spd;
    _max_speed  = max_spd;
}

follower_state line_follower::state()         const { return _state; }
bool           line_follower::is_calibrated() const { return _calibrated; }

void line_follower::update() {
    if (_state != follower_state::running) return;

    const uint16_t position = _follow_white
        ? _sensors.readLineWhite(_sensor_values)
        : _sensors.readLineBlack(_sensor_values);

    const int16_t error   = (int16_t)position - 2000;
    const int16_t d_error = error - _prev_error;
    const int16_t abs_err = error < 0 ? -error : error;

    const bool new_on_curve = _on_curve
        ? (abs_err >= _curve_threshold - LF_CURVE_HYST)
        : (abs_err >= _curve_threshold + LF_CURVE_HYST);

    if (new_on_curve != _on_curve) {
        PololuBuzzer::playFrequency(
            new_on_curve ? LF_CHIRP_CURVE_FREQ : LF_CHIRP_STRAIGHT_FREQ,
            LF_CHIRP_DURATION_MS, LF_CHIRP_VOLUME);
        _on_curve = new_on_curve;
    }

    const float kp = _on_curve ? _gains.kp_curve : _gains.kp_straight;
    const float kd = _on_curve ? _gains.kd_curve : _gains.kd_straight;

    const float   diff_f     = kp * (float)error + kd * (float)d_error;
    const int16_t speed_diff = diff_f >  (float)_max_speed ?  _max_speed
                             : diff_f < -(float)_max_speed ? -_max_speed
                             : (int16_t)diff_f;

    int16_t actual_base = _base_speed
        - (int16_t)(_gains.speed_reduction * (float)abs_err);
    actual_base = clamp16(actual_base, _min_speed, _max_speed);

    const int16_t left  = clamp16(actual_base + speed_diff, -_max_speed, _max_speed);
    const int16_t right = clamp16(actual_base - speed_diff, -_max_speed, _max_speed);

    Mot::setSpeeds(left, right);
    _prev_error = error;
}

}  // namespace pll::subsystems
