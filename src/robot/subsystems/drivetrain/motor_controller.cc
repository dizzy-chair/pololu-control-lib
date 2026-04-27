#include "motor_controller.hh"
#include "config.hh"

namespace pll {

using namespace pll::config;

static int16_t clamp_cmd(float v) {
    if (v >  (float)MOTOR_MAX_CMD) return  MOTOR_MAX_CMD;
    if (v < -(float)MOTOR_MAX_CMD) return -MOTOR_MAX_CMD;
    return (int16_t)v;
}

motor_controller::motor_controller(motor_gains_t gains)
    : _gains(gains), _integral(0.0f), _prev_error(0.0f) {}

void motor_controller::reset() {
    _integral   = 0.0f;
    _prev_error = 0.0f;
}

void motor_controller::set_gains(motor_gains_t gains) { _gains = gains; }
motor_gains_t motor_controller::gains() const { return _gains; }

int16_t motor_controller::update(float desired_vel, float measured_vel, float dt_s) {
    const float error = desired_vel - measured_vel;

    float output = _gains.kff * desired_vel;
    output += _gains.kp * error;

    if (dt_s > 1e-6f) {
        _integral += error * dt_s;

        const float integral_contribution = _gains.ki * _integral;
        if      (integral_contribution >  MC_INTEGRAL_MAX_CMD && _gains.ki != 0.0f)
            _integral =  MC_INTEGRAL_MAX_CMD / _gains.ki;
        else if (integral_contribution < -MC_INTEGRAL_MAX_CMD && _gains.ki != 0.0f)
            _integral = -MC_INTEGRAL_MAX_CMD / _gains.ki;

        output += _gains.ki * _integral;
    }

    if (dt_s > 1e-6f) {
        const float deriv = (error - _prev_error) / dt_s;
        output += _gains.kd * deriv;
    }
    _prev_error = error;

    // Reset integral at rest to prevent drift accumulation.
    if (desired_vel == 0.0f && measured_vel == 0.0f) {
        _integral = 0.0f;
    }

    return clamp_cmd(output);
}

}  // namespace pll
