#pragma once

#include <stdint.h>

namespace pll {

struct motor_gains_t {
    float kff;  // [motor_cmd / (m/s)]   feedforward
    float kp;   // [motor_cmd / (m/s)]   proportional
    float ki;   // [motor_cmd / m]       integral
    float kd;   // [motor_cmd / (m/s²)]  derivative
};

// Discrete-time PID controller with velocity feedforward for one drive wheel.
class motor_controller {
public:
    explicit motor_controller(motor_gains_t gains);

    // Returns a motor command in [-400, 400].
    int16_t update(float desired_vel, float measured_vel, float dt_s);

    void          reset();
    void          set_gains(motor_gains_t gains);
    motor_gains_t gains() const;

private:
    motor_gains_t _gains;
    float         _integral;    // [m]    accumulated velocity_error × dt
    float         _prev_error;  // [m/s]  velocity error from previous step
};

}  // namespace pll
