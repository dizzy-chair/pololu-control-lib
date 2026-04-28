#pragma once

#include <stdint.h>
#include <Pololu3piPlus32U4LineSensors.h>

namespace pll::subsystems {

struct line_follower_gains_t {
    float kp_straight;     // [motor_cmd / error_unit]
    float kd_straight;     // [motor_cmd / error_unit]
    float kp_curve;        // [motor_cmd / error_unit]
    float kd_curve;        // [motor_cmd / error_unit]
    float speed_reduction; // [motor_cmd / error_unit]
};

enum class follower_state : uint8_t { idle, calibrating, sensing, running };

// Calibrated sensor snapshot returned by read_sensors().
struct sensor_reading_t {
    int16_t  error;            // position - 2000, range [-2000, 2000]
    uint16_t position;         // readLine output [0, 4000]
    uint16_t sensor_values[5]; // per-sensor calibrated [0, 1000]
    bool     line_lost;        // all sensors > 900 OR |error| > LF_LINE_LOST_THRESHOLD
};

// PD line follower with adaptive speed and dual gain sets (straight / curve).
class line_follower {
public:
    explicit line_follower(bool follow_white);

    // Sweep the robot back and forth to calibrate line sensors. Blocks until complete.
    void calibrate();

    // Compute and apply motor commands from current sensor reading. No-op when not running.
    void update();

    // Transition from idle to running. Requires prior calibration.
    void start();

    // Cut motors and return to idle.
    void stop();

    void set_gains(line_follower_gains_t gains);
    void set_speed_profile(int16_t base, int16_t min_spd, int16_t max_spd);

    // Switch to sensing state so read_sensors() may be called. Requires calibration.
    // Motor control becomes the caller's responsibility.
    void arm_sensing();

    // Read calibrated sensors and return a snapshot. Valid in sensing or running state.
    sensor_reading_t read_sensors();

    follower_state state()         const;
    bool           is_calibrated() const;

private:
    Pololu3piPlus32U4::LineSensors _sensors;
    line_follower_gains_t          _gains;

    int16_t        _base_speed;
    int16_t        _min_speed;
    int16_t        _max_speed;
    int16_t        _curve_threshold;
    bool           _follow_white;
    bool           _calibrated;
    follower_state _state;
    uint16_t       _sensor_values[5];
    int16_t        _prev_error;
    bool           _on_curve;
};

}  // namespace pll::subsystems
