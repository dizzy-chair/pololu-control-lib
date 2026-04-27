#pragma once

#include <stdint.h>

namespace pll {

struct pose_t {
    float x;      // [m]   X position in world frame
    float y;      // [m]   Y position in world frame
    float theta;  // [rad] heading, counter-clockwise positive, 0 = +X axis
};

struct twist_t {
    float vx;    // [m/s]   forward velocity in robot frame
    float omega; // [rad/s] angular velocity, counter-clockwise positive
};

struct accel_t {
    float ax;    // [m/s²]   forward linear acceleration (finite-difference)
    float alpha; // [rad/s²] angular acceleration (finite-difference)
};

class odometry {
public:
    odometry();

    // Integrate encoder and gyro data for one control step.
    void update(int16_t delta_left, int16_t delta_right, float dt_s,
                int16_t gyro_z_raw, bool use_gyro);

    void reset();

    pose_t  pose()  const;
    twist_t twist() const;
    accel_t accel() const;

private:
    pose_t  _pose;
    twist_t _twist;
    twist_t _prev_twist;
    accel_t _accel;
};

}  // namespace pll
