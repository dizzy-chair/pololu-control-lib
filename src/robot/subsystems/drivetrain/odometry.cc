#include "odometry.hh"
#include "config.hh"
#include <math.h>

namespace pll {

using namespace pll::config;

odometry::odometry() { reset(); }

void odometry::reset() {
    _pose       = {0.0f, 0.0f, 0.0f};
    _twist      = {0.0f, 0.0f};
    _prev_twist = {0.0f, 0.0f};
    _accel      = {0.0f, 0.0f};
}

void odometry::update(int16_t delta_left, int16_t delta_right, float dt_s,
                      int16_t gyro_z_raw, bool use_gyro) {
    const float dl = (float)delta_left  * METERS_PER_COUNT;
    const float dr = (float)delta_right * METERS_PER_COUNT;

    const float ds         = (dl + dr) * 0.5f;
    const float dtheta_enc = (dr - dl) / TRACK_WIDTH_M;

    float dtheta;
    if (use_gyro && dt_s > 1e-6f) {
        const float dtheta_gyro = (float)gyro_z_raw * GYRO_Z_SIGN
                                * GYRO_RAD_PER_S_PER_LSB * dt_s;
        dtheta = GYRO_FUSION_WEIGHT       * dtheta_gyro
               + (1.0f - GYRO_FUSION_WEIGHT) * dtheta_enc;
    } else {
        dtheta = dtheta_enc;
    }

    const float heading_mid = _pose.theta + dtheta * 0.5f;
    _pose.x     += ds * cosf(heading_mid);
    _pose.y     += ds * sinf(heading_mid);
    _pose.theta += dtheta;

    _prev_twist = _twist;
    if (dt_s > 1e-6f) {
        _twist.vx    = ds / dt_s;
        _twist.omega = dtheta / dt_s;
        _accel.ax    = (_twist.vx    - _prev_twist.vx)    / dt_s;
        _accel.alpha = (_twist.omega - _prev_twist.omega) / dt_s;
    }
}

pose_t  odometry::pose()  const { return _pose; }
twist_t odometry::twist() const { return _twist; }
accel_t odometry::accel() const { return _accel; }

}  // namespace pll
