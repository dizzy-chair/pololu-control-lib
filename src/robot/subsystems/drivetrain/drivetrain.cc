// Pololu3piPlus32U4IMU.h must be included in exactly one translation unit.
#include <Pololu3piPlus32U4IMU.h>
#include <Pololu3piPlus32U4Encoders.h>
#include <Pololu3piPlus32U4Motors.h>

#include "drivetrain.hh"
#include "config.hh"

namespace pll::subsystems {

using namespace pll::config;
using Enc = Pololu3piPlus32U4::Encoders;
using Mot = Pololu3piPlus32U4::Motors;

static constexpr pll::motor_gains_t DEFAULT_GAINS = {
    MC_KFF, MC_KP, MC_KI, MC_KD
};

drivetrain::drivetrain()
    : _left_ctrl(DEFAULT_GAINS),
      _right_ctrl(DEFAULT_GAINS),
      _desired{0.0f, 0.0f},
      _left_vel_mps(0.0f),
      _right_vel_mps(0.0f),
      _gyro_bias(0),
      _imu_ok(false) {}

bool drivetrain::init() {
    _imu_ok = _imu.init();
    if (!_imu_ok) return false;

    _imu.configureForTurnSensing();

    int32_t total = 0;
    for (uint16_t i = 0; i < GYRO_CALIBRATION_SAMPLES; i++) {
        while (!_imu.gyroDataReady()) {}
        _imu.readGyro();
        total += _imu.g.z;
    }
    _gyro_bias = (int16_t)(total / (int32_t)GYRO_CALIBRATION_SAMPLES);

    return true;
}

void drivetrain::set_speeds(pll::chassis_speeds speeds) { _desired = speeds; }

void drivetrain::stop() {
    _desired = {0.0f, 0.0f};
    Mot::setSpeeds(0, 0);
    _left_ctrl.reset();
    _right_ctrl.reset();
}

const pll::odometry& drivetrain::get_odometry() const { return _odometry; }

pll::motor_controller& drivetrain::left_motor_ctrl()  { return _left_ctrl; }
pll::motor_controller& drivetrain::right_motor_ctrl() { return _right_ctrl; }

void drivetrain::to_wheel_speeds(pll::chassis_speeds speeds,
                                 float& left_mps, float& right_mps) {
    const float half_track = TRACK_WIDTH_M * 0.5f;
    left_mps  = speeds.vx - speeds.omega * half_track;
    right_mps = speeds.vx + speeds.omega * half_track;
}

void drivetrain::update(float dt_s) {
    const int16_t dl = Enc::getCountsAndResetLeft();
    const int16_t dr = Enc::getCountsAndResetRight();

    if (dt_s > 1e-6f) {
        _left_vel_mps  = (float)dl * METERS_PER_COUNT / dt_s;
        _right_vel_mps = (float)dr * METERS_PER_COUNT / dt_s;
    }

    int16_t gyro_z = 0;  // [LSB]
    if (_imu_ok && _imu.gyroDataReady()) {
        _imu.readGyro();
        gyro_z = _imu.g.z - _gyro_bias;
    }

    _odometry.update(dl, dr, dt_s, gyro_z, _imu_ok);

    float desired_left_mps  = 0.0f;
    float desired_right_mps = 0.0f;
    to_wheel_speeds(_desired, desired_left_mps, desired_right_mps);

    const int16_t left_cmd  = _left_ctrl.update(desired_left_mps,  _left_vel_mps,  dt_s);
    const int16_t right_cmd = _right_ctrl.update(desired_right_mps, _right_vel_mps, dt_s);

    Mot::setSpeeds(left_cmd, right_cmd);
}

}  // namespace pll::subsystems
