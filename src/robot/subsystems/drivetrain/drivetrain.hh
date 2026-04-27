#pragma once

// Pololu3piPlus32U4IMU.h must be included in exactly one translation unit.
#include <Pololu3piPlus32U4IMU_declaration.h>

#include "chassis_speeds.hh"
#include "motor_controller.hh"
#include "odometry.hh"

namespace pll::subsystems {

class drivetrain {
public:
    drivetrain();

    // Call once from setup(), with the robot held still for gyro calibration.
    // Returns false if the IMU was not detected.
    bool init();

    // Call every control loop iteration.
    void update(float dt_s);

    void set_speeds(pll::chassis_speeds speeds);

    // Cut power immediately and clear velocity setpoint.
    void stop();

    const pll::odometry&    get_odometry()     const;
    pll::motor_controller&  left_motor_ctrl();
    pll::motor_controller&  right_motor_ctrl();

private:
    static void to_wheel_speeds(pll::chassis_speeds speeds,
                                float& left_mps, float& right_mps);

    Pololu3piPlus32U4::IMU   _imu;
    pll::motor_controller    _left_ctrl;
    pll::motor_controller    _right_ctrl;
    pll::odometry            _odometry;

    pll::chassis_speeds _desired;
    float               _left_vel_mps;   // [m/s]
    float               _right_vel_mps;  // [m/s]

    int16_t _gyro_bias;   // [LSB]
    bool    _imu_ok;
};

}  // namespace pll::subsystems
