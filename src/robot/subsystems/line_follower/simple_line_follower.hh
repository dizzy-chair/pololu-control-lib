#pragma once

#include <Pololu3piPlus32U4LineSensors.h>
#include "robot/subsystems/drivetrain/drivetrain.hh"

namespace pll::subsystems {

enum class simple_follower_state : uint8_t { idle, running };

class simple_line_follower {
public:
    simple_line_follower(pll::subsystems::drivetrain& dt, bool follow_white);

    // Blocking calibration sweep — call from setup or button handler.
    void calibrate();

    // Transition idle → running.
    void start();

    // Cut motors and return to idle.
    void stop();

    // Run one PD control tick — call each loop iteration when state == running.
    void update();

    simple_follower_state state()         const;
    bool                  is_calibrated() const;

private:
    pll::subsystems::drivetrain&   _drivetrain;
    Pololu3piPlus32U4::LineSensors _sensors;

    bool                  _follow_white;
    bool                  _calibrated = false;
    simple_follower_state _state      = simple_follower_state::idle;
    int16_t               _prev_error = 0;
};

}  // namespace pll::subsystems
