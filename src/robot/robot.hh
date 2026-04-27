#pragma once

#include "subsystems/drivetrain/drivetrain.hh"

namespace pll {

class robot {
public:
    robot();

    // Call once from setup(), with the robot held still. Returns false if IMU not detected.
    bool init();

    void update(float dt_s);

    pll::subsystems::drivetrain& drivetrain();

private:
    pll::subsystems::drivetrain _drivetrain;
};

}  // namespace pll
