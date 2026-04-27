#pragma once

namespace pll {

// Desired velocity of the robot's chassis, expressed in the robot frame.
struct chassis_speeds {
    float vx;    // [m/s]   forward (+) / backward (-) linear velocity
    float omega; // [rad/s] counter-clockwise (+) angular velocity
};

}  // namespace pll
