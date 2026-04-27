#include "robot.hh"

namespace pll {

robot::robot() = default;

bool robot::init() {
    return _drivetrain.init();
}

void robot::update(float dt_s) {
    _drivetrain.update(dt_s);
}

pll::subsystems::drivetrain& robot::drivetrain() {
    return _drivetrain;
}

}  // namespace pll
