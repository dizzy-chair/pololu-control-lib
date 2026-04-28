#pragma once

#include <stdint.h>

namespace pll::config {

constexpr float    SLF_KP                  = 0.40f;
constexpr float    SLF_KD                  = 2.7f;
constexpr int16_t  SLF_BASE_SPEED          = 350;
constexpr int16_t  SLF_MOTOR_MAX_CMD       = 400;
constexpr uint16_t SLF_CALIBRATION_SAMPLES = 250;
constexpr int16_t  SLF_CALIBRATION_SPEED   = 60;
constexpr bool     SLF_FOLLOW_WHITE        = true;

}  // namespace pll::config
