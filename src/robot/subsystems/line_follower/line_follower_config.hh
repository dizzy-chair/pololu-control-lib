#pragma once

#include <stdint.h>

namespace pll::config {

constexpr float    LF_KP_STRAIGHT     = 0.005f;
constexpr float    LF_KD_STRAIGHT     = 0.5f;

constexpr float    LF_KP_CURVE        = 0.11f;
constexpr float    LF_KD_CURVE        = 0.25f;

constexpr int16_t  LF_BASE_SPEED      = 200;   // [motor_cmd]
constexpr int16_t  LF_MIN_SPEED       = 40;    // [motor_cmd]
constexpr int16_t  LF_MAX_SPEED       = 400;   // [motor_cmd]

constexpr float    LF_SPEED_REDUCTION = 0.05f; // [motor_cmd / error_unit]

constexpr int16_t  LF_CURVE_THRESHOLD = 300;   // [error_units]
constexpr int16_t  LF_CURVE_HYST      = 75;    // [error_units]

constexpr uint16_t LF_CHIRP_CURVE_FREQ    = 2000;  // [Hz]
constexpr uint16_t LF_CHIRP_STRAIGHT_FREQ = 1000;  // [Hz]
constexpr uint16_t LF_CHIRP_DURATION_MS   = 40;    // [ms]
constexpr uint8_t  LF_CHIRP_VOLUME        = 15;    // [0-15]

constexpr uint16_t LF_CALIBRATION_SAMPLES = 100;
constexpr int16_t  LF_CALIBRATION_SPEED   = 60;   // [motor_cmd]

// true = white line on dark background, false = black line on light background
constexpr bool LF_FOLLOW_WHITE = true;

}  // namespace pll::config
