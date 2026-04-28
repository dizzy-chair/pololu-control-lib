#pragma once

#include <stdint.h>

namespace pll::config {

constexpr float    LF_KP_STRAIGHT     = 0.0200f;
constexpr float    LF_KD_STRAIGHT     = 0.0800f;

constexpr float    LF_KP_CURVE        = 0.0350f;
constexpr float    LF_KD_CURVE        = 0.1500f;

constexpr int16_t  LF_BASE_SPEED      = 300;   // [motor_cmd]
constexpr int16_t  LF_MIN_SPEED       = 80;    // [motor_cmd]
constexpr int16_t  LF_MAX_SPEED       = 400;   // [motor_cmd]

constexpr float    LF_SPEED_REDUCTION = 0.3f; // [motor_cmd / error_unit]

constexpr int16_t  LF_CURVE_THRESHOLD = 400;   // [error_units]
constexpr int16_t  LF_CURVE_HYST      =  10;    // [error_units]

constexpr uint16_t LF_CHIRP_CURVE_FREQ    = 2000;  // [Hz]
constexpr uint16_t LF_CHIRP_STRAIGHT_FREQ = 1000;  // [Hz]
constexpr uint16_t LF_CHIRP_DURATION_MS   = 40;    // [ms]
constexpr uint8_t  LF_CHIRP_VOLUME        = 12;    // [0-15]

constexpr uint16_t LF_CALIBRATION_SAMPLES = 200;
constexpr int16_t  LF_CALIBRATION_SPEED   = 60;   // [motor_cmd]

// true = white line on dark background, false = black line on light background
constexpr bool LF_FOLLOW_WHITE = true;

constexpr int16_t LF_LINE_LOST_THRESHOLD = 1800; // [error_units]  |error| above this = line lost

}  // namespace pll::config
