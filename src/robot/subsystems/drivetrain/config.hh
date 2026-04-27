#pragma once

#include <stdint.h>

namespace pll::config {

constexpr float   WHEEL_RADIUS_M  = 0.016f;   // [m]
constexpr float   TRACK_WIDTH_M   = 0.096f;   // [m]

constexpr float   ENCODER_CPR       = 360.0f; // [counts / revolution]
constexpr float   METERS_PER_COUNT  = (2.0f * 3.14159265f * WHEEL_RADIUS_M) / ENCODER_CPR;

constexpr int16_t MOTOR_MAX_CMD     = 400;    // [-400, 400]

constexpr float   GYRO_DPS_PER_LSB       = 0.07f; // [dps / LSB]  configureForTurnSensing
constexpr float   GYRO_RAD_PER_S_PER_LSB = GYRO_DPS_PER_LSB * (3.14159265f / 180.0f);
constexpr uint16_t GYRO_CALIBRATION_SAMPLES = 512;
constexpr float   GYRO_Z_SIGN        = 1.0f;  // set to -1.0f if heading is inverted
constexpr float   GYRO_FUSION_WEIGHT = 0.98f; // complementary filter weight [0=encoder, 1=gyro]

constexpr float MC_KFF              = 300.0f;  // [motor_cmd / (m/s)]
constexpr float MC_KP               = 100.0f;  // [motor_cmd / (m/s)]
constexpr float MC_KI               = 10.0f;   // [motor_cmd / m]
constexpr float MC_KD               = 0.0f;    // [motor_cmd / (m/s²)]
constexpr float MC_INTEGRAL_MAX_CMD = 100.0f;  // [motor_cmd]  integral anti-windup limit

}  // namespace pll::config
