#pragma once

#include <stdint.h>

namespace pll::config {

constexpr int16_t  TR_SPEED_MIN            = 100;  // [motor_cmd]  first speed level tested
constexpr int16_t  TR_SPEED_MAX            = 400;  // [motor_cmd]  last speed level tested
constexpr int16_t  TR_SPEED_STEP           = 10;   // [motor_cmd]  increment per level

constexpr uint16_t TR_LOOP_PERIOD_MS       = 16; // 16;   // [ms]  fixed tick period (~62.5 Hz)

constexpr int16_t  TR_RELAY_OUTPUT         = 80;   // [motor_cmd]  relay differential amplitude
constexpr uint8_t  TR_RELAY_MIN_CROSSINGS  = 6;    // half-cycles required to accept Pu
constexpr uint32_t TR_RELAY_MAX_MS         = 8000; // [ms]  relay phase timeout
constexpr uint32_t TR_RELAY_SETTLE_MS      = 300;  // [ms]  discard initial transients
constexpr int16_t  TR_RELAY_AMP_LIMIT      = 1900; // [error_units]  peak error ceiling; above = abort

// Ziegler-Nichols PD coefficients from relay identification.
// kp = TR_ZN_KP_FACTOR * Ku
// kd = TR_ZN_KD_FACTOR * Ku * (Pu_ms / TR_LOOP_PERIOD_MS)  [discrete-time]
constexpr float    TR_ZN_KP_FACTOR         = 2.5f;
constexpr float    TR_ZN_KD_FACTOR         = 0.35f;

// Curve gain scaling: kp_curve = kp_straight * (BASE + level_index * SPEED)
// level_index is 0 at TR_SPEED_MIN, increments by 1 per TR_SPEED_STEP
constexpr float    TR_CURVE_KP_BASE_SCALE  = 2.0f;
constexpr float    TR_CURVE_KP_SPEED_SCALE = 0.15f;
constexpr float    TR_CURVE_KD_BASE_SCALE  = 1.2f;
constexpr float    TR_CURVE_KD_SPEED_SCALE = 0.05f;

constexpr uint32_t TR_VERIFY_DURATION_MS   = 4000; // [ms]  PD verification run duration
constexpr uint32_t TR_VERIFY_SETTLE_MS     = 600;  // [ms]  discard initial samples in verify
constexpr uint32_t TR_VERIFY_MSE_LIMIT     = 18750;// scaled MSE limit (= 300000 >> 4)
constexpr uint8_t  TR_VERIFY_FAIL_SAMPLES  = 20;   // consecutive line-lost samples = failure (unused — kept for reference)

constexpr float    TR_SAFETY_FACTOR        = 0.75f;// multiply gains by this on relay failure

// ── Line recovery ─────────────────────────────────────────────────────────────
constexpr uint8_t  TR_RECOVER_TRIGGER_SAMPLES = 5;    // consecutive lost samples → enter recovery
constexpr uint32_t TR_RECOVER_MAX_MS          = 10000;// [ms]  max time allowed to reacquire line
constexpr uint32_t TR_RECOVER_BACKUP_MS       = 1000;  // [ms]  reverse before spinning
constexpr int16_t  TR_RECOVER_BACKUP_SPEED    = 80;   // [motor_cmd]  reverse speed
constexpr uint8_t  TR_RECOVER_SPOKES          = 8;    // [count]  radial probes (~45° each)
constexpr uint32_t TR_RECOVER_SPIKE_ROTATE_MS = 200;  // [ms]  rotation between spokes (~45°)
constexpr uint32_t TR_RECOVER_SPIKE_FWD_MS    = 400;  // [ms]  forward probe duration
constexpr int16_t  TR_RECOVER_SPIKE_SPEED     = 80;   // [motor_cmd]  rotation and probe speed
constexpr uint8_t  TR_RECOVER_FOUND_SAMPLES   = 5;    // consecutive on-line samples = reacquired

constexpr uint16_t TR_TONE_RECOVER_HZ      = 600;  // [Hz]  recovery initiated
constexpr uint16_t TR_TONE_RECOVER_MS      = 300;  // [ms]
constexpr uint16_t TR_TONE_RELAY_START_HZ  = 1200; // [Hz]
constexpr uint16_t TR_TONE_RELAY_START_MS  = 100;  // [ms]
constexpr uint16_t TR_TONE_TUNING_DONE_HZ  = 2400; // [Hz]
constexpr uint16_t TR_TONE_TUNING_DONE_MS  = 200;  // [ms]
constexpr uint16_t TR_TONE_VERIFY_PASS_HZ  = 3000; // [Hz]
constexpr uint16_t TR_TONE_VERIFY_PASS_MS  = 300;  // [ms]
constexpr uint16_t TR_TONE_FAIL_HZ         = 400;  // [Hz]
constexpr uint16_t TR_TONE_FAIL_MS         = 500;  // [ms]
constexpr uint16_t TR_TONE_FINAL_DONE_HZ   = 4000; // [Hz]
constexpr uint16_t TR_TONE_FINAL_DONE_MS   = 600;  // [ms]
constexpr uint8_t  TR_TONE_VOLUME          = 12;   // [0-15]

}  // namespace pll::config
