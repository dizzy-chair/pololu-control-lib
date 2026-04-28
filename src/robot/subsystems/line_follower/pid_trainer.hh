#pragma once

#include <stdint.h>
#include <Pololu3piPlus32U4OLED.h>
#include "line_follower.hh"
#include "pid_trainer_config.hh"

namespace pll::trainer {

enum class trainer_phase : uint8_t {
    idle,
    relay_settle,   // running relay, discarding initial transients
    relay_measure,  // counting zero-crossings, accumulating Pu and amplitude
    compute_gains,  // one-shot ZN calculation; no sensor read
    verify_settle,  // PD control active, discarding initial samples
    verify_measure, // accumulating MSE, watching for line loss
    recover,        // line lost — backing up and spinning to reacquire
    advance_speed,  // increment speed, setup next level or finish
    done,
};

struct trainer_result_t {
    pll::subsystems::line_follower_gains_t gains;
    int16_t  base_speed;  // [motor_cmd]
    uint32_t verify_mse;  // sum(error²>>4) / samples from verify phase
    bool     valid;       // false = this level failed relay or verification
};

// Autonomous PD gain trainer using relay feedback identification (Åström-Hägglund)
// followed by Ziegler-Nichols PD tuning formulas. Runs through a configurable
// speed sweep (TR_SPEED_MIN → TR_SPEED_MAX) and outputs the best validated gain set.
class pid_trainer {
public:
    pid_trainer(pll::subsystems::line_follower& follower,
                Pololu3piPlus32U4::OLED&        oled);

    // Call once after follower.calibrate() to begin the sweep.
    void start();

    // Non-blocking tick. Call every TR_LOOP_PERIOD_MS from loop().
    // Returns true while training is running, false when done.
    bool update();

    // After update() returns false, fills out with the highest-speed validated result.
    // Returns false if no level passed verification.
    bool best_result(trainer_result_t& out) const;

    trainer_phase phase() const;

private:
    pll::subsystems::line_follower& _follower;
    Pololu3piPlus32U4::OLED&        _oled;

    trainer_phase _phase;
    int16_t  _current_speed;       // [motor_cmd]  speed being tested this level
    uint8_t  _speed_level;         // 0-based index into _results
    uint8_t  _best_valid_level;    // index of highest valid result; 0xFF = none

    // Relay measurement state
    uint32_t _phase_start_ms;
    int16_t  _relay_amplitude;     // [motor_cmd]  effective relay, clamped to headroom
    int16_t  _relay_cmd;           // [motor_cmd]  current relay sign: +amp or -amp
    int16_t  _prev_error;          // [error_units] error from previous tick
    uint32_t _last_crossing_ms;    // timestamp of most recent zero-crossing
    uint32_t _sum_half_period_ms;  // accumulated sum of half-periods
    uint8_t  _crossing_count;
    int16_t  _relay_peak;          // [error_units]  maximum |error| observed during relay

    // Verification state
    uint32_t _verify_sum_sq;       // accumulates (error²) >> 4 to prevent overflow
    uint32_t _verify_samples;
    uint8_t  _fail_streak;         // consecutive line-lost samples

    // Recovery state
    uint32_t      _recover_start_ms;     // millis() when recovery phase began
    uint32_t      _recover_step_start_ms;// millis() when current sub-step began
    uint8_t       _recover_found_streak; // consecutive on-line samples during recovery
    uint8_t       _recover_sub_step;     // 0=BACKUP 1=FORWARD 2=RETURN 3=ROTATE
    uint8_t       _recover_spoke_idx;    // current spoke (0 to TR_RECOVER_SPOKES-1)
    trainer_phase _recover_return_phase; // phase to resume after reacquisition

    pll::subsystems::line_follower_gains_t _candidate_gains;

    static constexpr uint8_t MAX_LEVELS =
        (pll::config::TR_SPEED_MAX - pll::config::TR_SPEED_MIN)
        / pll::config::TR_SPEED_STEP + 1;

    trainer_result_t _results[MAX_LEVELS];

    trainer_phase tick_relay_settle (const pll::subsystems::sensor_reading_t& r);
    trainer_phase tick_relay_measure(const pll::subsystems::sensor_reading_t& r);
    trainer_phase tick_compute_gains();
    trainer_phase tick_verify_settle (const pll::subsystems::sensor_reading_t& r);
    trainer_phase tick_verify_measure(const pll::subsystems::sensor_reading_t& r);
    trainer_phase tick_recover(const pll::subsystems::sensor_reading_t& r);
    trainer_phase tick_advance_speed();

    void enter_relay_settle();
    trainer_phase enter_recovery(trainer_phase return_to);
    void apply_relay(int16_t error);
    void apply_pd(int16_t error);
    void store_failed_result();
    void render_display();
    void play_tone(uint16_t hz, uint16_t ms);

    static int16_t clamp16(int16_t v, int16_t lo, int16_t hi);
};

}  // namespace pll::trainer
