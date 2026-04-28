#include "pid_trainer.hh"
#include "line_follower_config.hh"
#include <Pololu3piPlus32U4Motors.h>
#include <PololuBuzzer.h>
#include <Arduino.h>

namespace pll::trainer {

using namespace pll::config;
using Mot = Pololu3piPlus32U4::Motors;

static constexpr float PI_F = 3.14159265f;

// ── Helpers ───────────────────────────────────────────────────────────────────

int16_t pid_trainer::clamp16(int16_t v, int16_t lo, int16_t hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

void pid_trainer::play_tone(uint16_t hz, uint16_t ms) {
    PololuBuzzer::playFrequency(hz, ms, TR_TONE_VOLUME);
}

void pid_trainer::apply_relay(int16_t error) {
    _relay_cmd = (error > 0) ? _relay_amplitude : -_relay_amplitude;
    Mot::setSpeeds(clamp16(_current_speed + _relay_cmd, -400, 400),
                   clamp16(_current_speed - _relay_cmd, -400, 400));
}

void pid_trainer::apply_pd(int16_t error) {
    const int16_t d_error = error - _prev_error;
    const int16_t abs_err = error < 0 ? -error : error;

    const float diff_f = _candidate_gains.kp_straight * (float)error
                       + _candidate_gains.kd_straight * (float)d_error;
    const int16_t speed_diff = (diff_f >  400.0f) ?  400
                             : (diff_f < -400.0f) ? -400
                             : (int16_t)diff_f;

    const int16_t min_spd = _current_speed / 2;
    int16_t actual_base = _current_speed
        - (int16_t)(_candidate_gains.speed_reduction * (float)abs_err);
    actual_base = clamp16(actual_base, min_spd, _current_speed);

    Mot::setSpeeds(clamp16(actual_base + speed_diff, -400, 400),
                   clamp16(actual_base - speed_diff, -400, 400));
    _prev_error = error;
}

void pid_trainer::store_failed_result() {
    if (_speed_level > 0 && _results[_speed_level - 1].valid) {
        _candidate_gains.kp_straight = _results[_speed_level - 1].gains.kp_straight * TR_SAFETY_FACTOR;
        _candidate_gains.kd_straight = _results[_speed_level - 1].gains.kd_straight * TR_SAFETY_FACTOR;
        _candidate_gains.kp_curve    = _results[_speed_level - 1].gains.kp_curve    * TR_SAFETY_FACTOR;
        _candidate_gains.kd_curve    = _results[_speed_level - 1].gains.kd_curve    * TR_SAFETY_FACTOR;
        _candidate_gains.speed_reduction = _results[_speed_level - 1].gains.speed_reduction;
    } else {
        _candidate_gains = {0.02f, 0.01f, 0.04f, 0.012f,
                            (float)(_current_speed / 2) / (float)LF_CURVE_THRESHOLD};
    }
    _results[_speed_level] = {_candidate_gains, _current_speed, 0, false};
}

// ── OLED display ──────────────────────────────────────────────────────────────

void pid_trainer::render_display() {
    _oled.clear();

    _oled.gotoXY(0, 0);
    _oled.print(F("S:"));
    _oled.print(_current_speed);
    _oled.print(' ');
    switch (_phase) {
        case trainer_phase::relay_settle:
        case trainer_phase::relay_measure:  _oled.print(F("REL")); break;
        case trainer_phase::compute_gains:  _oled.print(F("CMP")); break;
        case trainer_phase::verify_settle:
        case trainer_phase::verify_measure: _oled.print(F("VFY")); break;
        case trainer_phase::recover:        _oled.print(F("RCV")); break;
        case trainer_phase::advance_speed:  _oled.print(F("ADV")); break;
        case trainer_phase::done:           _oled.print(F("DONE")); break;
        default: break;
    }

    if (_phase == trainer_phase::verify_settle  ||
        _phase == trainer_phase::verify_measure ||
        _phase == trainer_phase::advance_speed  ||
        _phase == trainer_phase::done) {

        _oled.gotoXY(0, 1);
        _oled.print(F("Ks:"));
        _oled.print((int16_t)(_candidate_gains.kp_straight * 10000.0f));

        _oled.gotoXY(0, 2);
        _oled.print(F("Ds:"));
        _oled.print((int16_t)(_candidate_gains.kd_straight * 10000.0f));

        _oled.gotoXY(0, 3);
        _oled.print(F("Kc:"));
        _oled.print((int16_t)(_candidate_gains.kp_curve * 10000.0f));
    }
}

// ── Phase entry helpers ───────────────────────────────────────────────────────

void pid_trainer::enter_relay_settle() {
    _relay_amplitude = TR_RELAY_OUTPUT;
    const int16_t headroom = 400 - _current_speed;
    if (_relay_amplitude > headroom) _relay_amplitude = headroom;
    if (_relay_amplitude < 10)       _relay_amplitude = 10;

    _relay_cmd          = _relay_amplitude;
    _crossing_count     = 0;
    _sum_half_period_ms = 0;
    _relay_peak         = 0;
    _prev_error         = 0;
    _fail_streak        = 0;
    _phase_start_ms     = millis();

    _follower.arm_sensing();
    play_tone(TR_TONE_RELAY_START_HZ, TR_TONE_RELAY_START_MS);
}

trainer_phase pid_trainer::enter_recovery(trainer_phase return_to) {
    Mot::setSpeeds(0, 0);
    _recover_return_phase  = return_to;
    _recover_start_ms      = millis();
    _recover_step_start_ms = millis();
    _recover_found_streak  = 0;
    _recover_sub_step      = 0;  // begin with BACKUP
    _recover_spoke_idx     = 0;
    _fail_streak           = 0;
    play_tone(TR_TONE_RECOVER_HZ, TR_TONE_RECOVER_MS);
    return trainer_phase::recover;
}

// ── Phase tick handlers ───────────────────────────────────────────────────────

trainer_phase pid_trainer::tick_relay_settle(const pll::subsystems::sensor_reading_t& r) {
    if (r.line_lost) {
        _fail_streak++;
        if (_fail_streak >= TR_RECOVER_TRIGGER_SAMPLES)
            return enter_recovery(trainer_phase::relay_settle);
    } else {
        _fail_streak = 0;
        apply_relay(r.error);
        _prev_error = r.error;
    }

    if (millis() - _phase_start_ms >= TR_RELAY_SETTLE_MS) {
        _last_crossing_ms = millis();
        _fail_streak      = 0;
        return trainer_phase::relay_measure;
    }
    return trainer_phase::relay_settle;
}

trainer_phase pid_trainer::tick_relay_measure(const pll::subsystems::sensor_reading_t& r) {
    const uint32_t now     = millis();
    const uint32_t elapsed = now - _phase_start_ms;

    // Non-recoverable termination: timeout or measurement limits.
    if (elapsed > TR_RELAY_MAX_MS || _crossing_count > 40 || _relay_peak > TR_RELAY_AMP_LIMIT) {
        Mot::setSpeeds(0, 0);
        if (_crossing_count >= TR_RELAY_MIN_CROSSINGS)
            return trainer_phase::compute_gains;
        store_failed_result();
        play_tone(TR_TONE_FAIL_HZ, TR_TONE_FAIL_MS);
        return trainer_phase::advance_speed;
    }

    if (r.line_lost) {
        _fail_streak++;
        if (_fail_streak >= TR_RECOVER_TRIGGER_SAMPLES)
            return enter_recovery(trainer_phase::relay_settle);
        // Keep driving toward last known direction while streak is building.
        apply_relay(_prev_error);
    } else {
        _fail_streak = 0;

        // Track peak oscillation amplitude.
        const int16_t abs_err = r.error < 0 ? -r.error : r.error;
        if (abs_err > _relay_peak) _relay_peak = abs_err;

        // Zero-crossing detection: sign change in error.
        const bool prev_pos = (_prev_error > 0);
        const bool curr_pos = (r.error    > 0);
        if (prev_pos != curr_pos) {
            _sum_half_period_ms += now - _last_crossing_ms;
            _last_crossing_ms    = now;
            _crossing_count++;
        }

        apply_relay(r.error);
        _prev_error = r.error;
    }

    if (_crossing_count >= TR_RELAY_MIN_CROSSINGS) {
        Mot::setSpeeds(0, 0);
        return trainer_phase::compute_gains;
    }
    return trainer_phase::relay_measure;
}

trainer_phase pid_trainer::tick_compute_gains() {
    float kp, kd;

    if (_crossing_count >= TR_RELAY_MIN_CROSSINGS && _relay_peak > 0) {
        const float pu_ms = 2.0f * (float)_sum_half_period_ms / (float)_crossing_count;
        const float ku    = (4.0f * (float)_relay_amplitude) / (PI_F * (float)_relay_peak);
        kp = TR_ZN_KP_FACTOR * ku;
        kd = TR_ZN_KD_FACTOR * ku * (pu_ms / (float)TR_LOOP_PERIOD_MS);
    } else {
        store_failed_result();
        play_tone(TR_TONE_FAIL_HZ, TR_TONE_FAIL_MS);
        return trainer_phase::advance_speed;
    }

    const float lvl            = (float)_speed_level;
    const float kp_curve_scale = TR_CURVE_KP_BASE_SCALE + lvl * TR_CURVE_KP_SPEED_SCALE;
    const float kd_curve_scale = TR_CURVE_KD_BASE_SCALE + lvl * TR_CURVE_KD_SPEED_SCALE;
    const float speed_reduction = (float)(_current_speed / 2) / (float)LF_CURVE_THRESHOLD;

    _candidate_gains = {
        kp, kd,
        kp * kp_curve_scale,
        kd * kd_curve_scale,
        speed_reduction,
    };

    play_tone(TR_TONE_TUNING_DONE_HZ, TR_TONE_TUNING_DONE_MS);

    _verify_sum_sq  = 0;
    _verify_samples = 0;
    _fail_streak    = 0;
    _prev_error     = 0;
    _phase_start_ms = millis();
    _follower.arm_sensing();

    return trainer_phase::verify_settle;
}

trainer_phase pid_trainer::tick_verify_settle(const pll::subsystems::sensor_reading_t& r) {
    if (r.line_lost) {
        _fail_streak++;
        if (_fail_streak >= TR_RECOVER_TRIGGER_SAMPLES)
            return enter_recovery(trainer_phase::verify_settle);
    } else {
        _fail_streak = 0;
        apply_pd(r.error);
    }

    if (millis() - _phase_start_ms >= TR_VERIFY_SETTLE_MS) {
        _phase_start_ms = millis();
        _fail_streak    = 0;
        return trainer_phase::verify_measure;
    }
    return trainer_phase::verify_settle;
}

trainer_phase pid_trainer::tick_verify_measure(const pll::subsystems::sensor_reading_t& r) {
    if (r.line_lost) {
        _fail_streak++;
        if (_fail_streak >= TR_RECOVER_TRIGGER_SAMPLES)
            return enter_recovery(trainer_phase::verify_settle);
    } else {
        _fail_streak = 0;
        apply_pd(r.error);
        _verify_sum_sq += (uint32_t)((int32_t)r.error * r.error) >> 4;
        _verify_samples++;
    }

    if (millis() - _phase_start_ms >= TR_VERIFY_DURATION_MS) {
        Mot::setSpeeds(0, 0);
        const uint32_t mse   = _verify_samples > 0 ? _verify_sum_sq / _verify_samples : 0;
        const bool     valid = (mse <= TR_VERIFY_MSE_LIMIT);

        _results[_speed_level] = {_candidate_gains, _current_speed, mse, valid};

        if (valid) {
            _best_valid_level = _speed_level;
            play_tone(TR_TONE_VERIFY_PASS_HZ, TR_TONE_VERIFY_PASS_MS);
        } else {
            play_tone(TR_TONE_FAIL_HZ, TR_TONE_FAIL_MS);
        }
        return trainer_phase::advance_speed;
    }
    return trainer_phase::verify_measure;
}

trainer_phase pid_trainer::tick_recover(const pll::subsystems::sensor_reading_t& r) {
    // Line reacquisition check — runs every tick regardless of sub-step.
    if (!r.line_lost) {
        _recover_found_streak++;
        if (_recover_found_streak >= TR_RECOVER_FOUND_SAMPLES) {
            Mot::setSpeeds(0, 0);
            play_tone(TR_TONE_RELAY_START_HZ, TR_TONE_RELAY_START_MS);

            if (_recover_return_phase == trainer_phase::relay_settle) {
                enter_relay_settle();
                return trainer_phase::relay_settle;
            }
            // verify_settle: retain candidate gains, reset measurement state.
            _verify_sum_sq  = 0;
            _verify_samples = 0;
            _fail_streak    = 0;
            _prev_error     = 0;
            _phase_start_ms = millis();
            _follower.arm_sensing();
            return trainer_phase::verify_settle;
        }
    } else {
        _recover_found_streak = 0;
    }

    // Global recovery timeout.
    if (millis() - _recover_start_ms >= TR_RECOVER_MAX_MS) {
        Mot::setSpeeds(0, 0);
        store_failed_result();
        play_tone(TR_TONE_FAIL_HZ, TR_TONE_FAIL_MS);
        return trainer_phase::advance_speed;
    }

    // Spike/star search: backup → then for each spoke: rotate 45°, probe forward, return.
    const uint32_t step_elapsed = millis() - _recover_step_start_ms;

    switch (_recover_sub_step) {
        case 0:  // BACKUP — reverse away from current position
            Mot::setSpeeds(-TR_RECOVER_BACKUP_SPEED, -TR_RECOVER_BACKUP_SPEED);
            if (step_elapsed >= TR_RECOVER_BACKUP_MS) {
                Mot::setSpeeds(0, 0);
                _recover_spoke_idx     = 0;
                _recover_sub_step      = 1;  // → FORWARD
                _recover_step_start_ms = millis();
            }
            break;

        case 1:  // FORWARD — probe along current spoke direction
            Mot::setSpeeds(TR_RECOVER_SPIKE_SPEED, TR_RECOVER_SPIKE_SPEED);
            if (step_elapsed >= TR_RECOVER_SPIKE_FWD_MS) {
                _recover_sub_step      = 2;  // → RETURN
                _recover_step_start_ms = millis();
            }
            break;

        case 2:  // RETURN — drive back to hub of star
            Mot::setSpeeds(-TR_RECOVER_SPIKE_SPEED, -TR_RECOVER_SPIKE_SPEED);
            if (step_elapsed >= TR_RECOVER_SPIKE_FWD_MS) {
                _recover_spoke_idx++;
                if (_recover_spoke_idx >= TR_RECOVER_SPOKES) {
                    // All spokes exhausted — give up on this speed level.
                    Mot::setSpeeds(0, 0);
                    store_failed_result();
                    play_tone(TR_TONE_FAIL_HZ, TR_TONE_FAIL_MS);
                    return trainer_phase::advance_speed;
                }
                Mot::setSpeeds(0, 0);
                _recover_sub_step      = 3;  // → ROTATE
                _recover_step_start_ms = millis();
            }
            break;

        case 3:  // ROTATE — spin ~45° clockwise to next spoke
            Mot::setSpeeds(TR_RECOVER_SPIKE_SPEED, -TR_RECOVER_SPIKE_SPEED);
            if (step_elapsed >= TR_RECOVER_SPIKE_ROTATE_MS) {
                Mot::setSpeeds(0, 0);
                _recover_sub_step      = 1;  // → FORWARD
                _recover_step_start_ms = millis();
            }
            break;
    }

    return trainer_phase::recover;
}

trainer_phase pid_trainer::tick_advance_speed() {
    _speed_level++;
    _current_speed += TR_SPEED_STEP;

    if (_speed_level >= MAX_LEVELS || _current_speed > TR_SPEED_MAX) {
        Mot::setSpeeds(0, 0);
        play_tone(TR_TONE_FINAL_DONE_HZ, TR_TONE_FINAL_DONE_MS);
        return trainer_phase::done;
    }

    enter_relay_settle();
    return trainer_phase::relay_settle;
}

// ── Public API ────────────────────────────────────────────────────────────────

pid_trainer::pid_trainer(pll::subsystems::line_follower& follower,
                         Pololu3piPlus32U4::OLED&        oled)
    : _follower(follower),
      _oled(oled),
      _phase(trainer_phase::idle),
      _current_speed(TR_SPEED_MIN),
      _speed_level(0),
      _best_valid_level(0xFF),
      _phase_start_ms(0),
      _relay_amplitude(TR_RELAY_OUTPUT),
      _relay_cmd(TR_RELAY_OUTPUT),
      _prev_error(0),
      _last_crossing_ms(0),
      _sum_half_period_ms(0),
      _crossing_count(0),
      _relay_peak(0),
      _verify_sum_sq(0),
      _verify_samples(0),
      _fail_streak(0),
      _recover_start_ms(0),
      _recover_step_start_ms(0),
      _recover_found_streak(0),
      _recover_sub_step(0),
      _recover_spoke_idx(0),
      _recover_return_phase(trainer_phase::relay_settle),
      _candidate_gains{},
      _results{} {}

void pid_trainer::start() {
    _speed_level      = 0;
    _current_speed    = TR_SPEED_MIN;
    _best_valid_level = 0xFF;

    for (uint8_t i = 0; i < MAX_LEVELS; i++) _results[i] = {};

    enter_relay_settle();
    _phase = trainer_phase::relay_settle;
    render_display();
}

bool pid_trainer::update() {
    if (_phase == trainer_phase::done || _phase == trainer_phase::idle) return false;

    pll::subsystems::sensor_reading_t r{};
    if (_phase != trainer_phase::compute_gains &&
        _phase != trainer_phase::advance_speed) {
        r = _follower.read_sensors();
    }

    const trainer_phase prev = _phase;

    switch (_phase) {
        case trainer_phase::relay_settle:   _phase = tick_relay_settle(r);   break;
        case trainer_phase::relay_measure:  _phase = tick_relay_measure(r);  break;
        case trainer_phase::compute_gains:  _phase = tick_compute_gains();   break;
        case trainer_phase::verify_settle:  _phase = tick_verify_settle(r);  break;
        case trainer_phase::verify_measure: _phase = tick_verify_measure(r); break;
        case trainer_phase::recover:        _phase = tick_recover(r);        break;
        case trainer_phase::advance_speed:  _phase = tick_advance_speed();   break;
        default: break;
    }

    if (_phase != prev) render_display();

    return _phase != trainer_phase::done;
}

bool pid_trainer::best_result(trainer_result_t& out) const {
    if (_best_valid_level == 0xFF) return false;
    out = _results[_best_valid_level];
    return true;
}

trainer_phase pid_trainer::phase() const { return _phase; }

}  // namespace pll::trainer
