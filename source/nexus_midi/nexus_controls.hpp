/*=======================================================================================
    Copyright (c) 2016 Cycfi Research

    Distributed under the MIT License [ https://opensource.org/licenses/MIT ]
   =======================================================================================*/

#if !defined(CYCFI_NEXUS_CONTROLS_HPP_JUNE_25_2026)
#define CYCFI_NEXUS_CONTROLS_HPP_JUNE_25_2026

#include "midi.hpp"
#include "util.hpp"
#include "MspFlash.h"
#ifdef NEXUS_SIM
#include "sim/nexus_sim.hpp"
#endif

///////////////////////////////////////////////////////////////////////////////
// Environment provided by the driver translation unit: the firmware sketch on
// the device, or the host test harness off-device. These are referenced by the
// controllers below and defined by whoever drives them.
//   midi_out:         where MIDI messages are written
//   last_cc_time:     timestamp of the last CC send (gates pitch bend)
//   reset_save_delay: defer the next flash save after a state change
///////////////////////////////////////////////////////////////////////////////
extern cycfi::midi::midi_stream midi_out;
extern uint32_t last_cc_time;
void reset_save_delay();

///////////////////////////////////////////////////////////////////////////////
// Control Mapping:
//
// Main:
//
//    program_change       5-way switch
//    channel_volume       analog
//    pitch_change         analog
//    modulation           analog
//    effect_1             analog
//    effect_2             analog
//    sustain              momentary switch
//
// Aux:
//
//    program_change +5    momentary switch
//    program_change -5    momentary switch
//    program_change +1    momentary switch
//    program_change -1    momentary switch
//    bank_select +1       momentary switch
//    bank_select -1       momentary switch
//
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Constants
///////////////////////////////////////////////////////////////////////////////
int const ch9  = P2_0; //digital
int const ch10 = P1_0; //analog and digital
int const ch11 = P1_3; //analog and digital
int const ch12 = P1_4; //analog and digital
int const ch13 = P1_5; //analog and digital
int const ch14 = P1_6; //analog and digital
int const ch15 = P1_7; //analog and digital

int const aux1 = P2_1; //digital
int const aux2 = P2_2; //digital
int const aux3 = P2_3; //digital
int const aux4 = P2_4; //digital
int const aux5 = P2_5; //digital
int const aux6 = P2_6; //digital


///////////////////////////////////////////////////////////////////////////////
// Flash Utility for persisting MIDI 7 bit data
//
// MSP430 has SEGMENT_B to SEGMENT_D available for applications to use,
// where each segment has 64 bytes. When erased, data in the flash is
// read as 0xff. You can only write once to a data slot, per segment,
// per erase cycle (actually you can write more than once but once a bit
// is reset, you cannot set it with a subsequent write).
//
// MSP430 is spec'd to allow a minimum guaranteed 10,000 erase cycles
// (100,000 erase cycles typical). We store 7-bits data into flash memory
// in a ring buffer fashion to minimize erase cycles and increase the
// possible write cycles. See link below:
//
// http://processors.wiki.ti.com/index.php/Emulating_EEPROM_in_MSP430_Flash)
//
///////////////////////////////////////////////////////////////////////////////
struct flash
{
   typedef unsigned char byte;

   flash(byte* segment_)
      : _segment(segment_)
   {}

   void erase()
   {
      Flash.erase(_segment);
   }

   bool empty() const
   {
      return (*_segment == 0xff);
   }

   byte read() const
   {
      if (empty())
         return 0xff;
      if (byte* p = find_free())
         return *(p - 1);
      return _segment[63];
   }

   void write(byte val)
   {
      byte* p = find_free();
      if (p == 0)
      {
         erase();
         Flash.write(_segment, &val, 1);
      }
      else
      {
         Flash.write(p, &val, 1);
      }
   }

   private:

   byte* find_free() const
   {
      for (int i = 0; i != 64; ++i)
         if (_segment[i] == 0xff)
            return &_segment[i];
      return 0;
   }

   byte* _segment;
};

// Persistent storage for program change / bank select (defined by the driver).
extern flash flash_b;
extern flash flash_c;

namespace cycfi
{

///////////////////////////////////////////////////////////////////////////////
// ADC seam. Production reads the hardware; a NEXUS_SIM build injects synthetic,
// time-driven test data so every control -- including pitch bend, which
// bypasses analog_read() -- can be stress-tested without moving a knob.
///////////////////////////////////////////////////////////////////////////////
inline uint16_t raw_adc(uint16_t pin)
{
#ifdef NEXUS_SIM
   return nexus_sim::sample(pin, millis());
#else
   return analogRead(pin);
#endif
}

// Pots don't travel to the physical ends of their range; clamp to the
// effective 2%–98% travel window and remap to the full 0–1023 range.
constexpr uint16_t min_x = 1024 * 0.02;
constexpr uint16_t max_x = 1024 * 0.98;

inline uint16_t analog_read(uint16_t pin)
{
   uint16_t x = raw_adc(pin);
   if (x < min_x)
      x = min_x;
   else if (x > max_x)
      x = max_x;
   return map(x, min_x, max_x, 0, 1023);
}

// Small signed-integer helpers (constexpr implies inline).
constexpr int32_t iabs(int32_t x)
{
   return x < 0 ? -x : x;
}

// Branch-free clamp. Assumes lo <= hi with small operands (no subtraction overflow), which
// holds at every call site. Adds the low correction when x < lo, the high one when x > hi.
constexpr int32_t clamp(int32_t x, int32_t lo, int32_t hi)
{
   return x + ((lo - x) & -int32_t(x < lo)) + ((hi - x) & -int32_t(x > hi));
}


///////////////////////////////////////////////////////////////////////////////
// Generic controller handling (with course and fine controls)
///////////////////////////////////////////////////////////////////////////////
template <midi::cc::controller ctrl>
struct controller
{
   // Gate in 10-bit space before shifting down to 7-bit MIDI CC. Otherwise
   // noise around a CC boundary can chatter, e.g. 126, 127, 126, 127.
   static constexpr uint16_t cc_window = 8;
   static constexpr uint16_t endpoint_snap = 0x07;
   static constexpr uint16_t endpoint_release = 0x0f;

   controller()
    : prev(0xff)
    , prev_val(0)
   {}

   void init(uint32_t val)
   {
      lp1.y = val * 8;
      lp2.y = val * 16;
      prev_val = val;
      prev = uint8_t(val >> 3);
      // Publish the seeded startup value.
      midi_out << midi::control_change{0, ctrl, prev};
   }

   void operator()(uint32_t val_)
   {
      uint32_t val = lp2(lp1(val_));
      if (val <= endpoint_snap || (prev == 0 && val <= endpoint_release))
         val = 0;
      else if (
         val >= (0x3ff - endpoint_snap)
         || (prev == 127 && val >= (0x3ff - endpoint_release)))
         val = 0x3ff;

      uint32_t delta = val > prev_val ? val - prev_val : prev_val - val;
      if (delta <= cc_window)
         return;

      prev_val = val;
      uint8_t cc = uint8_t(val >> 3);
      if (cc != prev)
      {
         prev = cc;
         last_cc_time = millis();
         midi_out << midi::control_change{0, ctrl, cc};
      }
   }

   lowpass<8, int32_t>  lp1;
   lowpass<16, int32_t> lp2;
   uint32_t             prev_val;
   uint8_t              prev;
};

///////////////////////////////////////////////////////////////////////////////
// Pitch bend controller -- FEED-FORWARD hysteresis prediction (Stage 2, v2)
//
// 10-bit ADC -> 14-bit MIDI. The spring-return hysteresis is a PREDICTABLE
// function of the peak deflection (fitted from hardware: rest = C0 + k*peak,
// k ~= 0.043), not something to learn reactively. So we COMPUTE the offset from
// the input instead of chasing the output -- which makes the off-center latch
// (the reactive servo learning a held bend) structurally impossible: the
// correction is always proportional to the deflection, so a held 1 ST bend earns
// only a ~0.04 ST nudge and is preserved, while a full pull earns the full null.
//
//   dev       = (pos - center) - C0          deflection from the DC center
//   M         = leaky peak-detector of dev, decaying to a 2/3-of-peak FLOOR over
//               ~16 s (the persistent creep -- it settles at 2/3 and STAYS)
//   corrected = dev - k*M                     hysteresis removed -> centers on release
//
// C0 (the only per-unit term) is a SLOW DC servo, gated to near-center output so a
// held bend can't drag it. A watchdog re-seeds C0 if the output is stuck off-center
// past a timeout. See the numbered notes after the class for per-field rationale.
///////////////////////////////////////////////////////////////////////////////
struct pitch_bend_controller
{
   static constexpr int32_t  center  = 8192;
   static constexpr int32_t  pb_max  = 2 * center - 1;   // 16383, 14-bit MIDI ceiling
   static constexpr int32_t  scale   = 16;      // one 10-bit step -> 14-bit
   static constexpr int32_t  nominal = 512;     // ADC nominal center

   // Input smoothing: an adaptive dynamic_smoother replaces the old lp1<8>+lp2<16>
   // cascade -- fast tracking on moves, heavy smoothing at rest. Tuned on recorded
   // gestures (deep-bend lag 22 -> 11 ms); G0 ~ the old rest smoothing.
   static constexpr int  sm_g0    = 24;
   static constexpr int  sm_sense = 512;        // power of two -> sense term is a shift

   // Hysteresis slope k = k_num / k_den. A FIXED per-unit compile-time constant (note 1):
   // the C0 servo does the adaptive centering, so k only removes the bulk hysteresis and
   // only has to be roughly right. Set it per unit at build time (note 2).
   static constexpr int32_t  k_den = 4096;
   static constexpr int32_t  k_num = 176;       // ~0.043 (per-unit; see calibration note)

   // M creep memory: tau ~ 2^14 ms ~ 16 s, decays to a 2/3-of-peak floor (note 3).
   static constexpr int      mem_shift = 14;
   static constexpr int32_t  mem_one   = 1 << mem_shift;
   static constexpr int      floor_num = 2;
   static constexpr int      floor_den = 3;

   // C0 DC servo: tau ~ 2^11 ms ~ 2 s, frozen when |deflection| > c0_gate (note 4).
   static constexpr int      c0_shift = 16;
   static constexpr int32_t  c0_one   = 1 << c0_shift;
   static constexpr int32_t  c0_clip  = 736;    // +/-46 LSB (~1.3 ST): the max physical DC wander
                                                // (hysteresis + thermal); C0 is hard-clamped here
   static constexpr int32_t  c0_gate  = 546;    // ~0.8 ST: a real bend (>1 ST) freezes C0

   // Post error-correction (note 4): nulls the predictor's leftover at a SETTLED rest. Fast
   // (tau ~ 2^post_shift ms ~ 2 s) but bounded to +/- post_clip authority and gated on stillness,
   // so it never chases a live bend and never holds a standing offset (it relaxes to 0 off-rest).
   // It does the rest-nulling the now-very-slow C0 servo can't, WITHOUT re-coupling to the bend.
   static constexpr int      post_shift  = 11;
   static constexpr int32_t  post_clip   = 460;  // ~0.8 ST: the post stage's hard authority limit
   static constexpr int32_t  still_band  = 2 * scale; // "still" = bar within ~2 LSB ...
   static constexpr uint32_t still_dwell = 400;       // ... for this long (displacement-over-time)

   static constexpr int32_t  pb_window = 16;    // output send deadband (1 step)
   // Schmitt center detent: snap-in < release so a rest at the edge can't ping-pong (note 5).
   static constexpr int32_t  detent_snap    = 2 * scale;
   static constexpr int32_t  detent_release = 5 * scale;
   static constexpr int32_t  band_thresh    = 256;  // smoother band (Q8) above this = moving -> no detent
                                                    // (256 keeps the vibrato lobes; the residual center
                                                    // toggle it leaves is softened by out_ma below)
   static constexpr uint32_t engage_dwell_ms = 50;  // stillness must hold this long before the detent
                                                    // latches -- a brief mild-vibrato crossing slips
                                                    // through un-clipped; a real rest still snaps
   static constexpr int32_t  dt_max = 32;       // clamp ms/loop (creep/servo are real-time)

   // Input slew gate. The arm tops out ~4 LSB/ms (measured); a programmer-contact glitch shifts
   // the ADC reference ~10x faster (a ~40 LSB jump in one ~1 ms loop). Reject any raw step faster
   // than this and hold the last good reading -- since a glitched level is SUSTAINED, every reading
   // stays a too-fast jump from the held value, so it is rejected for the whole event and never
   // reaches the predictor. Prevents the off-center servo latch at the source. dt is clamped so a
   // long loop (e.g. the first one after muting) can't widen the allowance enough to let one through.
   static constexpr int32_t  slew_rate   = 6;   // max physical |d raw| per ms (LSB); arm peaks ~4
   static constexpr int32_t  slew_slack  = 4;   // always allow this much (noise floor, tiny-dt loops)
   static constexpr int32_t  slew_dt_max = 6;   // clamp the per-step ms in the allowance

   // Output gain: scale the (linear, sub-full-scale) sensor swing up, symmetric so dive
   // and pull feel the same, and let the pitch clamp take the overshoot (note 7).
   static constexpr int32_t  out_num = 307;     // x1.2 (256 = x1.0)
   static constexpr int32_t  out_den = 256;

   // Startup / validity gate: mute until a valid settled rest (note 6).
   static constexpr int32_t  valid_lo   = 400;  // plausible REST band around sensor center
   static constexpr int32_t  valid_hi   = 640;  // (~512); outside = rail / disconnect / bend
   static constexpr int32_t  settle_lsb = 6;    // "settled" = pos within this of itself ...
   static constexpr uint32_t settle_ms  = 300;  // ... for this long (and past startup_holdoff)
   static constexpr uint32_t diag_ms    = 100;  // NEXUS_DIAG free-running telemetry interval (note 8)

   // Deflection of a smoothed ADC reading from nominal center, in 14-bit units.
   static int32_t deflection(int32_t pos) { return (pos - nominal) * scale; }

   pitch_bend_controller();

   void init(uint16_t pin_);
   void operator()();

   bool    startup_gate(int32_t pos, uint32_t now);
   int32_t slew_limit(int32_t raw, uint32_t now);   // reject glitch-fast input jumps at the source
   void    update_memory(int32_t dev, int32_t adev, int32_t dt);
   int32_t apply_detent(int32_t corrected, int32_t ac, int32_t band, uint32_t now);
   void    update_center(int32_t defl, int32_t c0, int32_t dt, bool still);
   void    send_pitch(
              int32_t pitch, int32_t raw, int32_t pos, int32_t c0,
              int32_t hyst);
#ifdef NEXUS_DIAG
   void    send_diag(int32_t raw, int32_t pos, int32_t c0, int32_t hyst);  // one telemetry frame
   void    send_diag_signature(uint8_t b1, uint32_t field);
#endif

   uint16_t pin;
   int32_t  held_raw;            // last raw reading the slew gate accepted (glitch reject)
   uint16_t prev;                // last emitted pitch (send-deadband reference)
   int32_t  M_acc;               // deflection memory (M = M_acc >> mem_shift)
   int32_t  peak;                // |captured peak deviation| (sets the 2/3 floor)
   int32_t  c0_acc;              // DC center estimate (C0 = c0_acc >> c0_shift)
   int32_t  c0_seed;             // startup-rest deflection: FIXED ref for M + the C0 clip (note 4)
   int32_t  post_acc;            // post error-correction (post = post_acc >> post_shift)
   int32_t  dwell_ref;           // displacement-dwell reference for the post-correction stillness
   uint32_t dwell_since;         // millis() the bar's current displacement-still streak began
   dynamic_smoother<sm_g0, sm_sense> smoother;   // adaptive input lowpass
   moving_average_n<3> out_ma;   // 3-pt output average: softens the residual center toggle
   uint32_t last_ms;             // millis() at the previous loop (real-time dt)
   uint32_t last_diag_ms;        // millis() of the last telemetry frame (free-running diag clock)
   uint32_t still_since;         // millis() the detent's still condition began (engage dwell)
   bool     centered;            // Schmitt detent state (true = snapped to center)
   bool     muted;               // startup/validity gate: holding center until a valid rest
   uint32_t valid_since;         // millis() the input started its current valid+stable streak
   int32_t  valid_ref;           // pos the startup stability check compares against
   uint32_t boot_ms;             // millis() at init -- startup hold-off (wait out the Vcc ramp)
   uint32_t startup_holdoff;     // min ms muted after boot (member so host tests can shorten it)
};

///////////////////////////////////////////////////////////////////////////////
// Notes (pitch_bend_controller)
//
// 1. Fixed k (no online learning). k is a compile-time per-unit constant, not adapted at
//    runtime. An earlier build learned k from "landings"; replaying real sessions showed
//    it did not help (a wash, slightly worse with the servo on), never converged to a
//    stable value, and fired rarely -- the C0 servo (note 4) does the real centering, so k
//    only has to be roughly right. The learner and the k flash persistence were removed.
//
// 2. Per-unit calibration. k_num ~0.043 is fine for most units since the servo absorbs the
//    rest. To set a unit exactly: a full dive and a full pull, each released and settled;
//    k = (rest_pull - rest_dive)/(peak_pull - peak_dive). Set k_num to that at build time
//    (per-unit firmware). No flash, no runtime writes.
//
// 3. M memory. Reversal-aware peak detector of dev: fast-attack each new extreme,
//    then decay toward a 2/3-of-peak floor that tracks the physical creep (the
//    hysteresis settles at ~2/3 and PERSISTS). Decaying to 0 would let the
//    corrected rest drift up as M shrinks below the bar's actual rest.
//
// 4. C0 baseline servo + post-correction. The center is split into two decoupled stages so
//    NOTHING in the bend path feeds the center (the old feedback loop made it history-dependent
//    and let it chase/strand). (a) C0: a VERY slow (tau ~65s) leaky integrator toward the RAW
//    deflection -- not defl-hyst -- stepping only when the bar is STILL and near neutral, hard-
//    clipped to +/- c0_clip of the seed. Being far slower than any gesture, it follows only the
//    thermal/bias baseline, never a per-bend shift; the clip means a detection slip can never
//    reach a real bend. (b) M is computed off the FIXED seed (defl - c0_seed), never C0, so the
//    predictor cannot feed the servo. (c) The hysteresis the predictor leaves is nulled by the
//    relaxing post-correction below, fast but bounded and still-gated, off the servo loop.
//    No watchdog: the clip is the guarantee. Verified by replaying the real captures
//    (test/replay_captures.py) -- the chase and watchdog strands go to 0, clean gestures stay 0.
//
// 5. Schmitt detent. release > snap (hysteresis) so a rest parked at the detent
//    edge can't ping-pong across the boundary, which was the rest chatter seen on
//    hardware.
//
// 6. Startup gate. The analog front-end can read a FALSE stable rest for a moment
//    before it swings on the Vcc ramp, so don't trust an early "settled" reading:
//    also require startup_holdoff ms since boot before going live. A missing whammy
//    (ADC pinned at 0) is outside the rest band, so it never goes live.
//
// 7. Output gain. The front-end is deliberately kept LINEAR (note 4 reasoning: the
//    predictor needs M to be a faithful measure of deflection, so the amp must not
//    saturate). That leaves the sensor swing filling only part of +/-8192: a full dive
//    reaches -8192 but a full pull rails at the analog amp around +7121 (+10.4 ST).
//    out_num scales the output up (x1.2) and the pitch clamp takes whatever overshoots,
//    so the pull now reaches full and the dive just clips its deepest sliver -- a single
//    SYMMETRIC gain, so both sides feel the same. Applied at the VERY END in send_pitch,
//    only to the value handed to MIDI, so the detent, the send deadband, prev, and ALL
//    the predictor math (M, hysteresis, C0 servo) stay in un-scaled units --
//    the gain changes only the emitted magnitude, never the behavior. Calibrated on
//    hardware 2026-06-26 (firm full pull peaked +7121; x1.2 maps it past full, dive clips
//    ~17% of its deepest travel).
//
// 8. Free-running diag (NEXUS_DIAG only). The send-paired diag rides each pitch_bend, so the
//    silent stretches emit nothing -- the power-on settle climb (muted) and a dead-centered
//    rest (send-deadband). To sample those, operator() also emits a frame on a diag_ms clock,
//    BEFORE the startup mute returns. send_diag resets that clock on every frame (paired or
//    clocked), so the two paths never double up within diag_ms. Telemetry only -- it reads
//    state and writes MIDI, never touches control. Keep diag_ms generous (observer effect:
//    each frame is ~3 ms of UART; pairing a frame with EVERY loop floods the bus and perturbs
//    timing). The clocked frame uses last loop's C0/M (a few ms stale); raw/pos are current.
///////////////////////////////////////////////////////////////////////////////

inline pitch_bend_controller::pitch_bend_controller()
 : pin(0)
 , held_raw(0)
 , prev(center)
 , M_acc(0)
 , peak(0)
 , c0_acc(0)
 , c0_seed(0)
 , post_acc(0)
 , dwell_ref(0)
 , dwell_since(0)
 , last_ms(0)
 , last_diag_ms(0)
 , still_since(0)
 , centered(true)
 , muted(true)
 , valid_since(0)
 , valid_ref(0)
 , boot_ms(0)
 , startup_holdoff(5000)        // ride out the Hall front-end's power-on settle
{}

inline void pitch_bend_controller::init(uint16_t pin_)
{
   pin = pin_;
   int32_t pos = raw_adc(pin);
   held_raw = pos;                // seed the slew gate to the startup reading
   smoother = pos;
   out_ma.init(center);
   int32_t defl = deflection(pos);
   M_acc = 0;
   peak = 0;
   c0_acc = defl * c0_one;        // C0 := startup deflection -> output starts centered
   c0_seed = defl;                // fixed neutral for M + the clip
   post_acc = 0;
   dwell_ref = defl;
   dwell_since = millis();
   prev = center;
   last_ms = millis();
   centered = true;
   muted = true;                  // start MUTED: wait for a valid settled rest (note 6)
   valid_ref = pos;
   valid_since = millis();
   boot_ms = millis();            // hold-off reference: don't go live during the Vcc ramp
   midi_out << midi::pitch_bend{0, uint16_t(center)};
#ifdef NEXUS_DIAG
   send_diag_signature(0x7f, startup_holdoff / 100);   // boot signature (raw=16383)
#endif
}

// Input slew gate: reject a raw jump faster than the arm can physically move (a programmer-
// contact glitch shifts the ADC reference ~10x faster than a real bend) and hold the last good
// reading. A glitched level is SUSTAINED, so every reading stays a too-fast jump from the held
// value and is rejected for the whole event -- the input never leaves the true rest, the C0 servo
// never freezes off-center, and the output can't latch. Prevents the stuck state at the source.
inline int32_t pitch_bend_controller::slew_limit(int32_t raw, uint32_t now)
{
   int32_t dt = clamp(int32_t(now - last_ms), 0, slew_dt_max);
   if (iabs(raw - held_raw) > slew_rate * dt + slew_slack)
      return held_raw;                           // non-physical jump -> reject, hold last good
   held_raw = raw;
   return raw;
}

inline void pitch_bend_controller::operator()()
{
   uint32_t now = millis();
   int32_t  raw = slew_limit(raw_adc(pin), now); // reject glitch-fast jumps before anything sees them
   int32_t  pos = smoother(raw);                 // adaptive-smoothed 10-bit

#ifdef NEXUS_DIAG
   // Free-running telemetry (note 8): emit on a clock, BEFORE the mute return and regardless of
   // the send-deadband, so the power-on settle climb and the quiet rest get sampled too -- not
   // just active sends. c0/M are last loop's (one loop ~ a few ms stale; raw/pos are current).
   if (now - last_diag_ms >= diag_ms)
      send_diag(raw, pos, c0_acc >> c0_shift, (int32_t(M_acc >> mem_shift) * k_num) / k_den);
#endif

   if (startup_gate(pos, now))                   // held at center until a valid rest
      return;

   int32_t defl = deflection(pos);               // raw deflection (14-bit)
   int32_t c0   = c0_acc >> c0_shift;            // DC center estimate
   int32_t dev  = defl - c0;                     // deflection from the LIVE center -> output
   int32_t dev_m = defl - c0_seed;               // deflection from the FIXED seed -> predictor only
                                                 //   (note 4: M off the seed, never C0 -> no loop)

   // Real-time step: creep and DC servo advance by elapsed ms, not per loop, so
   // they track physical time regardless of the device's loop rate.
   int32_t dt = clamp(int32_t(now - last_ms), 0, dt_max);
   last_ms = now;

   update_memory(dev_m, iabs(dev_m), dt);        // advance M off the FIXED-seed deflection
   int32_t M    = M_acc >> mem_shift;
   int32_t hyst = (M * k_num) / k_den;           // k * M (the predicted hysteresis)

   int32_t corrected = dev - hyst;               // bend with hysteresis removed
   int32_t ac = iabs(corrected);

   // Post error-correction (note 4). Stillness by displacement-over-time (the smoother band can't
   // see a slow ramp). When settled AND already inside the post authority, integrate the leftover
   // away; otherwise relax to 0 -- so a live bend is never chased and no standing offset survives.
   if (iabs(defl - dwell_ref) > still_band) { dwell_ref = defl; dwell_since = now; }
   bool still = (now - dwell_since >= still_dwell);
   int32_t post = post_acc >> post_shift;
   post_acc += ((still && ac <= post_clip) ? (corrected - post) : (0 - post)) * dt;
   post_acc = clamp(post_acc, -(post_clip << post_shift), post_clip << post_shift);
   post = post_acc >> post_shift;
   corrected -= post;                            // remove the settled residual
   ac = iabs(corrected);

   int32_t pitch = apply_detent(corrected, ac, smoother.band(), now); // dwelled velocity detent
   pitch = out_ma(pitch);                        // 3-pt average -> soften residual toggle
   update_center(defl, c0, dt, still);           // C0 baseline servo (raw defl, gated, clipped)
   send_pitch(pitch, raw, pos, c0, hyst);        // emit (deadband) + diagnostics
}

// Startup / validity gate: hold center (MUTED) until the input is a valid, SETTLED
// rest, suppressing the power-on swing and the no-whammy case (note 6). Seeds C0 to
// the rest on the first valid + stable reading (also handles a hot-plug after boot).
// Returns true while still muted so the caller returns and holds center; once live
// it stays live (a mid-session disconnect is left to the watchdog).
inline bool pitch_bend_controller::startup_gate(int32_t pos, uint32_t now)
{
   if (!muted)
      return false;

   int32_t dp = iabs(pos - valid_ref);
   if (pos >= valid_lo && pos <= valid_hi && dp <= settle_lsb)
   {
      if (now - valid_since >= settle_ms && now - boot_ms >= startup_holdoff)
      {
         int32_t defl0 = deflection(pos);
         c0_acc = defl0 * c0_one;             // seed the center to the settled rest
         c0_seed = defl0;                     // fixed neutral for M + the clip
         post_acc = 0;
         dwell_ref = defl0;
         dwell_since = now;
         M_acc = 0;
         peak = 0;
         last_ms = now;
         centered = true;
         muted = false;
#ifdef NEXUS_DIAG
         send_diag_signature(0x7e, (now - boot_ms) / 10);   // go-live (raw=16382)
#endif
      }
   }
   else
   {
      valid_ref = pos;                        // moved / out of range -> restart the wait
      valid_since = now;
   }
   if (prev != center)                        // park the output at center while muted
   {
      prev = uint16_t(center);
      midi_out << midi::pitch_bend{0, uint16_t(center)};
   }
   return true;
}

// Advance the creep memory M: a reversal-aware peak detector of dev (note 3).
inline void
pitch_bend_controller::update_memory(int32_t dev, int32_t adev, int32_t dt)
{
   int32_t M  = M_acc >> mem_shift;
   int32_t aM = iabs(M);
   bool reversed = ((dev < 0) != (M < 0)) && M != 0;
   if (reversed || adev > aM)                 // new excursion / extending -> capture peak
   {
      M_acc = dev * mem_one;
      peak  = adev;
   }
   else                                       // decay toward sign(M) * 2/3 * peak
   {
      int32_t target = (M < 0 ? -peak : peak) * floor_num / floor_den;
      M_acc += (target - M) * dt;             // millis-based leaky (tau = 2^mem_shift ms)
   }
}

// Schmitt center detent: snap a small rest band to exact center, with hysteresis
// (release > snap) so a rest at the edge can't ping-pong (note 5). Returns pitch.
inline int32_t
pitch_bend_controller::apply_detent(int32_t corrected, int32_t ac, int32_t band, uint32_t now)
{
   // Velocity-gated Schmitt center detent with an engage-dwell. Release is purely on
   // amplitude, so the rest wander never un-centers a still rest. Re-engagement is gated two
   // ways: by the band (a fast vibrato crossing is moving -> won't snap) AND by a dwell (the
   // still condition must hold engage_dwell_ms) -- the time discriminator the band can't give,
   // so a brief mild/slow crossing slips through un-clipped while a real rest still snaps.
   if (centered)
   {
      if (ac > detent_release)
         centered = false;
   }
   else if (ac <= detent_snap && band <= band_thresh)
   {
      if (now - still_since >= engage_dwell_ms)   // sustained stillness -> snap to center
         centered = true;
   }
   else
   {
      still_since = now;                          // moving / off-center -> restart the dwell
   }
   int32_t pitch = centered ? center : center + corrected;
   return clamp(pitch, 0, pb_max);
}

// C0 baseline servo (note 4). Tracks the RAW deflection -- NOT defl-hyst -- so it has no path to
// the predictor (M is computed off the fixed seed, note 4); that breaks the feedback loop that
// made the center history-dependent. It steps only when the bar is STILL (displacement dwell) AND
// near neutral (|defl-c0| within the gate), and is far slower than any gesture (tau ~65s), so it
// follows only the thermal/bias baseline, never a per-bend hysteresis (the predictor owns that).
// Finally it is hard-CLIPPED to +/- c0_clip of the seed = the max the rest can physically sit off
// center, so even a detection slip cannot reach a real bend or strand a release. No watchdog --
// the clip is the guarantee; the 15 s re-seed only ever centered a held bend and stranded it.
inline void pitch_bend_controller::update_center(int32_t defl, int32_t c0, int32_t dt, bool still)
{
   if (still && iabs(defl - c0) <= c0_gate)
      c0_acc += (defl - c0) * dt;             // very slow leaky integrator toward the raw rest
   c0_acc = clamp(c0_acc, (c0_seed - c0_clip) * c0_one, (c0_seed + c0_clip) * c0_one);
}

// Emit on the send-deadband; a NEXUS_DIAG build pairs each send with raw / smoothed
// pos / total offset (C0 + hysteresis) so the host can watch the predictor work.
inline void pitch_bend_controller::send_pitch(
   int32_t pitch, int32_t raw, int32_t pos, int32_t c0, int32_t hyst)
{
   int32_t delta = iabs(pitch - prev);
   if (delta == 0)
      return;                                   // unchanged -- nothing to send
   if (delta <= pb_window && pitch != center)
      return;                                   // send-deadband, but never swallow a center snap
   prev = uint16_t(pitch);
   // Output gain at the VERY END (note 7): scale ONLY the value handed to MIDI, so the
   // detent, the send deadband above, prev, and all predictor math stay un-scaled.
   int32_t out = center + ((pitch - center) * out_num) / out_den;
   midi_out << midi::pitch_bend{0, uint16_t(clamp(out, 0, pb_max))};
#ifdef NEXUS_DIAG
   send_diag(raw, pos, c0, hyst);                // pair a frame with the send (also resets the clock)
#endif
}

#ifdef NEXUS_DIAG
// One telemetry frame: raw / smoothed pos / total offset (C0 + hysteresis), each 14-bit on the
// 0x4364 SysEx. Resets the free-running clock so a send-paired frame and the clocked frame can't
// double up within diag_ms (note 8).
inline void
pitch_bend_controller::send_diag(int32_t raw, int32_t pos, int32_t c0, int32_t hyst)
{
   last_diag_ms = millis();
   int32_t off = (c0 + hyst) / scale + nominal;
   uint8_t buf[6] =
   {
      uint8_t((raw >> 7) & 0x7f), uint8_t(raw & 0x7f),
      uint8_t((pos >> 7) & 0x7f), uint8_t(pos & 0x7f),
      uint8_t((off >> 7) & 0x7f), uint8_t(off & 0x7f)
   };
   midi_out << midi::sysex<6>(0x4364, buf);
}

// Diagnostic boot / go-live signature on the 0x4364 frame: b1 = 0x7f (boot, decodes
// to raw=16383) or 0x7e (go-live, raw=16382); field = holdoff/100 or muted-ms/10.
inline void
pitch_bend_controller::send_diag_signature(uint8_t b1, uint32_t field)
{
   uint8_t sig[6] =
   {
      0x7f, b1, 0x00, 11,
      uint8_t((field >> 7) & 0x7f), uint8_t(field & 0x7f)
   };
   midi_out << midi::sysex<6>(0x4364, sig);
}
#endif

///////////////////////////////////////////////////////////////////////////////
// Program change controller
///////////////////////////////////////////////////////////////////////////////
struct program_change_controller
{
   program_change_controller()
      : curr{0}
      , base{0}
   {}

   void load()
   {
      if (!flash_b.empty())
         base = flash_b.read();
   }

   void save()
   {
      uint8_t base_ = max(min(base, 127), 0);
      if (base_ != flash_b.read())
         flash_b.write(base_);
   }

   uint8_t get()
   {
      return uint8_t{max(min(curr + base, 127), 0)};
   }

   void transmit()
   {
      midi_out << midi::program_change{0, get()};
   }

   void operator()(uint32_t val_)
   {
      uint32_t curr_ = curr * 205;
      int diff = curr_ - val_;
      if (diff < 0)
         diff = -diff;
      if (diff < 8)
         return;

      uint8_t val = (val_ * 5) / 1024;
      if (val != curr)
      {
         curr = val;
         transmit();
      }
   }

   void up(bool sw)
   {
      if (btn_up(sw) && (base < 127))
      {
         ++base;
         reset_save_delay();
         transmit();
      }
   }

   void down(bool sw)
   {
      if (btn_down(sw) && (base > 0))
      {
         --base;
         reset_save_delay();
         transmit();
      }
   }

   void group_up(bool sw)
   {
      if (grp_btn_up(sw) && (base < 127))
      {
         base += 5;
         reset_save_delay();
         transmit();
      }
   }

   void group_down(bool sw)
   {
      if (grp_btn_down(sw) && (base > 0))
      {
         base -= 5;
         reset_save_delay();
         transmit();
      }
   }

   int16_t curr;
   int16_t base;
   repeat_button<> btn_up;
   repeat_button<> btn_down;
   repeat_button<> grp_btn_up;
   repeat_button<> grp_btn_down;
};

///////////////////////////////////////////////////////////////////////////////
// Sustain control
///////////////////////////////////////////////////////////////////////////////
struct sustain_controller
{
   void init(bool sw)
   {
      // Avoid a duplicate startup edge and publish the initial state.
      edge.init(sw);
      midi_out << midi::control_change{
         0, midi::cc::sustain, uint8_t(sw ? 0 : 127)};
   }

   void operator()(bool sw)
   {
      int state = edge(sw);
      if (state == 1)
         midi_out << midi::control_change{0, midi::cc::sustain, 0};
      else if (state == -1)
         midi_out << midi::control_change{0, midi::cc::sustain, 127};
   }

   edge_detector<> edge;
};

///////////////////////////////////////////////////////////////////////////////
// Bank Select controller
///////////////////////////////////////////////////////////////////////////////
struct bank_select_controller
{
   bank_select_controller()
      : curr{0}
   {}

   void load()
   {
      if (!flash_c.empty())
         curr = flash_c.read();
   }

   void save()
   {
      uint8_t curr_ = max(min(curr, 127), 0);
      if (curr_ != flash_c.read())
         flash_c.write(curr_);
   }

   void transmit()
   {
      midi_out << midi::control_change{0, midi::cc::bank_select, curr};
   }

   void up(bool sw)
   {
      if (btn_up(sw) && (curr < 127))
      {
         ++curr;
         reset_save_delay();
         transmit();
      }
   }

   void down(bool sw)
   {
      if (btn_down(sw) && (curr > 0))
      {
         --curr;
         reset_save_delay();
         transmit();
      }
   }

   uint8_t curr;
   repeat_button<> btn_up;
   repeat_button<> btn_down;
};

}  // namespace cycfi

#endif
