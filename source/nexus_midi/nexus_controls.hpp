/*=======================================================================================
    Copyright (c) 2016 Cycfi Research

    Distributed under the MIT License [ https://opensource.org/licenses/MIT ]
   =======================================================================================*/

#if !defined(CYCFI_NEXUS_CONTROLS_HPP_JUNE_25_2026)
#define CYCFI_NEXUS_CONTROLS_HPP_JUNE_25_2026

#include "midi.hpp"
#include "util.hpp"
#include "nexus_adc.hpp"
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
// ADC seam. Production reads the background oversampler (nexus_adc.hpp): a
// (10 + os_shift)-bit value per channel, refreshed faster than the loop. A
// NEXUS_SIM build injects synthetic data (shifted up by os_shift so the whole
// pipeline runs at the same bit depth as the device).
///////////////////////////////////////////////////////////////////////////////

// Map a wired analog pin to its ADC10 input channel (A0, A3..A7).
inline uint8_t adc_channel_of(uint16_t pin)
{
   return pin == ch10 ? 0 : pin == ch11 ? 3 : pin == ch12 ? 4
        : pin == ch14 ? 6 : pin == ch15 ? 7 : 5;   // default A5 = pitch (ch13)
}

inline uint16_t raw_adc(uint16_t pin)
{
#ifdef NEXUS_SIM
   return uint16_t(nexus_sim::sample(pin, millis()) << adc::os_shift);  // 10-bit sim -> os bits
#else
   return adc::read_os(adc_channel_of(pin));   // blocking oversampled read
#endif
}

// Pots don't travel to the physical ends of their range; clamp to the effective
// 2%-98% travel window and remap to the full 0-1023 range (the CC controllers
// stay 10-bit; the oversampling just cleans the input).
constexpr uint16_t adc_full = 1024 << adc::os_shift;
constexpr uint16_t min_x = uint16_t(adc_full * 0.02);
constexpr uint16_t max_x = uint16_t(adc_full * 0.98);

inline uint16_t analog_read(uint16_t pin)
{
   uint16_t x = raw_adc(pin);
   if (x < min_x)
      x = min_x;
   else if (x > max_x)
      x = max_x;
   return map(x, min_x, max_x, 0, 1023);
}

// iabs / clamp live in util.hpp, shared with the general processor blocks (slew_gate, stillness,
// dc_servo, ...). The pitch-specific blocks are defined just above this controller.


///////////////////////////////////////////////////////////////////////////////
// Generic controller handling (with course and fine controls)
///////////////////////////////////////////////////////////////////////////////
template <midi::cc::controller ctrl>
struct controller
{
   // Gate in 10-bit space before shifting down to 7-bit MIDI CC. Otherwise
   // noise around a CC boundary can chatter, e.g. 126, 127, 126, 127.
   static constexpr uint16_t cc_window = 8;
   static constexpr uint16_t endpoint_snap = 0x0f;     // snap to 0 / 0x3ff within this
   static constexpr uint16_t endpoint_release = 0x1f;  // hysteresis out of the snap (> snap)

   controller()
    : prev(0xff)
    , prev_val(0)
   {}

   void init(uint32_t val)
   {
      smoother = int32_t(val);
      prev_val = val;
      prev = uint8_t(val >> 3);
      // Publish the seeded startup value.
      midi_out << midi::control_change{0, ctrl, prev};
   }

   void operator()(uint32_t val_)
   {
      uint32_t val = uint32_t(smoother(int32_t(val_)));
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

   dynamic_smoother<24, 512> smoother;   // adaptive (fast on moves, calm at rest)
   uint32_t                  prev_val;
   uint8_t                   prev;
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
// C0 (the only per-unit term) is a VERY slow DC servo (the Pre-DC servo block), gated to
// still + near-center and hard-clipped to +/- the max physical wander, so a held bend can't
// drag it and a slip can't strand a release. See the numbered notes after the class.
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Common constants -- shared by pitch_bend_controller and 2+ of its aux blocks,
// plus the tuning for the GENERAL util.hpp blocks the controller instantiates
// (slew_gate, stillness, dc_servo, dynamic_smoother). Each pitch-specific aux
// block below keeps its OWN constants in-class.
///////////////////////////////////////////////////////////////////////////////
struct constants
{
   static constexpr int32_t  center  = 8192;
   static constexpr int32_t  pb_max  = 2 * center - 1;   // 16383, 14-bit MIDI ceiling
   // Input is (10 + adc::os_shift) bits (the oversampler); scale/nominal rescale
   // with it so the 14-bit DEFLECTION domain (and downstream) is unchanged.
   static constexpr int32_t  scale   = 16 >> adc::os_shift;   // input step -> 14-bit
   static constexpr int32_t  nominal = 512 << adc::os_shift;  // ADC nominal center

   // Input smoothing (adaptive dynamic_smoother): fast on moves, heavy at rest.
   static constexpr int      sm_g0    = 24;
   static constexpr int      sm_sense = 512 >> adc::os_shift;  // input-scaled

   // Input slew gate: reject raw steps faster than the arm can move (~4 LSB/ms; a
   // programmer-contact glitch is ~10x faster). Holds the last good reading (note 4).
   static constexpr int      slew_rate   = 6 << adc::os_shift;
   static constexpr int      slew_slack  = 4 << adc::os_shift;
   static constexpr int      slew_dt_max = 6;

   // C0 DC servo: VERY slow (tau 2^16 ms ~ 65 s), frozen past the gate, clipped to the
   // max physical DC wander (note 4).
   static constexpr int      c0_shift = 16;
   static constexpr int32_t  c0_gate  = 546;    // ~0.8 ST: a real bend (>1 ST) freezes C0
   static constexpr int32_t  c0_clip  = 736;    // +/-46 LSB (~1.3 ST): max physical DC wander

   // Stillness, by displacement-over-time: within still_band for still_dwell ms.
   static constexpr int32_t  still_band  = 32;  // ~2 LSB (14-bit deflection, fixed)
   static constexpr uint32_t still_dwell = 400;

   static constexpr int32_t  dt_max  = 32;      // clamp ms/loop (creep/servo are real-time)
   static constexpr uint32_t diag_ms = 100;     // NEXUS_DIAG telemetry interval (note 8)
};

///////////////////////////////////////////////////////////////////////////////
// Pitch-specific aux blocks -- used only by pitch_bend_controller, declared here
// just before it. Non-template (the tuning is in-class constexpr; shared values
// come from `constants` above); operator() bodies are out-of-line below. The
// general, reusable blocks (slew_gate, stillness, dc_servo, dynamic_smoother,
// moving_average_n) stay templated in util.hpp. Unit-tested in unit_blocks.cpp.
///////////////////////////////////////////////////////////////////////////////

// hysteresis_predictor: the feed-forward creep memory. M is a reversal-aware peak
// detector of the input deflection -- fast-attack to each new extreme, then a leaky
// decay toward sign(M) * (floor_num/floor_den) * peak, where it PERSISTS (the spring
// steel hysteresis settles at ~2/3 of peak and stays). operator() advances M by the
// elapsed dt and returns the predicted hysteresis k*M (k = k_num/k_den). Driven off
// the FIXED seed deflection, never the live center (notes 1-4).
struct hysteresis_predictor
{
   static constexpr int      mem_shift = 14;    // tau ~ 2^14 ms ~ 16 s
   static constexpr int32_t  k_num = 176, k_den = 4096;   // k ~ 0.043 (per-unit, notes 1-2)
   static constexpr int      floor_num = 2, floor_den = 3;   // 2/3-of-peak persistent floor

   hysteresis_predictor() : m_acc(0), peak(0) {}
   void init() { m_acc = 0; peak = 0; }

   int32_t operator()(int32_t dev, int32_t dt);   // advance M by dt, return k*M

   int32_t hyst() const { return ((m_acc >> mem_shift) * k_num) / k_den; }   // k * M
   int32_t M() const    { return m_acc >> mem_shift; }

   int32_t m_acc;
   int32_t peak;
};

// post_corrector: nulls the predictor's leftover at a SETTLED rest -- the fast rest
// nulling the slow dc_servo can't do, off the servo loop. Still + inside authority ->
// integrate the leftover away; else RELAX to 0. Bounded to +/-post_clip (note 4).
struct post_corrector
{
   static constexpr int      post_shift = 11;   // tau ~ 2^11 ms ~ 2 s
   static constexpr int32_t  post_clip  = 460;  // ~0.8 ST: hard authority limit
   // int32_t(post_clip): a bare post_clip<<post_shift overflows the device's 16-bit int.
   static constexpr int32_t  lim = int32_t(post_clip) << post_shift;
   static_assert((lim >> post_shift) == post_clip, "post_corrector authority overflows int32_t");

   post_corrector() : acc(0) {}
   void init() { acc = 0; }

   int32_t operator()(int32_t corrected, bool still, int32_t dt);   // -> offset to subtract

   int32_t acc;
};

// center_detent: a velocity+dwell-gated Schmitt detent that snaps a small rest band to
// EXACT center. Amplitude-only release with hysteresis (release > snap, note 5); re-engage
// needs a low smoother band AND dwell_ms of stillness. Returns the corrected value, or 0
// when snapped (the caller adds the center offset).
struct center_detent
{
   static constexpr int32_t  snap        = 32;    // ~+/-2 LSB rest band (14-bit)
   static constexpr int32_t  release     = 80;    // un-center hysteresis (14-bit)
   static constexpr int32_t  band_thresh = 256 << adc::os_shift;  // band: moving above
   static constexpr uint32_t dwell_ms    = 50;    // stillness must hold this long to latch

   center_detent() : centered(true), since(0) {}
   void init(uint32_t now) { centered = true; since = now; }

   int32_t operator()(int32_t corrected, int32_t band, uint32_t now);   // corrected, or 0 if snapped

   bool     centered;
   uint32_t since;
};

// output_stage: the send-deadband + the symmetric output gain (note 7). The gain hits ONLY
// the value handed to MIDI; prev (the deadband ref) stays un-scaled. operator() returns true
// + writes the gained MIDI value when the pitch should send (changed by > window), false to
// suppress -- but a center snap is never swallowed.
struct output_stage
{
   static constexpr int32_t  out_num = 307, out_den = 256;   // x1.2 (256 = x1.0)
   static constexpr int32_t  window  = 16;      // output send deadband (1 step)

   output_stage() : prev(constants::center) {}
   void init() { prev = constants::center; }

   bool operator()(int32_t pitch, int32_t* out);   // true + *out (gained) if it should send

   int32_t value() const { return prev; }                 // last accepted pitch (un-scaled)
   void reset(int32_t pitch) { prev = pitch; }            // force the deadband ref (muted park)

   int32_t prev;
};

// settle_gate: the startup "is this a valid settled rest yet" detector. True once pos has
// held within settle_lsb of itself, inside the rest band [valid_lo,valid_hi], for settle_ms
// AND at least `holdoff` ms since boot. Any move / out-of-band restarts the wait (note 6).
struct settle_gate
{
   static constexpr int32_t  valid_lo = 400 << adc::os_shift;  // plausible rest band
   static constexpr int32_t  valid_hi = 640 << adc::os_shift;  // (input units)
   static constexpr int32_t  settle_lsb = 6 << adc::os_shift;  // "settled" within this
   static constexpr uint32_t settle_ms  = 300;  // ... for this long (past the hold-off)

   settle_gate() : ref(0), since(0) {}
   void init(int32_t pos, uint32_t now) { ref = pos; since = now; }

   bool operator()(int32_t pos, uint32_t now, uint32_t boot, uint32_t holdoff);   // ready?

   int32_t  ref;
   uint32_t since;
};

struct pitch_bend_controller
{
   // Deflection of a smoothed ADC reading from nominal center, in 14-bit units.
   static int32_t deflection(int32_t pos) { return (pos - constants::nominal) * constants::scale; }

   pitch_bend_controller();

   void init(uint16_t pin_);
   void operator()();

   bool    startup_gate(int32_t pos, uint32_t now);
   void    send_pitch(
              int32_t pitch, int32_t raw, int32_t pos, int32_t c0,
              int32_t hyst);
#ifdef NEXUS_DIAG
   void    send_diag(int32_t raw, int32_t pos, int32_t c0, int32_t hyst);  // one telemetry frame
   void    send_diag_signature(uint8_t b1, uint32_t field);
#endif

   uint16_t pin;
   slew_gate<constants::slew_rate, constants::slew_slack, constants::slew_dt_max> slew;  // glitch reject
   output_stage out_stage;       // deadband + output gain
   hysteresis_predictor predictor;  // k*M feed-forward
   dc_servo<constants::c0_shift, constants::c0_gate, constants::c0_clip> servo;  // Pre-DC baseline servo
   post_corrector postc;         // null the predictor's leftover at a settled rest
   stillness<constants::still_band, constants::still_dwell> dwell;  // "is the bar settled"
   dynamic_smoother<constants::sm_g0, constants::sm_sense> smoother;   // adaptive input lowpass
   moving_average_n<3> out_ma;   // 3-pt output average: softens the residual center toggle
   uint32_t last_ms;             // millis() at the previous loop (real-time dt)
   uint32_t last_diag_ms;        // millis() of the last telemetry frame (free-running diag clock)
   center_detent detent;         // snap a still rest to exact center
   bool     muted;               // startup/validity gate: holding center until a valid rest
   settle_gate settle;           // valid-settled-rest detector
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

///////////////////////////////////////////////////////////////////////////////
// Aux-block operator() bodies (compact declarations above, before the controller).
///////////////////////////////////////////////////////////////////////////////

inline int32_t
hysteresis_predictor::operator()(int32_t dev, int32_t dt)
{
   int32_t M    = m_acc >> mem_shift;
   int32_t adev = iabs(dev);
   bool reversed = ((dev < 0) != (M < 0)) && M != 0;
   if (reversed || adev > iabs(M))        // new excursion / extending -> capture peak
   {
      m_acc = dev * (int32_t(1) << mem_shift);
      peak  = adev;
   }
   else                                   // decay toward sign(M)*floor*peak, then persist
   {
      int32_t target = (M < 0 ? -peak : peak) * floor_num / floor_den;
      m_acc += (target - M) * dt;         // millis-based leaky (tau = 2^mem_shift ms)
   }
   return hyst();
}

inline int32_t
post_corrector::operator()(int32_t corrected, bool still, int32_t dt)
{
   int32_t post = acc >> post_shift;
   int32_t ac   = iabs(corrected);
   acc += ((still && ac <= post_clip) ? (corrected - post) : (0 - post)) * dt;
   acc = clamp(acc, -lim, lim);
   return acc >> post_shift;
}

inline int32_t
center_detent::operator()(int32_t corrected, int32_t band, uint32_t now)
{
   int32_t ac = iabs(corrected);
   if (centered)
   {
      if (ac > release) centered = false;            // amplitude-only release
   }
   else if (ac <= snap && band <= band_thresh)
   {
      if (now - since >= dwell_ms) centered = true;  // sustained stillness -> snap
   }
   else
   {
      since = now;                                   // moving / off-center -> restart dwell
   }
   return centered ? 0 : corrected;
}

inline bool
output_stage::operator()(int32_t pitch, int32_t* out)
{
   int32_t delta = iabs(pitch - prev);
   if (delta == 0) return false;                       // unchanged
   if (delta <= window && pitch != constants::center)  // deadband (but never a center snap)
      return false;
   prev = pitch;
   *out = clamp(constants::center + ((pitch - constants::center) * out_num) / out_den,
                0, constants::pb_max);
   return true;
}

inline bool
settle_gate::operator()(int32_t pos, uint32_t now, uint32_t boot, uint32_t holdoff)
{
   if (pos >= valid_lo && pos <= valid_hi && iabs(pos - ref) <= settle_lsb)
      return now - since >= settle_ms && now - boot >= holdoff;
   ref = pos; since = now;                             // moved / out of band -> restart
   return false;
}

inline pitch_bend_controller::pitch_bend_controller()
 : pin(0)
 , last_ms(0)
 , last_diag_ms(0)
 , muted(true)
 , boot_ms(0)
 , startup_holdoff(5000)        // ride out the Hall front-end's power-on settle
{}

inline void pitch_bend_controller::init(uint16_t pin_)
{
   pin = pin_;
   int32_t pos = raw_adc(pin);
   slew.init(pos, millis());      // seed the slew gate to the startup reading
   smoother = pos;
   out_ma.init(constants::center);
   int32_t defl = deflection(pos);
   predictor.init();
   servo.init(defl);              // C0 := startup deflection -> output starts centered
   postc.init();
   dwell.init(defl, millis());
   out_stage.init();
   last_ms = millis();
   detent.init(millis());
   muted = true;                  // start MUTED: wait for a valid settled rest (note 6)
   settle.init(pos, millis());
   boot_ms = millis();            // hold-off reference: don't go live during the Vcc ramp
   midi_out << midi::pitch_bend{0, uint16_t(constants::center)};
#ifdef NEXUS_DIAG
   send_diag_signature(0x7f, startup_holdoff / 100);   // boot signature (raw=16383)
#endif
}

inline void pitch_bend_controller::operator()()
{
   uint32_t now = millis();
   int32_t  raw = slew(raw_adc(pin), now);       // reject glitch-fast jumps before anything sees them
   int32_t  pos = smoother(raw);                 // adaptive-smoothed 10-bit

#ifdef NEXUS_DIAG
   // Free-running telemetry (note 8): emit on a clock, BEFORE the mute return and regardless of
   // the send-deadband, so the power-on settle climb and the quiet rest get sampled too -- not
   // just active sends. c0/M are last loop's (one loop ~ a few ms stale; raw/pos are current).
   if (now - last_diag_ms >= constants::diag_ms)
      send_diag(raw, pos, servo.value(), predictor.hyst());
#endif

   if (startup_gate(pos, now))                   // held at center until a valid rest
      return;

   int32_t defl = deflection(pos);               // raw deflection (14-bit)
   int32_t c0   = servo.value();                 // DC center estimate (live)
   int32_t dev  = defl - c0;                     // deflection from the LIVE center -> output
   int32_t dev_m = defl - servo.seed_value();    // deflection from the FIXED seed -> predictor only
                                                 //   (note 4: M off the seed, never C0 -> no loop)

   // Real-time step: creep and DC servo advance by elapsed ms, not per loop, so
   // they track physical time regardless of the device's loop rate.
   int32_t dt = clamp(int32_t(now - last_ms), 0, constants::dt_max);
   last_ms = now;

   int32_t hyst = predictor(dev_m, dt);          // advance M off the FIXED-seed deflection -> k*M

   int32_t corrected = dev - hyst;               // bend with hysteresis removed

   // Post error-correction (note 4): stillness by displacement-over-time (the smoother band can't
   // see a slow ramp), then null the predictor's settled leftover -- off the servo loop.
   bool still = dwell(defl, now);
   corrected -= postc(corrected, still, dt);     // remove the settled residual

   int32_t pitch = clamp(constants::center + detent(corrected, smoother.band(), now), 0, constants::pb_max);
   pitch = out_ma(pitch);                        // 3-pt average -> soften residual toggle
   servo(defl, still, dt);                       // advance the C0 baseline servo (raw defl, gated, clipped)
   send_pitch(pitch, raw, pos, c0, hyst);        // emit (deadband) + diagnostics
}

// Startup / validity gate: hold center (MUTED) until the input is a valid, SETTLED
// rest, suppressing the power-on swing and the no-whammy case (note 6). Seeds C0 to
// the rest on the first valid + stable reading (also handles a hot-plug after boot).
// Returns true while still muted so the caller returns and holds center; once live it
// stays live (a mid-session disconnect rails the input -- the slew gate rejects the jump
// and the C0 clip bounds the center, so it can't strand off-center).
inline bool pitch_bend_controller::startup_gate(int32_t pos, uint32_t now)
{
   if (!muted)
      return false;

   if (settle(pos, now, boot_ms, startup_holdoff))   // a valid settled rest past the hold-off
   {
      int32_t defl0 = deflection(pos);
      servo.init(defl0);                      // seed the center to the settled rest
      postc.init();
      dwell.init(defl0, now);
      predictor.init();
      last_ms = now;
      detent.init(now);
      muted = false;
#ifdef NEXUS_DIAG
      send_diag_signature(0x7e, (now - boot_ms) / 10);   // go-live (raw=16382)
#endif
   }
   if (out_stage.value() != constants::center)  // park the output at center while muted
   {
      out_stage.reset(constants::center);
      midi_out << midi::pitch_bend{0, uint16_t(constants::center)};
   }
   return true;
}

// Emit on the send-deadband; a NEXUS_DIAG build pairs each send with raw / smoothed
// pos / total offset (C0 + hysteresis) so the host can watch the predictor work.
inline void pitch_bend_controller::send_pitch(
   int32_t pitch, int32_t raw, int32_t pos, int32_t c0, int32_t hyst)
{
   int32_t out;
   if (!out_stage(pitch, &out))                  // send-deadband + output gain (note 7)
      return;                                     // unchanged or within the deadband -- nothing to send
   midi_out << midi::pitch_bend{0, uint16_t(out)};
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
   int32_t off = (c0 + hyst) / constants::scale + constants::nominal;
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
