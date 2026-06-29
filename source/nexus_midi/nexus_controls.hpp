/*==============================================================================
    Copyright (c) 2016 Cycfi Research

    Distributed under the MIT License [ https://opensource.org/licenses/MIT ]
   ===========================================================================*/

#if !defined(CYCFI_NEXUS_CONTROLS_HPP_JUNE_25_2026)
#define CYCFI_NEXUS_CONTROLS_HPP_JUNE_25_2026

#include "midi.hpp"
#include "util.hpp"
#include "nexus_adc.hpp"
#include "MspFlash.h"
#ifdef NEXUS_SIM
#include "sim/nexus_sim.hpp"
#endif

////////////////////////////////////////////////////////////////////////////////
// Environment provided by the driver translation unit: the firmware
// sketch on the device, or the host harness off-device. Referenced by
// the controllers below and defined by whoever drives them.
//
//   midi_out:         where MIDI messages are written
//   last_cc_time:     timestamp of the last CC send (gates pitch bend)
//   reset_save_delay: defer the next flash save after a state change
////////////////////////////////////////////////////////////////////////////////
extern cycfi::midi::midi_stream midi_out;
extern uint32_t last_cc_time;
void reset_save_delay();

////////////////////////////////////////////////////////////////////////////////
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
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Constants
////////////////////////////////////////////////////////////////////////////////
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


////////////////////////////////////////////////////////////////////////////////
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
////////////////////////////////////////////////////////////////////////////////
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

// Persistent storage for program change / bank select (set by the driver).
extern flash flash_b;
extern flash flash_c;

namespace cycfi
{

////////////////////////////////////////////////////////////////////////////////
// ADC seam. Production reads the background oversampler (nexus_adc.hpp): a
// (10 + os_shift)-bit value per channel, refreshed faster than the loop. A
// NEXUS_SIM build injects synthetic data (shifted up by os_shift so the
// whole pipeline runs at the same bit depth as the device).
////////////////////////////////////////////////////////////////////////////////

// Map a wired analog pin to its ADC10 input channel (A0, A3..A7).
inline uint8_t adc_channel_of(uint16_t pin)
{
   return pin == ch10 ? 0 : pin == ch11 ? 3 : pin == ch12 ? 4
        : pin == ch14 ? 6 : pin == ch15 ? 7 : 5;  // A5 = pitch (ch13)
}

inline uint16_t raw_adc(uint16_t pin)
{
#ifdef NEXUS_SIM
   // 10-bit sim value shifted up to the os_shift bit depth.
   return uint16_t(nexus_sim::sample(pin, millis()) << adc::os_shift);
#else
   return adc::read_os(adc_channel_of(pin));   // blocking oversampled read
#endif
}

// Pots don't travel to the physical ends of their range; clamp to the
// effective 2%-98% travel window and remap to the full 0-1023 range (the CC
// controllers stay 10-bit; the oversampling just cleans the input).
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

// iabs / clamp live in util.hpp, shared with the general processor blocks
// (slew_gate, stillness, dc_servo, ...). The pitch-specific blocks are
// defined just above this controller.


////////////////////////////////////////////////////////////////////////////////
// Generic controller handling (with course and fine controls)
////////////////////////////////////////////////////////////////////////////////
template <midi::cc::controller ctrl>
struct controller
{
   // Gate in 10-bit space before shifting down to 7-bit MIDI CC. Otherwise
   // noise around a CC boundary can chatter, e.g. 126, 127, 126, 127.
   static constexpr uint16_t cc_window = 8;
   static constexpr uint16_t endpoint_snap = 0x0f;     // snap to 0 / 0x3ff
   static constexpr uint16_t endpoint_release = 0x1f;  // snap hysteresis

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

   dynamic_smoother<24, 512> smoother;  // adaptive: fast moves, calm rest
   uint32_t                  prev_val;
   uint8_t                   prev;
};

////////////////////////////////////////////////////////////////////////////////
// Pitch bend controller -- inertial centering
//
// 10-bit ADC -> 14-bit MIDI. The output is the deflection from a slow DC
// center, run through the inertial_corrector: it TRACKS the bend and lands
// only a HANG (a release that stalls near zero) back to center, so vibrato,
// slow bends and held bends pass through untouched while a real release
// returns cleanly. There is no feed-forward model and nothing on the bend
// path feeds the center, so the off-center latch is structurally impossible.
//
//   dev    = (pos - nominal) - C0            deflection from the DC center
//   output = center + inertial(dev)          tracked; a stalled release
//   lands to 0
//
// C0 (the only per-unit term) is a VERY slow DC servo (the Pre-DC servo
// block), gated to still + near-center and hard-clipped to +/- the max
// physical wander, so a held bend can't drag it and a slip can't strand a
// release. A baseline that jumps PAST the gate is caught by the
// freeze_watchdog. See the notes after the class.
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Common constants -- shared by pitch_bend_controller and 2+ of its aux
// blocks, plus the tuning for the GENERAL util.hpp blocks the controller
// instantiates (slew_gate, stillness, dc_servo, dynamic_smoother). Each
// pitch-specific aux block below keeps its OWN constants in-class.
////////////////////////////////////////////////////////////////////////////////
struct constants
{
   static constexpr int32_t  center  = 8192;
   static constexpr int32_t  pb_max  = 2 * center - 1;  // 16383, 14-bit ceil
   // Input is (10 + adc::os_shift) bits (the oversampler); scale and nominal
   // rescale with it so the 14-bit DEFLECTION domain (and downstream) holds.
   static constexpr int32_t  scale   = 16 >> adc::os_shift;  // 14-bit step
   static constexpr int32_t  nominal = 512 << adc::os_shift; // ADC center

   // Input smoothing (dynamic_smoother): fast on moves, heavy at rest.
   static constexpr int      sm_g0    = 24;
   static constexpr int      sm_sense = 512 >> adc::os_shift;  // scaled

   // Input slew gate: reject steps faster than the arm moves (~4 LSB/ms;
   // a programmer glitch is ~10x faster). Holds the last good reading
   // (note 1).
   static constexpr int      slew_rate   = 6 << adc::os_shift;
   static constexpr int      slew_slack  = 4 << adc::os_shift;
   static constexpr int      slew_dt_max = 6;

   // C0 servo: VERY slow (tau 2^16 ms ~ 65 s), frozen past gate, clipped
   // to the max physical DC wander (note 1).
   static constexpr int      c0_shift = 16;
   static constexpr int32_t  c0_gate  = 546;  // ~0.8 ST: a bend freezes C0
   static constexpr int32_t  c0_clip  = 736;  // ~1.1 ST: max DC wander

   // Stillness by displacement: within still_band for still_dwell ms.
   static constexpr int32_t  still_band  = 32;  // ~2 LSB (14-bit defl)
   static constexpr uint32_t still_dwell = 400;

   static constexpr int32_t  dt_max  = 32;   // clamp ms/loop (real-time)
   static constexpr uint32_t diag_ms = 100;  // NEXUS_DIAG frame interval
};

////////////////////////////////////////////////////////////////////////////////
// Pitch-specific aux blocks -- used only by pitch_bend_controller, declared
// here just before it. Non-template (tuning is in-class constexpr; shared
// values come from `constants` above); operator() bodies are out-of-line
// below. The general, reusable blocks (slew_gate, stillness, dc_servo,
// dynamic_smoother, moving_average_n) stay templated in util.hpp;
// unit-tested in unit_blocks.cpp.
////////////////////////////////////////////////////////////////////////////////

// inertial_corrector: track the bend, land only a HANG from a real release.
//
//   * SLOPE -- a forward difference is mostly jitter, so it is low-passed
//     (slope_k) into a clean velocity. A hang is a RELEASE that STALLS
//     (slope ~0) near zero, landed to 0 -- but ONLY after the bar bent past
//     +/-4 ST (ARMED): a shallower bend leaves an inaudible residual the
//     gate/servo handle. A vibrato never stalls near zero (crosses fast).
//
//   * LAND -- not a fixed-rate ramp but an EXPONENTIAL decay sized by the
//     descent's PEAK velocity vpk: tau = dist/vpk, clamped to [tau_min,
//     tau_max] loops (vpk itself clamped vs a glitch / mis-read). The first
//     step matches how fast the bar was moving -- vigorous lands snappy,
//     gentle eases in -- then it relaxes in; the offset is clamped so it
//     never pulls PAST center. The gate mutes the sub-gate_lo tail.
//
//   * GATE -- a hysteresis noise gate mutes the rest: once a swing crosses
//     gate_hi it stays open until |x| sits below gate_lo for gate_dwell, a
//     vibrato's troughs pass (no chop) and only a real settle re-closes it.
//     The DC hands off to the slow C0 servo. The sole centering path.
struct inertial_corrector
{
   static constexpr int32_t  bend_th     = 2731; // 4 ST: arm this deep
   static constexpr int32_t  hang_band   = 546;  // ~0.8 ST: land within
   static constexpr int32_t  gate_hi     = 68;   // ~0.10 ST: gate OPEN
   static constexpr int32_t  gate_lo     = 34;   // ~0.05 ST: gate CLOSE
   static constexpr int32_t  gate_dwell  = 90;   // ms below lo to close
   static constexpr int32_t  slope_k     = 3;    // slope low-pass (>>k)
   static constexpr int32_t  stall_slope = 14;   // |slope|/loop = stall
   static constexpr int32_t  tau_min     = 4;    // land tau clamp: min
   static constexpr int32_t  tau_max     = 24;   //   ...max (no drag)
   static constexpr int32_t  vpk_min     = 14;   // peak-slope: floor
   static constexpr int32_t  vpk_max     = 256;  //   ...ceiling
   static constexpr int32_t  land_rate   = 8;    // units/ms: gate-close
   static constexpr int32_t  leak_rate   = 3;    // units/ms: leak/track

   inertial_corrector()
    : off(0), prev(0), vlp(0), vpk(0), tau(0), gd(0)
    , armed(false), gopen(false), landing(false) {}
   void init()
   {
      off = 0; prev = 0; vlp = 0; vpk = 0; tau = 0;
      gd = 0; armed = false; gopen = false; landing = false;
   }

   // Move v toward target by at most |step| units (step floored at 1).
   static int32_t toward(int32_t v, int32_t target, int32_t step)
   {
      if (step < 1) step = 1;
      int32_t d = target - v, ad = iabs(d);
      return v + (d < 0 ? -1 : 1) * (ad < step ? ad : step);
   }

   int32_t operator()(int32_t x, int32_t dt);   // x = corrected -> output

   int32_t off;      // correction offset: output = x - off
   int32_t prev;     // last input, for the slope
   int32_t vlp;      // low-passed slope (clean velocity)
   int32_t vpk;      // peak velocity of the descent (sizes the land)
   int32_t tau;      // land time constant (loops), latched at onset
   int32_t gd;       // ms |x| below gate_lo (gate close dwell)
   bool    armed;    // bar bent past +/-4 ST -> a release may land
   bool    gopen;    // noise gate open (passing) vs closed (muted)
   bool    landing;  // a hang-land is in progress (latches tau)
};

// output_stage: the send-deadband + the symmetric output gain (note 3). The
// gain hits ONLY the value handed to MIDI; prev (the deadband ref) stays
// un-scaled. operator() returns true + writes the gained MIDI value when the
// pitch should send (changed by > window), false to suppress -- but a center
// snap is never swallowed.
struct output_stage
{
   static constexpr int32_t  out_num = 307, out_den = 256;  // x1.2 gain
   static constexpr int32_t  window  = 16;  // output send deadband (1 step)

   output_stage() : prev(constants::center) {}
   void init() { prev = constants::center; }

   bool operator()(int32_t pitch, int32_t* out);  // true + *out if it sends

   int32_t value() const { return prev; }   // last accepted (un-scaled)
   void reset(int32_t pitch) { prev = pitch; }   // force the deadband ref

   int32_t prev;
};

// settle_gate: the startup "is this a valid settled rest yet" detector. True
// once pos has held within settle_lsb of itself, inside the rest band
// [valid_lo,valid_hi], for settle_ms AND at least `holdoff` ms since boot. A
// move / out-of-band reading restarts the wait (note 2).
struct settle_gate
{
   static constexpr int32_t  valid_lo = 400 << adc::os_shift;  // rest lo
   static constexpr int32_t  valid_hi = 640 << adc::os_shift;  // rest hi
   static constexpr int32_t  settle_lsb = 6 << adc::os_shift;  // settled
   static constexpr uint32_t settle_ms  = 300;  // ...held this long

   settle_gate() : ref(0), since(0) {}
   void init(int32_t pos, uint32_t now) { ref = pos; since = now; }

   // -> true when pos is a valid settled rest (past boot + holdoff).
   bool operator()(int32_t pos, uint32_t now,
                   uint32_t boot, uint32_t holdoff);

   int32_t  ref;
   uint32_t since;
};

// freeze_watchdog: a last-resort backstop for an output STUCK off-center
// indefinitely (e.g. a programmer disconnect shifts the C0 baseline past
// c0_gate, freezing the servo so it never re-centers). A held bend and a
// hang are IDENTICAL instant-to-instant -- a steady off-center output --
// so only TIME separates them: a player releases within a second or two, a
// hang persists. So fire only after the output holds within `band`, off
// center, for a LONG `freeze_ms` (10 s, past any real held bend), then flush
// to center (re-seed C0). Detect on the output: the smoother + deadband
// collapse even a jittery input into a steady value. Tunable via FRZ_*.
// (May be moot: the latch it guarded was a property of the removed
// post_corrector.)
struct freeze_watchdog
{
   static constexpr int32_t  band      = 20;    // pitch stuck == frozen
   static constexpr uint32_t freeze_ms = 10000; // ...this long (10 s)
   static constexpr int32_t  off_min   = 64;    // ...and >~0.1 ST off-ctr

   freeze_watchdog() : ref(0), since(0) {}
   void init(int32_t pitch, uint32_t now) { ref = pitch; since = now; }

   // -> true once the output has held a stuck off-center hang for freeze_ms.
   bool operator()(int32_t pitch, uint32_t now)
   {
#ifdef NEXUS_SIM
      static int32_t  B  = getenv("FRZ_BAND")
                         ? atoi(getenv("FRZ_BAND")) : band;
      static uint32_t MS = getenv("FRZ_MS")
                         ? uint32_t(atoi(getenv("FRZ_MS"))) : freeze_ms;
      static int32_t  OF = getenv("FRZ_OFF")
                         ? atoi(getenv("FRZ_OFF")) : off_min;
#else
      const int32_t  B  = band;
      const uint32_t MS = freeze_ms;
      const int32_t  OF = off_min;
#endif
      // moved -> not frozen
      if (iabs(pitch - ref) > B) { ref = pitch; since = now; return false; }
      if (now - since < MS) return false;          // not frozen long enough
      return iabs(pitch - constants::center) > OF; // frozen, and off center
   }

   int32_t  ref;
   uint32_t since;
};

struct pitch_bend_controller
{
   // Deflection of a smoothed ADC reading from nominal, in 14-bit units.
   static int32_t deflection(int32_t pos)
   {
      return (pos - constants::nominal) * constants::scale;
   }

   pitch_bend_controller();

   void init(uint16_t pin_);
   void operator()();

   bool    startup_gate(int32_t pos, uint32_t now);
   void    send_pitch(int32_t pitch);
#ifdef NEXUS_DIAG
   void    send_diag(int32_t raw, int32_t pos, int32_t c0);  // telemetry
   void    send_diag_signature(uint8_t b1, uint32_t field);
#endif

   uint16_t pin;

   // Long instantiated types aliased for width (templates live in util.hpp;
   // typedef not `using` -- the device GCC 4.6 lacks alias-declarations).
   typedef slew_gate<constants::slew_rate, constants::slew_slack,
                     constants::slew_dt_max> slew_t;
   typedef dc_servo<constants::c0_shift, constants::c0_gate,
                    constants::c0_clip> servo_t;
   typedef stillness<constants::still_band, constants::still_dwell> dwell_t;
   typedef dynamic_smoother<constants::sm_g0, constants::sm_sense> smooth_t;

   slew_t   slew;                // reject faster-than-arm jumps (note 1)
   output_stage out_stage;       // deadband + output gain
   servo_t  servo;               // Pre-DC baseline servo (note 1)
   inertial_corrector inertial;  // the sole centering path
   dwell_t  dwell;               // "is the bar settled"
   smooth_t smoother;            // adaptive input low-pass
   moving_average_n<3> out_ma;   // 3-pt output average
   freeze_watchdog freeze_wd;    // stuck-output backstop (note 1)
   uint32_t last_ms;             // millis() at the previous loop
   uint32_t last_diag_ms;        // millis() of the last diag frame
   bool     muted;               // startup gate: holding center
   settle_gate settle;           // valid-settled-rest detector
   uint32_t boot_ms;             // millis() at init (startup hold-off)
   uint32_t startup_holdoff;     // min ms muted after boot
};

////////////////////////////////////////////////////////////////////////////////
// Notes (pitch_bend_controller)
//
// 1. C0 baseline servo. The center is a VERY slow (tau ~65 s) leaky
//    integrator toward the raw deflection, stepping ONLY when the bar is
//    STILL and within c0_gate of neutral, hard-clipped to +/- c0_clip of the
//    startup seed. Far slower than any gesture, it follows only the
//    thermal/bias baseline, not a per-bend shift; the gate keeps a held bend
//    from dragging it and the clip keeps a detection slip from stranding a
//    release. Nothing on the bend path feeds it, so it can't chase or go
//    history-dependent. The one case it can't recover from on its own -- a
//    baseline that jumps PAST the gate (a programmer disconnect) -- is
//    backstopped by the freeze_watchdog.
//
// 2. Startup gate. The analog front-end can read a FALSE stable rest for a
//    moment before it swings on the Vcc ramp, so don't trust an early
//    "settled" reading: also require startup_holdoff ms since boot before
//    going live. A missing whammy (ADC pinned at 0) is outside the rest
//    band, so it never goes live.
//
// 3. Output gain. The sensor swing fills only part of +/-8192: a full dive
//    reaches -8192 but a full pull rails at the analog amp around +7121
//    (+10.4 ST). out_num scales the output up (x1.2) and the pitch clamp
//    takes the overshoot, so the pull now reaches full and the dive just
//    clips its deepest sliver -- one SYMMETRIC gain, both sides equal.
//    Applied at the VERY END in send_pitch, only to the value handed to
//    MIDI, so the deflection, the C0 servo, the send deadband and prev all
//    stay un-scaled -- the gain changes only the emitted magnitude, never
//    the behavior. Calibrated 2026-06-26 (a firm pull peaked +7121).
//
// 4. Free-running diag (NEXUS_DIAG only). operator() emits a telemetry frame
//    on a diag_ms clock, BEFORE the startup mute returns, so the silent
//    stretches get sampled too -- the power-on settle climb (muted) and a
//    dead-centered rest (send-deadband). Telemetry only: it reads state and
//    writes MIDI, never touches control. Keep diag_ms generous -- each frame
//    is ~3 ms of UART, and pairing one with every loop floods the bus and
//    perturbs timing. The clocked frame uses last loop's C0 (a few ms
//    stale); raw/pos are current.
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Aux-block operator() bodies (declarations are above, beside the class).
////////////////////////////////////////////////////////////////////////////////

inline int32_t
inertial_corrector::operator()(int32_t x, int32_t dt)
{
   int32_t BT = bend_th, HB = hang_band, GH = gate_hi, GL = gate_lo,
           GD = gate_dwell, SK = slope_k, ST = stall_slope, LR = land_rate,
           LK = leak_rate, TMIN = tau_min, TMAX = tau_max, VMIN = vpk_min,
           VMAX = vpk_max;
#ifdef NEXUS_SIM
   { const char* e;
     if ((e = getenv("BT")))   BT = atoi(e);
     if ((e = getenv("HB")))   HB = atoi(e);
     if ((e = getenv("GH")))   GH = atoi(e);
     if ((e = getenv("GL")))   GL = atoi(e);
     if ((e = getenv("GD")))   GD = atoi(e);
     if ((e = getenv("ST")))   ST = atoi(e);
     if ((e = getenv("TMIN"))) TMIN = atoi(e);
     if ((e = getenv("TMAX"))) TMAX = atoi(e);
     if ((e = getenv("VMIN"))) VMIN = atoi(e);
     if ((e = getenv("VMAX"))) VMAX = atoi(e);
     if ((e = getenv("LAND"))) LR = atoi(e);
     if ((e = getenv("LEAK"))) LK = atoi(e); }
#endif
   if (dt <= 0) dt = 1;
   int32_t ax = iabs(x);

   // Clean slope: low-pass the forward difference so jitter doesn't swamp
   // the velocity.
   vlp += ((x - prev) - vlp) >> SK; prev = x;
   int32_t av = iabs(vlp);
   bool moving = av > ST;   // clean velocity above the stall floor

   // Track the PEAK velocity of the descent to center (a vigorous release
   // peaks high, a gentle one low). Reset when the bar moves AWAY (a
   // fresh bend) so each release lands on its energy; hold it through the
   // stall so the land can read it.
   if (moving) vpk = ((x ^ vlp) < 0) ? (av > vpk ? av : vpk) : 0;

   // Hysteresis noise gate on |x|: mute the rest, but once a swing opens it
   // stay open until |x| sits below gate_lo for gate_dwell ms -- a vibrato's
   // troughs pass, only a real settle closes.
   if (gopen)
   {
      if (ax < GL) { if ((gd += dt) >= GD) gopen = false; } else gd = 0;
   }
   else if (ax > GH) { gopen = true; gd = 0; }

   if (ax > BT) armed = true;        // a real bend -> a release may land
   else if (ax < GL) armed = false;  // back at center -> disarm

   if (!gopen)                       // gate closed: rest near center -> mute
   {
      off = toward(off, x, LR * dt); // track to the rest, clean re-open
      armed = false; landing = false; vpk = 0;
      return 0;
   }

   if (armed && !moving && ax < HB)  // stalled near 0 (after >4 ST): a hang
   {
      if (!landing)  // ONSET: size the exponential to the descent
      {
         int32_t v0 = clamp(vpk, VMIN, VMAX);  // clamp the slope (guard)
         tau = clamp(iabs(x - off) / v0, TMIN, TMAX);  // tau ~ dist/vpk
         landing = true;
      }
      int32_t d = x - off, step = d / tau;  // EXPONENTIAL decay
      off += step ? step : (d > 0) - (d < 0);  // 1-unit floor to 0
      if (((x - off) ^ x) < 0) off = x;  // never pull PAST center
   }
   else if (moving)  // a real move -> release the offset, track
   {
      off = toward(off, 0, LK * dt);
      landing = false;
   }
   else landing = false;  // still but not a hang (held bend) -> hold
   return x - off;
}

inline bool
output_stage::operator()(int32_t pitch, int32_t* out)
{
   int32_t delta = iabs(pitch - prev);
   if (delta == 0) return false;  // unchanged
   // within the send-deadband -- but a center snap is never swallowed:
   if (delta <= window && pitch != constants::center)
      return false;
   prev = pitch;
   int32_t g = (pitch - constants::center) * out_num / out_den;  // x1.2
   *out = clamp(constants::center + g, 0, constants::pb_max);
   return true;
}

inline bool
settle_gate::operator()(int32_t pos, uint32_t now,
                        uint32_t boot, uint32_t holdoff)
{
   if (pos >= valid_lo && pos <= valid_hi && iabs(pos - ref) <= settle_lsb)
      return now - since >= settle_ms && now - boot >= holdoff;
   ref = pos; since = now;  // moved / out of band -> restart
   return false;
}

inline pitch_bend_controller::pitch_bend_controller()
 : pin(0)
 , last_ms(0)
 , last_diag_ms(0)
 , muted(true)
 , boot_ms(0)
 , startup_holdoff(5000)  // ride out the power-on settle
{}

inline void pitch_bend_controller::init(uint16_t pin_)
{
   pin = pin_;
   int32_t pos = raw_adc(pin);
   slew.init(pos, millis());  // seed the slew gate to the rest
   smoother = pos;
   out_ma.init(constants::center);
   freeze_wd.init(constants::center, millis());
   int32_t defl = deflection(pos);
   servo.init(defl);  // C0 := startup deflection -> centered out
   inertial.init();
   dwell.init(defl, millis());
   out_stage.init();
   last_ms = millis();
   muted = true;  // start MUTED: wait for a settled rest (note 2)
   settle.init(pos, millis());
   boot_ms = millis();  // hold-off ref: don't go live on the Vcc ramp
   midi_out << midi::pitch_bend{0, uint16_t(constants::center)};
#ifdef NEXUS_DIAG
   send_diag_signature(0x7f, startup_holdoff / 100);  // boot (raw=16383)
#endif
}

inline void pitch_bend_controller::operator()()
{
   uint32_t now = millis();
   int32_t  raw = slew(raw_adc(pin), now);  // reject glitch-fast jumps
   int32_t  pos = smoother(raw);            // adaptive-smoothed 10-bit

#ifdef NEXUS_DIAG
   // Free-running telemetry (note 4): emit on a clock, BEFORE the mute
   // return and regardless of the send-deadband, so the power-on settle
   // climb and the quiet rest get sampled too -- not just active sends. c0
   // is last loop's (one loop ~ a few ms stale; raw/pos are current).
   if (now - last_diag_ms >= constants::diag_ms)
      send_diag(raw, pos, servo.value());
#endif

   if (startup_gate(pos, now))  // held at center until a valid rest
      return;

   int32_t defl = deflection(pos);  // raw deflection (14-bit)
   int32_t c0   = servo.value();    // DC center estimate (live)
   int32_t dev  = defl - c0;        // deflection from LIVE center = output

   // Real-time step: creep and DC servo advance by elapsed ms, not per
   // loop, so they track physical time regardless of the loop rate.
   int32_t dt = clamp(int32_t(now - last_ms), 0, constants::dt_max);
   last_ms = now;

   bool still = dwell(defl, now);  // stillness -> gates the C0 servo

   // The inertial_corrector is the sole centering path: it tracks the bend
   // and lands only a HANG -- a release that stalls near zero -- back to
   // center, with no off-center latch.
   int32_t pitch =
      clamp(constants::center + inertial(dev, dt), 0, constants::pb_max);
   pitch = out_ma(pitch);  // 3-pt average -> soften residual toggle
   servo(defl, still, dt); // advance C0 (raw defl, gated, clipped)

   // output stuck off-center for freeze_ms == a hang the corrector can't
   // see there -> flush to center:
   if (freeze_wd(pitch, now))
   {
#ifdef NEXUS_DIAG
      // freeze-flush marker (raw=16381)
      send_diag_signature(0x7d, iabs(pitch - constants::center) / 10);
#endif
      servo.init(defl);  // recenter: C0 := current defl -> output 0
      inertial.init();   // drop the corrector's offset/arm state
      out_ma.init(constants::center);  // clear the average (no pull-back)
      pitch = constants::center;  // emit center (snap bypasses deadband)
      freeze_wd.init(constants::center, now);  // re-arm at center
   }
   send_pitch(pitch);  // emit (deadband)
}

// Startup / validity gate: hold center (MUTED) until the input is a valid,
// SETTLED rest, suppressing the power-on swing and the no-whammy case
// (note 2). Seeds C0 to the rest on the first valid + stable reading (also
// handles a hot-plug after boot). Returns true while still muted so the
// caller returns and holds center; once live it stays live (a mid-session
// disconnect rails the input -- the slew gate rejects the jump and the C0
// clip bounds the center, so it can't strand off-center).
inline bool pitch_bend_controller::startup_gate(int32_t pos, uint32_t now)
{
   if (!muted)
      return false;

   // a valid settled rest past the hold-off:
   if (settle(pos, now, boot_ms, startup_holdoff))
   {
      int32_t defl0 = deflection(pos);
      servo.init(defl0);  // seed the center to the settled rest
      inertial.init();
      freeze_wd.init(constants::center, now);
      dwell.init(defl0, now);
      last_ms = now;
      muted = false;
#ifdef NEXUS_DIAG
      send_diag_signature(0x7e, (now - boot_ms) / 10);  // go-live raw=16382
#endif
   }
   if (out_stage.value() != constants::center)  // park output at center
   {
      out_stage.reset(constants::center);
      midi_out << midi::pitch_bend{0, uint16_t(constants::center)};
   }
   return true;
}

// Emit the pitch bend through the send-deadband; the x1.2 output gain
// (note 3) is applied here, only to the value handed to MIDI.
inline void pitch_bend_controller::send_pitch(int32_t pitch)
{
   int32_t out;
   if (!out_stage(pitch, &out))  // send-deadband + gain (note 3)
      return;  // unchanged or within the deadband
   midi_out << midi::pitch_bend{0, uint16_t(out)};
   // Do NOT pair a diag frame with the send. During a gesture send_pitch
   // fires ~every loop (~175/s); stapling an 11-byte sysex onto each pushes
   // ~2.7 kB/s into the 31250-baud UART (~3.1 kB/s ceiling), overflowing TX
   // and truncating diag frames into stray 0xF7 bytes. Telemetry rides the
   // free-running diag_ms clock in operator(); the fine output is in the
   // host pitch-bend log.
}

#ifdef NEXUS_DIAG
// One telemetry frame: raw / smoothed pos / C0 center, each 14-bit on the
// 0x4364 SysEx. Resets the free-running diag_ms clock (note 4).
inline void
pitch_bend_controller::send_diag(int32_t raw, int32_t pos, int32_t c0)
{
   last_diag_ms = millis();
   int32_t off = c0 / constants::scale + constants::nominal;
   uint8_t buf[6] =
   {
      uint8_t((raw >> 7) & 0x7f), uint8_t(raw & 0x7f),
      uint8_t((pos >> 7) & 0x7f), uint8_t(pos & 0x7f),
      uint8_t((off >> 7) & 0x7f), uint8_t(off & 0x7f)
   };
   midi_out << midi::sysex<6>(0x4364, buf);
}

// Diagnostic boot / go-live signature on the 0x4364 frame: b1 = 0x7f (boot,
// decodes to raw=16383) or 0x7e (go-live, raw=16382); field = holdoff/100
// or muted-ms/10.
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

////////////////////////////////////////////////////////////////////////////////
// Program change controller
////////////////////////////////////////////////////////////////////////////////
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

////////////////////////////////////////////////////////////////////////////////
// Sustain control
////////////////////////////////////////////////////////////////////////////////
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

////////////////////////////////////////////////////////////////////////////////
// Bank Select controller
////////////////////////////////////////////////////////////////////////////////
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
