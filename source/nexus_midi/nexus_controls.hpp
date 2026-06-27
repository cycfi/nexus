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
// Pitch bend controller
///////////////////////////////////////////////////////////////////////////////
// Pitch bend uses the raw ADC range, not analog_read(), because the eWhammy
// center/deadband behavior matters more than remapping the pot travel endpoints.
struct adc_sampler
{
   void init(uint16_t pin_)
   {
      pin = pin_;
      // Seed the filters from the live input so startup does not ramp from 0.
      int32_t seed = adc_read();
      ma.init(int16_t(seed));
      lp1.y = seed * 8;    // lowpass<8>:  output = y/8
      lp2.y = seed * 16;   // lowpass<16>: output = y/16
   }

   uint16_t adc_read() const
   {
      return raw_adc(pin);
   }

   int32_t operator()()
   {
      int32_t val = lp2(lp1(ma(adc_read())));
      // Expand 10-bit ADC space into 14-bit MIDI pitch bend space by
      // replicating the low bits, preserving the full 0..16383 range.
      return (val << 4) + (val % 16);
   }

   uint16_t pin;
   moving_average<4, int16_t>    ma;
   lowpass<8, int32_t>           lp1;
   lowpass<16, int32_t>          lp2;
};

struct pitch_centering_servo
{
   static constexpr int32_t center = 8192;

   // eWhammy mechanical center/deadband is about +/-2.5%.  The servo is only
   // allowed to learn drift here, so sustained bends are not pulled to center.
   static constexpr int32_t center_window = 16384 / 40;  // +/-2.5%

   void init(int32_t val)
   {
      servo.init(val);
   }

   int32_t operator()(int32_t val)
   {
      // offset_servo removes slow Hall sensor drift; adding MIDI center turns
      // the signed offset back into the 14-bit pitch bend value.
      int32_t out = servo(val) + center;
      out = max(int32_t(0), min(out, int32_t(16383)));

      if (out >= (center - center_window) && out <= (center + center_window))
         servo.update(val);

      return out;
   }

   offset_servo<13> servo;
};

struct pitch_bend_controller
{
   static constexpr int32_t center = pitch_centering_servo::center;

   // Normal pitch-bend delta gate window. The separate center window is kept
   // much smaller so light vibrato can cross zero without being muted.
   static constexpr int16_t pb_window = 40;
   static constexpr int16_t arm_center_window = 80;
   static constexpr int16_t center_window = 12;
   static constexpr uint32_t center_idle_ms = 30;

   // Wider gate while nearby CC controls are moving; those controls can couple
   // into the pitch ADC briefly, so require a larger movement before sending PB.
   static constexpr int16_t pb_window_high = 80;
   static constexpr uint32_t cc_idle_ms = 100;

   pitch_bend_controller()
    : prev_out(center)
    , pitch_active(false)
    , startup_guard(true)
    , center_pending(false)
    , center_time(0)
   {}

   void init(uint16_t pin)
   {
      // Give the Hall sensor/reference a short settling time before seeding the
      // ADC filters and servo from the live hardware state.
      delay(100);
      adc.init(pin);
      servo.init(adc());
      int32_t val = servo(adc());
      post_lp.y = val * 4;  // lowpass<4>: ~40 Hz at the 1 kHz loop rate
      gt.init(val);
      prev_out = center;
      pitch_active = false;
      startup_guard = true;
      center_pending = false;
      // Publish neutral startup pitch bend.
      midi_out << midi::pitch_bend{0, uint16_t(center)};
   }

   void operator()()
   {
      auto val = servo(adc());
      // CC movement widens the pitch-bend gate for a short window to suppress
      // crosstalk without permanently making pitch bend feel less responsive.
      gt.set_window(((millis() - last_cc_time) < cc_idle_ms)
         ? pb_window_high : pb_window);

      // Centered samples are a separate state: they may be a zero crossing in
      // active vibrato, or a real return to rest after a short dwell.
      if (process_center_dwell(val))
         return;

      // Any non-centered sample cancels a pending return-to-center dwell.
      center_pending = false;

      // At boot, ignore near-center sensor drift until the first deliberate
      // movement. Once movement escapes this band, normal center handling takes
      // over for the rest of the run.
      if (startup_guard)
      {
         if (is_within_center(val, arm_center_window))
         {
            gt.init(val);
            post_lp.y = center * 4;
            prev_out = center;
            return;
         }
         startup_guard = false;
      }

      // The delta gate arms pitch bend only after a meaningful movement, then
      // all later samples are sent until the center dwell above disarms it.
      if (gt(val))
         pitch_active = true;

      if (pitch_active)
         send_pitch_bend(val);
   }

   bool process_center_dwell(int32_t val)
   {
      if (!is_centered(val))
         return false;

      // Do not close on a brief zero crossing; light vibrato passes through
      // center every cycle. Close only after the input dwells at center.
      if (!center_pending)
      {
         center_pending = true;
         center_time = millis();
      }

      if ((millis() - center_time) < center_idle_ms)
      {
         if (pitch_active)
            send_pitch_bend(center);
         return true;
      }

      gt.init(center);
      pitch_active = false;
      send_pitch_bend(center);
      return true;
   }

   static bool is_centered(int32_t val)
   {
      return is_within_center(val, center_window);
   }

   static bool is_within_center(int32_t val, int16_t window)
   {
      int32_t delta = val - center;
      if (delta < 0)
         delta = -delta;
      return delta <= window;
   }

   void send_pitch_bend(int32_t val)
   {
      int32_t out = post_lp(val);
      if (out != prev_out)
      {
         prev_out = out;
         midi_out << midi::pitch_bend{0, uint16_t(out)};
      }
   }

   adc_sampler             adc;
   pitch_centering_servo   servo;
   lowpass<4, int32_t>     post_lp;
   delta_gate<40, int16_t> gt;
   int32_t                 prev_out;
   bool                    pitch_active;
   bool                    startup_guard;
   bool                    center_pending;
   uint32_t                center_time;
};

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
