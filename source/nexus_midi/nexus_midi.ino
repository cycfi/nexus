/*=======================================================================================
    Copyright (c) 2016 Cycfi Research

    Distributed under the MIT License [ https://opensource.org/licenses/MIT ]
   =======================================================================================*/
#include "midi.hpp"
#include "util.hpp"
#include "MspFlash.h"

// Drive MIDI TX (P1.2) HIGH as early as possible — before global constructors
// and before setup(). The internal pull-up (~50kΩ) is too weak to overcome
// the MIDI output circuit (220Ω + LED), so we actively drive the pin HIGH as
// an output. Serial.begin() in midi_out.start() will reconfigure P1.2 as
// UART TX; while in UART reset (UCSWRST=1, the default at power-on), the
// UART peripheral holds TX HIGH, so the transition is glitch-free.
void __attribute__((naked, section(".init3"), used)) _midi_tx_drive_high()
{
   P1DIR |= BIT2;   // set P1.2 as output
   P1OUT |= BIT2;   // drive P1.2 HIGH (MIDI mark = idle)
}

using namespace cycfi;

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

constexpr int noise_window = 2;  // CC gate window (10-bit)

///////////////////////////////////////////////////////////////////////////////
// The main MIDI out stream
///////////////////////////////////////////////////////////////////////////////
midi::midi_stream midi_out;

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

// We use SEGMENT_B and SEGMENT_C to store program change and bank select data
flash flash_b(SEGMENT_B);
flash flash_c(SEGMENT_C);

// Save delay: We lazily save data to flash to minimize writes to flash
// and thus conserve erase cycles. To do this, we avoid eagerly saving
// data when the user is actively changing states (e.g. buttons are pushed).
// We delay the actual save N milliseconds after the last state change.

uint32_t const save_delay = 3000;  // 3 seconds delay
int32_t save_delay_start_time = -1;

void reset_save_delay()
{
   save_delay_start_time = millis();
}


// The effective range of our controls (e.g. pots) is within 2% of the travel
constexpr uint16_t min_x = 1024 * 0.02;
constexpr uint16_t max_x = 1024 * 0.98;

uint16_t analog_read(uint16_t pin)
{
   uint16_t x = analogRead(pin);
   if (x < min_x)
      x = min_x;
   else if (x > max_x)
      x = max_x;
   return map(x, min_x, max_x, 0, 1023);
}


// Timestamp of the last CC send. Used by pitch_bend_controller to
// raise its gate threshold during CC activity, suppressing crosstalk.
uint32_t last_cc_time = 0;

///////////////////////////////////////////////////////////////////////////////
// Generic controller handling (with course and fine controls)
///////////////////////////////////////////////////////////////////////////////
template <midi::cc::controller ctrl>
struct controller
{
   controller() : prev(0xff), prev_raw(0), gt(noise_window) {}

   void init(uint32_t val)
   {
      lp1.y = val * 8;
      lp2.y = val * 16;
      prev_raw = val;
      prev = uint8_t(val >> 3);
   }

   void operator()(uint32_t val_)
   {
      uint32_t val = lp2(lp1(val_));
      uint32_t delta = val > prev_raw ? val - prev_raw : prev_raw - val;
      if (gt(delta))
      {
         prev_raw = val;
         uint8_t cc = uint8_t(val >> 3);
         if (cc != prev)
         {
            prev = cc;
            last_cc_time = millis();
            midi_out << midi::control_change{0, ctrl, cc};
         }
      }
   }

   lowpass<8, int32_t> lp1;
   lowpass<16, int32_t> lp2;
   gate<uint32_t> gt;
   uint32_t prev_raw;
   uint8_t prev;
};

///////////////////////////////////////////////////////////////////////////////
// Pitch bend controller
///////////////////////////////////////////////////////////////////////////////
struct pitch_bend_controller
{
   // MIDI pitch bend: 14-bit offset binary, center = 8192.
   static constexpr int32_t center = 8192;

   // Hardware deadband: eWhammy ±2.5% around center.
   // Servo only updates within this window so sustained bends
   // are never absorbed as drift.
   static constexpr int32_t center_window = 16384 / 40;  // 409

   // Gate threshold: normal sensitivity and suppressed (during CC activity).
   static constexpr int32_t threshold      = 32;
   static constexpr int32_t threshold_high = 64;

   // CC idle time before gate returns to normal threshold.
   static constexpr uint32_t cc_idle_ms = 80;
   static constexpr int32_t settle_delta = 1;
   static constexpr int32_t settle_window = threshold;
   static constexpr uint16_t settle_count_required = 150;   // ~150 ms at 1 kHz

   pitch_bend_controller()
    : prev_out(center)
    , prev_offset(0)
    , settle_count(0)
    , startup_blank(true)
    , gt(threshold)
   {}

   struct sample
   {
      int32_t s;
      int32_t out;
   };

   void init(uint16_t pin)
   {
      // Wait for Hall effect sensor and ADC reference to stabilize.
      delay(100);

      // Pre-seed all filters with the first live reading so there is no
      // filter ramp-up transient (filters start in a pre-converged state).
      // lowpass<k> stores the accumulator y = output * k, so seed y = seed * k.
      int32_t seed = analog_read(pin);
      ma.init(int16_t(seed));
      lp1.y = seed * 8;    // lowpass<8>:  output = y/8
      lp2.y = seed * 16;   // lowpass<16>: output = y/16

      // Fast-convergence burn-in: drive the servo to the sensor's startup
      // resting value with TC = 64 samples (fast_shift = 6) instead of the
      // normal TC = 8192 samples.  500 iterations ≈ 500 ms → >99.9% converged.
      // This replaces the old IIR s_avg estimate which undershot because the
      // IIR started from 0 and only 200 samples were collected.
      int32_t s_last = (seed << 4) + (seed % 16);
      servo.init(s_last);
      for (int i = 0; i < 500; ++i)
      {
         int32_t val = lp2(lp1(ma(analog_read(pin))));
         s_last = (val << 4) + (val % 16);
         servo.fast_update(s_last, 6);
         delay(1);
      }

      int32_t out = servo(s_last) + center;
      out = max(int32_t(0), min(out, int32_t(16383)));
      prev_out = out;
      prev_offset = servo.offset();
      settle_count = 0;
      startup_blank = true;
   }

   void operator()(uint32_t val_)
   {
      sample x = process_signal(val_);
      update_servo(x.s, x.out);

      int32_t delta = update_offset_delta();
      int32_t abs_bend_offset = abs_value(x.out - center);

      if (handle_startup_blank(delta, abs_bend_offset))
         return;

      update_dynamic_threshold();
      send_pitch_bend(x.out);
   }

   sample process_signal(uint32_t val_)
   {
      // Signal chain: ma → lp1 → lp2 (10-bit output)
      int32_t val = lp2(lp1(ma(val_)));

      // Expand 10-bit → 14-bit (bit-replicate LSBs for full range)
      int32_t s = (val << 4) + (val % 16);

      // Apply drift compensation: servo removes slow DC offset so the
      // output stays centered at 8192 regardless of sensor drift.
      int32_t out = servo(s) + center;
      out = max(int32_t(0), min(out, int32_t(16383)));
      return {s, out};
   }

   void update_servo(int32_t s, int32_t out)
   {
      // Update servo estimate only within the hardware deadband.
      if (out >= (center - center_window) && out <= (center + center_window))
         servo.update(s);
   }

   int32_t update_offset_delta()
   {
      int32_t offset = servo.offset();
      int32_t delta = offset - prev_offset;
      if (delta < 0)
         delta = -delta;
      prev_offset = offset;
      return delta;
   }

   bool handle_startup_blank(int32_t delta, int32_t abs_bend_offset)
   {
      // Startup blanking is a one-shot startup phase only.
      //
      // While active, pitch bend output is muted so any initial Hall sensor /
      // servo settling does not produce stray MIDI pitch bend messages.
      //
      // Once startup blanking ends, it must never re-arm during normal playing,
      // otherwise pitch bend would become unresponsive whenever the signal
      // passes near center.
      if (!startup_blank)
         return false;

      // Escape hatch: if the user makes a deliberate bend outside the hardware
      // deadband around center, treat that as intentional input and disable
      // startup blanking immediately. This avoids the controller feeling dead
      // if the player moves the eWhammy before the startup settle phase ends.
      if (abs_bend_offset > center_window)
      {
         startup_blank = false;
         settle_count = settle_count_required;
         return false;
      }

      // Automatic settle detection:
      //
      // We count consecutive samples only while BOTH conditions are true:
      //   1. The servo offset is changing very little (delta <= settle_delta)
      //   2. The bend output is close to center (abs_bend_offset <= settle_window)
      //
      // If either condition fails, the counter resets. This means startup
      // blanking ends only after the controller has been quiet and centered
      // continuously for long enough.
      if ((delta <= settle_delta) && (abs_bend_offset <= settle_window))
      {
         if (settle_count < settle_count_required)
            ++settle_count;
      }
      else
      {
         settle_count = 0;
      }

      // Still settling: keep pitch bend muted.
      //
      // Force prev_out to center while blanked so there is no stale non-center
      // state carried into normal operation once startup blanking ends.
      if (settle_count < settle_count_required)
      {
         prev_out = center;
         return true;
      }

      // Settled long enough: permanently exit the one-shot startup blank phase.
      startup_blank = false;
      return false;
   }

   void update_dynamic_threshold()
   {
      // Dynamic threshold: wider during CC activity to suppress crosstalk.
      gt.threshold = ((millis() - last_cc_time) < cc_idle_ms)
         ? threshold_high : threshold;
   }

   void send_pitch_bend(int32_t out)
   {
      // Noise gate: pass MIDI only when the pitch is significantly bent
      // away from center. When the gate closes on return to center, send
      // one final center value so the receiver zeroes out the bend.
      if (gt(out - center))
      {
         if (out != prev_out)
         {
            prev_out = out;
            midi_out << midi::pitch_bend{0, uint16_t(out)};
         }
      }
      else if (prev_out != center)
      {
         prev_out = center;
         midi_out << midi::pitch_bend{0, uint16_t(center)};
      }
   }

   static int32_t abs_value(int32_t x)
   {
      return x < 0 ? -x : x;
   }

   // Signal chain: ma → lp1 → lp2
   //   ma:  moving average N=16 → ~4× noise reduction, 8ms latency.
   //   lp1: leaky integrator k=8  → ~21 Hz at 1 kHz.
   //   lp2: leaky integrator k=16 → ~10 Hz at 1 kHz.
   //        Cascaded lp1+lp2 gives sub-10 Hz combined cutoff.
   moving_average<4, int16_t> ma;
   lowpass<8, int32_t> lp1;
   lowpass<16, int32_t> lp2;

   // Drift compensation: tracks slow Hall effect sensor offset (temperature,
   // age). Shift=13 → TC ≈ 8 s at 1 kHz. Only updates within deadband.
   offset_servo<13> servo;

   int32_t prev_out;
   int32_t prev_offset;
   uint16_t settle_count;
   bool startup_blank;
   gate<int32_t> gt;
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

///////////////////////////////////////////////////////////////////////////////
// The controls
///////////////////////////////////////////////////////////////////////////////
controller<midi::cc::channel_volume>   volume_control;
controller<midi::cc::effect_1>         fx1_control;
controller<midi::cc::effect_2>         fx2_control;
controller<midi::cc::modulation>       modulation_control;
pitch_bend_controller                  pitch_bend;
program_change_controller              program_change;
sustain_controller                     sustain_control;
bank_select_controller                 bank_select_control;

///////////////////////////////////////////////////////////////////////////////
// setup
///////////////////////////////////////////////////////////////////////////////
void setup()
{
   midi_out.start();
   midi_out << uint8_t(0xFF);

   pinMode(ch9 , INPUT_PULLUP);
   pinMode(ch10, INPUT);
   pinMode(ch11, INPUT);
   pinMode(ch12, INPUT);
   pinMode(ch13, INPUT);
   pinMode(ch14, INPUT);
   pinMode(ch15, INPUT);

   pinMode(aux1, INPUT_PULLUP);
   pinMode(aux2, INPUT_PULLUP);
   pinMode(aux3, INPUT_PULLUP);
   pinMode(aux4, INPUT_PULLUP);
   pinMode(aux5, INPUT_PULLUP);
   pinMode(aux6, INPUT_PULLUP);

   // Prime controller filters from the live hardware state so the first loop
   // iteration does not ramp from zero.
   volume_control.init(analog_read(ch10));
   fx1_control.init(analog_read(ch11));
   fx2_control.init(analog_read(ch12));
   pitch_bend.init(ch13);
   modulation_control.init(analog_read(ch15));

   // Load the program_change and bank_select_control states from flash
   program_change.load();
   bank_select_control.load();

   // Transmit the current program_change and bank select state
   program_change.transmit();
   bank_select_control.transmit();

}

uint32_t prev_time = 0;

void loop()
{
   // Wish we used timer interrupts. Anyway, at least make sure we don't
   // exceed a 1kHz processing loop.
   if (prev_time != millis())
   {
      sustain_control(digitalRead(ch9));
      volume_control(analog_read(ch10));
      fx1_control(analog_read(ch11));
      fx2_control(analog_read(ch12));
      pitch_bend(analog_read(ch13));
      program_change(analog_read(ch14));
      modulation_control(analog_read(ch15));

      program_change.up(!digitalRead(aux1));
      program_change.down(!digitalRead(aux2));
      program_change.group_up(!digitalRead(aux3));
      program_change.group_down(!digitalRead(aux4));
      bank_select_control.up(!digitalRead(aux5));
      bank_select_control.down(!digitalRead(aux6));

      prev_time = millis();
   }

   // Save the program_change and bank_select_control if needed
   if ((save_delay_start_time != -1)
      && (millis() > (save_delay_start_time + save_delay)))
   {
      program_change.save();
      bank_select_control.save();
      save_delay_start_time = -1;
   }
}
