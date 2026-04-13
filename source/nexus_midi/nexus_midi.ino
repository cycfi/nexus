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


// Pots don't travel to the physical ends of their range; clamp to the
// effective 2%–98% travel window and remap to the full 0–1023 range.
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
   controller() : prev(0xff) {}

   void init(uint32_t val)
   {
      lp1.y = val * 8;
      lp2.y = val * 16;
      prev = uint8_t(val >> 3);
   }

   void operator()(uint32_t val_)
   {
      uint32_t val = lp2(lp1(val_));
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
      return analogRead(pin);
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
   static constexpr int16_t center_window = 12;
   static constexpr uint32_t center_idle_ms = 30;

   // Wider gate while nearby CC controls are moving; those controls can couple
   // into the pitch ADC briefly, so require a larger movement before sending PB.
   static constexpr int16_t pb_window_high = 80;
   static constexpr uint32_t cc_idle_ms = 100;
   static constexpr uint32_t startup_blank_ms = 300;

   pitch_bend_controller()
    : prev_out(center)
    , pitch_active(false)
    , center_pending(false)
    , center_time(0)
    , blank_until(0)
   {}

   void init(uint16_t pin)
   {
      // Give the Hall sensor/reference a short settling time before seeding the
      // ADC filters and servo from the live hardware state.
      delay(100);
      adc.init(pin);
      servo.init(adc());
      int32_t val = servo(adc());
      post_lp.y = val * 8;  // lowpass<8>: ~20 Hz at the 1 kHz loop rate
      gt.init(val);
      prev_out = center;
      pitch_active = false;
      center_pending = false;
      blank_until = millis() + startup_blank_ms;
   }

   void operator()()
   {
      auto val = servo(adc());
      // CC movement widens the pitch-bend gate for a short window to suppress
      // crosstalk without permanently making pitch bend feel less responsive.
      gt.set_window(((millis() - last_cc_time) < cc_idle_ms)
         ? pb_window_high : pb_window);

      if (millis() < blank_until)
      {
         // During startup blanking, keep the delta gate tracking the live value
         // but force logical output state to center so no stale bend escapes.
         gt.init(val);
         post_lp.y = center * 8;
         prev_out = center;
         pitch_active = false;
         center_pending = false;
         return;
      }

      if (is_centered(val))
      {
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
            return;
         }

         gt.init(center);
         pitch_active = false;
         send_pitch_bend(center);
         return;
      }

      center_pending = false;

      if (gt(val))
         pitch_active = true;

      if (pitch_active)
         send_pitch_bend(val);
   }

   bool is_centered(int32_t val)
   {
      int32_t delta = val - center;
      if (delta < 0)
         delta = -delta;
      return delta <= center_window;
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
   lowpass<8, int32_t>     post_lp;
   delta_gate<40, int16_t> gt;
   int32_t                 prev_out;
   bool                    pitch_active;
   bool                    center_pending;
   uint32_t                center_time;
   uint32_t                blank_until;
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
      pitch_bend();
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
