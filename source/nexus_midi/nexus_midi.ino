/*==============================================================================
    Copyright (c) 2016 Cycfi Research

    Distributed under the MIT License [ https://opensource.org/licenses/MIT ]
   ===========================================================================*/

#include "nexus_controls.hpp"

// Drive MIDI TX HIGH before global constructors. Device-only (pokes P1DIR).
#ifndef NEXUS_HOST
// Drive MIDI TX (P1.2) HIGH as early as possible — before global
// constructors and before setup(). The internal pull-up (~50kΩ) is too
// weak to overcome the MIDI output circuit (220Ω + LED), so we actively
// drive the pin HIGH as an output. Serial.begin() in midi_out.start() will
// reconfigure P1.2 as UART TX; while in UART reset (UCSWRST=1, the
// default at power-on), the UART peripheral holds TX HIGH, so the
// transition is glitch-free.
void __attribute__((naked, section(".init3"), used)) _midi_tx_drive_high()
{
   P1DIR |= BIT2;   // set P1.2 as output
   P1OUT |= BIT2;   // drive P1.2 HIGH (MIDI mark = idle)
}
#endif

using namespace cycfi;

// The main MIDI out stream.
midi::midi_stream midi_out;

// We use SEGMENT_B and SEGMENT_C to store program change and bank select
// data.
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

// Timestamp of the last CC send. Used by pitch_bend_controller to
// raise its gate threshold during CC activity, suppressing crosstalk.
uint32_t last_cc_time = 0;

////////////////////////////////////////////////////////////////////////////////
// The controls
////////////////////////////////////////////////////////////////////////////////
controller<midi::cc::channel_volume>   volume_control;
controller<midi::cc::effect_1>         fx1_control;
controller<midi::cc::effect_2>         fx2_control;
controller<midi::cc::modulation>       modulation_control;
pitch_bend_controller                  pitch_bend;
program_change_controller              program_change;
sustain_controller                     sustain_control;
bank_select_controller                 bank_select_control;

////////////////////////////////////////////////////////////////////////////////
// setup
////////////////////////////////////////////////////////////////////////////////
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

   // Start the background ADC oversampler before any analog read (it fills
   // the per-channel values during the flash load / transmit below, no-op on
   // host).
   adc::start();

   // Load the program_change and bank_select_control states from flash
   program_change.load();
   bank_select_control.load();

   program_change.transmit();
   bank_select_control.transmit();

   // Prime controller filters from the live hardware state so the first loop
   // iteration does not ramp from zero. Each init() also transmits the
   // seeded startup state.
   sustain_control.init(digitalRead(ch9));
   volume_control.init(analog_read(ch10));
   fx1_control.init(analog_read(ch11));
   fx2_control.init(analog_read(ch12));
   pitch_bend.init(ch13);
   modulation_control.init(analog_read(ch15));

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
