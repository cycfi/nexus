/*=======================================================================================
   Copyright (c) 2016 Cycfi Research

   Distributed under the MIT License [ https://opensource.org/licenses/MIT ]
=======================================================================================*/
#include "midi.hpp"
#include "util.hpp"

#define NEXUS_TEST
//#define NEXUS_TEST_VOLUME
//#define NEXUS_TEST_PITCH_BEND
#define NEXUS_TEST_PROGRAM_CHANGE

using namespace cycfi;

///////////////////////////////////////////////////////////////////////////////
// Control Mapping:
//
// Main:
//
//    program_change       5-way switch
//    channel_volume       analog
//    expression           analog
//    pitch_change         analog
//    modulation           analog
//    effect_1             analog
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
int const ch9  = P2_0;
int const ch10 = P1_0;
int const ch11 = P1_3;
int const ch12 = P1_4;
int const ch13 = P1_5;
int const ch14 = P1_6;
int const ch15 = P1_7;

#ifdef NEXUS_TEST
int const noise_window = 4;
#else
int const noise_window = 0;
#endif

midi::midi_stream midi_out;

///////////////////////////////////////////////////////////////////////////////
// Play notes (for testing only)
///////////////////////////////////////////////////////////////////////////////
#ifdef NEXUS_TEST
struct note
{
   void operator()(bool sw)
   {
      int state = edge(sw);
      if (state == 1)
         midi_out << midi::note_on{0, 80, 127};
      else if (state == -1)
         midi_out << midi::note_off{0, 80, 127};
   }

   edge_detector<> edge;
};

note _note;
#endif

///////////////////////////////////////////////////////////////////////////////
// Generic controller handling (with course and fine controls)
///////////////////////////////////////////////////////////////////////////////
template <midi::cc::controller ctrl>
struct controller
{
   static midi::cc::controller const ctrl_lsb = midi::cc::controller(ctrl | 0x20);

   void operator()(uint32_t val_)
   {
      uint32_t val = lp(val_);
      if (gt(val))
      {
         uint8_t const msb = val >> 3;
         uint8_t const lsb = (val << 4) & 0x7F;
         midi_out << midi::control_change{0, ctrl, msb};
         midi_out << midi::control_change{0, ctrl_lsb, lsb};
      }
   }

   lowpass<256, int32_t> lp;
   gate<noise_window, int32_t> gt;
};

///////////////////////////////////////////////////////////////////////////////
// Pitch bend controller
///////////////////////////////////////////////////////////////////////////////
struct pitch_bend_controller
{
   void operator()(uint32_t val_)
   {
      uint32_t val = lp(val_);
      if (gt(val))
         midi_out << midi::pitch_bend{0, uint16_t{val << 4}};
   }

   lowpass<256, int32_t> lp;
   gate<noise_window, int32_t> gt;
};

///////////////////////////////////////////////////////////////////////////////
// Program change controller
///////////////////////////////////////////////////////////////////////////////
struct program_change_controller
{
   void operator()(uint32_t val_)
   {
      uint8_t val = (val_ * 5) / 1024;
      if (val != prev)
      {
         midi_out << midi::program_change{0, val};
         prev = val;
      }
   }

   int prev;
};

///////////////////////////////////////////////////////////////////////////////
// The controls
///////////////////////////////////////////////////////////////////////////////
controller<midi::cc::channel_volume>   volume_control;
pitch_bend_controller                  pitch_bend;
program_change_controller              program_change;

///////////////////////////////////////////////////////////////////////////////
// setup
///////////////////////////////////////////////////////////////////////////////
void setup()
{
   pinMode(ch9 , INPUT);
   pinMode(ch10, INPUT);
   pinMode(ch11, INPUT);
   pinMode(ch12, INPUT);
   pinMode(ch13, INPUT);
   pinMode(ch14, INPUT);
   pinMode(ch15, INPUT);

   midi_out.start();
}

///////////////////////////////////////////////////////////////////////////////
// loop
///////////////////////////////////////////////////////////////////////////////
#ifdef NEXUS_TEST
void loop()
{
   _note(digitalRead(ch12));

#ifdef NEXUS_TEST_VOLUME
   volume_control(analogRead(ch11));
#endif

#ifdef NEXUS_TEST_PITCH_BEND
   pitch_bend(analogRead(ch11));
#endif

#ifdef NEXUS_TEST_PROGRAM_CHANGE
   program_change(analogRead(ch11));
#endif
}

#else // !NEXUS_TEST

void loop()
{
   volume_control(analogRead(ch11));
   pitch_bend(analogRead(ch11));
}

#endif // NEXUS_TEST
