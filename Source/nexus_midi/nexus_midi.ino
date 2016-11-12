/*=======================================================================================
   Copyright (c) 2016 Cycfi Research

   Distributed under the MIT License [ https://opensource.org/licenses/MIT ]
=======================================================================================*/
#include "midi.hpp"
#include "util.hpp"

#define NEXUS_TEST
#define NEXUS_TEST_VOLUME
//#define NEXUS_TEST_PITCH_BEND

using namespace cycfi;

// Hard-wired channels to analog and digital pins
int const ch9  = P2_0;
int const ch10 = P1_0;
int const ch11 = P1_3;
int const ch12 = P1_4;
int const ch13 = P1_5;
int const ch14 = P1_6;
int const ch15 = P1_7;

#ifdef NEXUS_TEST
int const control_gate_window = 1;
int const pitch_bend_gate_window = 4;
#else
int const control_gate_window = 0;
int const pitch_bend_gate_window = 0;
#endif

midi::midi_stream midi_out;

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

template <midi::cc::controller ctrl>
struct controller
{
   void operator()(uint16_t val_)
   {
      uint16_t val = lp(val_);
      if (gt(val))
         midi_out << midi::control_change{0, ctrl, uint8_t(val)};
   }

   lowpass<64> lp;
   gate<control_gate_window> gt;
};

controller<midi::cc::channel_volume> volume_control;

struct pitch_bend_controller
{
   void operator()(uint32_t val_)
   {
      uint32_t val = lp(val_);
      if (gt(val))
         midi_out << midi::pitch_bend{0, uint16_t(val)};
   }

   lowpass<256, int32_t> lp;
   gate<pitch_bend_gate_window, int32_t> gt;
   int prev;
};

pitch_bend_controller pitch_bend;

void setup()
{
   pinMode(ch9 , INPUT);   // digital
   pinMode(ch10, INPUT);   // analog
   pinMode(ch11, INPUT);   // analog
   pinMode(ch12, INPUT);   // analog
   pinMode(ch13, INPUT);   // analog
   pinMode(ch14, INPUT);   // analog
   pinMode(ch15, INPUT);   // analog

   midi_out.start();
}

template <int bits>
int read(int ch)
{
   if (bits > 10)
      return analogRead(ch) << (bits-10);
   else
      return analogRead(ch) >> (10-bits);
}

void loop()
{
   unsigned long start = millis();

#ifdef NEXUS_TEST
   _note(digitalRead(ch12));
#endif

#ifdef NEXUS_TEST

#ifdef NEXUS_TEST_VOLUME
   volume_control(read<7>(ch11));
#endif

#ifdef NEXUS_TEST_PITCH_BEND
   pitch_bend(read<14>(ch11));
#endif

#endif



   //unsigned long delta = millis() - start;
   //if (delta < 10)
   //   delay(10-delta);
}
