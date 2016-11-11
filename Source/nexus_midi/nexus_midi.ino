/*=======================================================================================
   Copyright (c) 2016 Cycfi Research

   Distributed under the MIT License [ https://opensource.org/licenses/MIT ]
=======================================================================================*/
#include "midi.hpp"
#include "util.hpp"

#define NEXUS_TEST

using namespace cycfi;

// Hard-wired channels to analog and digital pins
int const ch9  = P2_0;
int const ch10 = P1_0;
int const ch11 = P1_3;
int const ch12 = P1_4;
int const ch13 = P1_5;
int const ch14 = P1_6;
int const ch15 = P1_7;

midi::midi_stream midi_out;

template <int ch>
struct note
{
   void operator()()
   {
      int state = edge(digitalRead(ch));
      if (state == 1)
         midi_out << midi::note_on{0, 80, 127};
      else if (state == -1)
         midi_out << midi::note_off{0, 80, 127};
   }

   edge_detector<> edge;
};

note<ch12> _note;

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

void loop()
{
   _note();
}
