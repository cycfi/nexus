/*
Project Name: Nexus MIDI Controller
Project Description: Allows MSP MCU to send MIDI data controlled by Guitar CV(Control Voltage) 
                     5-way Switch and Potentiometer
Date Created: 5/14/2016
Date Last Modified: 5/27/2016
*/

//#include "nexus_midi_controller.h"

/*Hard-wired Control VOltage Channel to MCU analog and digital pins*/

#include "nexus.h"

uint16_t const ch9 = P2_0;
uint16_t const ch10 = P1_0;
uint16_t const ch11 = P1_3;
uint16_t const ch12 = P1_4;
uint16_t const ch13 = P1_5;
uint16_t const ch14 = P1_6;
uint16_t const ch15 = P1_7;

TEST test_tone;
VOLUME v;
PITCH p;
MODULATION m;
FX1 f;
BANK b;
PROGRAM pr;

void setup()
{
  pinMode(ch9, INPUT); //digital
  pinMode(ch10, INPUT); //analog
  pinMode(ch11, INPUT); //analog
  pinMode(ch12, INPUT); //analog
  pinMode(ch13, INPUT); //analog
  pinMode(ch14, INPUT); //analog
  pinMode(ch15, INPUT); //analog
  //v.channel = ch11;
  //p.channel = ch11;
  //m.channel = ch11;
  //f.channel = ch11;
  //b.up_channel = ch12;
  //b.down_channel = ch13;
  //pr.channel = ch11;
  startMIDI();
}
void loop()
{
//test_tone.start(P1_4);
//v.out();
//p.out();
//m.out();
//f.out();
//b.up();
//b.down();
//pr.out(0, 45, 67, 80, 100);
}

