/*
Project Name: API for MIDI Controller
Project Description: Allows MSP MCU to send MIDI data from CV(Control Voltage) 5-way Switch and Potentiometer
Date Created: 5/14/2016
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

uint8_t maxTask;
volatile uint8_t task = 1; 


#pragma vector=TIMER0_A0_VECTOR
__interrupt  void timerA0ISR(void)
{
    if (task > maxTask)
    task = 1;
    task++;
}

SCHEDULER scheduler;
TEST test_tone;
VOLUME v;

void setup()
{
  pinMode(ch14,OUTPUT); // make the LED pins outputs
  scheduler.set();
  scheduler.start();
  maxTask = 2;
  v.channel = ch11;
  pinMode(ch9, INPUT); //digital
  pinMode(ch10, INPUT); //analog
  pinMode(ch11, INPUT); //analog
  pinMode(ch12, INPUT); //analog
  pinMode(ch13, INPUT); //analog
  pinMode(ch14, INPUT); //analog
  pinMode(ch15, INPUT); //analog
  startMIDI();
  test_tone.prev_state = 0;
}
void loop()
{
 //bankup(ch9);
 //bankdown(ch10);
// volume_control(chP1_3);
 //pitch_bend_control(ch14);
 //modulation_control(ch15);
 //fx1_control(ch14);
 //program_change_control(ch15, 20, 45, 67, 80, 100);
// switch(task)
// {
//   case 1 : scheduler.stop(); test_tone.start(P1_4); scheduler.start(); break;
//   case 2 : scheduler.stop(); v.out(); scheduler.start(); break;
////   case 3 :  pitch_bend_control(ch14); break;
////   case 4 :  modulation_control(ch15); break;
////   case 5 :  bankup(ch9); break;
////   case 6 :  bankdown(ch10); break;
// //  default : test_tone.start(P1_4); break;
// }

test_tone.start(P1_4);
delay(1);

v.out(); 
delay(1);

}

