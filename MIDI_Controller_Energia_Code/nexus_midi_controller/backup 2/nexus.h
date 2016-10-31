
#ifndef _NEXUS_H_
#define _NEXUS_H_

#include <energia.h>
#include <..\..\libraries\MIDI\MIDI.h>

class SCHEDULER
{
public:
  void set();
  void start();
  void stop();   
};


bool debounce(uint16_t ch); //integral debounce function


typedef class VOLUME
{
uint8_t prev_volume_param; // = 0;
public:
  uint16_t channel;
  void out(); //adjust the volume
};


typedef class PITCH
{
int16_t prev_pitch_bend_param; // = 0;
public:
  uint16_t channel;
  void pitch_bend_control(); //adjust the frequncy/pitch
};

typedef class MODULATION
{
uint8_t prev_modulation_param; // = 0;
public:
  uint16_t channel;
  void modulation_control(); //modulate
};

typedef class FX1
{
uint8_t prev_fx1_param; // = 0;	
public:
  uint16_t channel;
  void fx1_control(); //effects
};

typedef class PROGRAM
{
int16_t program_number;
uint8_t prev_program_number; // = 1;
public:
  /*
  programChangeControl function use 5-way switch and resistor network to generate specific data.
  Position 1 = 4.8V;
  Position 2 = 3.3V;
  Position 3 = 2.4V;
  Position 4 = 1.2V;
  Position 5 = 0.0V;
  */
  uint16_t channel;
  void program_change_control(uint8_t pos1, uint8_t pos2, uint8_t pos3, uint8_t pos4, uint8_t pos5); //select instrument
};

typedef class BANK
{
uint16_t bank_data; // = 0;
bool prev_bankup_state; //  = 0;
bool prev_bankdown_state; // = 0;
bool bankup_state;
bool bankdown_state;
public:
  uint16_t up_channel, down_channel;
  void bankup(); //bank
  void bankdown();
};

class TEST
{
/*Variable for testing purposes(for test_tone function)*/
bool state;
public:
  bool prev_state;
  void start(uint16_t ch); //generate note for testing
};

void test();
void startMIDI();

#endif
