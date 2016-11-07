
#ifndef _NEXUS_H_
#define _NEXUS_H_

#define NEXUS_TEST

#include <energia.h>
#include <..\..\libraries\MIDI\MIDI.h>
extern midi::MidiInterface<HardwareSerial> MIDI;
  
namespace cycfi
{

   bool debounce(uint16_t ch); //integral debounce function
  
   void startMIDI();
  
   class midi_volume_control
   {
   uint8_t prev_volume_param; // = 0;
   public:
      uint16_t channel;
      void out(); //adjust the volume
   };
  
   class midi_pitch_control
   {
   int16_t prev_pitch_bend_param; // = 0;
   public:
      uint16_t channel;
      void out(); //adjust the frequncy/pitch
   };
  
   class midi_modulation_control
   {
   uint8_t prev_modulation_param; // = 0;
   public:
      uint16_t channel;
      void out(); //modulate
   };
  
   class midi_fx1_control
   {
   uint8_t prev_fx1_param; // = 0;	
   public:
      uint16_t channel;
      void out(); //effects
   };
  
   class midi_program_change_control
   {
   int16_t program_number;
   uint8_t prev_program_number;
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
      void out(uint8_t pos1, uint8_t pos2, uint8_t pos3, uint8_t pos4, uint8_t pos5); //select instrument
   };
  
   class midi_bank_control
   {
   uint16_t bank_data; // = 0;
   bool prev_bankup_state; //  = 0;
   bool prev_bankdown_state; // = 0;
   bool bankup_state;
   bool bankdown_state;
   public:
      uint16_t up_channel, down_channel;
      void up(); //bank
      void down();
   };
    
   struct midi_controls
   {
      midi_volume_control         volume;
      midi_pitch_control          pitch;
      midi_modulation_control     modulation;
      midi_fx1_control            fx1;
      midi_bank_control           bank;
      midi_program_change_control program_change;
   };
  
   void setup();
   void loop();
}

#endif
