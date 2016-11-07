#include "nexus.h"
midi::MidiInterface<HardwareSerial> MIDI((HardwareSerial&)Serial);

namespace cycfi
{

   bool debounce(uint16_t ch)
   {
      int8_t    counter=0; //clear counter
      uint8_t   threshold=10; //will determine the logic levels when reached
      uint16_t  prev_time=millis();
      uint16_t  current_time=0;
      uint8_t   interval=2; //2ms interval for reading the digital input
      while ( abs(counter) < threshold ) // check samples every 2ms until threshold was reached
      {
         current_time = millis();
         if ((digitalRead(ch) == HIGH) && ((current_time - prev_time) > interval))
         {   counter++; prev_time=current_time;  }
         else if ((digitalRead(ch) == LOW) && ((current_time - prev_time) > interval)) 
         {   counter--; prev_time=current_time; }
      }     
     if (counter>0)
        return 1;
     else if (counter<0)
        return 0;
   }

   void midi_volume_control::out()
   {
      uint8_t volume_param = map(analogRead(channel), 0, 1023, 0, 127);
      if (volume_param < (prev_volume_param - 1)  || volume_param > (prev_volume_param + 1) )
      {
         MIDI.sendControlChange(7, volume_param, 1);
         prev_volume_param = volume_param;
      }
   }

   void midi_pitch_control::out()
   {
      int16_t pitch_bend_param = map(analogRead(channel), 0, 1023, -8191, 8192);
      if ( pitch_bend_param < (prev_pitch_bend_param - 100)  ||  pitch_bend_param > (prev_pitch_bend_param + 100) )
      {
         MIDI.sendPitchBend(pitch_bend_param, 1);
         prev_pitch_bend_param = pitch_bend_param;
      }
   }
    
   void midi_modulation_control::out()
   {
      uint8_t modulation_param = map(analogRead(channel), 0, 1023, 0, 127);
      if ( modulation_param < (prev_modulation_param - 1)  ||  modulation_param > (prev_modulation_param + 1) )
      {
         MIDI.sendControlChange(1,modulation_param, 1);
         prev_modulation_param = modulation_param;
      }
   }
    
   void midi_fx1_control::out()
   {
      uint8_t fx1_param = map(analogRead(channel), 0, 1023, 0, 127);
      if ( fx1_param < (prev_fx1_param - 1)  ||  fx1_param > (prev_fx1_param + 1) )
      {
         MIDI.sendControlChange(12, fx1_param, 1);
         prev_fx1_param = fx1_param;
      }
   }
    
   void midi_program_change_control::out(uint8_t pos1, uint8_t pos2, uint8_t pos3, uint8_t pos4, uint8_t pos5)
   {  
      float voltage = analogRead(channel);
      if ( voltage > 800 && voltage < 1023 )
         program_number = pos5; // first program
      else if ( voltage > 600 && voltage < 750 )
         program_number = pos4; //second program
      else if ( voltage > 450 && voltage < 550)
         program_number = pos3; //third program
      else if ( voltage > 200 && voltage < 400 )
         program_number = pos2; //fourth program
      else if ( voltage < 100 )
         program_number = pos1; //fifth program
      
      if ( prev_program_number != program_number )
      {
         MIDI.sendProgramChange(program_number, 1);
         prev_program_number = program_number;
      }
   }
    
   void midi_bank_control::up()
   {
      bankup_state = debounce(up_channel);
      if (prev_bankup_state != bankup_state)
      {
          if (debounce(up_channel) == 1)
          {
             if (bank_data == 127) 
             bank_data=126;
             bank_data++;
             MIDI.sendControlChange(0, bank_data, 1);
          }
          prev_bankup_state = bankup_state;
      }  
   }
    
   void midi_bank_control::down()
   {
      bankdown_state = debounce(down_channel);
      if (prev_bankdown_state != bankdown_state)
      {
         if ( debounce(down_channel) == 1 )
         {
            if ( bank_data == 0 ) 
            bank_data=1;
            bank_data--;
            MIDI.sendControlChange(0, bank_data, 1);
         }
         prev_bankdown_state = bankdown_state;
       }   
   }
    
   void startMIDI()
   {
      MIDI.begin(4);
   }
    
   #ifndef NEXUS_TEST
    
   midi_controls _midi_controls;
    
   /*Hard-wired channels to analog and digital pins*/
   uint16_t const ch9  = P2_0;
   uint16_t const ch10 = P1_0;
   uint16_t const ch11 = P1_3;
   uint16_t const ch12 = P1_4;
   uint16_t const ch13 = P1_5;
   uint16_t const ch14 = P1_6;
   uint16_t const ch15 = P1_7;
    
   void setup()
   {
      pinMode(ch9 , INPUT); //digital
      pinMode(ch10, INPUT); //analog
      pinMode(ch11, INPUT); //analog
      pinMode(ch12, INPUT); //analog
      pinMode(ch13, INPUT); //analog
      pinMode(ch14, INPUT); //analog
      pinMode(ch15, INPUT); //analog
      _midi_controls.bank.up_channel        = ch9;
      _midi_controls.bank.down_channel      = ch10;
      _midi_controls.volume.channel         = ch11;
      _midi_controls.pitch.channel          = ch12;
      _midi_controls.modulation.channel     = ch13;
      _midi_controls.fx1.channel            = ch14;
      _midi_controls.program_change.channel = ch15;
      startMIDI();
   }
   void loop()
   {
      _midi_controls.volume.out();
      _midi_controls.pitch.out();
      _midi_controls.modulation.out();
      _midi_controls.fx1.out();
      _midi_controls.bank.up();
      _midi_controls.bank.down();
      _midi_controls.program_change.out(0, 45, 67, 80, 100);
   }
    
   #endif

}
