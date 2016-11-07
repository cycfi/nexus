
#include "test.h"
#include "nexus.h"
//midi::MidiInterface<HardwareSerial> MIDI((HardwareSerial&)Serial);

namespace cycfi
{
   void note::out(uint16_t ch)
   {
   if (prev_state != state)
      {
      if (debounce(ch) == 1 )
         MIDI.sendNoteOn(80, 127, 1);
      else if (debounce(ch) == 0)
         MIDI.sendNoteOff(80, 0, 1);
    
      prev_state = state;
      }
   state = debounce(ch);
   }

   #ifdef NEXUS_TEST

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
  
       _midi_controls.bank.up_channel        = ch12;
       _midi_controls.bank.down_channel      = ch13;
       _midi_controls.volume.channel         = ch11;
       _midi_controls.pitch.channel          = ch11;
       _midi_controls.modulation.channel     = ch11;
       _midi_controls.fx1.channel            = ch11;
       _midi_controls.program_change.channel = ch11;
       startMIDI();
   }  

   void loop()
   {
      note _note;
      _note.out(ch12);
      
      #ifdef TEST_VOLUME
      _midi_controls.volume.out();
      #endif
      
      #ifdef TEST_PITCH
      _midi_controls.pitch.out();
      #endif
      
      #ifdef TEST_MODULATION
      _midi_controls.modulation.out();
      #endif
      
      #ifdef TEST_FX1
      _midi_controls.fx1.out();
      #endif
      
      #ifdef TEST_PROGRAM_CHANGE
      _midi_controls.program_change.out(0, 45, 67, 80, 100);
      #endif
      
      #ifdef TEST_BANK
      _midi_controls.bank.up();
      _midi_controls.bank.down();
      #endif
   }

   #endif

}

