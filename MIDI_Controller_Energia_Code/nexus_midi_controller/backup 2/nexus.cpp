
#include "nexus.h"

MIDI_CREATE_DEFAULT_INSTANCE();

void SCHEDULER :: set()
{
  // Bits 15-10: Unused
  // Bits 9-8: Clock source select: set to SMCLK (16MHz)
  // Bits 7-6: Input divider: set to 8
  // Bits 5-4: Mode control: Count up to TACCRO and reset
  // Bit 3: Unused
  // Bits 2: TACLR : set to initially clear timer system
  // Bit 1: Enable interrupts from TA0
  // Bit 0: Interrupt (pending) flag : set to zero (initially)
  TA0CTL = 0b0000001011010010; 
  TACCR0 = 988; // Set TACCR0 = 988 to generate a 1ms timebase @ 16MHz with a divisor of 8
} 

void SCHEDULER :: start()
{
  __enable_interrupt( );
  TACCTL0 = BIT4; // Enable interrupts when TAR = TACCR0
}

void SCHEDULER :: stop()
{
 // __disable_interrupt( );
  TACCTL0 &= ~BIT4;
}

bool debounce(uint16_t ch)
{
  int8_t counter=0; //clear counter
  uint8_t threshold=10; //will determine the logic levels when reached
  uint16_t prev_time=millis();
  uint16_t current_time=0;
  uint8_t interval=2; //2ms interval for reading the digital input
  while ( abs(counter) < threshold ) // check samples every 2ms until threshold was reached
  {
    current_time = millis();
    if ( (digitalRead(ch) == HIGH) && ((current_time - prev_time) > interval) )
    {counter++; prev_time=current_time; }
    else if ( (digitalRead(ch) == LOW) && ((current_time - prev_time) > interval) ) 
    {counter--; prev_time=current_time; }
  }     
  if (counter>0)
    return 1;
  else if (counter<0)
    return 0;
}

void VOLUME :: out()
{
  uint8_t volume_param = map(analogRead(channel), 0, 1023, 0, 127);
  if (volume_param < (prev_volume_param - 1)  || volume_param > (prev_volume_param + 1) )
  {
    MIDI.sendControlChange(7, volume_param, 1);
    prev_volume_param = volume_param;
  }
}

void PITCH :: pitch_bend_control()
{
  int16_t pitch_bend_param = map(analogRead(channel), 0, 1023, -8191, 8192);
  if ( pitch_bend_param < (prev_pitch_bend_param - 100)  ||  pitch_bend_param > (prev_pitch_bend_param + 100) )
  {
    MIDI.sendPitchBend(pitch_bend_param, 1);
    prev_pitch_bend_param = pitch_bend_param;
  }
}

void MODULATION :: modulation_control()
{
  uint8_t modulation_param = map(analogRead(channel), 0, 1023, 0, 127);
  if ( modulation_param < (prev_modulation_param - 1)  ||  modulation_param > (prev_modulation_param + 1) )
  {
    MIDI.sendControlChange(1,modulation_param, 1);
    prev_modulation_param = modulation_param;
  }
}

void FX1 :: fx1_control()
{
  uint8_t fx1_param = map(analogRead(channel), 0, 1023, 0, 127);
  if ( fx1_param < (prev_fx1_param - 1)  ||  fx1_param > (prev_fx1_param + 1) )
  {
    MIDI.sendControlChange(12, fx1_param, 1);
    prev_fx1_param = fx1_param;
  }
}

void PROGRAM :: program_change_control(uint8_t pos1, uint8_t pos2, uint8_t pos3, uint8_t pos4, uint8_t pos5)
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

void BANK :: bankup()
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

void BANK :: bankdown()
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

void TEST :: start(uint16_t ch)
{
  if (prev_state != state)
  {
    if (debounce(ch) == 1 )
    {
      MIDI.sendNoteOn(80, 127, 1);
    }
    else if (debounce(ch) == 0)
    {
      MIDI.sendNoteOff(80, 0, 1);
    } 
    prev_state = state;
  }
  state = debounce(ch);
}

void test()
{
       MIDI.sendNoteOn(80, 127, 1);
       delay(1000);
}
void startMIDI()
{
       MIDI.begin(4);
}

