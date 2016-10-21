/*
Project Name: API for MIDI Controller
Project Description: Allows MSP MCU to send MIDI data from CV(Control Voltage) 5-way Switch and Potentiometer
Date Created: 5/14/2016
*/

#include <MIDI.h>

MIDI_CREATE_DEFAULT_INSTANCE();

/*Hard-wired Control VOltage Channel to MCU analog and digital pins*/
uint16_t const ch9 = P2_0;
uint16_t const ch10 = P1_0;
uint16_t const ch11 = P1_3;
uint16_t const ch12 = P1_4;
uint16_t const ch13 = P1_5;
uint16_t const ch14 = P1_6;
uint16_t const ch15 = P1_7;

/*Variable for testing purposes(for test_tone function)*/
bool prev_state=false;
bool state;


/*MIDI Parameter Variables*/
int16_t program_number;
uint8_t prev_volume_param=0;
int16_t prev_pitch_bend_param=0;
uint8_t prev_modulation_param=0;
uint8_t prev_fx1_param=0;
uint8_t prev_program_number=1;

bool prev_bankup_state = 0;
bool prev_bankdown_state = 0;
bool bankup_state;
bool bankdown_state;
uint16_t bank_data=0;

/*Function Prototypes*/

//MIDI Controls API
void volume_control(uint16_t ch); //adjust the volume
void pitch_bend_control(uint16_t ch); //adjust the frequncy/pitch
void modulation_control(uint16_t ch); //modulate
void fx1_control(uint16_t ch); //effects
void program_change_control(uint16_t ch, uint8_t pos1, uint8_t pos2, uint8_t pos3, uint8_t pos4, uint8_t pos5); //select instrument
void bankup(uint16_t ch); //banks
void bankdown(uint16_t ch);
//Supports
void test_tone(uint16_t ch); //generate note for testing
bool debounce(uint16_t ch); //integral debounce function

void setup()
{
  //pinMode(BUT, INPUT);
  pinMode(ch9, INPUT); //digital
  pinMode(ch10, INPUT); //analog
  pinMode(ch11, INPUT); //analog
  pinMode(ch12, INPUT); //analog
  pinMode(ch13, INPUT); //analog
  pinMode(ch14, INPUT); //analog
  pinMode(ch15, INPUT); //analog
  MIDI.begin(4);          // Launch MIDI and listen to channel 4
}

void loop()
{
 //test_tone(ch9);
 //bankup(ch9);
 // delay(1);
 //bankdown(ch10);
 //delay(1);
 volume_control(ch13);
 delay(1);
 pitch_bend_control(ch14);
 delay(1);
 //modulation_control(ch15);
 //fx1_control(ch14);
 program_change_control(ch15, 20, 45, 67, 80, 100);
 delay(1);
}

   
void volume_control(uint16_t ch)
{
  uint8_t volume_param = map(analogRead(ch), 0, 1023, 0, 127);
  if (volume_param < (prev_volume_param - 1)  || volume_param > (prev_volume_param + 1) )
  {
    MIDI.sendControlChange(7, volume_param, 1);
    prev_volume_param = volume_param;
  }
}

void pitch_bend_control(uint16_t ch)
{
  int16_t pitch_bend_param = map(analogRead(ch), 0, 1023, -8191, 8192);
  if ( pitch_bend_param < (prev_pitch_bend_param - 100)  ||  pitch_bend_param > (prev_pitch_bend_param + 100) )
  {
    MIDI.sendPitchBend(pitch_bend_param, 1);
    prev_pitch_bend_param = pitch_bend_param;
  }
}

void modulation_control(uint16_t ch)
{
  uint8_t modulation_param = map(analogRead(ch), 0, 1023, 0, 127);
  if ( modulation_param < (prev_modulation_param - 1)  ||  modulation_param > (prev_modulation_param + 1) )
  {
    MIDI.sendControlChange(1,modulation_param, 1);
    prev_modulation_param = modulation_param;
  }
}

void fx1_control(uint16_t ch)
{
  uint8_t fx1_param = map(analogRead(ch), 0, 1023, 0, 127);
  if ( fx1_param < (prev_fx1_param - 1)  ||  fx1_param > (prev_fx1_param + 1) )
  {
    MIDI.sendControlChange(12, fx1_param, 1);
    prev_fx1_param = fx1_param;
  }
}

/*
programChangeControl function use 5-way switch and resistor network to generate specific data.
Position 1 = 4.8V;
Position 2 = 3.3V;
Position 3 = 2.4V;
Position 4 = 1.2V;
Position 5 = 0.0V;
*/
void program_change_control(uint16_t ch, uint8_t pos1, uint8_t pos2, uint8_t pos3, uint8_t pos4, uint8_t pos5)
{
  float voltage = analogRead(ch);
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

void bankup(uint16_t ch)
{
   bankup_state = debounce(ch);
   if (prev_bankup_state != bankup_state)
   {
    if (debounce(ch) == 1)
    {
     if (bank_data == 127) 
     bank_data=126;
     bank_data++;
     MIDI.sendControlChange(0, bank_data, 1);
    }
    prev_bankup_state = bankup_state;
   }  
}

void bankdown(uint16_t ch)
{
   bankdown_state = debounce(ch);
   if (prev_bankdown_state != bankdown_state)
   {
      if ( debounce(ch) == 1 )
      {
       if ( bank_data == 0 ) 
       bank_data=1;
       bank_data--;
       MIDI.sendControlChange(0, bank_data, 1);
      }
      prev_bankdown_state = bankdown_state;
   }   
 }

void test_tone(uint16_t ch)
{
   if(prev_state != state)
   {
     if(debounce(ch) == 1 )
      {
       MIDI.sendNoteOn(80, 127, 1);
      }
     else if(debounce(ch) == 0)
      {
        MIDI.sendNoteOff(80, 0, 1);
      } 
     prev_state = state;
   }
   state = debounce(ch);
}

bool debounce(uint16_t tact_switch)
{
    int8_t counter=0; //clear counter
    uint8_t threshold=10; //will determine the logic levels when reached
    uint16_t prev_time=millis();
    uint16_t current_time=0;
    uint8_t interval=2; //2ms interval for reading the digital input
    while ( abs(counter) < threshold ) // check samples every 2ms until threshold was reached
    {
     current_time = millis();
     if ( (digitalRead(tact_switch) == HIGH) && ((current_time - prev_time) > interval) )
     {counter++; prev_time=current_time;}
     else if ( (digitalRead(tact_switch) == LOW) && ((current_time - prev_time) > interval) ) 
     {counter--; prev_time=current_time; }
    }     
    if (counter>0)
    return 1;
    else if (counter<0)
    return 0;
}
