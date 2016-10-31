/*
Project Name: API for MIDI Controller
Project Description: Allows MSP MCU to send MIDI data from CV(Control Voltage) 5-way Switch and Potentiometer
Date Created: 5/14/2016
*/

//#include "nexus_midi_controller.h"

/*Hard-wired Control VOltage Channel to MCU analog and digital pins*/
//#include <msp430.h>
#include"NexusMidiControls.h"
MIDI_CREATE_DEFAULT_INSTANCE();

uint16_t const ch9 = P2_0;
uint16_t const ch10 = P1_0;
uint16_t const ch11 = P1_3;
uint16_t const ch12 = P1_4;
uint16_t const ch13 = P1_5;
uint16_t const ch14 = P1_6;
uint16_t const ch15 = P1_7;

class SCHEDULER
{
 public:
    void set(uint16_t t)
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
    TACCR0 = 988; // Set TACCR0 = 2000 to generate a 1ms timebase @ 16MHz with a divisor of 8
     } 
    void start()
    {__enable_interrupt( );
    TACCTL0 = BIT4; // Enable interrupts when TAR = TACCR0
    }
    void stop()
    {
    TACCTL0 &= ~BIT4;
    }
    
}scheduler;

#pragma vector=TIMER0_A0_VECTOR
__interrupt  void timerA0ISR(void)
{
    static int state=0;
    state=~state;             // toggle state
    digitalWrite(P1_6,state); // Write to LED
}

class DEB
{
public:
	bool start(uint16_t ch) //integral debounce function
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
}debounce;

typedef class VOLUME
{
	uint8_t prev_volume_param; // = 0;
	
public:
        uint16_t channel;
	void volume_control() //adjust the volume
	{
	  uint8_t volume_param = map(analogRead(channel), 0, 1023, 0, 127);
	  if (volume_param < (prev_volume_param - 1)  || volume_param > (prev_volume_param + 1) )
	  {
		MIDI.sendControlChange(7, volume_param, 1);
		prev_volume_param = volume_param;
	  }
	}
};

typedef class PITCH
{
	int16_t prev_pitch_bend_param; // = 0;
	
public:
        uint16_t channel;
	void pitch_bend_control() //adjust the frequncy/pitch
    	{
      	   int16_t pitch_bend_param = map(analogRead(channel), 0, 1023, -8191, 8192);
      	   if ( pitch_bend_param < (prev_pitch_bend_param - 100)  ||  pitch_bend_param > (prev_pitch_bend_param + 100) )
      	   {
      	   MIDI.sendPitchBend(pitch_bend_param, 1);
      	   prev_pitch_bend_param = pitch_bend_param;
      	   }
    	}
};

typedef class MODULATION
{
    uint8_t prev_modulation_param; // = 0;
	
public:
        uint16_t channel;
	void modulation_control() //modulate
	{
	  uint8_t modulation_param = map(analogRead(channel), 0, 1023, 0, 127);
	  if ( modulation_param < (prev_modulation_param - 1)  ||  modulation_param > (prev_modulation_param + 1) )
	  {
		MIDI.sendControlChange(1,modulation_param, 1);
		prev_modulation_param = modulation_param;
	  }
	}
};

typedef class FX1
{
    uint8_t prev_fx1_param; // = 0;	
public:
        uint16_t channel;
	void fx1_control() //effects
		{
		  uint8_t fx1_param = map(analogRead(channel), 0, 1023, 0, 127);
		  if ( fx1_param < (prev_fx1_param - 1)  ||  fx1_param > (prev_fx1_param + 1) )
		  {
			MIDI.sendControlChange(12, fx1_param, 1);
			prev_fx1_param = fx1_param;
		  }
		}
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
	void program_change_control(uint16_t ch, uint8_t pos1, uint8_t pos2, uint8_t pos3, uint8_t pos4, uint8_t pos5) //select instrument
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
	void bankup() //banks
	{
	   bankup_state = debounce.start(up_channel);
	   if (prev_bankup_state != bankup_state)
	   {
		if (debounce.start(up_channel) == 1)
		{
		 if (bank_data == 127) 
		 bank_data=126;
		 bank_data++;
		 MIDI.sendControlChange(0, bank_data, 1);
		}
		prev_bankup_state = bankup_state;
	   }  
	}

	void bankdown()
	{
	   bankdown_state = debounce.start(down_channel);
	   if (prev_bankdown_state != bankdown_state)
	   {
		  if ( debounce.start(down_channel) == 1 )
		  {
		   if ( bank_data == 0 ) 
		   bank_data=1;
		   bank_data--;
		   MIDI.sendControlChange(0, bank_data, 1);
		  }
		  prev_bankdown_state = bankdown_state;
	   }   
	 }
};

class TEST
{
	/*Variable for testing purposes(for test_tone function)*/
	bool prev_state;
	bool state;
public:
	void start(uint16_t ch) //generate note for testing
	{
	   if (prev_state != state)
	   {
		 if (debounce.start(ch) == 1 )
		  {
		   MIDI.sendNoteOn(80, 127, 1);
		  }
		 else if (debounce.start(ch) == 0)
		  {
			MIDI.sendNoteOff(80, 0, 1);
		  } 
		 prev_state = state;
	   }
	   state = debounce.start(ch);
	}
}test_tone;


void setup()
{
  pinMode(P1_6,OUTPUT); // make the LED pins outputs
  pinMode(P1_0,OUTPUT);
  scheduler.set(1000);
  scheduler.start();
  //pinMode(BUT, INPUT);
//  pinMode(ch9, INPUT); //digital
//  pinMode(ch10, INPUT); //analog
//  pinMode(ch11, INPUT); //analog
//  pinMode(ch12, INPUT); //analog
//  pinMode(ch13, INPUT); //analog
//  pinMode(ch14, INPUT); //analog
//  pinMode(ch15, INPUT); //analog
//  MIDI.begin(4);          // Launch MIDI and listen to channel 4
}

void loop()
{
 //test_tone(ch9);
 //bankup(ch9);
 // delay(1);
 //bankdown(ch10);
 //delay(1);
 //volume_control(ch13);
 delay(1);
 //pitch_bend_control(ch14);
 delay(1);
 //modulation_control(ch15);
 //fx1_control(ch14);
 //program_change_control(ch15, 20, 45, 67, 80, 100);
 delay(1);
}

