
#ifndef _TEST_H_
#define _TEST_H_

#include <energia.h>

#define TEST_VOLUME
#define TEST_PITCH
#define TEST_MODULATION
#define TEST_FX1
#define TEST_BANK
#define TEST_PROGRAM_CHANGE


namespace cycfi
{
   class note
   {
   /*Variable for testing purposes(for test_tone function)*/
   bool state;
   public:
      bool prev_state;
      void out(uint16_t ch); //generate note for testing
   };
  
   void setup();
   void loop();
}

#endif
