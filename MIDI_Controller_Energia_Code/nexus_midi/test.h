
#ifndef _TEST_H_
#define _TEST_H_

#include <energia.h>

#define TEST_VOLUME

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
