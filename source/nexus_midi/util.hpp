/*=============================================================================
   Copyright (c) 2016 Cycfi Research

   Distributed under the MIT License [ https://opensource.org/licenses/MIT ]
=============================================================================*/
#if !defined(CYCFI_UTIL_HPP_NOVEMBER_11_2016)
#define CYCFI_UTIL_HPP_NOVEMBER_11_2016
//
#include <stdint.h>
#include <energia.h>

namespace cycfi
{
   ////////////////////////////////////////////////////////////////////////////
   // debouncer: A switch debouncer
   ////////////////////////////////////////////////////////////////////////////
   template <int samples = 10>
   struct debouncer
   {
      debouncer()
       : counter(0)
       , result(false)
      {}

      bool operator()(bool sw)
      {
         if (sw)
         {
            if (counter == samples)
               result = true;
            else
               ++counter;
         }
         else
         {
            if (counter == 0)
               result = false;
            else
               --counter;
         }
         return result;
      }

   private:

      int counter;
      bool result;
   };

   ////////////////////////////////////////////////////////////////////////////
   // edge_detector: A switch edge detector
   ////////////////////////////////////////////////////////////////////////////
   template <int samples = 10>
   struct edge_detector : debouncer<samples>
   {
      typedef debouncer<samples> base_type;

      edge_detector()
       : prev(false)
      {}

      int operator()(bool sw)
      {
         bool curr = base_type::operator()(sw);
         if (prev != curr)
         {
            prev = curr;
            return curr ? 1 : -1;
         }
         return 0;
      }

   private:

      bool prev;
   };

   ////////////////////////////////////////////////////////////////////////////
   // repeat_button: A key detector with delay and repeat rate
   ////////////////////////////////////////////////////////////////////////////
   template <int delay_ = 1000, int rate = 100, int samples = 10>
   struct repeat_button : edge_detector<samples>
   {
      typedef edge_detector<samples> base_type;

      repeat_button()
       : start_time(-1)
      {}

      bool operator()(bool sw)
      {
         int state = base_type::operator()(sw);

         // rising edge
         if (state == 1)
         {
            start_time = millis();
            delay = delay_; // initial delay
            return true;
         }

         // falling edge
         else if (state == -1)
         {
            // reset
            start_time = -1;
            return false;
         }

         if (start_time == -1)
            return 0;

         // repeat button handling
         int now = millis();
         if (now > (start_time + delay))
         {
            start_time = now;
            delay = rate; // repeat delay
            return true;
         }
         return 0;
      }
      int start_time;
      int delay;
   };

   ////////////////////////////////////////////////////////////////////////////
   // Basic leaky-integrator filter. k will determine the effect of the
   // filter. Choose k to be a power of 2 for efficiency (the compiler
   // will optimize the computation using shifts). k = 16 is a good starting
   // point.
   //
   // This simulates the RC filter in digital form. The equation is:
   //
   //    y[i] = rho * y[i-1] + s
   //
   // where rho < 1. To avoid floating point, we use k instead which
   // allows for integer operations. In terms of k, rho = 1 - (1 / k).
   // So the actual formula is:
   //
   //    y[i] += s - (y[i-1] / k);
   //
   // k will also be the filter gain, so the final result should be
   // divided by k.
   //
   ////////////////////////////////////////////////////////////////////////////
   template <int k, typename T = int>
   struct lowpass
   {
      lowpass()
       : y(0)
      {}

      T operator()(T s)
      {
         y += s - (y / k);
         return y / k;
      }

      T y;
   };

   ////////////////////////////////////////////////////////////////////////////
   // Noise gate. Returns true if the signal, s, is above or below the given
   // window. For example, if window is 5, the previous signal is 20 and the
   // current signal, s, is within 15 to 25, the function returns false,
   // otherwise true.
   ////////////////////////////////////////////////////////////////////////////
   template <unsigned window, typename T = int>
   struct gate
   {
      gate()
       : val(0)
      {}

      bool operator()(T s)
      {
         if ((s < (val-window)) || (s > (val+window)))
         {
            val = s;
            return true;
         }
         return false;
      }

      T val;
   };
}

#endif
