/*=============================================================================
   Copyright (c) 2016 Cycfi Research

   Distributed under the MIT License [ https://opensource.org/licenses/MIT ]
=============================================================================*/
#if !defined(CYCFI_UTIL_HPP_NOVEMBER_11_2016)
#define CYCFI_UTIL_HPP_NOVEMBER_11_2016

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
}

#endif
