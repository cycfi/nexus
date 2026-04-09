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
   // dynamic_smoother: Adaptive lowpass filter. Integer port of the Q DSP
   // dynamic_smoother (Andrew Simper, Cytomic, 2016). The bandpass output
   // (low1 - low2) modulates the cutoff, giving fast tracking on transients
   // and heavy smoothing when the signal is stable.
   //
   // State is stored in Q8 (×256) for sub-integer precision.
   // G0:    base gain in Q8 [0..256]. Analogous to g0 = 2·tan(π·fc/fs).
   //        E.g. G0=16 → g≈0.06, ~10 Hz base cutoff at 1 kHz.
   // Sense: sensitivity. Scales the 10-bit band magnitude before adding to
   //        g (also Q8). Analogous to sense = sensitivity × 4 in the float
   //        version but expressed for a 10-bit signal.
   ////////////////////////////////////////////////////////////////////////////
   template <int G0, int Sense, int OutShift = 8, typename T = int32_t>
   struct dynamic_smoother
   {
      T operator()(T s)
      {
         T const low1z = low1;
         T const low2z = low2;
         T band = low1z - low2z;
         if (band < 0)
            band = -band;

         // band is Q8; >> 8 converts to 10-bit units, second >> 8 applies Q8 scale
         int32_t g = G0 + ((int32_t(Sense) * (band >> 8)) >> 8);
         if (g > 256)
            g = 256;

         low1 = low1z + (int32_t(g) * ((s << 8) - low1z) >> 8);
         low2 = low2z + (int32_t(g) * (low1 - low2z) >> 8);
         return low2 >> OutShift;
      }

      dynamic_smoother() : low1(0), low2(0) {}

      T low1;
      T low2;
   };

   ////////////////////////////////////////////////////////////////////////////
   // dc_block: High-pass (DC blocking) filter. Removes slow DC offset/drift.
   // K is the time constant in samples; use powers of 2 for efficiency
   // (division becomes a shift). TC = K / sample_rate.
   // E.g. K=8192 at 1 kHz → TC ≈ 8.2 s.
   ////////////////////////////////////////////////////////////////////////////
   template <int Shift, typename T = int32_t>
   struct dc_block
   {
      // TC = 2^Shift / sample_rate. E.g. Shift=16 at 1 kHz → TC ≈ 65.5 s.
      dc_block() : _lp(0) {}

      // Pre-load the DC estimate so the filter starts converged.
      void init(T s) { _lp = s << Shift; }

      T operator()(T s)
      {
         _lp += s - (_lp >> Shift);
         return s - (_lp >> Shift);
      }

      T _lp;
   };

   //////////////////////////////////////////////////////////////////////////////
   // offset_servo: Tracks and removes slow offset drift near the center.
   //////////////////////////////////////////////////////////////////////////////
   template <int Shift, typename T = int32_t>
   struct offset_servo
   {
      offset_servo()
      : _i(0)
      {}

      void init(T s)
      {
         _i = s << Shift;
      }

      void update(T s)
      {
         _i += s - (_i >> Shift);
      }

      T operator()(T s) const
      {
         return s - (_i >> Shift);
      }

      T offset() const
      {
         return _i >> Shift;
      }

      T _i;
   };

   ////////////////////////////////////////////////////////////////////////////
   // moving_average: Simple boxcar FIR filter. Optimal for reducing white
   // (broadband) noise. N = 2^Shift samples. Noise is reduced by sqrt(N)
   // and latency is N/2 samples. Use a small T (e.g. int16_t) to save RAM
   // when the signal range fits — 10-bit ADC (0–1023) fits in int16_t and
   // the sum of up to 32 samples (32×1023=32736) also fits in int16_t.
   ////////////////////////////////////////////////////////////////////////////
   template <int Shift, typename T = int32_t>
   struct moving_average
   {
      static int constexpr size = 1 << Shift;

      moving_average() : _sum(0), _index(0)
      {
         for (int i = 0; i < size; ++i)
            _buf[i] = 0;
      }

      void init(T s)
      {
         for (int i = 0; i < size; ++i)
            _buf[i] = s;
         _sum = s * size;
         _index = 0;
      }

      T operator()(T s)
      {
         _sum -= _buf[_index];
         _buf[_index] = s;
         _sum += s;
         _index = (_index + 1) & (size - 1);
         return _sum >> Shift;
      }

      T   _buf[size];
      T   _sum;
      int _index;
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
