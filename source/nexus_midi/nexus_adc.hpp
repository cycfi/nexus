/*=======================================================================================
    Copyright (c) 2016 Cycfi Research

    Distributed under the MIT License [ https://opensource.org/licenses/MIT ]
   =======================================================================================*/
#if !defined(CYCFI_NEXUS_ADC_HPP_JUNE_2026)
#define CYCFI_NEXUS_ADC_HPP_JUNE_2026

#include <stdint.h>

///////////////////////////////////////////////////////////////////////////////
// ADC oversampler for nexus_midi.
//
// The MSP430G2553 ADC10 is a 10-bit SAR. We oversample-decimate each channel for
// extra bits: sum 4^os_shift conversions, shift right os_shift -> +os_shift real
// bits (the Hall front-end's own noise is the dither) and sqrt(4^os_shift)x less
// noise. Default 16x -> 12-bit (+2 bits).
//
// read_os() is a BLOCKING burst on a single channel, built from the same one-shot
// conversion the stock analogRead() uses (proven to work standalone) -- NOT a free-
// running MSC + ISR. An earlier background-ISR version read 0 on the bench: the
// free-run + channel-switch-from-ISR was fragile and only "worked" with the
// programmer attached (which masks standalone behavior). Blocking is deterministic,
// uses NO .bss (no value[]/ISR), and the time-based control loop absorbs the ~0.1 ms
// per channel (its dynamics advance by elapsed ms, not per loop).
//
// A throwaway conversion leads each burst so the S&H settles on the freshly selected
// channel before the 16 we keep -- otherwise the first sample (charged from the
// previous channel) drags the average down and the top of the range never lands.
//
// Device-only: on the host (NEXUS_HOST) the controllers keep their existing
// sim / analogRead path. The decimation math is unit-tested (test/unit_blocks.cpp).
///////////////////////////////////////////////////////////////////////////////
namespace cycfi { namespace adc
{
   // Output is os_shift bits wider than the 10-bit ADC: oversample 4^os_shift.
   static constexpr int      os_shift = 2;              // +2 bits -> 12-bit output
   static constexpr int      os_ratio = 1 << (2 * os_shift);   // 4^os_shift samples (16)
   static constexpr int32_t  out_max  = (1024 << os_shift) - 1;   // 12-bit full scale (4095)

   // Decimate one channel's accumulated block: sum of os_ratio 10-bit samples ->
   // an (10 + os_shift)-bit value. Pure + host-testable.
   inline int32_t decimate(uint32_t sum) { return int32_t(sum >> os_shift); }

#ifndef NEXUS_HOST
   // Enable the analog inputs on the wired channels (A0, A3..A7). Once, from setup().
   inline void start() { ADC10AE0 = BIT0 | BIT3 | BIT4 | BIT5 | BIT6 | BIT7; }

   // Blocking oversampled read of ADC10 input channel `inch` (0..7) -> 0..out_max.
   inline uint16_t read_os(uint8_t inch)
   {
      ADC10CTL0 &= ~ENC;                                  // ENC=0 to edit CTL1
      ADC10CTL1 = (uint16_t(inch) << 12) | ADC10SSEL_0;   // channel, ADC10OSC, single (CONSEQ_0)
      ADC10CTL0 = ADC10SHT_2 | ADC10ON | SREF_0;          // 16-cyc S&H, Vcc ref, no interrupt
      ADC10CTL0 |= ENC | ADC10SC;                         // throwaway: settle S&H on the new channel
      while (ADC10CTL1 & ADC10BUSY) {}
      (void) ADC10MEM;
      uint16_t sum = 0;
      for (int i = 0; i < os_ratio; ++i)
      {
         ADC10CTL0 |= ENC | ADC10SC;                      // one conversion
         while (ADC10CTL1 & ADC10BUSY) {}
         sum += ADC10MEM;                                 // read (clears the flag)
      }
      return uint16_t(decimate(sum));
   }
#else
   inline void     start() {}                  // host: the oversampler is not used
   inline uint16_t read_os(uint8_t) { return 0; }
#endif
}}

#endif
