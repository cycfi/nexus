/*=============================================================================
   Copyright (c) 2016 Cycfi Research

   Distributed under the MIT License [ https://opensource.org/licenses/MIT ]
=============================================================================*/
#if !defined(CYCFI_NEXUS_SIM_HPP_JUNE_25_2026)
#define CYCFI_NEXUS_SIM_HPP_JUNE_25_2026

#include <stdint.h>

///////////////////////////////////////////////////////////////////////////////
// Synthetic ADC source for NEXUS_SIM builds.
//
// Deterministic and driven by the millisecond clock, so each boot (device) or
// each run (host) replays the exact same stimulus -- which makes before/after
// comparisons meaningful. The firmware's raw_adc() seam calls sample(pin, ms)
// in place of analogRead(pin); both the CC controls and pitch bend feed from
// here.
//
// This is the seed generator: a slow full-scale triangle on every channel.
// Per-control test patterns (vibrato through center, sustained off-center
// holds, return-to-center dwells, crosstalk spikes, ...) get filled in next.
///////////////////////////////////////////////////////////////////////////////
namespace nexus_sim
{
   inline uint16_t sample(uint16_t /*pin*/, uint32_t ms)
   {
      uint32_t const period = 4000;                  // 4 s
      uint32_t const half   = period / 2;
      uint32_t t   = ms % period;
      uint32_t tri = (t < half) ? t : (period - t);  // 0 .. half .. 0
      return uint16_t((tri * 1023) / half);          // 0 .. 1023
   }
}

#endif
