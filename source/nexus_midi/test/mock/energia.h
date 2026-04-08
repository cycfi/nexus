// Minimal Energia stub for host-side testing.
// Only provides what util.hpp needs on the host.
#pragma once
#include <stdint.h>

inline uint32_t millis()
{
   static uint32_t count = 0;
   return count++;
}
