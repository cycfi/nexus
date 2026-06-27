// Host harness: drives the REAL firmware controllers with synthetic ADC data
// (NEXUS_SIM) on a virtual clock, and decodes the MIDI they emit. Built with
// the host compiler — NOT cross-compiled for the MSP430.
//
//   cmake -S test -B test/build && cmake --build test/build
//   ./test/build/host_sim [ticks] [-q]
//
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <energia.h>
#include <MspFlash.h>

// Definitions for the extern globals the mocks declare.
uint32_t _sim_millis = 0;
int (*_sim_digital)(uint8_t) = 0;
std::vector<uint8_t> _sim_midi;
_SerialT Serial;
_FlashT Flash;
_MockFlashMem _flashmem;

// The firmware itself — the thin sketch brings in setup(), loop(), the control
// instances, and the environment singletons.
#include "nexus_midi.ino"

// ---------------------------------------------------------------------------
// MIDI decode
// ---------------------------------------------------------------------------
struct stats
{
   unsigned reset = 0, cc = 0, pc = 0, pb = 0;
   unsigned pb_min = 0xffffu, pb_max = 0, pb_last = 8192;
   int pc_last = -1;
};

static const char* cc_name(uint8_t c)
{
   switch (c)
   {
      case 0:  return "bank_select";
      case 1:  return "modulation";
      case 7:  return "channel_volume";
      case 12: return "effect_1";
      case 13: return "effect_2";
      case 64: return "sustain";
      default: return "cc";
   }
}

// The firmware writes whole messages, so the buffer holds complete messages.
static void drain(uint32_t t, bool verbose, stats& st)
{
   std::vector<uint8_t>& m = _sim_midi;
   size_t i = 0;
   while (i < m.size())
   {
      uint8_t s = m[i];
      uint8_t hi = s & 0xF0;
      if (s == 0xFF)
      {
         ++st.reset;
         if (verbose) printf("[t=%5u] RESET\n", (unsigned)t);
         i += 1;
      }
      else if (hi == 0xB0 && i + 3 <= m.size())
      {
         uint8_t cc = m[i + 1], v = m[i + 2];
         ++st.cc;
         if (verbose)
            printf("[t=%5u] CC%-3u %-15s= %u\n", (unsigned)t, cc, cc_name(cc), v);
         i += 3;
      }
      else if (hi == 0xC0 && i + 2 <= m.size())
      {
         ++st.pc; st.pc_last = m[i + 1];
         if (verbose) printf("[t=%5u] program_change   = %u\n", (unsigned)t, m[i + 1]);
         i += 2;
      }
      else if (hi == 0xE0 && i + 3 <= m.size())
      {
         unsigned v = m[i + 1] | (unsigned(m[i + 2]) << 7);
         ++st.pb; st.pb_last = v;
         if (v < st.pb_min) st.pb_min = v;
         if (v > st.pb_max) st.pb_max = v;
         if (verbose) printf("[t=%5u] pitch_bend       = %5u\n", (unsigned)t, v);
         i += 3;
      }
      else
      {
         i += 1;   // unexpected; skip a byte
      }
   }
   m.clear();
}

int main(int argc, char** argv)
{
   unsigned ticks = (argc > 1) ? (unsigned)strtoul(argv[1], 0, 10) : 2000;
   bool verbose = !(argc > 2 && strcmp(argv[2], "-q") == 0);
   stats st;

   printf("=== nexus_midi host sim — %u ticks, seed triangle on all channels ===\n", ticks);
   printf("--- startup ---\n");
   setup();
   drain(0, true, st);

   printf("--- run (%s) ---\n", verbose ? "verbose" : "quiet");
   for (unsigned k = 1; k <= ticks; ++k)
   {
      _sim_millis = k;
      loop();
      drain(k, verbose, st);
   }

   printf("--- summary ---\n");
   printf("  messages: reset=%u  cc=%u  program_change=%u  pitch_bend=%u\n",
          st.reset, st.cc, st.pc, st.pb);
   printf("  pitch_bend range: %u..%u  last=%u  (center 8192)\n",
          st.pb_min, st.pb_max, st.pb_last);
   printf("  program_change last=%d\n", st.pc_last);
   return 0;
}
