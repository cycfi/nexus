/*=============================================================================
   Unit tests for the processor blocks: the general ones (slew_gate, stillness,
   dc_servo) live in util.hpp, the pitch-specific ones in nexus_controls.hpp.

   Each block is exercised in ISOLATION, so a failure localizes to one block --
   unlike the whole-loop replay (replay_captures.py), which is the integration
   gate. Both run in CI: the unit tests pin each block's contract, the replay
   pins their composition. Built host-side:

     cmake -S test -B test/build && cmake --build test/build
     ./test/build/unit_blocks
=============================================================================*/
#include "nexus_controls.hpp"   // the pitch-specific blocks live next to the controller now;
                               // util.hpp (general blocks) comes in transitively
#include <cstdio>

using namespace cycfi;

static int g_fail = 0;
#define CHECK(cond, msg) \
   do { if (!(cond)) { std::printf("  FAIL [%s]: %s\n", __func__, msg); ++g_fail; } } while (0)

//----------------------------------------------------------------------------
// slew_gate: a too-fast jump is held (and stays held while sustained); physical
// motion within the rate passes untouched.
static void test_slew_gate()
{
   slew_gate<6, 4, 6> g;                  // 6 LSB/ms + 4 slack, dt clamp 6 ms
   g.init(500, 0);

   CHECK(g(505, 1) == 505, "a 5-LSB step (<= 6*1+4) passes");
   CHECK(g(600, 2) == 505, "a 95-LSB jump is rejected -> holds last good");
   CHECK(g(600, 3) == 505, "a SUSTAINED glitch stays rejected (never creeps onto it)");
   CHECK(g(508, 4) == 508, "input back near the held value passes again");

   // dt clamp: a long gap can't license an arbitrarily large jump.
   slew_gate<6, 4, 6> h;
   h.init(500, 0);
   CHECK(h(900, 1000) == 500, "dt is clamped to 6 ms, so 6*6+4=40 < 400 -> still rejected");
}

//----------------------------------------------------------------------------
// stillness: a slow ramp reads as MOVING (the smoother's velocity band can't
// see it); a settled rest reads STILL, but only after the dwell elapses.
static void test_stillness()
{
   stillness<32, 400> s;                  // 32-unit band, 400 ms dwell

   s.init(0, 0);
   CHECK(!s(0, 100), "not still yet -- dwell (400 ms) not elapsed");
   CHECK(!s(2, 399), "still within band but dwell still short");
   CHECK(s(2, 400), "settled within band for >= 400 ms -> still");

   // Slow ramp ~0.2 unit/ms (like rec07_take2's slow 1 ST). It crosses the
   // 32-band every ~160 ms < 400 ms, so the dwell keeps restarting.
   s.init(0, 1000);
   bool any_still = false;
   for (int t = 1; t <= 3000; ++t)
      any_still |= s(int32_t(t / 5), 1000 + t);     // t/5 == 0.2*t, integer
   CHECK(!any_still, "a slow ramp never reads still (displacement dwell catches it)");
   CHECK(s(600, 1000 + 3000 + 500), "once the ramp stops, still returns after the dwell");
}

//----------------------------------------------------------------------------
// dc_servo (Pre-DC servo): tracks the baseline only when still + near neutral,
// freezes on motion or a real bend, and is hard-clipped to the seed +/- clip.
static void test_dc_servo()
{
   dc_servo<16, 546, 736> s;              // tau 2^16 ms, gate 546, clip +/-736 of seed
   s.init(1000);
   CHECK(s.value() == 1000, "seeds C0 to the rest");
   CHECK(s.seed_value() == 1000, "remembers the FIXED seed (clip center)");

   for (int i = 0; i < 2000; ++i) s(1010, true, 1000);
   CHECK(s.value() == 1010, "still + near neutral: tracks the slow baseline");

   s.init(1000);
   for (int i = 0; i < 2000; ++i) s(1010, false, 1000);
   CHECK(s.value() == 1000, "not still: frozen (won't follow even a near-neutral target)");

   s.init(1000);
   for (int i = 0; i < 2000; ++i) s(2000, true, 1000);
   CHECK(s.value() == 1000, "a bend past the gate (|defl-C0| > 546): frozen, never dragged onto it");

   s.init(1000);
   for (int i = 0; i < 1000; ++i) s(s.value() + 500, true, 1000);   // always within gate, walks up
   CHECK(s.value() == 1000 + 736, "hard-clipped to seed + clip: can never reach a real bend");
}

//----------------------------------------------------------------------------
// output_stage: suppress unchanged / within-deadband pitches (but never a center
// snap), apply the symmetric x1.2 gain to the emitted value, clamp to pb_max.
static void test_output_stage()
{
   output_stage o;   // x1.2 gain, center 8192, deadband 16 (in-class constexpr)
   int32_t out = -1;

   CHECK(!o(8192, &out), "unchanged from the seeded center: suppressed");
   CHECK(!o(8200, &out), "within the deadband (|8200-8192| = 8 <= 16): suppressed");
   CHECK(o(9192, &out) && out == 8192 + (1000 * 307) / 256,
         "beyond the deadband: sent, with the x1.2 symmetric gain applied");

   o.reset(8196);
   CHECK(o(8192, &out) && out == 8192, "a center snap is sent even within the deadband");

   o.reset(0);
   CHECK(o(16000, &out) && out == 16383, "the gained value clamps to pb_max (14-bit ceiling)");
}

//----------------------------------------------------------------------------
// settle_gate: the startup valid-settled-rest detector. Climbing input never
// fires; a steady in-band rest fires only after the dwell AND the hold-off; an
// out-of-band reading never fires.
static void test_settle_gate()
{
   settle_gate g;   // rest band [valid_lo,valid_hi], settle settle_lsb / 300 ms (in-class)
   const uint32_t boot = 0, holdoff = 5000;
   const int32_t in  = (settle_gate::valid_lo + settle_gate::valid_hi) / 2;  // mid rest band
   const int32_t out = settle_gate::valid_hi + 100;                          // out of band
   const int32_t jit = settle_gate::settle_lsb + 5;                          // a move > settle_lsb

   g.init(in, 0);
   bool ready = false;
   for (uint32_t t = 1; t <= 400; ++t)
      ready |= g(in + (t & 1 ? jit : 0), 6000 + t, boot, holdoff);   // always moving -> never settles
   CHECK(!ready, "a moving input never reads settled");

   g.init(in, 0);
   CHECK(!g(in, 400, boot, holdoff), "settled 300+ ms but before the 5 s hold-off: not ready");

   g.init(in, 6000);
   CHECK(!g(in, 6200, boot, holdoff), "in band but the settle dwell not yet met (200 < 300 ms)");
   CHECK(g(in, 6400, boot, holdoff), "held in band 300+ ms past the hold-off -> ready");

   g.init(out, 6000);
   CHECK(!g(out, 9000, boot, holdoff), "out of the rest band: never ready");
}

//----------------------------------------------------------------------------
// adc::decimate: os_ratio (= 4^os_shift) raw 10-bit samples sum -> a value
// os_shift bits wider (same physical level, +os_shift real bits via dither).
static void test_adc_decimate()
{
   using namespace cycfi::adc;
   CHECK(decimate(os_ratio * 512) == (512 << os_shift), "oversample of 10-bit 512 -> +os_shift bits");
   CHECK(decimate(os_ratio * 1023) == (1023 << os_shift), "oversample of full scale -> +os_shift bits");
   CHECK(decimate(0) == 0, "zero -> zero");
}

int main()
{
   test_slew_gate();
   test_stillness();
   test_dc_servo();
   test_output_stage();
   test_settle_gate();
   test_adc_decimate();
   if (g_fail) { std::printf("unit_blocks: %d CHECK(s) FAILED\n", g_fail); return 1; }
   std::printf("unit_blocks: PASSED (5 blocks + adc::decimate)\n");
   return 0;
}
