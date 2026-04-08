// Tests for cycfi::dynamic_smoother and cycfi::dc_block (util.hpp)
//
// dynamic_smoother tests (OutShift=8, 10-bit output mode):
//
//  1. Convergence: a constant input should drive the filter to a fixed point
//     equal to the input within ±1 (the ±1 tolerance is inherent — Q8 integer
//     rounding can leave the state up to 15/256 below the exact target, which
//     may truncate to -1 in 10-bit output units).
//
//  2. Step response: after settling at 0, a step to full scale (1023) should
//     reach 90 % within 20 samples.  The adaptive gain (Sense) opens up the
//     cutoff when the bandpass output is large, giving much faster tracking
//     than a fixed-gain filter at the same base cutoff.  The test confirms
//     this by comparing against a Sense=0 instance, which needs ~60 samples.
//
//  3. Noise suppression: with the filter settled, alternating ±1 jitter
//     should be attenuated to ≤1 in the output.  When the signal is stable
//     the bandpass output is near zero, so g stays at G0 (heavy smoothing)
//     and the sense term does not open up the cutoff on sub-LSB noise.
//
// dynamic_smoother tests (OutShift=4, 14-bit output mode):
//
//  4. Convergence to 14-bit fixed point: same as test 1 but output is scaled
//     to [0, 16383].  Input 512 should converge to 8192 ±1.
//
//  5. Step response at 14-bit scale: 90 % of 16383 (≥14745) within 20 samples.
//     Confirms that the extra output bits do not affect adaptive behaviour.
//
// dc_block tests:
//
//  6. DC removal: a constant input held for many samples should decay to near 0.
//     Uses K=64 (short TC) so the test runs in a few hundred iterations.
//
//  7. Fast signal passthrough: a sudden step mostly passes through the DC block
//     immediately (the high-pass nature lets transients through).
//
//  8. Bipolar centering: dc output + 8192 should stay within [0, 16383] for
//     typical ADC input range, and the center (no-bend) value should be 8192
//     when the input is at its DC equilibrium.
//
// A step-response table is also printed for manual inspection and tuning of
// the G0 / Sense template parameters.

#include "util.hpp"
#include <cstdio>
#include <cstdlib>

using namespace cycfi;

// ----------------------------------------------------------------------------
// Minimal test harness
// ----------------------------------------------------------------------------
static int tests_run    = 0;
static int tests_failed = 0;

#define CHECK(expr)                                                  \
   do {                                                              \
      ++tests_run;                                                   \
      if (!(expr)) {                                                 \
         ++tests_failed;                                             \
         printf("  FAIL: %s  (line %d)\n", #expr, __LINE__);        \
      }                                                              \
   } while (0)

// ----------------------------------------------------------------------------
// Test 1: convergence (10-bit / OutShift=8)
// ----------------------------------------------------------------------------
static void test_convergence()
{
   printf("Test 1: convergence to steady state (10-bit)\n");

   dynamic_smoother<16, 128> ds;

   int32_t out = 0;
   for (int i = 0; i < 2000; ++i)
      out = ds(512);

   printf("  output after 2000 samples at input=512: %d\n", (int)out);
   CHECK(out >= 511 && out <= 513);

   dynamic_smoother<16, 128> ds0, ds1;
   for (int i = 0; i < 2000; ++i) { ds0(0); ds1(1023); }
   CHECK(ds0(0) == 0);
   CHECK(ds1(1023) >= 1022 && ds1(1023) <= 1023);
}

// ----------------------------------------------------------------------------
// Test 2: step response (10-bit / OutShift=8)
// ----------------------------------------------------------------------------
static void test_step_response()
{
   printf("Test 2: step response tracking speed (10-bit)\n");

   dynamic_smoother<16, 128> ds;

   for (int i = 0; i < 500; ++i)
      ds(0);

   int     first_90pct = -1;
   int32_t final_out   = 0;
   for (int i = 0; i < 100; ++i)
   {
      int32_t out = ds(1023);
      if (first_90pct < 0 && out >= 920)
         first_90pct = i + 1;
      final_out = out;
   }

   printf("  90 %% reached at sample %d,  final output %d\n",
          first_90pct, (int)final_out);

   CHECK(first_90pct > 0 && first_90pct <= 20);
   CHECK(final_out >= 1000);

   dynamic_smoother<16, 0> ds_fixed;
   for (int i = 0; i < 500; ++i)
      ds_fixed(0);

   int fixed_90pct = -1;
   for (int i = 0; i < 500; ++i)
   {
      int32_t out = ds_fixed(1023);
      if (fixed_90pct < 0 && out >= 920)
         fixed_90pct = i + 1;
   }
   printf("  fixed-gain 90 %% reached at sample %d\n", fixed_90pct);
   CHECK(first_90pct < fixed_90pct);
}

// ----------------------------------------------------------------------------
// Test 3: noise suppression (10-bit / OutShift=8)
// ----------------------------------------------------------------------------
static void test_noise_suppression()
{
   printf("Test 3: noise suppression on stable signal (10-bit)\n");

   dynamic_smoother<16, 128> ds;

   for (int i = 0; i < 500; ++i)
      ds(512);

   int32_t max_dev = 0;
   for (int i = 0; i < 200; ++i)
   {
      int32_t noise = (i % 2 == 0) ? 513 : 511;
      int32_t out   = ds(noise);
      int32_t dev   = out - 512;
      if (dev < 0) dev = -dev;
      if (dev > max_dev) max_dev = dev;
   }

   printf("  max deviation from 512 under ±1 jitter: %d\n", (int)max_dev);
   CHECK(max_dev <= 1);
}

// ----------------------------------------------------------------------------
// Test 4: convergence (14-bit / OutShift=4)
//
// With OutShift=4 the output is low2>>4.  The Q8 state converges to within
// 15 Q8 units of the target; at the >>4 scale that is within ±2 of the
// 14-bit target.
// ----------------------------------------------------------------------------
static void test_convergence_14bit()
{
   printf("Test 4: convergence to steady state (14-bit, OutShift=4)\n");

   dynamic_smoother<16, 128, 4> ds;

   int32_t out = 0;
   for (int i = 0; i < 2000; ++i)
      out = ds(512);

   printf("  output after 2000 samples at input=512: %d  (expected ~8192)\n", (int)out);
   CHECK(out >= 8190 && out <= 8194);

   dynamic_smoother<16, 128, 4> ds0, ds1;
   for (int i = 0; i < 2000; ++i) { ds0(0); ds1(1023); }
   CHECK(ds0(0) == 0);
   CHECK(ds1(1023) >= 16360 && ds1(1023) <= 16383);
}

// ----------------------------------------------------------------------------
// Test 5: step response (14-bit / OutShift=4)
// ----------------------------------------------------------------------------
static void test_step_response_14bit()
{
   printf("Test 5: step response tracking speed (14-bit, OutShift=4)\n");

   dynamic_smoother<16, 128, 4> ds;

   for (int i = 0; i < 500; ++i)
      ds(0);

   int     first_90pct = -1;
   int32_t final_out   = 0;
   for (int i = 0; i < 100; ++i)
   {
      int32_t out = ds(1023);
      if (first_90pct < 0 && out >= 14745)  // 90% of 16383
         first_90pct = i + 1;
      final_out = out;
   }

   printf("  90 %% reached at sample %d,  final output %d\n",
          first_90pct, (int)final_out);

   CHECK(first_90pct > 0 && first_90pct <= 20);
   CHECK(final_out >= 16000);
}

// ----------------------------------------------------------------------------
// Test 6: DC removal
//
// Uses dc_block<6> (TC = 64 samples) so the test converges quickly.
// The lowpass-subtraction form (s - lp/K) fully removes DC at steady state:
// _lp converges to K*s, so _lp/K = s and the output goes to 0 (±1 rounding).
// ----------------------------------------------------------------------------
static void test_dc_removal()
{
   printf("Test 6: DC removal (dc_block<6>)\n");

   dc_block<6> dc;

   int32_t out = 0;
   for (int i = 0; i < 500; ++i)
      out = dc(1023);

   printf("  output after 500 samples of constant 1023: %d  (expected 0 or 1)\n", (int)out);
   CHECK(out >= 0 && out <= 1);
}

// ----------------------------------------------------------------------------
// Test 7: fast signal passthrough
//
// A sudden step should pass through the DC block largely intact on the first
// sample (the DC estimate is still tracking the old value).
// ----------------------------------------------------------------------------
static void test_dc_passthrough()
{
   printf("Test 7: fast signal passthrough (dc_block<6>)\n");

   dc_block<6> dc;

   // Settle with input at 0
   for (int i = 0; i < 500; ++i)
      dc(0);

   // Step to 1023 — first output should be close to 1023
   int32_t first = dc(1023);
   printf("  first output after step 0→1023: %d  (expected ~1023)\n", (int)first);
   CHECK(first >= 900);
}

// ----------------------------------------------------------------------------
// Test 8: bipolar centering for pitch bend
//
// Uses dc_block<13> on a 14-bit signal (as in pitch_bend_controller).
// After settling, dc output ≈ 0, so dc + 8192 ≈ 8192 (MIDI centre).
// Input extremes after settling at mid-scale should stay in [0, 16383].
// ----------------------------------------------------------------------------
static void test_dc_pitch_bend_centering()
{
   printf("Test 8: bipolar centering for 14-bit pitch bend\n");

   // Settle at mid-scale (14-bit: 8192 = centre)
   dc_block<13> dc_mid;
   for (int i = 0; i < 100000; ++i)
      dc_mid(8192);
   int32_t center_out = dc_mid(8192) + 8192;
   printf("  dc(8192)+8192 after settling: %d  (expected ~8192)\n", (int)center_out);
   CHECK(center_out >= 8191 && center_out <= 8193);

   // Transient at extremes: adding 8192 should stay in [0, 16383]
   dc_block<13> dc_lo, dc_hi;
   for (int i = 0; i < 100000; ++i) { dc_lo(8192); dc_hi(8192); }

   int32_t lo_out = dc_lo(0)     + 8192;
   int32_t hi_out = dc_hi(16383) + 8192;
   printf("  dc(0)+8192:     %d\n", (int)lo_out);
   printf("  dc(16383)+8192: %d\n", (int)hi_out);
   CHECK(lo_out >= 0 && lo_out <= 16383);
   CHECK(hi_out >= 0 && hi_out <= 16383);
}

// ----------------------------------------------------------------------------
// Step-response table (for visual inspection)
// ----------------------------------------------------------------------------
static void print_step_response()
{
   printf("\nStep response table — 10-bit (input steps 0→1023 at sample 0):\n");
   printf("  %6s  %6s  %6s\n", "sample", "output", "pct");

   dynamic_smoother<16, 128> ds;
   for (int i = 0; i < 30; ++i)
   {
      int32_t out = ds(1023);
      printf("  %6d  %6d  %5.1f%%\n", i, (int)out, 100.0 * out / 1023.0);
   }

   printf("\nStep response table — 14-bit (input steps 0→1023 at sample 0):\n");
   printf("  %6s  %6s  %6s\n", "sample", "output", "pct");

   dynamic_smoother<16, 128, 4> ds14;
   for (int i = 0; i < 30; ++i)
   {
      int32_t out = ds14(1023);
      printf("  %6d  %6d  %5.1f%%\n", i, (int)out, 100.0 * out / 16383.0);
   }
}

// ----------------------------------------------------------------------------

int main()
{
   printf("=== dynamic_smoother + dc_block tests ===\n\n");

   test_convergence();          printf("\n");
   test_step_response();        printf("\n");
   test_noise_suppression();    printf("\n");
   test_convergence_14bit();    printf("\n");
   test_step_response_14bit();  printf("\n");
   test_dc_removal();           printf("\n");
   test_dc_passthrough();       printf("\n");
   test_dc_pitch_bend_centering();
   print_step_response();

   printf("\n%d/%d tests passed\n", tests_run - tests_failed, tests_run);
   return (tests_failed == 0) ? 0 : 1;
}
