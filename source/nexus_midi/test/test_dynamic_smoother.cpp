// Tests for cycfi::dynamic_smoother (util.hpp)
//
// Three behavioural tests:
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
// Test 1: convergence
//
// A constant input should drive both poles to the same value so that the
// output equals the input exactly (within ±1 due to integer truncation).
// ----------------------------------------------------------------------------
static void test_convergence()
{
   printf("Test: convergence to steady state\n");

   dynamic_smoother<16, 128> ds;

   int32_t out = 0;
   for (int i = 0; i < 2000; ++i)
      out = ds(512);

   printf("  output after 2000 samples at input=512: %d\n", (int)out);
   CHECK(out >= 511 && out <= 513);

   // Endpoint values converge exactly (no rounding at the boundaries)
   dynamic_smoother<16, 128> ds0, ds1;
   for (int i = 0; i < 2000; ++i) { ds0(0); ds1(1023); }
   CHECK(ds0(0) == 0);
   CHECK(ds1(1023) >= 1022 && ds1(1023) <= 1023);
}

// ----------------------------------------------------------------------------
// Test 2: step response
//
// The adaptive gain should open up on a sudden step so the output reaches
// 90 % of the target within a small number of samples.  A fixed-gain filter
// with the same base cutoff (G0=16) would need ~100 samples to hit 90 %.
// ----------------------------------------------------------------------------
static void test_step_response()
{
   printf("Test: step response tracking speed\n");

   dynamic_smoother<16, 128> ds;

   // Settle at 0
   for (int i = 0; i < 500; ++i)
      ds(0);

   // Step to 1023 and track how quickly it rises
   int    first_90pct = -1;
   int32_t final_out  = 0;
   for (int i = 0; i < 100; ++i)
   {
      int32_t out = ds(1023);
      if (first_90pct < 0 && out >= 920)   // 920 / 1023 ≈ 90 %
         first_90pct = i + 1;
      final_out = out;
   }

   printf("  90 %% reached at sample %d,  final output %d\n",
          first_90pct, (int)final_out);

   CHECK(first_90pct > 0 && first_90pct <= 20);
   CHECK(final_out >= 1000);

   // Compare: a fixed lowpass<16> (equivalent to G0 with Sense=0) is slower
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
   CHECK(first_90pct < fixed_90pct);   // adaptive must be faster than fixed
}

// ----------------------------------------------------------------------------
// Test 3: noise suppression
//
// After settling, alternating ±1 jitter around a stable value should be
// heavily attenuated — the output should not track each individual jitter.
// ----------------------------------------------------------------------------
static void test_noise_suppression()
{
   printf("Test: noise suppression on stable signal\n");

   dynamic_smoother<16, 128> ds;

   // Settle at 512
   for (int i = 0; i < 500; ++i)
      ds(512);

   // Feed ±1 alternating noise
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
// Step-response table (for visual inspection)
// ----------------------------------------------------------------------------
static void print_step_response()
{
   printf("\nStep response table (input steps 0→1023 at sample 0):\n");
   printf("  %6s  %6s  %6s\n", "sample", "output", "pct");

   dynamic_smoother<16, 128> ds;
   for (int i = 0; i < 30; ++i)
   {
      int32_t out = ds(1023);
      printf("  %6d  %6d  %5.1f%%\n", i, (int)out, 100.0 * out / 1023.0);
   }
}

// ----------------------------------------------------------------------------

int main()
{
   printf("=== dynamic_smoother tests ===\n\n");

   test_convergence();
   printf("\n");
   test_step_response();
   printf("\n");
   test_noise_suppression();
   print_step_response();

   printf("\n%d/%d tests passed\n", tests_run - tests_failed, tests_run);
   return (tests_failed == 0) ? 0 : 1;
}
