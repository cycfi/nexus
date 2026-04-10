// Tests for cycfi::gate (util.hpp)
//
// gate<T> is a stateless dead-zone gate. Returns true when s lies outside
// the dead-zone around zero (i.e. the signal is not near zero).
//
//   Bipolar  (signed T):   passes if s > threshold  OR  s < -threshold
//   Unipolar (unsigned T): passes if s > threshold  (negative s can't exist)
//
// Signedness is detected at compile time via T(-1) < T(0).
//
// The caller controls what is passed:
//   - Pitch-bend noise gate: pass (out - center), threshold is the dead zone
//     around center; gate opens only when the pitch is significantly bent.
//   - CC change threshold: pass |val - prev|; gate opens when the change
//     exceeds the noise floor.
//
// Tests:
//
//  1. Unipolar (gate<uint32_t>): only passes above threshold; zero and
//     values at or below the threshold are blocked.
//
//  2. Bipolar (gate<int32_t>): passes above +threshold and below -threshold;
//     zero, ±1, and values within the dead-zone are blocked.
//
//  3. Boundary values: threshold itself is blocked; threshold+1 passes.
//
//  4. Default threshold=1: only the exact value 0 is blocked for signed,
//     0 and 1 are blocked for unsigned.
//
//  5. static_assert checks: confirm the compile-time signedness detection
//     produces the correct bool constant for signed and unsigned types.

#include "util.hpp"
#include <cstdio>
#include <cstdlib>
#include <cstdint>

using namespace cycfi;

// ----------------------------------------------------------------------------
// Minimal test harness (shared style with test_dynamic_smoother.cpp)
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
// Compile-time signedness detection
// ----------------------------------------------------------------------------
static_assert(!(uint32_t(-1) < uint32_t(0)), "uint32_t should be detected as unsigned");
static_assert(  int32_t(-1)  <  int32_t(0),  "int32_t should be detected as signed");

// ----------------------------------------------------------------------------
// Test 1: unipolar gate (unsigned T)
// ----------------------------------------------------------------------------
static void test_unipolar()
{
   printf("Test 1: unipolar gate<uint32_t> threshold=4\n");

   gate<uint32_t> g(4);

   // Dead-zone: 0 through threshold are blocked
   CHECK(!g(0));
   CHECK(!g(1));
   CHECK(!g(4));      // threshold itself blocked

   // Above threshold: passes
   CHECK( g(5));
   CHECK( g(100));
}

// ----------------------------------------------------------------------------
// Test 2: bipolar gate (signed T)
// ----------------------------------------------------------------------------
static void test_bipolar()
{
   printf("Test 2: bipolar gate<int32_t> threshold=4\n");

   gate<int32_t> g(4);

   // Dead-zone: -threshold through +threshold blocked
   CHECK(!g( 0));
   CHECK(!g( 1));
   CHECK(!g( 4));     // +threshold blocked
   CHECK(!g(-1));
   CHECK(!g(-4));     // -threshold blocked

   // Outside dead-zone: passes
   CHECK( g( 5));
   CHECK( g(-5));
   CHECK( g( 100));
   CHECK( g(-100));
}

// ----------------------------------------------------------------------------
// Test 3: exact boundary values
// ----------------------------------------------------------------------------
static void test_boundaries()
{
   printf("Test 3: boundary values\n");

   gate<int32_t> gs(10);
   CHECK(!gs( 10));   // at threshold: blocked
   CHECK(!gs(-10));   // at -threshold: blocked
   CHECK( gs( 11));   // one above: passes
   CHECK( gs(-11));   // one below: passes

   gate<uint32_t> gu(10);
   CHECK(!gu(10));    // at threshold: blocked
   CHECK( gu(11));    // one above: passes
}

// ----------------------------------------------------------------------------
// Test 4: default threshold=1
// ----------------------------------------------------------------------------
static void test_default_threshold()
{
   printf("Test 4: default threshold=1\n");

   gate<int32_t> gs;   // threshold=1
   CHECK(!gs( 0));
   CHECK(!gs( 1));
   CHECK(!gs(-1));
   CHECK( gs( 2));
   CHECK( gs(-2));

   gate<uint32_t> gu;  // threshold=1
   CHECK(!gu(0));
   CHECK(!gu(1));
   CHECK( gu(2));
}

// ----------------------------------------------------------------------------

int main()
{
   printf("=== gate tests ===\n\n");

   test_unipolar();    printf("\n");
   test_bipolar();     printf("\n");
   test_boundaries();  printf("\n");
   test_default_threshold();

   printf("\n%d/%d tests passed\n", tests_run - tests_failed, tests_run);
   return (tests_failed == 0) ? 0 : 1;
}
