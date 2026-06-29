// Host harness: drives the REAL firmware controllers with the NEXUS_SIM
// pitch timeline on a virtual clock, decodes the MIDI they emit, reports
// per phase, and asserts the expected pitch-bend behavior. Built host-side
// (not cross).
//
//   cmake -S test -B test/build && cmake --build test/build
//   ./test/build/host_sim       # report + assertions (exit != 0 on fail)
//   ./test/build/host_sim -v    # also dump every decoded message
//
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <energia.h>
#include <MspFlash.h>

uint32_t _sim_millis = 0;
int (*_sim_digital)(uint8_t) = 0;
std::vector<uint8_t> _sim_midi;
_SerialT Serial;
_FlashT Flash;
_MockFlashMem _flashmem;

#include "nexus_midi.ino"

// -----------------------------------------------------------------------------
// Decoded events
// -----------------------------------------------------------------------------
// kind: R reset, B pb(a=val), C cc(a=num,b=val), P pc(a=val)
struct ev { uint32_t t; char kind; int a, b; };
static std::vector<ev> events;

// Host-only real-data feed (set by --feed; consumed by sample() in
// nexus_sim.hpp)
namespace nexus_sim {
   const int16_t* g_feed = nullptr; uint32_t g_feed_len = 0;
}
static std::vector<int16_t> g_feedbuf;

static void drain(uint32_t t, bool verbose)
{
   std::vector<uint8_t>& m = _sim_midi;
   size_t i = 0;
   while (i < m.size())
   {
      uint8_t s = m[i], hi = s & 0xF0;
      if (s == 0xFF) { events.push_back({t,'R',0,0});
         if (verbose) printf("[t=%5u] RESET\n", (unsigned)t); i += 1; }
      else if (hi == 0xB0 && i + 3 <= m.size()) {
         events.push_back({t,'C',m[i+1],m[i+2]});
         if (verbose)
            printf("[t=%5u] CC%-3u = %u\n",(unsigned)t,m[i+1],m[i+2]);
         i += 3; }
      else if (hi == 0xC0 && i + 2 <= m.size()) {
         events.push_back({t,'P',m[i+1],0});
         if (verbose)
            printf("[t=%5u] program_change = %u\n",(unsigned)t,m[i+1]);
         i += 2; }
      else if (hi == 0xE0 && i + 3 <= m.size()) {
         int v = m[i+1] | (int(m[i+2]) << 7);
         events.push_back({t,'B',v,0});
         if (verbose) printf("[t=%5u] pitch_bend = %5d\n",(unsigned)t,v);
         i += 3; }
      else i += 1;
   }
   m.clear();
}

// -----------------------------------------------------------------------------
// Pitch-bend queries over a [t0,t1) window
// -----------------------------------------------------------------------------
struct pbwin { int count, first, last, lo, hi; };
static pbwin pb_in(uint32_t t0, uint32_t t1)
{
   pbwin w = {0, -1, -1, 1<<30, -(1<<30)};
   for (const ev& e : events)
      if (e.kind=='B' && e.t>=t0 && e.t<t1) {
         if (w.count==0) w.first=e.a;
         w.last=e.a;
         if (e.a<w.lo) w.lo=e.a; if (e.a>w.hi) w.hi=e.a; ++w.count;
      }
   if (w.count==0) { w.lo=w.hi=0; }
   return w;
}

// last pitch-bend value as of time t (8192 before any message)
static int pb_at(uint32_t t)
{
   int v = 8192;
   for (const ev& e : events) if (e.kind=='B' && e.t<=t) v = e.a;
   return v;
}

// -----------------------------------------------------------------------------
static int fails = 0;
static void check(const char* name, bool ok, const char* detail)
{
   printf("  [%s] %-34s %s\n", ok ? "PASS" : "FAIL", name, detail);
   if (!ok) ++fails;
}

int main(int argc, char** argv)
{
   bool verbose = false, dump = false;
   const char* feedfile = nullptr;
   // startup-gate hold-off; small here so the suite/replays run
   uint32_t holdoff = 300;
   // sim loop period (ms); hardware runs ~8 ms, not 1
   uint32_t loopms = 1;
   // lock c0 at the settled rest (isolate the feed-forward)
   bool freeze_servo = false;
   for (int i = 1; i < argc; ++i) {
      if      (!strcmp(argv[i], "-v"))     verbose = true;
      else if (!strcmp(argv[i], "--dump")) dump = true;
      else if (!strcmp(argv[i], "--feed") && i+1 < argc)
         feedfile = argv[++i];
      else if (!strcmp(argv[i], "--holdoff") && i+1 < argc)
         holdoff = atoi(argv[++i]);
      else if (!strcmp(argv[i], "--loopms") && i+1 < argc)
         loopms = atoi(argv[++i]);
      else if (!strcmp(argv[i], "--freeze-servo")) freeze_servo = true;
   }
   if (loopms < 1) loopms = 1;
   using namespace nexus_sim;
   uint32_t ticks = T_LATCH_END + 1000;

   if (feedfile)                              // replay logged ADC, 1/ms
   {
      FILE* f = fopen(feedfile, "r");
      if (!f) { fprintf(stderr, "cannot open %s\n", feedfile); return 2; }
      char ln[256]; std::vector<double> tv; std::vector<int> av;
      fgets(ln, sizeof ln, f);                      // header
      double ts; int p14, ps;
      while (fgets(ln, sizeof ln, f))
         if (sscanf(ln, "%lf,%d,%d", &ts, &p14, &ps) == 3) {
            int a = ps/16 + 512; if (a < 0) a = 0; if (a > 1023) a = 1023;
            tv.push_back(ts); av.push_back(a);
         }
      fclose(f);
      double t0 = tv.front();
      uint32_t len = (uint32_t)((tv.back() - t0) * 1000.0) + 1;
      g_feedbuf.assign(len, (int16_t)av[0]);
      size_t pi = 0;
      for (uint32_t ms = 0; ms < len; ++ms) {
         double t = t0 + ms / 1000.0;
         while (pi + 1 < tv.size() && tv[pi+1] <= t) ++pi;
         // Linearly interpolate between logged samples to 1 ms. The diag
         // is decimated (~10-100 ms), so step-holding would replay a
         // staircase whose jumps trip the firmware's slew gate on fast
         // ramps -- reconstruct the real smooth arm motion.
         if (pi + 1 < tv.size() && tv[pi+1] > tv[pi]) {
            double f = (t - tv[pi]) / (tv[pi+1] - tv[pi]);
            g_feedbuf[ms] =
               (int16_t)(av[pi] + (av[pi+1] - av[pi]) * f + 0.5);
         } else {
            g_feedbuf[ms] = (int16_t)av[pi];
         }
      }
      g_feed = g_feedbuf.data(); g_feed_len = len; ticks = len;
      fprintf(stderr, "fed %zu logged samples -> %u ms\n", tv.size(), len);
   }

   if (verbose) printf("--- startup ---\n");
   setup();
   // override the 2500 ms production default for tests
   pitch_bend.startup_holdoff = holdoff;
   bool c0_locked = false; int32_t c0_lock = 0;
   drain(0, verbose);
   if (verbose) printf("--- run ---\n");
   if (dump) printf("t,adc,out,c0\n");          // per-tick trace for plot
   for (uint32_t k = loopms; k <= ticks; k += loopms)
   {
      _sim_millis = k; loop(); drain(k, verbose);
      if (freeze_servo && !pitch_bend.muted) {
         if (!c0_locked) {
            c0_lock = pitch_bend.servo.acc; c0_locked = true;
         }
         else pitch_bend.servo.acc = c0_lock;
      }
      if (dump)
         printf("%u,%d,%d,%d\n", (unsigned)k, (int)sample(PITCH_PIN,k),
                (int)pitch_bend.out_stage.value(),
                (int)pitch_bend.servo.value());
   }
   if (dump) return 0;

   // ---- per-phase pitch report ----
   struct ph { uint32_t t0,t1; const char* name; };
   ph phases[] = {
      {0,           T_REST1,      "rest1"},
      {T_REST1,     T_DIVE,       "dive_down"},
      {T_DIVE,      T_DIVE_HOLD,  "dive_hold"},
      {T_DIVE_HOLD, T_DIVE_END,   "dive_up"},
      {T_DIVE_END,  T_REST2,      "rest2"},
      {T_REST2,     T_PULL,       "pull_up"},
      {T_PULL,      T_PULL_HOLD,  "pull_hold"},
      {T_PULL_HOLD, T_PULL_END,   "pull_down"},
      {T_PULL_END,  T_REST3,      "rest3"},
      {T_REST3,     T_WOBBLE_END, "wobble"},
      {T_WOBBLE_END,T_REST4,      "rest4"},
      {T_REST4,     T_UP1_H,      "up1_hold"},
      {T_UP1_H,     T_DN1_H,      "dn1_hold"},
      {T_DN1_H,     T_HOLD_END,   "release"},
      {T_LATCH_R,   T_LATCH_H,    "latch_17s"},
      {T_LATCH_H,   T_LATCH_END,  "latch_rel"},
   };
   printf("\n=== pitch-bend per phase (center = 8192) ===\n");
   printf("  %-10s %-13s %4s  %6s %6s %6s %6s\n",
          "phase","window(ms)","#PB","first","last","min","max");
   for (ph& p : phases) {
      pbwin w = pb_in(p.t0, p.t1);
      char win[24];
      snprintf(win,sizeof win,"%u..%u",(unsigned)p.t0,(unsigned)p.t1);
      printf("  %-10s %-13s %4d  %6d %6d %6d %6d\n",
             p.name, win, w.count, w.first, w.last, w.lo, w.hi);
   }
   int cc=0,pc=0;
   for (const ev& e:events){ if(e.kind=='C')++cc; if(e.kind=='P')++pc; }
   printf("  (CC msgs=%d, program_change msgs=%d)\n", cc, pc);

   // ---- assertions: DC-servo nulls hysteresis, preserves real bends ----
   printf("\n=== assertions ===\n");
   char d[96];
   const int DET = 32;                 // center tolerance (+/-2 LSB)

   // 1. startup publishes center
   pbwin s = pb_in(0, 1);
   snprintf(d,sizeof d,"first=%d count=%d", s.first, s.count);
   check("startup center 8192", s.count>=1 && s.first==8192, d);

   // 2. THE check: the moving window follows each bend OUT (to ~+/-2.4 ST)
   // and on release descends through the depth-scaled hysteresis rest, so
   // the servo nulls BOTH rests statically -- even the +576 after-pull the
   // too-tight window missed.
   int r2=pb_at(T_REST2-1), r3=pb_at(T_REST3-1), r4=pb_at(T_REST4-1);
   snprintf(d,sizeof d,"offset=%+d", r2-8192);
   check("servo nulls after-dive rest", abs(r2-8192)<=DET, d);
   snprintf(d,sizeof d,"offset=%+d (was +640, tight window)", r3-8192);
   check("servo nulls after-pull rest (static)", abs(r3-8192)<=DET, d);
   snprintf(d,sizeof d,"offset=%+d", r4-8192);
   check("final rest centered", abs(r4-8192)<=DET, d);

   // 3. real bends are preserved (reach near full). The feed-forward scales
   //    the bend by (1-k) ~ 0.957 at the peak -- the deflection that BECOMES
   //    hysteresis -- so full travel maps to ~96% of full-scale, not the
   //    rail.
   pbwin dv = pb_in(T_REST1, T_DIVE_END);
   snprintf(d,sizeof d,"min=%d (1-k scaling -> ~352)", dv.lo);
   check("dive bends near full-down", dv.lo<=600, d);
   pbwin pl = pb_in(T_REST2, T_PULL_END);
   snprintf(d,sizeof d,"max=%d (1-k scaling -> ~16016)", pl.hi);
   check("pull reaches near full-up", pl.hi>=15800, d);

   // 4. vibrato preserved: full amplitude kept (servo doesn't flatten it)
   pbwin wb = pb_in(T_REST3+150, T_WOBBLE_END);
   snprintf(d,sizeof d,"p-p=%d (range %+d..%+d)",
            wb.hi-wb.lo, wb.lo-8192, wb.hi-8192);
   check("vibrato preserved (amplitude)", (wb.hi-wb.lo) >= 700, d);

   // 5. +1 ST / -1 ST holds preserved (the SPICE +/-1 semitone hold test;
   //    the analog holds ~84% with a slow leak -- 1 ST = 683 units, 80% =
   //    546)
   int up1 = pb_at(T_UP1_H - 1) - 8192;
   int dn1 = pb_at(T_DN1_H - 1) - 8192;
   snprintf(d,sizeof d,"hold=%+d (ideal +683; SPICE ~+574 = 84%%)", up1);
   check("+1 ST bend preserved (>=80%)", up1 >= 546, d);
   snprintf(d,sizeof d,"hold=%+d (ideal -683)", dn1);
   check("-1 ST bend preserved (>=80%)", dn1 <= -546, d);

   // 6. freeze_watchdog: a STUCK off-center output (the LATCH phase is
   //    bit-exact frozen, no dither -- a programmer-disconnect-style hang
   //    the corrector can't see) is flushed to center within ~500 ms and
   //    stays there. (A real held bend jitters -> not flushed; see #5.)
   int latch_held = pb_at(T_LATCH_R + 14000) - 8192;  // +14 s into frozen
   int latch_late = pb_at(T_LATCH_H - 100) - 8192;    // ~+17 s, still frozen
   snprintf(d,sizeof d,"+14s=%+d, +17s=%+d (frozen hang flushed to center)",
            latch_held, latch_late);
   check("freeze_watchdog flushes stuck output",
         abs(latch_held) <= DET && abs(latch_late) <= DET, d);

   // 7. input slew gate: an injected programmer-glitch reference shift
   //    (faster than the arm can move) must be rejected, so the output
   //    stays centered instead of latching off-center.
   int glitch_out = pb_at(T_GLITCH + 250) - 8192;  // mid-glitch (0.5 s hold)
   snprintf(d,sizeof d,
            "mid-glitch out=%+d (held center; un-gated this latches off"
            "-center)", glitch_out);
   check("slew gate rejects programmer glitch", abs(glitch_out) <= 100, d);

   printf("\n%s (%d failure%s)\n",
          fails?"FAILED":"PASSED", fails, fails==1?"":"s");
   return fails ? 1 : 0;
}
