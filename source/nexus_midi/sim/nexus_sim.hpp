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
// run (host) replays the exact same stimulus. The firmware's raw_adc() seam
// calls sample(pin, ms) in place of analogRead(pin).
//
// Ideal whammy gesture: a sprung bar that rests at PERFECT center (ADC 512 ->
// pitch-bend 8192) and bends both ways. No drift, no noise. The dive and pull
// dwell briefly at the rail so the input smoother can settle there.
//
//   phase       window (ms)   pitch ADC      what it checks
//   ---------   -----------   -----------    --------------------------------
//   rest1       0    .. 600   512            "0" point is exactly 8192
//   dive_down   600  .. 850   512 -> 0       dive to full down
//   dive_hold   850  ..1050   0              dwell so the filter reaches 0
//   dive_up     1050 ..1300   0   -> 512     spring back to center
//   rest2       1300 ..1800   512            returns to exact 8192
//   pull_up     1800 ..2050   512 -> 1023    bend up to full
//   pull_hold   2050 ..2250   1023           dwell so the filter reaches 16368
//   pull_down   2250 ..2500   1023 -> 512    spring back
//   rest3       2500 ..3000   512            returns to exact 8192
//   wobble      3000 ..4000   512 +/-150     vibrato around center
//   rest4       4000 ..4500   512            settles to exact 8192
///////////////////////////////////////////////////////////////////////////////
namespace nexus_sim
{
   constexpr uint16_t PITCH_PIN    = 7;     // P1_5 / ch13
   constexpr int      PITCH_CENTER = 512;   // 10-bit ADC -> 8192 in 14-bit

   // Spring-return hysteresis as the feed-forward model expects it: the rest is
   // PROPORTIONAL to the peak deflection, rest = k*peak (k~0.043, fitted). A full
   // dive (peak -8192) -> -352 = adc 490; a full pull (peak +8176) -> +351 = adc
   // 534. The predictor nulls both (k*M); a 1 ST bend earns only ~0.04 ST. (The
   // real bar adds a per-unit DC bias C0~+250, handled by the slow C0 servo and
   // exercised by the --feed real-session replay; the synthetic timeline uses
   // C0=0 so the startup seed is already the center.)
   constexpr int REST_DIVE_ADC = 490;       // after-dive rest  k*peak ~ -352
   constexpr int REST_PULL_ADC = 534;       // after-pull rest  k*peak ~ +351

   constexpr uint32_t T_REST1      = 800;
   constexpr uint32_t T_DIVE       = 1050;
   constexpr uint32_t T_DIVE_HOLD  = 1250;
   constexpr uint32_t T_DIVE_END   = 1500;
   constexpr uint32_t T_REST2      = 3000;   // long rest: servo nulls the -48
   constexpr uint32_t T_PULL       = 3250;
   constexpr uint32_t T_PULL_HOLD  = 3450;
   constexpr uint32_t T_PULL_END   = 3700;
   constexpr uint32_t T_REST3      = 5200;   // long rest: servo must null the +576
   constexpr uint32_t T_WOBBLE_END = 6200;
   constexpr uint32_t T_REST4      = 7000;

   // +1 ST / -1 ST HOLD preservation test (mirrors the SPICE +/-1 semitone holds,
   // which the analog holds at ~84% with a slow leak). From the centered after-pull
   // rest (ADC 534, output 0), a +/-1 ST OUTPUT bend is +/-683/16 ~ 43 ADC codes ->
   // 577 / 491. The predictor must HOLD these (k*M earns only ~0.04 ST correction).
   constexpr int      UP1_ADC      = 555;   // +1 ST output bend from center (512 + 683/16)
   constexpr int      DN1_ADC      = 469;   // -1 ST output bend from center (512 - 683/16)
   constexpr uint32_t T_UP1_R      = 7200;  // ramp rest -> +1 ST
   constexpr uint32_t T_UP1_H      = 9200;  // HOLD +1 ST, 2 s
   constexpr uint32_t T_DN1_R      = 9400;  // ramp +1 ST -> -1 ST
   constexpr uint32_t T_DN1_H      = 11400; // HOLD -1 ST, 2 s
   constexpr uint32_t T_HOLD_END   = 11700; // ramp back to rest

   // Latch test: the output gets STUCK off-center (a mechanical jam / power-on-bent
   // -- here ~+2 ST) and is held past the 15 s timeout. The watchdog must re-acquire.
   constexpr int      LATCH_ADC    = 633;   // ~+2 ST off-center "stuck" position
   constexpr uint32_t T_LATCH_R    = 12200; // ramp to the stuck position
   constexpr uint32_t T_LATCH_H    = 29200; // hold 17 s (> 15 s timeout)
   constexpr uint32_t T_LATCH_END  = 30200; // release

   // Programmer-glitch test: a contact reference shift -- an instant, faster-than-physical jump
   // (bypasses pitch_adc_phys) that the firmware's input slew gate must reject, holding the rest
   // so the output never goes off-center and the servo never freezes. Without the gate this is
   // exactly the captured stuck (a ~40 LSB drop -> servo latch -> 15 s recovery).
   constexpr uint32_t T_GLITCH     = 6400;  // mid rest4 (a settled center), then the ref shifts
   constexpr uint32_t T_GLITCH_END = 6900;  // shift sustained 0.5 s, then removed (rest4 ends 7000)
   constexpr int      GLITCH_LSB   = 44;    // ~the captured -42 LSB reference shift

   // linear interpolate a -> b as num/den goes 0 -> 1 (32-bit: the products
   // overflow 16-bit int on the MSP430).
   inline int lerp(int a, int b, uint32_t num, uint32_t den)
   {
      return a + int((long(b - a) * long(num)) / long(den));
   }

   inline int pitch_adc(uint32_t ms)
   {
      if (ms < T_REST1)      return 512;
      if (ms < T_DIVE)       return lerp(512, 0,    ms - T_REST1,     T_DIVE - T_REST1);
      if (ms < T_DIVE_HOLD)  return 0;
      if (ms < T_DIVE_END)   return lerp(0, REST_DIVE_ADC, ms - T_DIVE_HOLD, T_DIVE_END - T_DIVE_HOLD);
      if (ms < T_REST2)      return REST_DIVE_ADC;       // after-dive hysteresis rest
      if (ms < T_PULL)       return lerp(REST_DIVE_ADC, 1023, ms - T_REST2, T_PULL - T_REST2);
      if (ms < T_PULL_HOLD)  return 1023;
      if (ms < T_PULL_END)   return lerp(1023, REST_PULL_ADC, ms - T_PULL_HOLD, T_PULL_END - T_PULL_HOLD);
      if (ms < T_REST3)      return REST_PULL_ADC;       // after-pull hysteresis rest (k*peak)
      if (ms < T_WOBBLE_END)                             // ~4 Hz triangle +/-30 ADC (~vibrato)
      {                                                  // around center (its own small hysteresis)
         uint32_t ph = (ms - T_REST3) % 250;
         int v = (ph < 125) ? lerp(-30, 30, ph, 125)
                            : lerp(30, -30, ph - 125, 125);
         return 512 + v;
      }
      if (ms < T_REST4)      return 512;                 // rest4: small-excursion rest -> center
      if (ms < T_UP1_R)      return lerp(512, UP1_ADC, ms - T_REST4, T_UP1_R - T_REST4);
      if (ms < T_UP1_H)      return UP1_ADC;             // HOLD +1 ST (from center)
      if (ms < T_DN1_R)      return lerp(UP1_ADC, DN1_ADC, ms - T_UP1_H, T_DN1_R - T_UP1_H);
      if (ms < T_DN1_H)      return DN1_ADC;             // HOLD -1 ST (from center)
      if (ms < T_HOLD_END)   return lerp(DN1_ADC, 512, ms - T_DN1_H, T_HOLD_END - T_DN1_H);
      if (ms < T_LATCH_R)    return lerp(512, LATCH_ADC, ms - T_HOLD_END, T_LATCH_R - T_HOLD_END);
      if (ms < T_LATCH_H)    return LATCH_ADC;           // STUCK off-center, 17 s (latch test)
      if (ms < T_LATCH_END)  return lerp(LATCH_ADC, 512, ms - T_LATCH_H, T_LATCH_END - T_LATCH_H);
      return 512;                                        // final rest (center)
   }

   // The timeline above takes some phase boundaries as instant steps (e.g. rest->wobble),
   // which are non-physical: a real arm slews at most ~4 LSB/ms (measured). Rate-limit the
   // synthetic input to that, so it matches reality -- and so the firmware's input slew gate,
   // which exists to reject faster-than-physical jumps as programmer glitches, sees only
   // legitimate motion. (The replay path below feeds real recorded data and is left alone.)
   inline int pitch_adc_phys(uint32_t ms)
   {
      static int      held = 512;
      static uint32_t last = 0;
      int dt = int(ms - last);
      last = ms;
      int maxstep = 4 * (dt > 0 ? dt : 1);          // ~4 LSB/ms physical arm slew
      int d = pitch_adc(ms) - held;
      held += (d > maxstep) ? maxstep : (d < -maxstep ? -maxstep : d);
      return held;
   }

#ifdef NEXUS_HOST
   // Host-only: replay a real logged ADC stream (1 sample/ms) instead of the
   // synthetic timeline, so the servo can be driven by the actual recorded gestures.
   extern const int16_t* g_feed;
   extern uint32_t       g_feed_len;
#endif

   inline uint16_t sample(uint16_t pin, uint32_t ms)
   {
#ifdef NEXUS_HOST
      if (g_feed && pin == PITCH_PIN)
         return uint16_t(g_feed[ms < g_feed_len ? ms : g_feed_len - 1]);
#endif
      if (pin == PITCH_PIN)
      {
         int v = pitch_adc_phys(ms);
         if (ms >= T_GLITCH && ms < T_GLITCH_END)  // injected programmer-contact reference shift:
            v -= GLITCH_LSB;                        //   an instant jump the slew gate must reject
         return uint16_t(v);
      }
      return 512;                       // CC channels: steady mid for now
   }
}

#endif
