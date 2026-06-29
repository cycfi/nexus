// Host mock of the Energia API used by the nexus_midi firmware. Enough to
// compile and run the real controllers off-device against a virtual clock.
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <vector>

// Pin numbers — must match the real MSP-EXP430G2553LP variant so synthetic
// patterns key on the same pin values the device sees.
static const uint8_t P1_0 = 2;    // ch10  channel_volume
static const uint8_t P1_3 = 5;    // ch11  effect_1
static const uint8_t P1_4 = 6;    // ch12  effect_2
static const uint8_t P1_5 = 7;    // ch13  pitch bend
static const uint8_t P2_0 = 8;    // ch9   sustain
static const uint8_t P2_1 = 9;    // aux1
static const uint8_t P2_2 = 10;   // aux2
static const uint8_t P2_3 = 11;   // aux3
static const uint8_t P2_4 = 12;   // aux4
static const uint8_t P2_5 = 13;   // aux5
static const uint8_t P1_6 = 14;   // ch14  program_change
static const uint8_t P1_7 = 15;   // ch15  modulation
static const uint8_t P2_6 = 19;   // aux6

enum { LOW = 0, HIGH = 1, INPUT = 0, OUTPUT = 1, INPUT_PULLUP = 2 };

// Virtual millisecond clock — the harness drives _sim_millis.
extern uint32_t _sim_millis;
inline uint32_t millis() { return _sim_millis; }
// no-op; the clock is advanced by hand
inline void delay(uint32_t) {}

// Digital input. Default HIGH (switches open / pull-ups); a test can install
// _sim_digital to synthesize sustain / aux-button activity.
inline void pinMode(uint8_t, uint8_t) {}
extern int (*_sim_digital)(uint8_t pin);
inline int digitalRead(uint8_t pin)
{
   return _sim_digital ? _sim_digital(pin) : HIGH;
}

// Analog input — unused under NEXUS_SIM (raw_adc() routes to the synthetic
// generator); provided so the seam's #else branch and any direct call
// compile.
inline uint16_t analogRead(uint8_t) { return 512; }

inline long map(long x, long in_min, long in_max, long out_min, long out_max)
{
   return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Captured MIDI byte stream (mock UART). The harness drains _sim_midi.
extern std::vector<uint8_t> _sim_midi;
struct _SerialT
{
   void begin(long) {}
   void write(uint8_t b) { _sim_midi.push_back(b); }
};
extern _SerialT Serial;

// Energia's min/max are macros and the firmware relies on that (mixed int
// types). Define them LAST, after the std headers above, so nothing else
// is disturbed.
#ifndef min
#define min(a, b) ((a) < (b) ? (a) : (b))
#endif
#ifndef max
#define max(a, b) ((a) > (b) ? (a) : (b))
#endif
