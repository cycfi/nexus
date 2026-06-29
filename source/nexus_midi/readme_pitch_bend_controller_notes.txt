Pitch Bend Controller
=====================

The pitch bend controller reads the eWhammy pedal's analog sensor and converts
it into 14-bit MIDI pitch bend messages. The whole design exists to solve one
hard problem: keep the output dead-centered at rest (no off-center "latch", no
drift, no crosstalk spikes) WITHOUT flattening real playing -- bends, vibrato,
and slow expressive moves must pass through faithfully.

It is built as a pipeline of small, independently unit-tested processor blocks
(see test/unit_blocks.cpp). The blocks sorted by reusability: the generally
reusable ones (slew_gate, stillness, dc_servo) live in util.hpp; the
pitch-specific ones (inertial_corrector, output_stage, settle_gate,
freeze_watchdog) sit next to this controller in nexus_controls.hpp.


Signal Path (per loop)
----------------------

Each loop the controller runs this chain (pitch_bend_controller::operator()):

  raw_adc            background oversampler -> a 12-bit reading (10-bit ADC
                     + 2 oversampling bits, os_shift = 2; full scale 4095)
    |
  slew_gate          reject faster-than-physical jumps and HOLD the last good
                     reading (not clamp). The arm slews at most ~6 LSB/ms; a
                     programmer-contact glitch shifts the ADC reference ~10x
                     faster, so it is rejected at the source -- the cause of the
                     old off-center latch. Real motion passes untouched.
    |
  dynamic_smoother   adaptive low-pass (re-ported Q / Cytomic). Its cutoff opens
                     with the bandpass energy, so it tracks fast gestures with
                     low lag and smooths hard at rest (G0 = 24, Sense = 128).
                     Replaces the old lp1<8>+lp2<16> cascade (~30 ms release lag).
    |
  settle_gate        STARTUP only: hold the output at center until the input is
                     a genuinely settled rest (in [1600,2560], stable within
                     24 LSB for 300 ms, and at least 5 s past boot). Then seed
                     the C0 servo from it and go live. Suppresses the power-on
                     swing as Vcc ramps; a disconnected pedal stays silently
                     centered and hot-plugs cleanly.
    |
  deflection         (pos - 2048) * 4  ->  a 14-bit signed deflection
    |
  C0 servo (dc_servo)  the live DC center estimate. dev = deflection - C0 is
                     what becomes the output. See "Centering" below.
    |
  inertial_corrector THE sole centering path. Tracks the bend; lands only a
                     HANG (a release that stalls near zero) back to center.
    |
  + center (8192)    offset-binary MIDI form: 8192 = no bend
    |
  out_ma             3-point moving average -- softens a 1-LSB residual toggle
    |
  output_stage       send deadband (+/-16) with a center-snap exception, then
                     the symmetric x1.2 output gain (307/256) applied to ONLY
                     the value handed to MIDI. Emits the 14-bit message.
    |
  freeze_watchdog    last-resort backstop (runs after the send): flush an output
                     that is genuinely stuck off-center back to center.


Centering: the C0 servo + the inertial_corrector
-------------------------------------------------

Centering is split into two decoupled stages so that nothing in the bend path
can feed the center estimate (the old failure mode: a servo that chased the
bend, then stranded the release off-center).

C0 servo (dc_servo<16, 546, 736>) -- the slow baseline. A VERY slow leaky
integrator (tau ~65 s) that tracks ONLY the thermal/mechanical DC drift of the
rest position. It steps toward the raw deflection only when the bar is STILL
(stillness<32,400>: settled within ~2 LSB for 400 ms) AND near neutral
(|dev| <= c0_gate = 546, ~0.8 ST). A real bend (past the gate) FREEZES it, so a
sustained bend is never absorbed as drift. It is hard-clipped to its seed
+/- c0_clip (736, the max physical DC wander), so even a detection slip can
never drag the center onto a real bend.

inertial_corrector -- the return profile. It passes the deflection through
while the bar moves, and lands ONLY a hang: a release that STOPS near zero. The
mechanics:
  * STALL  -- a stop is detected by low-passed SLOPE (slope_k = 3): when the
    smoothed per-loop slope drops below stall_slope = 14, the bar has stopped.
  * ARM    -- landing is armed only after the bar bends past +/-4 ST (bend_th =
    2731): the depth where the hysteresis residual (k*peak) first turns audible.
    A shallower bend never arms -- its tiny residual rides the gate/servo.
  * GATE   -- a hysteresis noise gate (open 68 ~0.10 ST, close 34 ~0.05 ST,
    90 ms dwell) mutes the rest; a vibrato's troughs ride through it (no chop),
    only a real settle re-closes it.
  * LAND   -- a stalled, armed release within hang_band (546) eases to center with
    an EXPONENTIAL decay sized by the descent's PEAK velocity (tau = dist/vpk; both
    the slope vpk and tau are clamped, against a glitch and against a drag). A
    vigorous release lands snappy, a gentle one eases in -- the landing inherits how
    fast you actually moved the bar. The offset is clamped so the land never pulls
    PAST center. The DC is handed off to the slow C0 servo.

This makes the off-center latch structurally impossible while preserving bends
(held, never landed), vibrato (always moving, never stalls), and slow descents
(force-landed -- see the trade below).


Startup
-------

Boot is held MUTED at center until settle_gate confirms a valid settled rest
(see Signal Path). The C0 servo is then seeded from that rest, so the very first
live output is already centered -- no startup swing, no false offset locked in.
A pedal held off-center at boot simply never satisfies the gate and stays muted.


The freeze_watchdog (backstop)
------------------------------

A held bend and a true hang are IDENTICAL instant-to-instant -- both a steady
off-center output -- so only TIME separates them. The watchdog fires only after
the output holds within band = 20, off center by at least off_min = 64, for a
LONG freeze_ms = 10 s (past any real held bend), then flushes: re-seed C0 to the
current deflection, drop the corrector's state, and emit center.

This guards ONE real-world case: an attached programmer's analog loading shifts
the ADC reference (the rest reads ~1 ST off with the bar untouched), past the
C0 gate, so the servo freezes and can't re-center on its own. In normal use (no
programmer attached) the C0 clip bounds the center and the watchdog just sits
idle.


Landing trade: fast snap vs. slow-descent tracking
--------------------------------------------------

The slope-based landing above is the SHIPPED choice (now with the vigor-adaptive
exponential shape). Its one cost is that a deliberately SLOW manual descent
(slower than the sensor noise floor) reads as a stall and is force-landed early
-- the output eases to center while the bar is still gliding down.

A displacement-stall variant tracks those slow descents (lands only after the
bar truly stops) but at a softer ~500 ms landing; a raw-stall "have both"
version got both (snappy AND tracked) at the cost of more machinery. Both were
trialed on hardware and the fast snap won on feel. The slow-descent-tracking
variant is preserved on branch corrector-slow-descents if it is ever wanted.


Output
------

A 14-bit pitch bend message, clamped to 0..16383: 8192 = center (no bend), 0 =
full down, 16383 = full up. The symmetric x1.2 gain (output_stage) fills the
pull side to the rail and clips only the deepest ~17% of the dive, recovering
the full +/-8192 range from a deliberately-linear (un-railed) analog front end.
