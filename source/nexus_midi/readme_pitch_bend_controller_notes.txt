Pitch Bend Controller
=====================

The pitch bend controller reads a Hall effect sensor on the eWhammy pedal and
converts it into 14-bit MIDI pitch bend messages. It is designed around three
concerns: resolution, drift compensation, and crosstalk rejection.


Signal Path
-----------

Every millisecond, the main loop reads the analog sensor through analog_read(),
which clips the raw ADC value to the usable 2-98% travel range and maps it to
0-1023. This normalized value feeds the dynamic smoother.

The dynamic smoother (dynamic_smoother<16, 128, 4>) is an adaptive lowpass
filter. When the signal is stable it attenuates noise heavily; when the signal
moves quickly it opens up to track the gesture. Internally it stores state in
Q8 fixed-point (x256), and with OutShift=4 it outputs in a 14-bit range
(0-16383) rather than 10-bit -- the sub-integer precision in the smoother's
state is preserved and exposed as extra resolution, giving the full 14-bit
pitch bend range from a 10-bit ADC.


Drift Compensation
------------------

The smoothed value then passes through the offset servo (offset_servo<13>).
Hall effect sensors drift slowly with temperature and age, so the sensor's
rest position will not stay exactly at the nominal midpoint over time. The
servo tracks this drift with a leaky integrator (time constant ~8 seconds at
1 kHz) and subtracts the estimated offset from the signal, keeping the output
centered at zero. After the subtraction, 8192 is added to produce the MIDI
pitch bend offset-binary format where center is 8192.

The critical constraint is that the servo only updates its estimate when the
pedal is within the hardware deadband -- +/-2.5% around center (409 counts in
14-bit terms). Outside that window, the integrator is frozen. This means a
sustained bend is never slowly absorbed as drift: the servo only learns when
the pedal is genuinely at rest.


Startup
-------

During setup(), the smoother is warmed up with 100 passes of the current
sensor reading so it starts converged rather than ramping from zero. If that
reading falls within the center deadband, the servo is seeded from it -- a
good real-world starting point. If the pedal happens to be depressed at boot,
the reading falls outside the deadband and the servo is seeded from nominal
center (8192) instead, preventing a false offset from being locked in at
startup.


Crosstalk Rejection
-------------------

The MSP430 shares its ADC ground and supply rails across all analog controls.
When a volume knob or effects pot moves, the resulting ground bounce can appear
as a brief spurious excursion on the pitch bend channel. The pitch activity
filter distinguishes real eWhammy gestures from these artifacts based on a
physical property: real eWhammy motion is continuous, while crosstalk is
isolated.

The filter runs two gates in parallel. A fine gate<2> watches for any movement
of 2 units or more in the corrected pitch output -- sensitive enough to detect
the start of even a slow bend. Each time this small gate fires, a 4 ms
activity window is refreshed and the filter is marked active. If no motion is
seen for 4 ms, the filter goes inactive. A coarser gate<16> then serves as the
actual MIDI transmission gate -- it only allows a pitch bend message when the
value has moved enough to be worth sending. Both conditions must be true: the
activity filter must confirm that real sustained motion is occurring, and the
MIDI gate must confirm the value has changed meaningfully. An isolated one-off
spike from crosstalk will trip the MIDI gate but fail the activity test, and
is silently dropped.


Output
------

When both conditions are satisfied, a 14-bit MIDI pitch bend message is sent.
The output is clamped to the valid range 0-16383, with 8192 representing
center (no pitch change), 0 representing maximum downward bend, and 16383
maximum upward bend.
