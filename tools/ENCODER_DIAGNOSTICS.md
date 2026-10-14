# Encoder read-only diagnostics

These tools request GPIO INPUT events only: lines 0, 12, 5, 1. They do not
start motor_driver, pwm_proc or publish motion commands. Stop other encoder
readers before use. Inputs must be compatible with 3.3 V logic.

Python: /usr/bin/python3 ~/robot/tools/encoder_signal_probe.py --seconds 120

Native: gcc -O2 -Wall -Wextra -Werror -std=c11 ~/robot/tools/encoder_native_probe.c -o /tmp/encoder_native_probe
Then /tmp/encoder_native_probe 120. Duration 0 waits until SIGINT/SIGTERM;
use only when explicitly coordinating a manual test and stop after completion.

Counters report raw GPIO edge events, not quadrature displacement or wheel
revolutions. repeated_polarity means consecutive events have the same polarity;
possible explanations include event loss or driver/signal anomalies. The tool
cannot measure how many events were lost. nonmonotonic counts timestamps less
than OR EQUAL TO the previous timestamp. It does not prove time reversal.

2026-10-14: stationary counters remained stable. Manual right-wheel rotation
in the native reader still produced left A/B counts 800/843. Right A/B had
51509/51873 events, with 16103/19053 repeated-polarity events and 69/61
non-increasing timestamps. Test output is saved in
../data/encoder_native_acceptance_20261014.json. The native result rules out
Python overhead as the sole explanation; it does not establish hardware fault.

Do not enable wheel odometry or PID using these counts. Next isolate signals
and repeat at a known slow speed; compare raw A/B waveforms with the reported
events if a logic analyzer or oscilloscope is available. Existing QuadDecoder
also drains events grouped by line rather than globally by timestamp, so its
direction decoding needs separate verification before enabling closed loop.
