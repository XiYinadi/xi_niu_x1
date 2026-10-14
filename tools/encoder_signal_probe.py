#!/usr/bin/env python3
"""Read-only raw encoder event diagnostic; never requests output lines.

Reports raw edge counts, repeated edge polarity (possible missing events),
source mismatches and minimum event intervals. Counts are not displacement.
No filtering is applied: electrical/driver problems must remain visible.
"""
import argparse
import json
import sys
import time
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=120)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if not 0 < args.seconds <= 600:
        parser.error('--seconds must be within (0, 600]')
    import gpiod
    names = {0:'L_A', 12:'L_B', 5:'R_A', 1:'R_B'}
    stats = {off:dict(name=name, edges=0, rising=0, falling=0,
                     repeated_polarity=0, wrong_source=0, nonmonotonic=0,
                     min_interval_ns=None) for off,name in names.items()}
    previous = {}
    chip = gpiod.Chip('gpiochip0')
    bulk = chip.get_lines(list(names))
    requested = False
    start = time.monotonic()
    def report():
        print(json.dumps(dict(elapsed_s=round(time.monotonic()-start,2),
                              lines=list(stats.values())),ensure_ascii=False),flush=True)
    try:
        bulk.request(consumer='encoder_signal_probe', type=gpiod.LINE_REQ_EV_BOTH_EDGES,
                     flags=gpiod.LINE_REQ_FLAG_BIAS_PULL_UP)
        requested = True
        print('INPUT ONLY. Initial levels: '+repr(bulk.get_values()),flush=True)
        last = start
        while time.monotonic()-start < args.seconds:
            ready = bulk.event_wait(0, 20000000)
            for line in ready or []:
                off = line.offset()
                for _ in range(512):
                    if not line.event_wait(0,0): break
                    event = line.event_read()
                    t = event.sec*1000000000+event.nsec
                    polarity = event.type == gpiod.LineEvent.RISING_EDGE
                    s = stats[off]
                    s['edges'] += 1
                    s['rising' if polarity else 'falling'] += 1
                    if event.source.offset() != off: s['wrong_source'] += 1
                    if off in previous:
                        prev_t, prev_polarity = previous[off]
                        delta = t-prev_t
                        if polarity == prev_polarity: s['repeated_polarity'] += 1
                        if delta <= 0: s['nonmonotonic'] += 1
                        if delta > 0 and (s['min_interval_ns'] is None or delta < s['min_interval_ns']):
                            s['min_interval_ns'] = delta
                    previous[off] = (t,polarity)
            if time.monotonic()-last >= 2:
                report(); last = time.monotonic()
    except KeyboardInterrupt:
        print('Stopped by operator',flush=True)
    finally:
        if requested: bulk.release()
        chip.close()
    report()
    if args.output:
        args.output.write_text(json.dumps(dict(lines=list(stats.values())),indent=2)+'\n')


if __name__ == '__main__':
    main()
