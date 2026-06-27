#!/usr/bin/env python3
"""
Replay the REAL e-Whammy gesture captures through host_sim and report behavior.

This is the preferred development/test driver -- it feeds the firmware the actual
recorded ADC stream from hardware (programmer unplugged, free-running diag), instead
of the synthetic NEXUS_SIM timeline, which never modeled the real failure modes.

Captures live in the KB: <center-stability>/captures/2026-06-27_recNN_*.csv
(columns t_s,raw,pos,off). We convert each to host_sim --feed format on the fly
(t,p14,ps with ps=(raw-512)*16 so host_sim recovers the raw ADC), replay it, and
measure: gesture peak, and the post-release STRAND -- the longest stretch the output
sits off-center (>0.3 ST) while the bar is back at rest. The latch shows up as a big
strand; a fix must shrink it on rec06/07/07_take2 without growing it on the clean ones.

Usage:
    python3 replay_captures.py [captures_dir] [--plot]
Build host_sim first:  cmake -S test -B test/build && cmake --build test/build
"""
import csv, subprocess, sys, os, tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
HOST = os.path.join(HERE, "build", "host_sim")
DEFAULT_CAP = os.path.expanduser(
    "~/dev/cycfi/cycfi_ai_dev/e_whammy/center-stability/captures")
ST = 1.0 / 682.67          # 14-bit pitch units -> semitones

# recording -> expected verdict (the ground truth from the bench session)
RECS = [
    ("rec01_power-on",          "clean"),
    ("rec02_deep-bend",         "clean"),
    ("rec03_tall-pull",         "clean"),
    ("rec04_mild-vibrato",      "clean"),
    ("rec05_deep-vibrato",      "clean"),
    ("rec06_1ST-up",            "chase (mild)"),
    ("rec07_1ST-down",          "watchdog strand"),
    ("rec07_1ST-down_take2",    "chase strand"),
    ("rec08_2ST-up",            "clean"),
    ("rec09_2ST-down",          "clean"),
]

def replay(csv_path):
    rows = list(csv.DictReader(open(csv_path)))
    fd, feed = tempfile.mkstemp(suffix=".csv")
    with os.fdopen(fd, "w") as f:
        f.write("t,p14,ps\n")
        for r in rows:
            f.write("%s,0,%d\n" % (r["t_s"], (int(r["raw"]) - 512) * 16))
    out = subprocess.run([HOST, "--feed", feed, "--holdoff", "300", "--dump"],
                         capture_output=True, text=True).stdout
    os.unlink(feed)
    traj = []                              # (t_ms, adc, out_ST)
    for ln in out.splitlines()[1:]:        # skip header
        p = ln.split(",")
        if len(p) >= 3:
            try: traj.append((int(p[0]), int(p[1]), (int(p[2]) - 8192) * ST))
            except ValueError: pass
    return traj

def analyze(traj):
    if not traj: return None
    adcs = [a for _, a, _ in traj]
    rest = adcs[-1]                        # the settled rest (after any hysteresis shift)
    peak = max((o for _, _, o in traj), key=abs)
    # post-release strand: longest run where bar is near rest but output is off-center
    strand = cur = 0
    for _, a, o in traj:
        cur = cur + 1 if (abs(a - rest) <= 12 and abs(o) > 0.3) else 0
        strand = max(strand, cur)
    return peak, strand / 1000.0          # ms -> s (1 tick = 1 ms)

def main():
    cap = DEFAULT_CAP
    for a in sys.argv[1:]:
        if not a.startswith("-"): cap = a
    if not os.path.exists(HOST):
        sys.exit("host_sim not built: cmake -S test -B test/build && cmake --build test/build")
    print("captures: %s\n" % cap)
    print("  %-26s %-16s %8s %10s" % ("recording", "expected", "peak ST", "strand s"))
    print("  " + "-" * 64)
    for name, verdict in RECS:
        path = os.path.join(cap, "2026-06-27_%s.csv" % name)
        if not os.path.exists(path):
            print("  %-26s  (missing)" % name); continue
        r = analyze(replay(path))
        if r is None: print("  %-26s  (no output)" % name); continue
        peak, strand = r
        print("  %-26s %-16s %+7.2f %9.1f" % (name, verdict, peak, strand))
    print("\n  strand s = longest stretch output > 0.3 ST while the bar is at rest")
    print("  (the latch). A fix should cut it on the chase/watchdog rows, keep the")
    print("  clean rows low. Compare this table before vs after a firmware change.")

if __name__ == "__main__":
    main()
