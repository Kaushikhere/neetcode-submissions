#!/bin/bash
# Builds the simulator around the real firmware and runs a set of race scenarios.
# Usage: bash sim/run_tests.sh   (from the robot-speed-race folder)
set -e
cd "$(dirname "$0")/.."
OUT=${SIM_BIN:-/tmp/speed_sim}
g++ -O2 -std=c++17 -Isim sim/sim.cpp -o "$OUT"
run() { printf "%-34s " "$*"; "$OUT" "$@" | sed 's/ | fw:.*//'; }
run side=R
run side=L
run side=R seed=2
run side=R seed=3 noise=0.4 drop=0.08 spike=0.04
run side=R R=104
run side=R R=113
run side=R R=122
run side=R bump=2
run side=R panels=7 flat=0.5
run side=R panels=10 flat=1
run side=R panels=12 flat=1
run side=R z0=3
run side=R z0=14
run side=R psi0=12
run side=R psi0=-12
run side=R psi0=-20
run side=R psi0=-12 z0=4
run side=R noise=0.6 drop=0.15 spike=0.08
run side=R vl=70 vr=58
run side=R vr=62
run side=R vl=55 vr=60
run side=R tau=0.15 d0=0.15
run side=R track=18.5
run side=R cone=12
run side=R yawn=0.004
run side=L psi0=-15 bump=2
run side=L R=122 noise=0.4 drop=0.08
