#!/bin/bash
# AP68060 regression: every program under ten bus configurations -- the
# 68040-protocol ones (wait states, burst inhibit, retries, BCLK at half
# CLK) and the 68060 reset modes (ignore states with junk terminations in
# the ignored clocks, native TRA retries, extra write hold, CLA, BGR).
# The bench's slave checks the bus protocol on every transfer.
# Usage: run_tests.sh [program ...]   (default: the passing set)
# JOBS=<n> simulations run in parallel (default: half the CPUs).
set -e
cd "$(dirname "$0")"
PROGS=${*:-"smoke t_integer t_exceptions t_mmu t_cache t_atcprobe t_bitfield_cache t_bitfield_mmu t_movem_restart t_moves_fc t_fpu t_fpu_frames t_snoop t_btb t_stld t_pins t_snstress t_eredir t_cbsplit t_cmodes t_loops t_early"}
WORK=${WORK:-$(cd .. && pwd)/obj}; export WORK
JOBS=${JOBS:-$(( $(nproc) / 2 ))}
./build_asm.sh $PROGS
./build_sim.sh > build/sim_build.log 2>&1 || { tail -30 build/sim_build.log; exit 1; }
SIM=$WORK/obj_prog/tb_ap68060
CFGS=(
	""
	"+waits"
	"+waits +tbi=2 +retry=10"
	"+bclk2"
	"+bclk2 +waits +tbi=1 +retry=20"
	"+ign=3120 +ignjunk +waits"
	"+native +retry=20 +waits"
	"+whold +waits +tbi=2"
	"+cla +whold +ign=2101 +ignjunk +native +retry=10 +bclk2 +tbi=2"
	"+bgr +waits"
)
out=$(for p in $PROGS; do for c in "${CFGS[@]}"; do printf '%s|%s\n' "$p" "$c"; done; done |
	xargs -d '\n' -P "$JOBS" -I{} bash -c '
		p=${1%%|*}; c=${1#*|}
		r=$('"$SIM"' +prog=build/$p.hex $c +timeout=4000000 2>&1 | grep -E "^(PASS|FAIL)" | tail -1)
		printf "%-16s %-64s %s\n" "$p" "${c:-(zero wait)}" "${r:-FAIL (no result)}"' _ {})
printf '%s\n' "$out" | sort
if printf '%s\n' "$out" | grep -qv " PASS "; then echo "FAILURES"; exit 1; fi
echo "ALL PASSED"
