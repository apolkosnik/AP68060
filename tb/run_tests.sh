#!/bin/bash
# AP68060 regression: every program under eleven bus configurations -- the
# 68040-protocol ones (wait states, burst inhibit, retries, BCLK at half
# CLK), the 68060 reset modes (ignore states with junk terminations in
# the ignored clocks, native TRA retries, extra write hold, CLA, BGR) and
# IPL bits 1-0 changing a clock after bit 2 (unequal pin delays).
# The bench's slave checks the bus protocol on every transfer.
# Usage: run_tests.sh [program ...]   (default: the passing set)
# JOBS=<n> simulations run in parallel (default: half the CPUs).
set -e
cd "$(dirname "$0")"
PROGS=${*:-"smoke t_integer t_exceptions t_mmu t_cache t_atcprobe t_bitfield_cache t_bitfield_mmu t_movem_restart t_moves_fc t_fpu t_fpu_frames t_snoop t_btb t_stld t_pins t_snstress t_eredir t_cbsplit t_cmodes t_loops t_early t_stbuf t_berr t_irq7 t_mmu060 t_cache060 t_halt t_snirq t_iplglitch t_buslock t_bsrfault"}
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
	"+iplskew=3 +waits +retry=10"
)
out=$(for p in $PROGS; do for c in "${CFGS[@]}"; do printf '%s|%s\n' "$p" "$c"; done; done |
	xargs -d '\n' -P "$JOBS" -I{} bash -c '
		p=${1%%|*}; c=${1#*|}
		r=$('"$SIM"' +prog=build/$p.hex $c +timeout=4000000 2>&1 | grep -E "^(PASS|FAIL)" | tail -1)
		printf "%-16s %-64s %s\n" "$p" "${c:-(zero wait)}" "${r:-FAIL (no result)}"' _ {})
printf '%s\n' "$out" | sort
fails=0
printf '%s\n' "$out" | grep -qv " PASS " && fails=1
# the fetch unit alone: TCI instruction lines (tb_fetch_tci.sv)
./run_fetch_tci.sh || fails=1
if [ $fails != 0 ]; then echo "FAILURES"; exit 1; fi
echo "ALL PASSED"
