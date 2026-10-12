#!/bin/sh
# The fetch unit alone (tb_fetch_tci.sv): TCI instruction lines are not
# kept (MC68060UM 5.7.1).  Builds under $WORK, runs every case.
set -e
cd "$(dirname "$0")"
WORK=${WORK:-$(cd .. && pwd)/obj}
R=../rtl
verilator --binary --timing -j 8 -Wno-fatal --top-module tb_fetch_tci --Mdir "$WORK/obj_fetch_tci" -o tb_fetch_tci \
	-I$R $R/ap68060_pkg.sv $R/gen/ap68060_upkg.sv $R/ap68060_ram.sv $R/ap68060_atc.sv $R/ap68060_predec.sv \
	$R/ap68060_fetch.sv tb_fetch_tci.sv > "$WORK/fetch_tci_build.log" 2>&1 || { tail -20 "$WORK/fetch_tci_build.log"; exit 1; }
fail=0
for c in "+tci" "+tci +race" "+tci +sequential" "+tci +sequential +invalidate" ""; do
	r=$("$WORK/obj_fetch_tci/tb_fetch_tci" $c 2>&1 | grep -E 'PASS|FAIL' | head -1)
	printf '%-32s %s\n' "fetch_tci [$c]" "$r"
	case "$r" in PASS*) ;; *) fail=1;; esac
done
exit $fail
