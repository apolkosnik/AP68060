#!/bin/sh
# Synthesis-only area estimate of the core with parameter overrides:
# syn/mapvar.sh <name> [PARAM=value ...]   (results: <name>/ap68060.map.rpt)
set -e
here=$(cd "$(dirname "$0")" && pwd)
R=${R:-$here/../rtl}
name=$1; shift
d=$here/$name
rm -rf "$d"; mkdir -p "$d"
{
echo "set_global_assignment -name FAMILY \"Cyclone V\""
echo "set_global_assignment -name DEVICE 5CSEBA6U23I7"
echo "set_global_assignment -name TOP_LEVEL_ENTITY ap68060"
echo "set_global_assignment -name NUM_PARALLEL_PROCESSORS 4"
echo "set_instance_assignment -name VIRTUAL_PIN ON -to *"
for p in "$@"; do echo "set_parameter -name ${p%%=*} ${p#*=}"; done
for f in $R/ap68060_pkg.sv $R/gen/ap68060_upkg.sv $R/ap68060_alu.sv $R/ap68060_alu_slow.sv $R/ap68060_muldiv.sv $R/ap68060_biu.sv $R/ap68060_ram.sv $R/ap68060_atc.sv $R/ap68060_dmu.sv $R/ap68060_predec.sv $R/ap68060_fetch.sv $R/ap68060_decode.sv $R/ap68060_useq.sv $R/fpu/ap040_fpu.v $R/ap68060_fpif.sv $R/ap68060_backend.sv $R/ap68060_snoop.sv $R/ap68060.sv; do
  case "$f" in *.sv) echo "set_global_assignment -name SYSTEMVERILOG_FILE $(readlink -f $f)";; *) echo "set_global_assignment -name VERILOG_FILE $(readlink -f $f)";; esac
  echo "set_global_assignment -name SEARCH_PATH $(dirname $(readlink -f $f))"
done
} > "$d/ap68060.qsf"
echo 'PROJECT_REVISION = "ap68060"' > "$d/ap68060.qpf"
cd "$d"
quartus_map ap68060 > map.log 2>&1 || { echo "$name: MAP FAILED"; tail -20 map.log; exit 1; }
grep -E "Estimate of Logic utilization|Total registers" ap68060.map.rpt | head -3 | sed "s/^/$name /"
