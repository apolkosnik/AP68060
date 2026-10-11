#!/bin/sh
# Out-of-context fit of the whole core on the MiSTer part (5CSEBA6U23I7) at
# 60 MHz: syn/fitcore.sh <work dir name> [mkooc script]
#   R=<rtl dir>  (default: the rtl next to this script)   SEED=<n>
# Every port is a virtual pin, so the result is the core's register-to-
# register timing alone.  Results: <name>/ap68060.sta.summary (all corners),
# <name>/paths.txt (worst setup paths).  Judge a change on three seeds.
here=$(cd "$(dirname "$0")" && pwd)
R=${R:-$here/../rtl}
S=${2:-$here/mkooc.sh}
cd "$here"
exec $S $1 ap68060 16.667 $R/ap68060_pkg.sv $R/gen/ap68060_upkg.sv $R/ap68060_alu.sv $R/ap68060_alu_slow.sv $R/ap68060_muldiv.sv $R/ap68060_biu.sv $R/ap68060_ram.sv $R/ap68060_atc.sv $R/ap68060_dmu.sv $R/ap68060_predec.sv $R/ap68060_fetch.sv $R/ap68060_decode.sv $R/ap68060_useq.sv $R/fpu/ap040_fpu.v $R/ap68060_fpif.sv $R/ap68060_backend.sv $R/ap68060_snoop.sv $R/ap68060.sv
