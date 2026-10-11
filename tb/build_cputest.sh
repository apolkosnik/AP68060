#!/bin/sh
# Verilate the cputest replay bench (tb_cputest.sv) and assemble its monitor.
set -e
cd "$(dirname "$0")"
RTL=../rtl
WORK=${WORK:-$(cd .. && pwd)/obj}
mkdir -p "$WORK"
python3 ../tools/ucode.py
[ "${ASM:-1}" = 1 ] && ./build_asm.sh cputest_mon
verilator --binary --timing -j 16 --build-jobs 16 -O3 \
	-Wno-fatal -Werror-PINMISSING -Werror-IMPLICIT -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-UNUSEDSIGNAL -Wno-PINCONNECTEMPTY \
	--top-module tb_cputest --Mdir "$WORK/obj_cputest" -o tb_cputest \
	-I"$RTL" -I"$RTL/fpu" -CFLAGS "-O2 -march=native" \
	$RTL/ap68060_pkg.sv $RTL/gen/ap68060_upkg.sv $RTL/ap68060_alu.sv $RTL/ap68060_alu_slow.sv $RTL/ap68060_muldiv.sv \
	$RTL/ap68060_biu.sv $RTL/ap68060_ram.sv $RTL/ap68060_atc.sv $RTL/ap68060_dmu.sv $RTL/ap68060_predec.sv $RTL/ap68060_fetch.sv \
	$RTL/ap68060_decode.sv $RTL/ap68060_useq.sv $RTL/fpu/ap040_fpu.v $RTL/ap68060_fpif.sv $RTL/ap68060_backend.sv \
	$RTL/ap68060_snoop.sv $RTL/ap68060.sv m68060_bus_slave.sv tb_cputest.sv
echo "built $WORK/obj_cputest/tb_cputest"
