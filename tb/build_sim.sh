#!/bin/sh
# Verilate the program bench.  Object tree under $WORK (default
# $WORK (default ../obj), NOT /tmp: it is a small tmpfs on this host).
set -e
cd "$(dirname "$0")"
RTL=../rtl
WORK=${WORK:-$(cd .. && pwd)/obj}
mkdir -p "$WORK"
python3 ../tools/ucode.py
verilator --binary --timing -j 16 --build-jobs 16 -O3 \
	-Wno-fatal -Werror-PINMISSING -Werror-IMPLICIT -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-UNUSEDSIGNAL -Wno-PINCONNECTEMPTY \
	--top-module tb_ap68060 --Mdir "$WORK/obj_prog" -o tb_ap68060 \
	-I"$RTL" -I"$RTL/fpu" -CFLAGS "-O2 -march=native" $DEFS \
	$RTL/ap68060_pkg.sv $RTL/gen/ap68060_upkg.sv $RTL/ap68060_alu.sv $RTL/ap68060_alu_slow.sv $RTL/ap68060_muldiv.sv \
	$RTL/ap68060_biu.sv $RTL/ap68060_ram.sv $RTL/ap68060_atc.sv $RTL/ap68060_dmu.sv $RTL/ap68060_predec.sv $RTL/ap68060_fetch.sv \
	$RTL/ap68060_decode.sv $RTL/ap68060_useq.sv $RTL/fpu/ap040_fpu.v $RTL/ap68060_fpif.sv $RTL/ap68060_backend.sv \
	$RTL/ap68060_snoop.sv $RTL/ap68060.sv \
	$RTL/fastram/ap030_l2ram.sv $RTL/fastram/ap030_async_fifo.v $RTL/fastram/ap030_fastram_fe.v \
	$RTL/fastram/ap030_fastram_be.v $RTL/fastram/ap060_fastram_bus.sv $RTL/fastram/ap060_fastram.sv \
	m68060_bus_slave.sv m68060_alt_master.sv ddr_model.sv tb_ap68060.sv
echo "built $WORK/obj_prog/tb_ap68060"
