# AP68060 status

A pipelined MC68060 in SystemVerilog, converted from the AP68040-60 core,
aimed at the MiSTer Cyclone V (5CSEBA6U23I7) at 60 MHz in under 20,000
ALMs, validated with Verilator.

## Implemented (68060 behaviour, MC68060UM section in brackets)

- **Programming model** [3, 8]: SR (T1 only, M kept, cleared by an
  interrupt), single supervisor stack, VBR, SFC/DFC, CACR, TCR, URP/SRP,
  ITT/DTT, BUSCR, PCR.  Exception frames $0, $2, $3, $4; RTE format
  check; the 060's trace and interrupt priorities.  A double bus fault
  (an access or address error in the routine of an access error, an
  address error or reset) halts the processor: PST $1C, BR negated, the
  bus released until RSTI (7.9.3).
- **Integer unit**: the 060 instruction set; MOVEP, CAS2, CHK2/CMP2,
  64-bit MULx/DIVx and CAS on misaligned data trap to vector 61 (format
  $0), as the 060 leaves them to software.  PLPAR/PLPAW, LPSTOP, HALT,
  PULSE.
- **FPU** [6]: the 060 hardware subset; transcendentals, FMOVECR,
  FScc/FDBcc/FTRAPcc trap to vector 11 (format $2); packed decimal,
  denormal and unnormal operands to vector 55 (format $2 / $0 / $3);
  #imm.X/P, dynamic FMOVEM lists and FMOVEM.L #imm to several registers
  to vector 60.  FINT/FINTRZ in hardware.  Three-long-word state frames
  (NULL, IDLE, EXCP).
- **MMU** [4]: 4K/8K pages, 64-entry ATCs (half with FOTC/FITC), table
  searches that bypass the data cache, no ATC entry for a failed search,
  U/M updates per table 4-1, transparent translation, default (TCR)
  cache modes, PLPA, PFLUSH variants, FSLW per 8.4.
- **Caches** [5]: 8 KB four-way data and instruction caches, copyback /
  write-through / noncachable precise and imprecise, NAD/NAI no-allocate,
  FOC/FIC half caches, DPI, push buffer, the four-entry **store buffer**
  (ESB) with SBE access errors, branch cache (EBC, CABC, CUBC).
- **Bus** [7]: the 68060 bus -- BS3-BS0, SAS, TRA, CLA, BGR, TIP; BB or
  BTT arbitration, a BTT-only system (BB pulled up) included, BR the
  registered internal request; the reset modes taken from IPL2-IPL0 and
  D15-D0 (acknowledge-termination ignore states, native TRA retries,
  extra write hold); acknowledge cycles (IACK, breakpoint, LPSTOP), no
  bus cycle after the LPSTOP broadcast but the waking interrupt's
  acknowledge; RSTO for 512 clocks from the end of the transfer in
  progress (the fetch held, the store buffer drained); CIOUT negated on
  acknowledge cycles; 27 clocks of internal reset after RSTI negates;
  BUSCR locked sequences with LOCK on every transfer inside them,
  instruction fetches included, ended only by the LOCKE write, a bus
  error, an exception or BUSCR.L cleared.
  Snooping invalidates only (SNOOP with TT1 = 0, both caches, the push
  buffer and a push waiting for a retry included) and keeps up with
  snooped transfers on every bus clock: the snooper holds a copy of the
  data cache's tags and valid bits, so a miss costs nothing and a hit is
  one frozen DMU clock; a snoop has priority over the processor's next
  transfer.

## Fast RAM for an integration (rtl/fastram)

`ap060_fastram.sv` puts DDR3-backed Fast RAM on the 68060 bus: a bus slave
(`ap060_fastram_bus.sv`) in front of the AP030 Fast RAM front end's native
port (`ap030_fastram_fe.v`: two 16-byte line buffers, an 8 KiB cache below
the processor's, posted writes) and its DDR3 side (`ap030_fastram_be.v`, an
Avalon-MM master), joined by dual-clock FIFOs.  The AP030 files are copied
unchanged from the Minimig AP030 build.  A line read is one request whose
long words come back in the 68060's burst order; writes are posted beat by
beat with BS3-BS0; only TA is driven.  TA is asserted only for clocks the
processor samples (ignore states, extra write hold, BCLK at half CLK).
The integration supplies the window decode (`sel`, with TS) and the DDR3
64-bit word address.  The front end's cache is swept by reset only: no
other master may write the DDR3 behind it.  The bench maps
$50000000-$500FFFFF there, on a DDR3 model with random waits and latency in
a clock of its own; `t_fastram` runs in every configuration.

## Verification

- `tb/run_tests.sh`: 28 self-checking programs (tb/asm) under ten bus
  configurations -- wait states, burst inhibit, retries, BCLK at half the
  clock, and the 060 reset modes (ignore states with junk terminations in
  every ignored clock, native retries, write hold, CLA, BGR).  The
  bench's slave checks the bus protocol on every transfer (BS3-BS0, SAS
  timing, the A3:A2/CLA rule, the write hold); the bench also fails a
  run on: a 68060 transfer while the alternate master owns the bus, or
  while RSTO is asserted, or after the LPSTOP broadcast; another
  master's transfer inside a locked sequence with BGR negated; a halted
  processor that is not quiet (PST $1C, no TS, BR negated); a snoop
  invalidation dropped, or the snooper's tag copy disagreeing with the
  cache on any snoop.  All 280 runs pass.
- `tb/build_fpu_pipe.sh`: the pipelined FPU (FPU_PIPE=1) against the
  sequential one on random command streams (100,000 commands over four
  seeds match).
- `tb/run_bench.sh`: Dhrystone and Whetstone, self-checking (see
  Performance).
- WinUAE cputest corpus for the 68060 (`tb/run_cputest.py`, generated
  with `cpu_unimplemented=0`): all slices pass except the oracle
  deviations below.

### Known deviations of the corpus oracle (the core follows the manual)

- Denormal or unnormal FP source operands: WinUAE stacks a format $3
  frame ("simplification" in fpp.cpp); the manual (6.5.2, table 6-10)
  says format $0 for register and memory sources.  Both stack the next
  PC with the address register updated (the M68060SP emulates the
  instruction from the FSAVE frame and returns past it).
- An unimplemented FPU instruction with an extended -(An) operand: the
  frame's address is An - 4 (the decremented An + 8, as the M68060SP's
  _dcalc_ea expects and 8.2.4 states for the FPU-disabled frame);
  WinUAE reports An - 12.
- Traced multi-instruction rounds: the generator runs the sequence without
  the trace exceptions in between, so effects of those exceptions are not
  in its expectations (BUSCR L/LE copied to SL/SLE, 7.4), nor is TCR's
  default write protection (DWO) when a round writes TCR.
  This also covers the ODDEXC/ODDIRQ rounds (every vector odd) that trace
  an instruction followed by another before the terminating ILLEGAL.

### Where the core follows the manual and WinUAE differs (not in the corpus)

- MOVES to an alternate space (SFC/DFC 0, 3, 4, 7) uses the address as a
  physical one, untranslated and noncachable (4.2.5, 7.7.1); WinUAE
  translates it, FC bit 2 picking the root.
- PLPA works on the data side (data TTRs and ATC) whatever the DFC (4.7.3,
  appendix D); WinUAE uses the instruction side for DFC 2 and 6.
- A write meeting a write-protected root or pointer descriptor ends the
  table search with WP, its history untouched (4.2.6.3); WinUAE walks on.

### Known simplifications

- Branch prediction errors (FSLW.BPE access errors) are never generated:
  a stale branch cache entry is corrected in the decoder instead.
- Single issue: the 060's second integer pipeline (sOEP, PCR.ESS) is not
  implemented; PCR.ESS reads and writes but changes nothing.

## Performance

`tb/run_bench.sh` (vbcc -O=1023 for the 68060; `tb/c/crt0.s` with
COPYBACK: user mode, copyback data, data/instruction/branch caches and
the store buffer on, FPCR double precision).  Each program runs at two
counts and the difference is taken, so start-up and cold caches drop
out; the loops then run from the caches, and the bus setting makes no
difference (zero-wait memory and BCLK at half CLK with random wait states
give the same cycles).  Scores at 60 MHz, from RTL simulation:

| build                          | Dhrystone 2.1                          | Whetstone              |
|--------------------------------|----------------------------------------|------------------------|
| default (area options off)     | 934 cycles/run, 36.56 DMIPS, 0.609 DMIPS/MHz | 9.59 MWIPS (0.160/MHz) |
| FPU_PIPE, LOOP_PRED, EARLY_RES | 874 cycles/run, 39.07 DMIPS, 0.651 DMIPS/MHz | 10.40 MWIPS (0.173/MHz) |

- Dhrystone: `tb/c/dhry.c`, 2.1, checked against the published final
  values; 660 instructions per run (1.42 cycles per instruction).  DMIPS =
  Dhrystones/s / 1757.
- Whetstone: `tb/c/whet.c`, the classic double-precision Curnow-Wichmann
  modules and loop counts (one loop = 100,000 Whetstone instructions).
  The 68060 has no transcendental instructions -- an Amiga runs them in
  the 68060.library's software package -- so sin, cos, atan, exp and log
  are C routines in the program (within 4e-15 of libm); sqrt is FSQRT.
  The results are checked against the same source compiled on the host.
  With a different math library the score moves accordingly: module 7
  and 11 (the functions) are most of the time.
- Whetstone found a pipeline bug (fixed at the commit that added it): a
  multiply or divide finishing in the cycle the next one entered EX let
  the second skip its operation.  t_integer 300-305 cover it.

## Area and timing

See `syn/fitcore.sh` (out-of-context fit, every port a virtual pin).
Figures are recorded per commit below.

| commit  | ALMs | registers | M10K bits | DSP | 60 MHz slack |
|---------|------|-----------|-----------|-----|--------------|
| c91d41c | 24,574 | 15,150 | 337,920 | 15 | -0.210 ns (Fmax 59.25 MHz; DMU steal -> D-ATC -> tag compare -> dq1) |
| a4693f9 | not fitted: synthesis estimate 20,654 (c91d41c's was 22,575, so a fit near 22,500) | 13,231 | | | |
| 040fd85 | not fitted: synthesis estimate 20,929 (52578b9: 20,930; the snooper's tag copy is in M10Ks) | 13,260 | | | |

### Area options (rtl/ap68060.sv parameters, default 0)

Synthesis estimates at 8ca556f (all on: 22,651 ALMs) and cycles of the
tb/asm bench programs:

| option      | ALMs saved when 0 | cost when 0                               |
|-------------|-------------------|-------------------------------------------|
| FPU_PIPE    | ~990              | bench_fpu +7.1 %, bench_insn +4.8 %       |
| LOOP_PRED   | } ~1,000 together | bench_loop +2.5 %                         |
| EARLY_RES   | }                 | bench_insn +1.9 %                         |

A BALANCED-mode fit (instead of HIGH PERFORMANCE EFFORT) is no smaller
(24,658 ALMs).

### The 060-120mhz branch (2026-10-30)

Branched from main at 3bd8b4e to reach 120 MHz; area may grow here (the
20,000 ALM limit stays on main), the FPU is re-pipelined, no multicycle
or false-path exception anywhere.  ~110 commits, each a structural
change judged by an out-of-context fit at PERIOD=8.333 (`syn/fitcore.sh`,
`PERIOD=8.333`), the regression (393 PASS) and the sim's checks that every
registered hold, select and flag equals (or, where allowed, is no shorter
than) the value it replaces.

What was done, in order of effect: every pipeline hold a register formed
a clock ahead (the DMU's four, WB's, EX's, the FPU interface's `safe`,
AG's interlock); the register files' bank and front selects registered
from next clock's address for all seven ports, their tables written a
clock late from registered writes; operand-match flags for every stage's
capture of EX's result; the DMU's records' hit clears and the scan's copy
compared per invalidation source; the engine's line registered before the
byte take; fetch's next addresses as one AND-OR each; the ALU's other
results as one AND-OR; preserved twins placed by their consumers.

| fit (PERIOD 8.333)      | commit  | Fmax       | slack     | TNS      | failing endpoints | ALMs   |
|-------------------------|---------|------------|-----------|----------|-------------------|--------|
| main baseline           | 3bd8b4e | 60.7 MHz   | -8.1 ns   |          | 16,594            |        |
| f120_n                  | 9318c9f | 96.4 MHz   | -2.04 ns  | -717 ns  | 1,599             | 24,524 |
| f120_r                  | 1cadb07 | 101.6 MHz  | -1.51 ns  | -431 ns  | 1,066             | 24,906 |
| f120_ab (best)          | 5755964 | 102.65 MHz | -1.41 ns  | -194 ns  | 826               | 25,394 |
| f120_ac .. f120_ag      | ..629723a | 96.5-100.6 MHz | -1.6..-2.0 ns | -364..-699 ns | 1,160-1,943 | ~25,800 |
| f120_ah (stalls registered) | dbecc65 | 96.9 MHz | -1.99 ns | -226 ns | 711 | 25,302 |
| f120_az (best)          | 5fcb013 | 103.25 MHz | -1.35 ns  | -141 ns  | 539               | 26,032 |

The last six fits are near-identical netlists: their spread (TNS -194 to
-699, the top clusters moving between modules) is placement, not logic --
what fails is 3-4 LUTs with 6-8 ns of wire (the stall chain's fan-out,
the DMU-backend boundary, the file reads).  The fitter's AGGRESSIVE
PERFORMANCE mode was worse (100.4 MHz, -499 ns) than HIGH PERFORMANCE
EFFORT on the same RTL.

**Certified: 100 MHz.**  At PERIOD=10 the design closes with no
violations: HEAD 629723a 101.07 MHz (slack +0.106 ns, 25,472 ALMs), the
f120_ab netlist 102.98 MHz (+0.289 ns, 25,094 ALMs).

Benchmarks at HEAD (tb/run_bench.sh): Dhrystone 2.1 1,071 cycles per
iteration (main: the same), Whetstone 754,922 cycles per loop (main
747,389: +1.0 %, from the one conservative case of the registered FPU
`safe` -- a uop leaving EX as the FPU's enters).

**Every stage's stall is now a register** (dbecc65 and after; the design
study is doc/NOTES-regstall.md).  The DMU's four hold next-values turned
out to need only three block-local terms, two replicated exactly and one
(DC2's ready) allowed to be conservative by its contract, so no
two-process refactor was needed.  The stall registers' D-inputs take the
late decisions (EX's exception decision, the FPU interface's hold, the
skid's valid, AG's address compare) as the last picks; DT's own decision,
most of a clock, is out of them: the registers assume the access entering
DC2 ready and DC2's registered ready corrects the stall nets one LUT from
flip-flops.  With that the fit-to-fit spread collapsed (ah -226, an -210,
ao -244, ap -210, av -182, az -141 TNS) and the failing endpoints fell
from ~1,000-1,900 to 539; what remains fails by at most 1.35 ns: AG's
operand capture (the file read into the capture's AND-OR), the slow
ALU's flags, DC2's ready from DT's tag compare (-0.84), the engine's
stolen lookup (-0.82) -- each within ~1 ns of wire plus 3-4 LUTs.
Benchmarks unchanged (Dhrystone 1,071, Whetstone 754,922).

## Next

1. Area: about 2,500 ALMs still to go.  With the options off, by ALUTs:
   FPU 6,400, backend pipeline 4,600 (the ~410-bit uop through five
   stages, ~2,900 registers), DMU 5,300, fetch 4,600 (two predecoders
   670 each), decode 3,300 (five ~400-bit decoded records).  Planned:
   narrow the uop per stage, remove the 040-only FPU paths the 060 traps
   (checked against the corpus), then smaller queues.
2. 60 MHz closure on main, structurally (no seed sweeps); on this branch,
   120 MHz by the registered per-stage stalls (the DMU in two-process
   form first) and/or an AG operand stage.
3. Performance: superscalar dispatch (the 060's sOEP) within the budget.
