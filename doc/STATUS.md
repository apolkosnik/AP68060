# AP68060 status

A pipelined MC68060 in SystemVerilog, converted from the AP68040-60 core,
aimed at the MiSTer Cyclone V (5CSEBA6U23I7) at 60 MHz in under 20,000
ALMs, validated with Verilator.

## Implemented (68060 behaviour, MC68060UM section in brackets)

- **Programming model** [3, 8]: SR (T1 only, M kept, cleared by an
  interrupt), single supervisor stack, VBR, SFC/DFC, CACR, TCR, URP/SRP,
  ITT/DTT, BUSCR, PCR.  Exception frames $0, $2, $3, $4; RTE format
  check; the 060's trace and interrupt priorities.
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
- **Bus** [7]: the 68060 bus -- BS3-BS0, SAS, TRA, CLA, BGR, BTT/BB
  arbitration, TIP; the reset modes taken from IPL2-IPL0 and D15-D0
  (acknowledge-termination ignore states, native TRA retries, extra write
  hold); acknowledge cycles (IACK, breakpoint, LPSTOP); RSTO for 512
  clocks.  Snooping invalidates only (SNOOP with TT1 = 0, both caches,
  the push buffer included); a snoop has priority over the processor's
  next transfer.

## Verification

- `tb/run_tests.sh`: 23 self-checking programs (tb/asm) under ten bus
  configurations -- wait states, burst inhibit, retries, BCLK at half the
  clock, and the 060 reset modes (ignore states with junk terminations in
  every ignored clock, native retries, write hold, CLA, BGR).  The
  bench's slave checks the bus protocol on every transfer (BS3-BS0, SAS
  timing, the A3:A2/CLA rule, the write hold).  All 230 runs pass.
- WinUAE cputest corpus for the 68060 (`tb/run_cputest.py`, generated
  with `cpu_unimplemented=0`): all slices pass except the oracle
  deviations below.

### Known deviations of the corpus oracle (the core follows the manual)

- Denormal or unnormal FP source operands: WinUAE stacks a format $3
  post-instruction frame ("simplification" in fpp.cpp); the manual (6.5.2)
  says format $0, pre-instruction, for register and memory sources.
- Traced multi-instruction rounds: the generator runs the sequence without
  the trace exceptions in between, so effects of those exceptions are not
  in its expectations (BUSCR L/LE copied to SL/SLE, 7.4), nor is TCR's
  default write protection (DWO) when a round writes TCR.
  This also covers the ODDEXC/ODDIRQ rounds (every vector odd) that trace
  an instruction followed by another before the terminating ILLEGAL.

### Known simplifications

- Branch prediction errors (FSLW.BPE access errors) are never generated:
  a stale branch cache entry is corrected in the decoder instead.

## Area and timing

See `syn/fitcore.sh` (out-of-context fit, every port a virtual pin).
Figures are recorded per commit below.

| commit  | ALMs | registers | M10K bits | DSP | 60 MHz slack |
|---------|------|-----------|-----------|-----|--------------|
| c91d41c | 24,574 | 15,150 | 337,920 | 15 | -0.210 ns (Fmax 59.25 MHz; DMU steal -> D-ATC -> tag compare -> dq1) |

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

## Next

1. Area: bring the core under 20,000 ALMs (the FPU is 7,800 of 33,300
   ALUTs; decode, fetch and the DMU 4,000-5,200 each).
2. 60 MHz closure, structurally (no seed sweeps).
3. Superscalar dispatch (the 060's sOEP under PCR.ESS) within the budget.
