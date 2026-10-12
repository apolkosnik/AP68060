# AP68060 audit against the MC68060UM and WinUAE (2026-10-11, at 2dac5a7)

Four parallel audits (integer unit and exceptions, MMU, FPU, bus and
caches) of the RTL against the MC68060 User's Manual (text extraction
`/home/adam/ref68040/text/um.txt`), WinUAE (`/home/adam/WinUAE`) and, for
the FPU, Motorola's M68060SP (`fpsp.S`), which consumes the 060's FPU
frames on real systems.  Findings marked **[confirmed]** were reproduced
on the simulator; probe programs are in
`/home/adam/ap68060-work/audit/{int,mmu,fpu,bus}/`.  Findings that occur
in more than one area are listed once.

## Status (2026-10-11, at c02ec8a)

Every A finding is fixed, each with a regression test that fails on the
logic before the fix (checked by building the old logic, or a mutant
restoring it, against the new test):

| # | commit | regression test |
|---|--------|-----------------|
| A1-A5 | 957ed3c | t_fpu (76, 81, 323, 93-95, 121-129 and the chains recomputed), t_fpu_frames 400-442 |
| A6 | 84ab6a8 | t_mmu 55/56, 259/260/292 (MOVES FC0/FC4 physical) |
| A7 | 52578b9 | t_cache060 5-7 |
| A8 | 52578b9 | t_cache060 1-4 (data and instruction fills) |
| A9 | 84ab6a8 | t_irq7 |
| A10 | 040fd85 | t_halt (two double bus faults, the bench resetting the halted processor and checking it quiet) |
| A11 | bc5ac62 | t_berr 9-11 |
| A12 | 52578b9 | t_cache060 8-9 |
| A13 | bc5ac62 | t_berr 1-8 |
| A14 | bc5ac62 | t_mmu060 5-7 (c02ec8a: the bench counts locked transfers) |
| A15 | 040fd85 | t_snoop 57-60 (128 lines snooped back to back by writes and reads); the snooper now keeps a copy of the data cache's tags and valid bits, checked against the cache on every snoop |
| A16 | 040fd85 | t_snoop 61-63 |
| A17 | bc5ac62 | t_snoop 66-67 (c02ec8a) |
| A18 | 1b1d75d | t_snoop 64-65 (BTT-only arbitration) |
| A19 | 1b1d75d | t_pins 9 |
| A20 | 040fd85 | t_halt 1-4, and every run (the bench fails a bus cycle under RSTO or after the LPSTOP broadcast) |

From B: the table-descriptor W rule (the manual's, 84ab6a8), PLPA on the
data side whatever the DFC (84ab6a8), and the bench's PST false failure
and duplicate `+biutrace` line (040fd85) are done.  Still open from B:
the FPU items (FMOVECR EA, `#imm` EA, SNAN/OPERR on FMOVE OUT), FSLW.TTR
on bus errors, the user-write-to-S+W FSLW inconsistency, the SP-fault ATC
entry question, MOVE16 FA, FSLW SIZE for fetches, the integer WinUAE
choices (to be listed in STATUS.md), and the bus items: the dead bus
clock between cycles, BTT/BB a bus clock late, RTE stack reads allocating,
a fetch between a fill and its push, LOCK after a bus error until the next
transfer, the reset details, IPL synchronised on BCLK, the reset-vector
fetch of 0-$10, the I-cache critical word.

From C: the alternate master now runs back-to-back transfers in one
tenure (`$F28A/$F290`), the BTT protocol alone (`$F280` bit 10) and a
retry of a chosen 68060 write with the bus handed over (`$F280` bit 9);
the bench checks the 68060 never drives TS while another master owns the
bus.  Still open: zero-wait C2 answers, TCI, TEA/retry on acknowledge
cycles, retries on later beats, the remaining protocol checks (signal
stability, TIP, LOCK placement, attributes), and tests for PFLUSHN/G,
indirect descriptors, instruction-side TWE and MDIS.

## A. Wrong behaviour that software will notice

| # | Area | Finding | Manual | RTL |
|---|------|---------|--------|-----|
| A1 | FPU | Vector 55 for a denormal/unnormal **source** stacks the faulting instruction's PC and leaves (An)+/-(An) undone; the 060 stacks the **next** PC with An updated (format $0 is right). Under the M68060SP this loops forever. doc/STATUS.md's note on this is a misreading, and t_fpu 76/81/323 assert the wrong behaviour. **[confirmed]** | Table 6-10 ("next" for 0, 2, 3); fpsp.S `fu_in_exit` | fpif.sv:511, :559 (no xnext/xcommit) |
| A2 | FPU | OVFL/UNFL (always trapping on the 060) overwrite a destination FP register; the 060 leaves it unchanged (pre-instruction exception); the SP re-runs the operation from the register.  t_fpu 93-95, 121-122, 124-125, 128-129 assert the wrong behaviour. **[confirmed]** | 6.6.4, 6.6.5; fpsp.S `_fpsp_ovfl/_unfl`; WinUAE agrees | ap040_fpu.v:1131 (x_kill) |
| A3 | FPU | The FSAVE exception operand of a single/double denormal memory source is the raw bits sliced as extended (garbage); the SP expects exponent $3F80/$3C00 with the fraction. **[confirmed]** | fpsp.S `fix_skewed_ops` | ap040_fpu.v ~1614, ~1680 |
| A4 | FPU | FMOVE OUT of a denormal/unnormal register (post-instruction vector 55): (An)+/-(An) not applied for B/W/L/S/D, and the -(An) EA for .X is 8 off. **[confirmed]** | fpsp.S `fout`, `_calc_ea_fout` | fpif.sv:789-795 |
| A5 | FPU | PCR.DFP (FPU disable) does nothing; every FP instruction should take vector 11 with a format $4 frame. **[confirmed]** | PCR, 8.2.4; WinUAE `regs.pcr & 2` | backend.sv:1725 |
| A6 | MMU/bus | MOVES to alternate spaces (SFC/DFC 0, 3, 4, 7) is translated (TTRs, ATC, table search, faults, DWO) and cached; the 060 uses the address untranslated and noncachable (TT=2, CIOUT).  WinUAE translates too, so the corpus cannot see it; t_mmu 55 asserts the WinUAE behaviour. **[confirmed]** | 4.2.5 (um.txt 3549), 7.7.1, table 7-2 | dmu.sv:475-519 (xlate ignores FC) |
| A7 | Cache | Accesses locked through BUSCR use the data cache: a locked load hits with no bus cycle, a locked store does not push/invalidate a dirty line (memory ends up stale after CPUSHA).  TAS/CAS are right. **[confirmed]** | 5.2 (locked references bypass the cache) | dmu.sv:551 (m1_fast ignores lock for M_LD), WB store path |
| A8 | Bus | A bus error on beat 2-4 of a line read always raises an access error (data and instruction fills); the 060 faults only if the needed long word did not arrive (the line is not allocated). `errbeat` is produced and never used. **[confirmed]** | 7.9.1, 5.7.1, 8.4 | dmu.sv:1789-1796, fetch.sv:774-784, ap68060.sv:149 |
| A9 | Int | A level-7 request is recognised only on its edge; with IPL held at 7, lowering the mask (RTE/MOVE to SR) to below 7 must take it again. **[confirmed]** | 8.2.9, figure 8-3; WinUAE newcpu.cpp:4866 | backend.sv:1162-1164 |
| A10 | Int/bus | No double bus fault: an access or address error while processing an access error, address error or reset nests forever instead of halting (PST $1C, BR negated, bus three-stated). **[confirmed]** | 8.2.1, 8.2.2, 8.2.10, 7.9.3; WinUAE cpu_halt | backend.sv:1741 (only HALT halts) |
| A11 | Bus | A bus error on a push is dropped silently; the 060 takes an access error with FSLW.PBE (as for SBE). **[confirmed]** | 8.4.6 | FSLW_PBE never used |
| A12 | Cache | CINVP/CPUSHP of the data cache always compares 4K pages, ignoring TCR.P (8K). **[confirmed]** | CPUSH/CINV descriptions | dmu.sv:1690-1693 (fetch side is right) |
| A13 | MMU | FSLW.MA is not set on a bus error in the second part of a misaligned access (only on ATC faults). **[confirmed]** | 8.4.4.3, 8.4.5; WinUAE misalignednotfirst | dmu.sv:1409, 1523, 1794 |
| A14 | MMU/bus | Setting U in a table search is a plain read then a locked write, not a locked read-modify-write (another master's change can be lost); the write also asserts CIOUT, which table 7-2 says is negated for table searches. **[confirmed]** | 4.2.5 (um.txt 3518), 10.3, table 7-2 | dmu.sv:1920-1928, 2019-2026 |
| A15 | Snoop | The snoop queue drops an invalidation when its 8 entries are full (the code comment claims otherwise); sustained back-to-back snooped line DMA outruns the 6-clock lookup. (The bench fails the run if it happens; no test reaches it.) | 5.6, 7.12 | snoop.sv:91-97 |
| A16 | Bus | A push the snoop invalidated is still retried after a relinquish-and-retry, overwriting the other master's data. | 7.9.2 | biu.sv:368-384, dmu.sv:1853-1870 |
| A17 | Bus | A retried LOCKE write with BG and BGR negated gives up the bus, splitting the locked sequence. | 7.11.2, 2.3.8 | biu.sv:379-381, 462-465 |
| A18 | Bus | In a BTT-only arbitration system (BB pulled up), the core leaves alternate-master-explicit on BB "negated" right after the other master's TS and can start a cycle on top of it. | 7.11.2, 2.7.4 | biu.sv:344 |
| A19 | Bus | BR is negated while the core explicitly owns the bus even with requests pending. | 2.7.1, 7.11.1 | biu.sv:328 |
| A20 | Bus | Bus cycles continue after the LPSTOP broadcast (instruction prefetch; fetch `.stop` tied 0) and while RSTO is asserted (RESET does not wait for the store buffer, prefetch runs). **[confirmed]** | 7.8.2.1, 2.8.4 | ap68060.sv:351, backend.sv:1891 |

## B. Minor, or the manual and WinUAE disagree

- **FPU unimplemented `<op>.X -(An)` EA**: the SP expects An-12+8 (8 bytes "too big"), the core stacks An-12 (WinUAE's FPU-present path agrees with the core). Follow the SP? fpif.sv:464.
- **FPU**: FMOVECR with a nonzero EA field takes the unimplemented-instruction trap instead of F-line illegal; `#imm` source stacks EA 0 instead of the first immediate long word; enabled SNAN/OPERR on FMOVE OUT writes the default result first (manual says it does, the SP says not).
- **MMU**: FSLW.TTR not set on a bus error to a TTR-mapped access (manual 8.4.4.3/8.4.5 says set; WinUAE and t_eredir say not).  A write meeting W in a table descriptor keeps walking and sets U (4.2.6.3 says stop, WP, no history; WinUAE as the core).  A user write to an S+W page reports WP in the walk but SP from an ATC hit (inconsistent; WinUAE gives SP).  PLPA with DFC 2/6 uses the ITTs and the instruction ATC (WinUAE) vs the manual's data-side description.  Whether an SP fault in the walk loads an ATC entry (4.2.6.2 vs 8.4.5 step 5 vs WinUAE).  MOVE16 FA stacks the unaligned An (WinUAE aligns it).  SIZE in fetch and PLPA FSLWs is long (WinUAE word).
- **Int**: RTE to an odd PC stacks WinUAE's synthetic SR instead of the SR at the RTE's start (8.4); JMP to an odd address stacks opcode+2; a not-taken Bcc with an odd target still faults -- all WinUAE choices, deliberate, to be listed in STATUS.md.
- **Bus**: a dead bus clock between consecutive cycles (also inside a burst-inhibited line), BTT/BB one BCLK late, an extra BCLK entering explicit ownership (figures 7-13/7-17/7-43): performance only.  RTE stack reads allocate in the data cache (7.7.1/5.4.3 say not).  An instruction fetch can slip between a fill and its push (5.10).  LOCK stays asserted after a bus error/kill inside a locked sequence until the next transfer; LOCK/LOCKE are not negated a BCLK before being three-stated.  Reset: no 27-CLK internal hold, BB/TIP three-stated without the negated CLK.  IPL synchronised on BCLK, not CLK.  The fetch reads 0-$10 with TM=6 before the reset vectors.  An I-cache fill's critical word waits for the whole burst (performance).

## C. Test bench gaps

- The slave never answers in C2: "zero wait" still has a wait state, so TA at the end of C2 and zero-wait first burst beats are never exercised.
- TCI is never asserted (TCI handling and the push restore after it untested); no TEA or retry on acknowledge cycles; retries only on beat 0 (TA+TEA / TRA on beats 2-4 untested); the LPSTOP broadcast is answered like a read.
- The protocol checker does not check: A31-A4/attribute/R/W/SIZ stability through waits and bursts, TS width, TS while busy, TIP, D driven/released, LOCK/LOCKE placement, TT/TM/CIOUT/UPA/TLN, BR/BG/BB/BTT, or the core driving while the alternate master owns the bus.
- The alternate master uses only BB-protocol arbitration, at most 4 transfers per tenure: BTT-only systems, BR semantics and maximum-rate snooping are untested.
- No tests yet for: PFLUSHN/PFLUSHAN and the G bit, indirect descriptors/IL, instruction-side TWE, MDIS, misaligned bus errors (MA) -- the MMU audit's probes cover them and can become regression tests.
- Bench false failure: the PST check at tb_ap68060.sv:493 compares the registered `pst` with the combinational `lpstopped` and fires on LPSTOP wake-up.  `+biutrace` prints a duplicate client 2.

## D. Checked and correct (summary)

Integer: privilege checks for every privileged instruction, trap frames (CHK, DIV0, TRAPV, TRAPcc), format errors and RTE frame sizes, trace (format $2, T at start, TRAP not traced, RTE setting T), SR mask and stack switching, M bit, illegal opcodes (BGND, RTM, CALLM, CMP2 Dn, BKPT), MOVE16, vector 61 instructions (misaligned CAS, 64-bit MUL/DIV, CHK2/CMP2, MOVEP), MOVEM/LINK/PEA with A7 or the base register, STOP/LPSTOP wake-up, MOVEC masks, IACK/spurious/autovector, decode priorities.
MMU: TCR fields (P, NAD, NAI, FOTC/FITC, DCO/DUO/DWO, DCI/DUI), URP/SRP by FC2, TTR matching/S/W/priority, descriptor formats and invalid encodings, U/M per table 4-1, ATC FC2/G tags, all PFLUSH variants, PLPAR/PLPAW, MDIS, the restart model, FSLW for RMW, CAS, misaligned RMW, fetch TWE/PF, MOVES FC2/6, MOVE16.
FPU: pending arithmetic exceptions (format $0, next FP instruction), exception operands, BSUN, FMOVE OUT post-instruction exceptions and integer conversions, priorities, INEX2, control register writes, FPIAR, unimplemented instructions and packed frames/EAs, FScc EA, extended denormal source operand, the hardware/trap split (table 6-11), state frames.
Bus/caches: BS3-BS0 and misaligned splitting (table 7-3), both termination protocols, TBI/TCI/AVEC sampling, CLA, ignore states and SAS, write hold, retries, TAS/CAS locked RMW with push, IACK/BKPT/LPSTOP cycles, store buffer ordering, fill/push restore on failed or TCI fills, write-through and CI hits on dirty lines, CPUSH/CINV with DPI/CDIS, branch cache clearing, snoop TT1 filter and push-buffer snooping, implicit ownership, end of tenure, reset-mode latching, RSTO length.
