# AP68060 follow-up audit — CPU, MMU, FPU, and bus

Three defects reproduce in the final captured working tree: missed FPU underflow at a rounding boundary, a shortened reset pin-release interval with BCLK at half CLK, and premature SAS output enable on bus acquisition. The earlier FSLW.TTR disagreement with the manual also remains. No additional integer or MMU defect was established by this pass.

The six findings in the [earlier October 12 audit](AUDIT-2026-10-12.md) have received fixes. Their original reproductions now pass, including ordinary locked handoff and reset tenure termination. The reset finding below is a narrower timing case uncovered while checking those changes.

This audit did not edit production RTL or repository tests. It incorporated concurrent working-tree changes, then tested a final capture taken at **01:05:41 EDT on October 12, 2026**. Production RTL still matched that capture at the final comparison. The report describes simulation and source evidence, not certification or on-board validation.

## Fix status (2026-10-12)

All three findings were reproduced and are fixed, each with a regression test that fails without its fix:

| Finding | Fix | Regression test |
| --- | --- | --- |
| P2 underflow lost to a rounding carry | F_ROUND decides UNFL from the intermediate exponent before rounding (`e_w < emin`), as SoftFloat's 68K roundAndPack does; the existing UNFL path then traps (vector 51) and preserves the destination | t_fpu_frames 460-465 (the FSADD and FDADD boundary cases) |
| P2 reset between bus clock edges | RSTI is synchronized on bus clock edges (2.8.3, 7.13), and the BIU's reset branch acts only on bus clock edges, so the SAS/LOCK release lasts a whole bus clock | t_halt phases 2-3 reset at clocks 61 and 62 (both phases of a half-rate bus clock); the bench measures every SAS and LOCK/LOCKE release in CLKs |
| P3 SAS in the first TS clock | `sas_oe` uses `a_oe` delayed a bus clock, so SAS stays three-stated in the first TS clock of a tenure (2.4.3) | every run: the bench fails SAS driven in that clock |

The arithmetic pipeline (FPU_PIPE=1) takes only operands with exponents near 1.0, so tiny results always go through F_ROUND; the fix covers both settings.

## Findings

### P2 — Underflow disappears when rounding carries into the normal range

In [ap040_fpu.v:2378](/home/adam/AP68060/rtl/fpu/ap040_fpu.v:2378), `unf` tests the exponent *after* accounting for a rounding carry. An intermediate result immediately below the smallest normal value can round into the normal range and escape underflow detection. The FPU sets INEX2, updates the destination, and does not raise the required nonmaskable underflow exception.

MC68060UM §6.4.1, pp. 6-15–6-16, and §6.6.5, pp. 6-30–6-31, require checking the intermediate result for underflow before rounding. A floating-point register destination remains unchanged on this exception. WinUAE's [68K roundAndPackFloatx80](/home/adam/WinUAE/softfloat/softfloat.cpp:1398) raises underflow from the unrounded exponent; its [exception handling](/home/adam/WinUAE/fpp.cpp:649) treats UNFL as nonmaskable for the hardware subset.

Two complete CPU reproducers use round-to-nearest with exception-enable bits clear:

| Instruction | FP0 before | FP1 | Expected | Observed |
| --- | --- | --- | --- | --- |
| `FSADD.X FP1,FP0` | `2^-152` | `-2^-126` | UNFL + INEX2; FP0 preserved; vector 51 before the next FP instruction | INEX2 only; FP0 becomes `-2^-126`; no vector 51 |
| `FDADD.X FP1,FP0` | `2^-1077` | `-2^-1022` | Same exception and preservation rules | INEX2 only; FP0 becomes `-2^-1022`; no vector 51 |

Both fail with the ordinary bus and with half-rate BCLK, waits, and burst inhibit. A nearby case whose rounded result remains below the normal boundary takes vector 51 and passes. The [WinUAE reference calculation](/home/adam/AP68060/obj/audit-2026-10-12-followup/fpu-tiny-reference.log) independently reports UNFL and INEX for both boundary cases.

The broader arithmetic comparison finds five instances of this status error among 100,000 vectors, with two destination-value mismatches caused by missing preservation. The same five cases fail with `FPU_PIPE=0` and `FPU_PIPE=1`. No other result or EXC-status differences occur in that sample.

Determine underflow from the unrounded intermediate exponent, and preserve the unrounded state needed by the exception path. Merely setting a status bit after writeback would leave the destination and exception behavior wrong.

Evidence: [single program](/home/adam/AP68060/obj/audit-2026-10-12-followup/fpu_tiny_round_single.s), [double program](/home/adam/AP68060/obj/audit-2026-10-12-followup/fpu_tiny_round_double.s), [control](/home/adam/AP68060/obj/audit-2026-10-12-followup/fpu_tiny_control.s), [CPU results](/home/adam/AP68060/obj/audit-2026-10-12-followup/focused-final.log), [sequential arithmetic results](/home/adam/AP68060/obj/audit-2026-10-12-followup/fpu-allops0.log), [pipelined arithmetic results](/home/adam/AP68060/obj/audit-2026-10-12-followup/fpu-allops1.log).

### P2 — Reset between BCLK edges shortens the driven LOCK release interval

The revised reset path correctly generates BTT and negates BB/TIP before releasing them. However, [ap68060_biu.sv:269](/home/adam/AP68060/rtl/ap68060_biu.sv:269) starts reset handling on any CLK edge. It immediately releases the address bus and drives LOCK negated, preserving its enable through `lk_hold`. [Line 327](/home/adam/AP68060/rtl/ap68060_biu.sv:327) clears that hold at the next BCLK edge.

With BCLK at half CLK, recognition on the intervening processor-clock edge gives LOCK only **one CLK, or half a BCLK**, of driven negation. The full core can present this phase: its [reset synchronizer](/home/adam/AP68060/rtl/ap68060.sv:88) advances on every CLK without qualifying the result by `bclk_en`.

MC68060UM §2.3.7 requires an asserted LOCK to be driven negated for a full BCLK before being three-stated, one BCLK after the address bus is idled. §2.8.3 describes reset recognition at a clock-enabled edge; §7.11.2 requires reset from explicit ownership to pass through end tenure. SAS uses the same shortened hold in this path, against §2.4.3's full-BCLK release interval.

The isolated BIU probe completes a locked read and asserts reset before a non-BCLK CLK edge, holding reset for 16 BCLK periods. LOCK becomes negated on the first CLK and loses its output enable on the second. The full-rate reset control and ordinary BGR handoff both pass. This establishes a pin-timing violation; a downstream board failure was not measured.

Align the reset-entry release sequence with BCLK, or otherwise retain the outputs for the required complete interval. Keep the corrected BTT and BB/TIP behavior.

Evidence: [probe](/home/adam/AP68060/obj/audit-2026-10-12-followup/tb_bus_release_final.sv), [half-rate reset failure](/home/adam/AP68060/obj/audit-2026-10-12-followup/bus-reset-half-final.log), [full-rate reset control](/home/adam/AP68060/obj/audit-2026-10-12-followup/bus-reset-final.log), [handoff control](/home/adam/AP68060/obj/audit-2026-10-12-followup/bus-handoff-final.log).

### P3 — SAS drives during the first TS clock of a newly acquired tenure

[ap68060_biu.sv:673](/home/adam/AP68060/rtl/ap68060_biu.sv:673) assigns `sas_oe = a_oe || sas_hold`. Acquiring explicit ownership asserts `a_oe` together with TS, so SAS is immediately driven negated during C1.

MC68060UM §2.4.3, pp. 2-8–2-9, explicitly requires SAS to remain three-stated when newly gaining ownership and asserting TS, until the following BCLK period. The probe observes `sas_oe=1` during that first TS clock; its C2 check passes. This is an output-enable conformance defect, not a demonstrated loss of transferred data.

Delay the initial SAS enable until the clock after the first TS of a tenure, while preserving normal back-to-back transfers and the newly added delayed release.

Evidence: [SAS acquisition result](/home/adam/AP68060/obj/audit-2026-10-12-followup/bus-sas-start-final.log), using the same isolated BIU probe.

## Previous findings and reference disagreements

| Earlier finding | Recheck against final capture |
| --- | --- |
| LPSTOP broadcast TEA incorrectly causes an access error | Fixed; independent TEA injection passes |
| Later beat of a misaligned cache fill omits FSLW.MA | Fixed; burst and burst-inhibited probes pass |
| RTE stack reads allocate cache lines | Fixed; ordinary load after RTE now performs its own allocation |
| FMOVE OUT overflow always sets INEX2 | Fixed; focused program and earlier 12,000-vector oracle pass |
| Locked handoff omits driven negation | Fixed for ordinary handoff; separate output enable is exercised |
| Reset loses BTT and BB/TIP release sequencing | Original failure fixed; half-rate release timing remains as described above |

**FSLW.TTR remains a manual disagreement.** A bus error through DTT0 still produces FSLW `$01050020`, with TTR clear. The manual's §8.4.4.3 and §8.4.5 require it set for this case. WinUAE and the repository's `t_eredir` expectations agree with the core. The [probe](/home/adam/AP68060/obj/audit-2026-10-12-followup/ttr_bus_error.s) still fails the manual expectation in both bus configurations. This is kept separate from findings supported by both references.

The simultaneous SP/WP priority and SP-fault ATC-allocation ambiguities recorded in the earlier report remain unresolved. This audit does not promote them into confirmed defects.

## Validation and scope

| Check | Result |
| --- | --- |
| Final production program bench, default parameters | **280/280 pass**: [log](/home/adam/AP68060/obj/audit-2026-10-12-followup/regression-final.log) |
| Same suite with FPU_PIPE, LOOP_PRED, EARLY_RES all enabled | **280/280 pass**: [log](/home/adam/AP68060/obj/audit-2026-10-12-followup/regression-options.log) |
| WinUAE integer corpus, Basic/Default/AE, first slice, up to 300 records per slice | **378/380 slices pass**; 107,139 completed rounds: [log](/home/adam/AP68060/obj/audit-2026-10-12-followup/cputest.log) |
| Prior independent FPU oracle | **12,000/12,000 values and EXC bytes match**: [log](/home/adam/AP68060/obj/audit-2026-10-12-followup/fpu-oracle.log) |
| Expanded exponent-boundary, infinity, NaN, arithmetic and output-conversion sample | **60,000/60,000 values and EXC bytes match**: [log](/home/adam/AP68060/obj/audit-2026-10-12-followup/fpu-edge-oracle.log) |
| 28 arithmetic opmodes, rounding modes/precisions and varied exception enables | **5/100,000 mismatches**, all the underflow finding; same outcome for both FPU pipeline settings |
| Earliest legal bus responses | **1,536/1,536 pass**: [log](/home/adam/AP68060/obj/audit-2026-10-12-followup/bus-minlat-final.log) |
| Focused fault, MMU, FPU and pin probes | Findings and passing controls above: [combined results](/home/adam/AP68060/obj/audit-2026-10-12-followup/rerun-final.log) |

The two corpus exceptions are `Basic--MOVEC2--0001` and `Default--MOVEC2--0001`: ten execution timeouts and two register mismatches. They exercise the documented traced-sequence oracle differences. A [trace of Basic round 234](/home/adam/AP68060/obj/audit-2026-10-12-followup/corpus-trace/logs/Basic--MOVEC2--0001.log) shows TCR written with `$7FFF`, then trace-frame writes faulting under default write protection. The BUSCR cases expect `$A0000000` but see `$50000000` after intervening trace exceptions move L/LE into SL/SLE. These are not counted as newly identified core defects, and the corpus run is not reported as an unconditional pass. The corpus used `/home/adam/ap68060-work/cputest060b/data`, whose generation configuration specifies `cpu=68060` and `cpu_unimplemented=0`. Replay used the initial capture; CPU/MMU/FPU RTL was unchanged afterward, while subsequent bus changes were checked by the final program and BIU regressions.

The minimum-latency bus bench covers reads/writes, single/line transfers, all four line starting offsets, native and 68040 retry protocols, first-sample retries, CLA, extra write hold, ignore counts 0/1/15, and both clock ratios. It drives successful responses at the first legal sampling edge, including consecutive burst beats. This closes the earlier basic C2-response gap for the BIU; it does not replace whole-core testing with a minimum-latency memory model.

MMU review and tests cover descriptor walks and history updates, protection paths, alternate function-code spaces, PLPA, TTRs, ATCs and PFLUSH variants. The additional global-entry flush, TCI read, IACK TEA and BKPT TEA probes continue to pass. Integer replay supplements the ordinary instruction/exception regressions. Neither constitutes exhaustive coverage of every encoding, MMU/cache race, or fault boundary.

The arithmetic oracle compiles the local WinUAE SoftFloat implementation directly. Its adapter applies the 68060 rule that faulting arithmetic preserves the register destination. It checks result bits and EXC status; the complete CPU probes establish the underflow exception-delivery failure. It does not exhaustively check every FPSR field, unsupported operand encoding, software-support-package path, or dependent instruction stream. WinUAE is not used as a pin-waveform oracle. Synthesis, timing closure, metastability, and physical hardware behavior were not tested.

## Reproducible inputs

- AP68060 HEAD: `739d45b9fca1a80dd588a042c9d6a62d3529e63c`, plus the [captured working-tree patch](/home/adam/AP68060/obj/audit-2026-10-12-followup/working-tree-final.patch).
- Final [source capture](/home/adam/AP68060/obj/audit-2026-10-12-followup/source) and [RTL SHA256 manifest](/home/adam/AP68060/obj/audit-2026-10-12-followup/rtl-sha256-final.json). The captured program bench adds audit-only fault injection; production tests were untouched by this audit. Initial sources and intermediate bus results are retained separately and do not define the final findings.
- WinUAE: `7d84eae9aecaf98bcfee1e5cf2c765d2fbfe17b4`, clean working tree.
- [MC68060UM.pdf](/home/adam/Downloads/MC68060UM.pdf): SHA256 `313fd672dfac8f1d1bb580ad61e8e1583a38b07fb4e2891aff7f29ef49d3ef40`. References use printed section/page numbers.

Rebuild and rerun the focused reproductions:

```sh
bash obj/audit-2026-10-12-followup/run_audit.sh
```

The runner returns nonzero while these findings reproduce and parses printed FAIL results rather than relying on Verilator's exit status. All captured sources, vectors, generators, tests, and logs are under [obj/audit-2026-10-12-followup](/home/adam/AP68060/obj/audit-2026-10-12-followup). This artifact directory is ignored by Git and must be preserved separately when moving the report.
