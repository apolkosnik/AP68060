# AP68060 core audit — October 12, 2026

Three functional defects reproduce in the current core: stale data-cache hit metadata can lose a retired store, MOVE16 invalidates retained data while caching is disabled, and completed TCI instruction fills remain usable in two cases. These are gaps in the recent cache fixes. No additional integer, MMU, FPU, or bus defect was established by this pass. The previously reported FSLW.TTR disagreement with the manual remains.

This audit compares the CPU, MMU, FPU, caches, and bus against the local WinUAE checkout and MC68060 user manual. It changes no production RTL or repository tests. The final source capture includes the concurrent backend and FP-interface edits. References to manual pages use printed page numbers.

## Fix status (2026-10-12)

All three findings were reproduced and are fixed, each with a test that fails without its fix:

| Finding | Fix | Regression test |
| --- | --- | --- |
| P1 stale hit records after an engine invalidation | The repair of the stage records moved from the snoop path to the valid/dirty write port: whatever clears a way's valid bit (snoop, an inhibited or locked access's push, CINV/CPUSH) clears every matching hit -- DC2/EX/WB records, those loaded in that clock (from DC1's lookup and the stage before), both parts of splits, and the engine's own | t_cache060 23-24 (an inhibited write, then at once a cached load and a cached store, no MOVEC between) |
| P2 MOVE16 invalidates with the cache disabled | The MOVE16 destination's invalidation is gated by the data cache enable (CACR.EDC, not CDIS) | t_cache060 25-26 (EDC clear, CDIS) |
| P2 TCI line reused sequentially, completion/redirect race | A TCI fill is never kept: at its completion the long word being fetched goes straight to the decoder (unless a redirect takes the stream in that clock), and the next one is fetched anew | `tb/tb_fetch_tci.sv` (`tb/run_fetch_tci.sh`, run by `tb/run_tests.sh`): the audit's fetch-unit probe, made to answer every request the unit makes; the old fetch unit fails the race and sequential cases, the new one passes all five. t_cache060 27 also runs a TCI line sequentially in the whole CPU (there the pipeline's redirects already hid the case). |

The earlier cache audit's fix table overstated the inhibited-write and TCI fixes, as this audit says; both are complete now.

## Findings

### P1 — Internal cache invalidation leaves younger accesses with stale hit metadata, losing stores

The inhibited-store path now correctly pushes and invalidates its matching line. However, [E_PUSHV_W](/home/adam/AP68060/rtl/ap68060_dmu.sv:2002) changes the cache's valid bits without clearing matching hit information already captured by younger pipeline records. The repair of `m2`/`m3`/`m4` hit records at [line 2449](/home/adam/AP68060/rtl/ap68060_dmu.sv:2449) only runs for snoop invalidations.

A younger load can consequently reuse `m2.x` at [E_S_START](/home/adam/AP68060/rtl/ap68060_dmu.sv:1282), skip a fresh lookup, and read the invalidated line's old data. A younger copyback store can reach [wb_fast](/home/adam/AP68060/rtl/ap68060_dmu.sv:892) with a stale hit and write the invalid line without allocating it. A later CPUSH does not find that line, so the retired store is lost.

The complete-CPU reproducer maps user data as copyback and supervisor data as precise cache inhibited, using DTT0/DTT1 and SFC/DFC=1:

1. Memory at `$9000` starts at `$11112222`; a user-space MOVES store leaves `$AAAABBBB` dirty in the cache.
2. A supervisor inhibited store writes `$CCCCDDDD` to the same physical address. The trace confirms that the old dirty line is pushed first, then the new value is written externally.
3. An immediately following cacheable MOVES load incorrectly returns `$AAAABBBB` without a refill.
4. In the store variant, a following MOVES store of `$EEEEFFFF` retires. After `CPUSHA DC`, memory still contains `$CCCCDDDD`.

In the [store-loss trace](/home/adam/AP68060/obj/audit-core-2026-10-12/dcache_ci_store_loss.log), the inhibited write completes at cycle 608, the newer store retires at cycle 612, and the read after CPUSHA returns `$CCCCDDDD` at cycle 1410. The [pipeline trace](/home/adam/AP68060/obj/audit-core-2026-10-12/dcache-ci-pipeline-trace.log) shows the younger request retaining `x.hit=1` through the invalidation and older store's completion. Inserting a serializing MOVEC between the accesses makes both controls pass.

MC68060UM §5.4.1.2 and §5.4.2, p. 5-8, require allocation on an ordinary copyback write miss and coherent invalidation for inhibited accesses. WinUAE's [write_dcache040](/home/adam/WinUAE/newcpu.cpp:9943) pushes/invalidates the inhibited hit and evaluates subsequent accesses against the updated valid bits; its [copyback-miss path](/home/adam/WinUAE/newcpu.cpp:9960) allocates a new line.

The load failure reproduces with both precise and imprecise inhibited modes, and with clean or dirty initial lines. The lost-store case is demonstrated with precise mode. Both ordinary and half-rate/waited/burst-inhibited bus configurations fail. Selected cases also fail with FPU_PIPE, LOOP_PRED, and EARLY_RES enabled. The existing `t_cache060` inhibited-write test changes the TTR with MOVEC before its cacheable read, hiding this case.

Repair or revalidate in-flight hit records when the engine invalidates a line, including records advancing in that cycle and both parts of split accesses. Merely clearing the cache-array valid bit is insufficient.

Evidence: [load probe](/home/adam/AP68060/obj/audit-core-2026-10-12/dcache_ci_write_40.s), [load control](/home/adam/AP68060/obj/audit-core-2026-10-12/dcache_ci_serial_control.s), [lost-store probe](/home/adam/AP68060/obj/audit-core-2026-10-12/dcache_ci_store_loss.s), [store control](/home/adam/AP68060/obj/audit-core-2026-10-12/dcache_ci_store_loss_control.s).

### P2 — MOVE16 destination invalidation ignores the disabled data cache

The [MOVE16 destination branch](/home/adam/AP68060/rtl/ap68060_dmu.sv:1445) tests `e_x.hit` without checking `dc_en`. It clears the matching line's valid and dirty bits even when CACR.EDC is clear or CDIS is asserted. The enable guards added to other read/write paths do not cover this branch.

MC68060UM §5.3, pp. 5-6–5-7, requires accesses while CDIS is recognized to bypass the caches and leave their contents unchanged. WinUAE's [MOVE16 write path](/home/adam/WinUAE/cpummu.cpp:1367) invokes physical writes with caching inhibited; [write_dcache040](/home/adam/WinUAE/newcpu.cpp:9933) bypasses the cache before lookup when EDC is clear.

The full-CPU probe leaves `$AAAABBBB` dirty at `$9000`, disables caching, and uses MOVE16 to copy `$CCCCDDDD` there from `$A000`. After caching is enabled again, a user cacheable read should still see the retained `$AAAABBBB`. Instead, it refills from memory and returns `$CCCCDDDD`: MOVE16 discarded the retained dirty line. The CDIS variant waits sixteen NOPs after asserting the pin. Both disable mechanisms fail with both bus configurations and both sets of core parameters.

The enabled-cache MOVE16 control correctly invalidates its destination. An ordinary longword store while EDC is clear preserves the retained line and passes. Gate MOVE16's normal-access invalidation by the effective cache enable; explicit maintenance and snooping still operate while disabled.

Evidence: [EDC probe](/home/adam/AP68060/obj/audit-core-2026-10-12/move16_disabled_cacr.s), [CDIS probe](/home/adam/AP68060/obj/audit-core-2026-10-12/move16_disabled_cdis.s), [EDC trace](/home/adam/AP68060/obj/audit-core-2026-10-12/move16_disabled_cacr.log), [ordinary-store control](/home/adam/AP68060/obj/audit-core-2026-10-12/move16_disabled_single_control.s).

### P2 — Completed TCI instruction fills survive sequential requests and a completion/redirect race

The [fill completion path](/home/adam/AP68060/rtl/ap68060_fetch.sv:786) retains the last-fill buffer unless `stale` was already set. The new [redirect guard](/home/adam/AP68060/rtl/ap68060_fetch.sv:900) only drops it when the previous value of `lb_tci` is set. Two cases remain:

| Request after a TCI fill | Required | Observed |
| --- | --- | --- |
| Next sequential longword requested after burst completion | New transfer | Completed line buffer reused |
| Redirect to the same line on the clock asserting `b_done` for its first TCI fill | Discard old stream; new transfer | Completion sees old `stale=0`, redirect sees old `lb_tci=0`; old line remains valid |

MC68060UM §5.7.1, p. 5-12, permits sequential access to this buffer only until the last burst longword arrives, then requires its contents to be discarded. Retaining it for the rest of the sequential stream is not sufficient.

The isolated fetch probe supplies `$7001` in the first burst and `$7002` if another transfer is requested. Both failing cases return `$7001` with only one bus request. A later, noncoincident redirect now passes, returning `$7002` after a second request. Explicit invalidation also makes the sequential case pass; the ordinary cacheable control correctly retains `$7001`.

Deliver the outstanding fetch's requested data separately from retention of the completed line, and account for a redirect asserted on the completion clock. Simply clearing `lb_v` unconditionally would leave the current delivery path repeatedly missing. These failures are demonstrated at the production fetch-module interface; this audit does not claim a complete-CPU reproducer for these two timing cases or use WinUAE as a physical TCI-pin oracle.

Evidence: [fetch probe](/home/adam/AP68060/obj/audit-core-2026-10-12/tb_fetch_cache.sv), [completion race](/home/adam/AP68060/obj/audit-core-2026-10-12/fetch-completion-race.log), [sequential failure](/home/adam/AP68060/obj/audit-core-2026-10-12/fetch-sequential.log), [redirect control](/home/adam/AP68060/obj/audit-core-2026-10-12/fetch-redirect.log).

## Earlier findings and reference disagreements

The original inhibited-write **external writeback order** is fixed: the previous lost-write probe now passes. Its immediately following cacheable-load probes still fail for the separate pipeline-metadata reason above. Ordinary inhibited reads while caching is disabled and dirty-line preservation across reset now pass. The earlier TCI redirect probe passes, with the two remaining cases described above. Thus the prior cache audit's fix-status table overstates closure of these paths.

The earlier FPU underflow-rounding boundary programs, overflow-status program, and RTE no-allocation probes pass. The BIU probes for locked handoff, reset release at both clock ratios, and SAS acquisition also pass.

Two previously reported reference disagreements still reproduce:

- **FSLW.TTR:** a bus error through DTT0 leaves TTR clear, contrary to MC68060UM §§8.4.4.3–8.4.5, pp. 8-24–8-25. WinUAE agrees with the core. The [probe](/home/adam/AP68060/obj/audit-core-2026-10-12/ttr_bus_error.s) fails the manual expectation in both bus configurations.
- **Inhibited instruction-cache hits:** the [probe](/home/adam/AP68060/obj/audit-core-2026-10-12/icache_ci.s) still executes values 1, 2, 1 as an inhibited ITT mapping is enabled and removed. WinUAE invalidates the matching I-cache entry; the core retains it. The manual's explicit inhibited-hit invalidation language concerns data-cache entries, so this remains a reference disagreement rather than an additional confirmed manual violation.

The previous reports' CUBC supervisor-entry invalidation, single-issue execution, missing FSLW.BPE generation, and critical-word fetch latency differences remain. Half-cache geometry, clean-line DPI semantics, and simultaneous SP/WP-fault behavior remain unresolved. This pass does not promote those ambiguities into new defects or change the core to match known WinUAE cache-maintenance discrepancies. See the [cache audit](/home/adam/AP68060/doc/AUDIT-CACHE-2026-10-12.md) and [previous core audit](/home/adam/AP68060/doc/AUDIT-2026-10-12-followup.md) for their evidence.

## Validation

| Check against captured sources | Result |
| --- | --- |
| Existing complete program suite; default parameters; ten bus configurations | **290/290 pass** — [log](/home/adam/AP68060/obj/audit-core-2026-10-12/regression-final.log) |
| Same suite with FPU_PIPE, LOOP_PRED, EARLY_RES enabled | **290/290 pass** — [log](/home/adam/AP68060/obj/audit-core-2026-10-12/regression-options.log) |
| Cache-maintenance matrix; 288 programs, two bus configurations | **576/576 pass**, both default and all-options builds — [default](/home/adam/AP68060/obj/audit-core-2026-10-12/matrix-results.log), [options](/home/adam/AP68060/obj/audit-core-2026-10-12/matrix-results-options.log) |
| WinUAE SoftFloat arithmetic comparison; 28 hardware opmodes | **100,000/100,000 match**, separately for FPU_PIPE=0 and 1 — [sequential](/home/adam/AP68060/obj/audit-core-2026-10-12/fpu0.log), [pipelined](/home/adam/AP68060/obj/audit-core-2026-10-12/fpu1.log) |
| Same vectors with varied initial FPSR; accrued exceptions, nontrapping arithmetic CC, quotient preservation | **No mismatches**, both FPU pipeline settings — [sequential](/home/adam/AP68060/obj/audit-core-2026-10-12/fpu-status0.log), [pipelined](/home/adam/AP68060/obj/audit-core-2026-10-12/fpu-status1.log) |
| Earliest legal bus responses | **1,536/1,536 pass** — [log](/home/adam/AP68060/obj/audit-core-2026-10-12/bus-minlat.log) |
| Focused full-CPU and fetch probes, including controls | **45 pass, 20 fail**; 16 failures exercise the three findings, four the two reference disagreements — [results](/home/adam/AP68060/obj/audit-core-2026-10-12/focused-results.log) |
| WinUAE cputest corpus, slice 0001, up to 300 records per slice | **684/711 slices pass**; **198,739 completed rounds**, **1,846 mismatches**, **10 harness errors** — [results](/home/adam/AP68060/obj/audit-core-2026-10-12/cputest-final.log) |

The corpus uses `/home/adam/ap68060-work/cputest060b/data`, generated for the 68060 with `cpu_unimplemented=0`. It covers Basic, Default, AE, FFEXT_SRC/DST, BasicFPU, PackedFPU, IRQ, ODD_EXC, ODD_IRQ, and ODD_STK. This is a bounded sample, not a full corpus sweep. The failing slices are accounted for as follows; they are not silently excluded from the totals:

| Failing slices | Mismatches / harness errors | Evidence and classification |
| --- | --- | --- |
| Basic/Default MOVEC2: 2 | 2 / 10 | The same documented traced-sequence differences: BUSCR expected `$A0000000`, observed `$50000000` after trace exceptions move L/LE into SL/SLE; TCR.DWO causes trace-stack writes to fault in the timeout cases. |
| PackedFPU single-precision sources: 14 | 1,104 / 0 | Unsupported-data exception frame format: WinUAE expects format 3, core uses format 0 as specified for these operands by §6.5.2, Table 6-10. |
| ODD_EXC: 7; ODD_IRQ: 4 | 740 / 0 | Trace/vector sequence expectations. The actual first trace fetches an odd vector and raises address error before the next instruction; the oracle expects later sequence effects. |

A new [ODD_IRQ trace](/home/adam/AP68060/obj/audit-core-2026-10-12/corpus-trace/logs/ODD_IRQ--SWAP.W--0001.log) shows SWAP retiring at `$43900000`, vector 9 with next PC `$43900002`, then vector 3 with PC `$24` for the odd trace vector. The [ODD_EXC DIVS trace](/home/adam/AP68060/obj/audit-core-2026-10-12/corpus-trace/logs/ODD_EXC--DIVS.W--0001.log) shows the same transition. This matches §§8.2.2 and 8.2.6, rather than the oracle's address-error PC `$10` and separately expected trace. The existing MOVEC trace analysis is linked in the previous core audit. No additional mismatch category was found in this sample.

The FPU adapter compiles the unmodified local WinUAE SoftFloat implementation and applies destination preservation for killing 68060 exceptions. It samples all rounding modes and precisions, exponent boundaries, zeros, infinities, NaNs, and exception masks. The FPSR supplement derives accrued-status expectations from §6.1.3.4; it does not compare CC on killing exceptions. The pipelined run accepts 1,327 operations through the arithmetic pipeline; the remaining vectors use the ordinary path. It is not exhaustive coverage of dependent FPU streams, unsupported operands, or software support-package behavior.

The program suite covers instruction execution and exception delivery, MMU walks/history/protection, alternate function codes, PLPA, ATCs/PFLUSH, cache modes, snooping, store/push buffers, split accesses, bus errors, interrupts, arbitration, reset, and retry modes. The maintenance matrix crosses CINV/CPUSH, line/page/all scopes, cache selections, dirty-line DPI, disabled/CDIS/no-allocation conditions, and 4K/8K pages; paging translation is disabled in that matrix. The minimum-latency bus probe exercises single/line reads and writes, all line offsets, both retry protocols, CLA, extra write hold, ignore counts 0/1/15, and full/half BCLK.

Passing these tests does not exhaust every encoding, concurrency race, or fault boundary. No synthesis, timing closure, board testing, or cycle-equivalence certification was performed.

## Captured inputs and reproduction

- AP68060 HEAD: `e74c38f9885dfe5bd56fbd717b40ea7674124301`, plus the backend and FP-interface changes in the [final working-tree patch](/home/adam/AP68060/obj/audit-core-2026-10-12/working-tree-final.patch).
- Final source capture: **2026-10-12 12:21:34 UTC**. [Sources](/home/adam/AP68060/obj/audit-core-2026-10-12/source), [RTL SHA256 manifest](/home/adam/AP68060/obj/audit-core-2026-10-12/rtl-final-sha256.json). Production RTL matched the final capture at the final comparison. Initial results are retained separately; the final program and corpus runs include both concurrent edits. FPU and BIU unit sources did not change between captures.
- WinUAE HEAD: `7d84eae9aecaf98bcfee1e5cf2c765d2fbfe17b4`, clean working tree.
- [MC68060UM.pdf](/home/adam/Downloads/MC68060UM.pdf): SHA256 `313fd672dfac8f1d1bb580ad61e8e1583a38b07fb4e2891aff7f29ef49d3ef40`.

Rebuild the captured core and rerun the focused reproductions:

```sh
python3 obj/audit-core-2026-10-12/run_audit.py
```

Use `--no-build` to reuse the binaries. The runner checks printed FAIL results and returns nonzero while the findings and reference disagreements reproduce. All focused programs are reassembled. It uses the unmodified captured program bench and the independent fetch-interface bench.

Run the maintenance matrix with the captured default build:

```sh
python3 obj/audit-core-2026-10-12/run_matrix.py
```

The [artifact directory](/home/adam/AP68060/obj/audit-core-2026-10-12) contains sources, vector generator, vectors, tests, and logs. It is ignored by Git and must be preserved separately when sharing this report.
