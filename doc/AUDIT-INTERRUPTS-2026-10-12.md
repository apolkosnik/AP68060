# Interrupt audit 2026 10 12

**One new defect is confirmed: IPEND negates before the interrupt acknowledge cycle.** It reproduces at every interrupt level, with autovectors, device vectors, spurious responses, and retries. Handler execution and return pass the focused architectural checks. The previous finding about the first handler instruction masking a deferred interrupt is fixed in the audited revision.

The audit covers interrupt input qualification, priority and masking, acknowledgement, handler entry and nesting, stack switching, STOP/LPSTOP wakeup, and RTE. Audited AP68060 revision: `0cf0a79f08783c347271a3329557f36c09cd01b2`. References: [MC68060UM](/home/adam/Downloads/MC68060UM.pdf) and `/home/adam/WinUAE`, revision `7d84eae9aecaf98bcfee1e5cf2c765d2fbfe17b4`. Production RTL and test sources were not changed by this audit.

## Fix status (2026-10-12)

Fixed. The backend keeps a separate `iack_hold`, set when an interrupt is accepted and cleared by the DMU's new `iack_end` pulse at the terminal response of the interrupt acknowledge cycle (after any retries; another exception, a halt or a reset also clear it). IPEND is driven from `irq_pend || iack_hold`; whether to take another interrupt is still decided by `irq_pend` alone, so the first-instruction masking fix is untouched.

The production bench now checks on every run that IPEND is asserted at the TS and at the terminal response of every interrupt acknowledge cycle, and its slave may retry interrupt acknowledges at random in the retry configurations (LPSTOP and breakpoint acknowledges, TM 0, are still never retried). On the previous RTL the check fails in `t_irq7` and `t_exceptions` in every configuration.

## P2 IPEND negates before interrupt acknowledgement

The [external pending indication](/home/adam/AP68060/rtl/ap68060_backend.sv:2069) is assigned directly from `irq_pend`. At interrupt acceptance, the backend raises SR's interrupt mask, [clears the pending claim and consumes a level 7 edge](/home/adam/AP68060/rtl/ap68060_backend.sv:1940). The accepted request consequently stops satisfying the [pending expression](/home/adam/AP68060/rtl/ap68060_backend.sv:1242). The top level then [negates IPEND on the following processor clock](/home/adam/AP68060/rtl/ap68060.sv:134), although the IACK bus transaction has not started.

MC68060UM §7.8.1, p. 7-34, requires IPEND to negate **after the interrupt acknowledge bus cycle**. Section 2.9.2 describes it as status available to other bus masters. This is an externally visible status-timing violation. The tests still execute the correct interrupt handler and return correctly; they do not demonstrate a lost interrupt or a downstream bus-master failure.

The [level 2 autovector trace](/home/adam/AP68060/obj/audit-interrupts-2026-10-12/ipend-auto-trace.log) records:

| Event | Processor clock |
| --- | ---: |
| Interrupt accepted at the instruction boundary | 256 |
| IPEND negates after its brief assertion | 258 |
| IACK begins, with IPEND already negated | 270 |
| Slave supplies the terminal IACK response | 272 |

The independent assertion fails in **40/40 runs**, with IPEND negating **14–46 processor clocks before the terminal response**. Coverage comprises levels 1–7 with two forced retries, plus separate autovector, supplied-vector and TEA/spurious cases. Each runs in both MC68040 and native MC68060 termination modes, at full and half bus-clock rates; the slower cases include waits and acknowledge-ignore states. All 40 programs subsequently print an architectural PASS, so a runner that only checks the final PASS would miss the defect.

For example, the [level 7 native-mode trace](/home/adam/AP68060/obj/audit-interrupts-2026-10-12/extra/ipend-level7-half060.log) shows IPEND negating at clock 184, IACK attempts at 197, 205 and 215, and the terminal response at 220. Both retries occur while IPEND is already negated.

Keep an accepted-interrupt status indication asserted through the terminal IACK response, including retries. Keep that indication separate from the mask-qualified claim used to decide whether to take another interrupt. Reusing the old claim through handler entry would risk undoing the first-instruction masking fix.

## Architectural checks

The full production regression passes **363/363 runs**: 33 programs across 11 bus configurations. The fetch-TCI checks also pass. Full logs were checked for any FAIL, not only their final result. This includes `t_xnest`, `t_irq7`, `t_iplglitch`, exception/trace priority tests, and `t_snirq`, which overlaps interrupts with instruction-cache snoop invalidations.

The audit adds **2,180 passing architectural runs** and the **40 failing IPEND checks** described above:

| Focused coverage | Runs | Result |
| --- | ---: | --- |
| All 8 IPL values against all 8 masks, waking STOP or remaining masked until a separate level 7 request | 128 | Pass |
| Levels 1–7, all S/M combinations, all four byte offsets of ISP, store buffering on/off; frame SR/PC/vector, handler SR, ISP/USP and RTE restoration | 448 | Pass |
| All 256 supplied vector bytes with relocated VBR; format 0 and mask determined by IPL | 512 | Pass |
| Higher requests arriving during level 2 entry; first handler instruction masks levels 3/5/6, permits level 7, or allows nesting immediately after its NOP | 16 | Pass |
| Previous handler-mask probes and selected IACK response probes | 24 | Pass |
| All IPL/mask combinations waking LPSTOP | 128 | Pass |
| All transitions from levels 0–6 to levels 0–7, with each of 8 one-clock pin-skew masks | 896 | Pass |
| Held levels 1–7 retrigger at RTE to a lower mask, before resumed code executes | 14 | Pass |
| A new level 7 edge nests inside a level 7 handler at mask 7 | 2 | Pass |
| Two forced IACK retries, followed by autovector, supplied vector, or AVEC overriding nonzero bus data | 12 | Pass |
| IPEND lifetime across acknowledgement | 40 | Fail |

The first eight rows use full BCLK and half BCLK with native termination, waits and ignore states; the original matrix additionally enables random retries for ordinary bus transfers. The explicit IACK retry probes are necessary because the production slave excludes acknowledge cycles from its random retry generator. The audit slave forces exactly two IACK retries in each of 40 runs: 12 architectural controls and 28 IPEND cases. Both termination protocols complete correctly.

These checks support the interrupt rules in MC68060UM §§7.8.1, 8.2.9 and 8.3. WinUAE's [interrupt dispatch](/home/adam/WinUAE/newcpu.cpp:3540), [exception entry](/home/adam/WinUAE/newcpu.cpp:3033), [priority/level 7 recognition](/home/adam/WinUAE/newcpu.cpp:4866), and [RTE implementation](/home/adam/WinUAE/cpuemu_40.cpp:18215) provide the architectural comparison. Its [IACK implementation](/home/adam/WinUAE/newcpu.cpp:2752) directly returns the autovector for CPU models at or above 68020; it is not an oracle for MC68060 pin timing, supplied vectors, AVEC, or bus retries. Those checks use the manual.

## Remaining reference discrepancies

All **125 available IRQ and ODD_IRQ corpus slices** were sampled, with a limit of 300 rounds per slice: **9,295 rounds**, no skipped rounds and no harness errors. The ordinary IRQ group passes **117/117 slices, 8,271 rounds**, covering SR changes, RTE entry into the test, branches, calls, STOP, traps and other interrupt cases.

The eight ODD_IRQ slices cover EXT.B/L/W and SWAP.W, two slices each. They execute 1,024 rounds and report **128 mismatches in 64 affected rounds**: each affected round reports a missing captured trace and an address-error frame PC expected as `$10` but observed as `$24`.

The [representative trace](/home/adam/AP68060/obj/audit-interrupts-2026-10-12/corpus-trace/logs/ODD_IRQ--EXT.B--0001.log) shows the core taking vector 9 at PC `$43900002`. The trace-vector read returns odd address `$123`; the core then takes vector 3 with stacked PC `$24`. Thus, the missing-trace diagnostic does not mean the core omitted the trace exception. It never reaches the trace capture handler. This is the same odd-vector/trace discrepancy recorded in the [exception audit](/home/adam/AP68060/doc/AUDIT-EXCEPTIONS-2026-10-12.md), now reproduced in the interrupt corpus. It remains unresolved and is retained in the failure totals; the corpus is not an unconditional pass.

## Reproduction and limits

The [artifact directory](/home/adam/AP68060/obj/audit-interrupts-2026-10-12) contains the captured sources, generated assembly programs, complete logs, result JSON and [source manifest](/home/adam/AP68060/obj/audit-interrupts-2026-10-12/manifest.json). RTL and generator sources match the audited commit. Audit-only changes are confined to the captured program bench and bus slave. The manual SHA-256 is `313fd672dfac8f1d1bb580ad61e8e1583a38b07fb4e2891aff7f29ef49d3ef40`.

- [run_matrix.py](/home/adam/AP68060/obj/audit-interrupts-2026-10-12/run_matrix.py) regenerates and runs the 1,128 entry, mask, vector and nesting checks.
- [run_extra.py](/home/adam/AP68060/obj/audit-interrupts-2026-10-12/run_extra.py) regenerates and runs 1,052 additional architectural checks and the 40 IPEND reproductions. Its nonzero exit is expected on this revision.
- [Regression results](/home/adam/AP68060/obj/audit-interrupts-2026-10-12/regression-strict.json), [matrix results](/home/adam/AP68060/obj/audit-interrupts-2026-10-12/matrix-results.json), [additional results](/home/adam/AP68060/obj/audit-interrupts-2026-10-12/extra-results.json), and [corpus results](/home/adam/AP68060/obj/audit-interrupts-2026-10-12/corpus-all.log) preserve the pass/fail evidence.

The captured simulator can be rebuilt with `WORK=/home/adam/AP68060/obj/audit-interrupts-2026-10-12/probe-build obj/audit-interrupts-2026-10-12/source/tb/build_sim.sh`, then both audit scripts can be run with Python. Corpus replay uses the captured `tb/run_cputest.py`, `--full --group IRQ --group ODD_IRQ --round-limit 300`, and the captured cputest simulator and monitor.

The pin-skew tests model digital one-clock settling, not analog metastability. The simulations do not exhaust every interrupt arrival clock, every MMU/cache/fault interaction, or every FPU instruction interrupted at completion. No synthesis or board validation was performed.
