# Exception-sequencer audit — 2026-10-12

**Three confirmed defects remain:** the first handler instruction cannot reliably mask an interrupt arriving during exception entry; double-fault protection ends before the first handler instruction is fetched; and buffered stacking errors bypass double-fault detection. The previously documented RTE-to-odd-PC SR discrepancy also reproduces: the core follows WinUAE rather than the manual.

Audited RTL matches AP68060 commit `56a66c03be769dfe0a882e5dcf38c31d7c3d45a8`. References are `/home/adam/Downloads/MC68060UM.pdf` and `/home/adam/WinUAE`. The audit covers exception selection and entry, frame construction, vector acquisition, interrupt deferral, nested faults, reset entry, and RTE. No production source files were edited by this audit.

## Fix status (2026-10-12)

All three findings were reproduced and are fixed in `rtl/ap68060_backend.sv`. The new program `t_xnest` (in `tb/run_tests.sh`) covers them; on the previous backend it fails at test 1, and variants that skip ahead fail at the access-error and buffered-frame cases.

| Finding | Fix | Regression test |
| --- | --- | --- |
| P1 the first handler instruction cannot mask a deferred interrupt | New `x_first`: from the end of the exception routine until the handler's first instruction completes. No IPEND claim is formed during `xstk` or `x_first` (`xwin`), and claims are dropped at exception entry, so at that instruction's boundary the request is compared with the mask the instruction leaves (8.1, 8.2.9, 8.3). The bench's IPEND model follows the same rule | `t_xnest` 1-2: IPL 2 raised as TRAP #0 starts stacking and once its vector is read; the handler's first instruction raises the mask to 7: no interrupt. 3: the control, a handler leaving the mask: the interrupt after its first instruction |
| P1 first-handler fetch errors escape double-fault protection | A double fault also covers an access error with FSLW.IO (the fetch of the handler's first instruction, a refetch included) while `x_first` is set after an access error's, address error's or reset's routine. An operand fault of that instruction stays an ordinary access error | `t_xnest` 4 (TRAP to an unfetchable handler: ordinary access error, FA and IO), then access error, address error and reset (initial PC) with unfetchable handlers: each halts. New bench register `$F156`: a persistent fetch TEA that survives a timed reset |
| P2 buffered stack-write errors bypass double-fault detection | The last uop of an access error's, address error's or reset's routine waits for the store buffer to drain; a store buffer error (SBE) taken then is a double fault | `t_xnest`: CACR.ESB on, an access error and an address error whose frames are written past the bench's memory: each halts |

The RTE odd-PC SR choice (WinUAE's) is unchanged.

## Findings

### 1. P1 — A handler's first instruction cannot reliably mask a deferred interrupt

The [pending-interrupt latch](/home/adam/AP68060/rtl/ap68060_backend.sv:1539) continues recording mask-qualified interrupts during exception processing. The [interrupt condition](/home/adam/AP68060/rtl/ap68060_backend.sv:1242) accepts this latched claim even if the current instruction raises SR's interrupt mask. [take_irq](/home/adam/AP68060/rtl/ap68060_backend.sv:1279) delays delivery while `xstk` is set, but does not prevent that earlier qualification. Consequently, a level-2 request arriving during TRAP entry is taken immediately after the handler's first `MOVE.W #$2700,SR`.

MC68060UM §8.1, p. 8-3, and §8.2.9, p. 8-14, explicitly guarantee that the first handler instruction can raise the mask to prevent such an interrupt. Section 8.3, p. 8-18, describes sampling again after that instruction. Deferring delivery while retaining an earlier mask decision does not implement that rule.

The [stacking-window probe](/home/adam/AP68060/obj/audit-exceptions-2026-10-12/handler_mask_1_7.s) raises IPL2 only after TRAP #0 entry begins. At cycle 313 the core enters vector 32 with SR `$2000`. The handler's first instruction raises the mask to 7; at cycle 409 the core nevertheless enters vector 26 with stacked SR **`$2700`**, `ipend=1`, and PC `$446`, immediately after the SR write. See the [log](/home/adam/AP68060/obj/audit-exceptions-2026-10-12/handler_mask_1_7-full.log).

The [vector-read/first-fetch-window variant](/home/adam/AP68060/obj/audit-exceptions-2026-10-12/handler_mask_2_7.s) also fails. Both injection windows fail at full BCLK and at half BCLK with waits and burst inhibition: **4/4 failures**. Otherwise identical controls leaving the mask at zero correctly take the interrupt after the first instruction: **4/4 pass**.

Keep the interrupt qualification deferred through the first handler instruction's mask decision. Simply suppressing the final `take_irq` pulse during stacking is insufficient. Preserve the ordinary instruction-boundary pending-interrupt behavior separately; the existing suite exercises legitimate pending claims when normal code changes SR. WinUAE's [interrupt test](/home/adam/WinUAE/newcpu.cpp:4555) compares the request with the current mask, but this precise arrival window is established here by the manual and full-CPU simulation, not by a WinUAE pin-timing comparison.

### 2. P1 — First-handler fetch errors escape double-fault protection

The [exception-state lifetime](/home/adam/AP68060/rtl/ap68060_backend.sv:2034) clears `xstk` when the exception micro-routine's last operation retires. That operation reads the vector and branches to its value; the first handler instruction has not necessarily been fetched. The [double-fault condition](/home/adam/AP68060/rtl/ap68060_backend.sv:1295) requires `xstk`, so a later fault fetching the first opcode is processed as a fresh access error even though `xr_dbl` still identifies access-error, address-error, or reset processing.

MC68060UM Figure 8-1, p. 8-2, includes the first handler instruction fetch in exception processing. Sections 8.2.1–8.2.2, pp. 8-6–8-7, require a halt for a further access/address error during access-error or address-error processing. Figure 8-5, p. 8-16, explicitly routes a fault fetching the first reset instruction to double-fault halt.

The audit slave persistently returns TEA for instruction fetches at `$2000`; operand accesses and exception vector reads remain available. Results at both tested clock configurations:

| Original exception | Required | Observed |
| --- | --- | --- |
| Access error, vector 2 points to `$2000` | Halt | Repeated format-4 access-error frames, eventually test timeout |
| Address error, vector 3 points to `$2000` | Halt | Enters the ordinary access-error handler |
| Reset, initial PC is `$2000` | Halt | Enters the ordinary access-error handler |
| TRAP #0, vector 32 points to `$2000` | Ordinary access error | Pass |

In the [access-error trace](/home/adam/AP68060/obj/audit-exceptions-2026-10-12/first_fetch_access-full.log), the initial vector-2 entry occurs at cycle 266. At cycle 376 another vector-2 entry records PC/FA `$2000`, `xstk=0`, `xr_dbl=1`; subsequent entries repeat. The processor continues writing stack frames instead of halting. See also the [address-error probe](/home/adam/AP68060/obj/audit-exceptions-2026-10-12/first_fetch_address.s) and [reset probe](/home/adam/AP68060/obj/audit-exceptions-2026-10-12/first_fetch_reset.s). All **six cases requiring halt fail**.

Retain double-fault context until the initial instruction fetch succeeds and handler execution begins. Do not extend it indiscriminately through execution of the first instruction: the [operand-fault control](/home/adam/AP68060/obj/audit-exceptions-2026-10-12/handler_operand_control.s) successfully fetches that instruction, then faults its data read, which correctly starts a new access error. That control passes at both speeds. Thus the interrupt-deferral boundary in finding 1 and the double-fault boundary here need distinct treatment.

WinUAE's [MMU exception path](/home/adam/WinUAE/newcpu.cpp:3033) builds the frame, obtains the handler, and performs `fill_prefetch()` within exception processing. The [68060 execution loop](/home/adam/WinUAE/newcpu.cpp:5853) halts when the nested exception-processing call itself throws another fault. It does not model this RTL pipeline's `xstk` boundary.

### 3. P2 — Buffered exception-stack write errors start another exception instead of halting

The [stack-store path](/home/adam/AP68060/rtl/ap68060_dmu.sv:1680) can enqueue write-through exception-stack writes when CACR.ESB is set. Their later bus errors arrive through `sb_err`/`take_sbe`. However, [dbl_fault](/home/adam/AP68060/rtl/ap68060_backend.sv:1295) only recognizes an access/address exception carried by the retiring micro-operation; it does not include `take_sbe`. [Exception selection](/home/adam/AP68060/rtl/ap68060_backend.sv:1429) therefore starts a fresh format-4 access-error sequence even when the buffered fault belongs to an active access-error or address-error stack frame.

The [access-error stacking probe](/home/adam/AP68060/obj/audit-exceptions-2026-10-12/stack_fault_access_sb1.s) enables ESB, raises an operand access error, and injects TEA once on the first stack write at `$33FC`. At cycle 354 the [trace](/home/adam/AP68060/obj/audit-exceptions-2026-10-12/stack_fault_access_sb1-full.log) shows a new vector-2 entry with **`FSLW=$00002000` (SBE), `xstk=1`, and `xr_dbl=1`**. The access-error handler then runs; the core never takes the required halt. The [address-error variant](/home/adam/AP68060/obj/audit-exceptions-2026-10-12/stack_fault_address_sb1.s) fails the same way.

Both original exceptions fail at full and half BCLK: **4/4 failures**. With ESB disabled, the same injected stack-write faults correctly halt: **4/4 controls pass**. A stack-write error during an ordinary TRAP correctly becomes an access error with either ESB setting: **4/4 controls pass**.

MC68060UM §8.2.1, p. 8-6, and §7.9.3 require double-fault halt on a bus error during access/address-error processing, without excluding buffered stacking writes. Route buffered errors through the double-fault decision before initiating another exception sequence. This defect is independent of finding 2: here `xstk` is still asserted when the incorrect new exception starts. PBE variants were not independently injected in this focused test.

## Known manual/WinUAE disagreement

**RTE to an odd PC still changes the SR saved by the resulting address error.** MC68060UM §8.4, p. 8-19, specifies the SR from before RTE. The core's [special-case construction](/home/adam/AP68060/rtl/ap68060_backend.sv:1382) instead follows WinUAE's [RTE implementation](/home/adam/WinUAE/cpuemu_40.cpp:18215): it retains the old system byte and synthesizes CCR bits from the proposed new SR.

The [manual-expectation probe](/home/adam/AP68060/obj/audit-exceptions-2026-10-12/rte_odd_manual.s) starts RTE with SR `$2715`, a proposed new SR `$2000`, and odd target `$2001`. Its address-error frame contains SR **`$2700`**, not `$2715`. Both clock configurations fail the manual expectation and pass the [WinUAE-expectation control](/home/adam/AP68060/obj/audit-exceptions-2026-10-12/rte_odd_winuae.s). The stacked PC correctly identifies the RTE. This is the previously documented compatibility choice, not an additional newly discovered defect.

## Validation

| Check on the final captured RTL | Result |
| --- | --- |
| Complete production program suite, 32 programs × 11 configurations, every output checked for FAIL | **352/352 pass** — [results](/home/adam/AP68060/obj/audit-exceptions-2026-10-12/regression-strict.json) |
| Exception entry matrix | **768/768 pass** — [log](/home/adam/AP68060/obj/audit-exceptions-2026-10-12/entry-matrix.log), [generator](/home/adam/AP68060/obj/audit-exceptions-2026-10-12/run_entry_matrix.py) |
| RTE frame matrix | **512/512 pass** — [log](/home/adam/AP68060/obj/audit-exceptions-2026-10-12/rte-matrix.log), [generator](/home/adam/AP68060/obj/audit-exceptions-2026-10-12/run_rte_matrix.py) |
| Focused faults, IRQ windows, IACK, controls, and odd-PC RTE comparison | **40 pass, 16 fail**: 14 failures exercise the three new findings; two exercise the known RTE discrepancy — [results](/home/adam/AP68060/obj/audit-exceptions-2026-10-12/focused-results.log) |
| WinUAE cputest exception-related sample, slice 0001, up to 300 rounds per slice | **37/44 slices pass; 12,060 completed rounds; 676 mismatches; zero harness errors** — [log](/home/adam/AP68060/obj/audit-exceptions-2026-10-12/corpus.log) |

The entry matrix crosses 24 exception triggers, four byte offsets for the supervisor stack, user/supervisor mode, and both M-bit values, at two bus configurations. It checks relocated VBR, exact stack pointer, stacked and handler SR, PC, format/vector word, USP preservation, format-2 address, and format-4 fault address/FSLW. Triggers include all 16 TRAP vectors, illegal/A/F-line instructions, unimplemented MOVEP, TRAPcc/TRAPV, odd branches, and read access errors.

The RTE matrix crosses all 16 format nibbles, four stack offsets, and four restored SR values. Formats 0/2/3/4 restore state and pop the correct lengths; the other formats take vector 14 without consuming the original frame. Both modes and M-bit settings use the expected ISP/USP behavior. The existing suite additionally exercises trace and interrupt priority, FPU exception frames, MMU faults, instruction faults, restart, deferred write errors, and nested interrupts.

The focused IACK checks all pass: supplied vectors 0, 2, 15, 32, 64, and 255; AVEC; and TEA/spurious interrupt, each at both clock configurations. They verify the resulting format-0 frame, relocated vector lookup, mask 2, and M cleared in the handler while retained in stacked SR. Vector 0 is deliberately supplied with AVEC negated; it is not confused with the bench's default autovector convention. Vector-read errors during access/address-error entry correctly halt, while the TRAP vector-read error enters the access-error handler.

All seven failing corpus slices are in `ODD_EXC`: CHK.W/L, DIVS.W, DIVSL.L, DIVU.W, DIVUL.L, and TRAPV. Their **338 affected rounds each produce the same pair**: a missing captured trace and an address-error frame PC expected as `$10` but observed as `$24`. The [new representative trace](/home/adam/AP68060/obj/audit-exceptions-2026-10-12/corpus-trace/logs/ODD_EXC--DIVS.W--0001.log) shows vector 9 at PC `$43900002`, followed by vector 3 at `$24` because the trace vector is odd. This matches the previously documented odd-vector/trace discrepancy in [the core audit](/home/adam/AP68060/doc/AUDIT-CORE-2026-10-12.md). These failures are retained in the totals; the corpus is not reported as an unconditional pass. RTE slices in Basic, Default, and AE all pass, consistent with the deliberate WinUAE odd-PC SR behavior.

These are bounded simulations and source comparisons. No exhaustive interrupt-arrival sweep, every MMU fault combination, every exception-frame cache eviction, formal proof, synthesis, or board validation was performed. The new external-fault and interrupt-window probes use the manual as their behavioral oracle; the corpus does not cover those timing cases.

## Capture and reproduction

- Final RTL is identical to commit **`56a66c03be769dfe0a882e5dcf38c31d7c3d45a8`**. The source capture was updated at **2026-10-12 17:21:34 UTC** and later verified against that commit. Concurrent DMU timing, backend register-read timing, and decoder fault-queue fixes were included. The new `t_bsrfault` program is included in the final 352-run suite.
- [Final RTL SHA256 manifest](/home/adam/AP68060/obj/audit-exceptions-2026-10-12/rtl-final-sha256.json), [capture-time working-tree patch](/home/adam/AP68060/obj/audit-exceptions-2026-10-12/working-tree-final.patch). Initial captures and the intermediate backend/decoder versions are retained separately.
- WinUAE: `7d84eae9aecaf98bcfee1e5cf2c765d2fbfe17b4`. Corpus: `/home/adam/ap68060-work/cputest060b/data`, generated for the 68060 with `cpu_unimplemented=0`.
- [MC68060UM.pdf](/home/adam/Downloads/MC68060UM.pdf): SHA256 `313fd672dfac8f1d1bb580ad61e8e1583a38b07fb4e2891aff7f29ef49d3ef40`.
- The captured bench adds optional fault injectors/exception-entry logging and holds initial RSTI for 128 CLK. The [original bench](/home/adam/AP68060/obj/audit-exceptions-2026-10-12/tb_ap68060.base.sv) is retained. Fault injectors are inactive during the production regression. No production testbench was changed by this audit.

From the repository root:

```sh
python3 obj/audit-exceptions-2026-10-12/run_audit.py --suite
python3 obj/audit-exceptions-2026-10-12/run_entry_matrix.py
python3 obj/audit-exceptions-2026-10-12/run_rte_matrix.py
python3 obj/audit-exceptions-2026-10-12/run_reference.py
```

The focused and reference runners accept `--no-build`. The matrix runners use the focused runner's simulator. The focused runner returns nonzero while the defects/manual discrepancy reproduce; the reference runner returns nonzero for the seven corpus discrepancies. Any printed FAIL counts even if a later line says PASS. Commands and per-run results are preserved in JSON.

The [artifact directory](/home/adam/AP68060/obj/audit-exceptions-2026-10-12) is ignored by Git. Preserve it separately when sharing the report.
