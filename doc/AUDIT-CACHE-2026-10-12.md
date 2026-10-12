# AP68060 cache audit — October 12, 2026

Four functional defects reproduce: cache-inhibited stores leave stale data-cache lines, disabled-cache reads can push retained lines, completed TCI instruction fills remain reusable, and external reset invalidates cache contents. The first can overwrite a completed memory write with older data. Ordinary cache-maintenance instructions pass the additional matrix described below.

This audit compares the working tree with `/home/adam/WinUAE` and `/home/adam/Downloads/MC68060UM.pdf`. It changes no production RTL or repository tests. Reproducers, captured sources, and logs are in [obj/audit-cache-2026-10-12](/home/adam/AP68060/obj/audit-cache-2026-10-12). Printed manual page numbers are used throughout.

## Fix status (2026-10-12)

All four findings were reproduced against the manual (5.1, 5.3, 5.4.2, 5.7.1, 5.7.2) and are fixed, each with a regression test that fails without its fix:

| Finding | Fix | Regression test |
| --- | --- | --- |
| P1 inhibited writes leave stale lines | A cache-inhibited write that hits a line pushes it if dirty and invalidates it before the bus write, as a locked write does (both CM encodings, each part of a split write) | t_cache060 15-17 |
| P2 disabled-cache reads push lines | The push-and-invalidate of locked, inhibited and MOVE16 accesses (reads and writes) runs only with the data cache enabled (CACR.EDC and not CDIS); CINV/CPUSH and snoops are unchanged | t_cache060 18-20 (EDC clear, then CDIS) |
| P2 TCI instruction line reused | A TCI line is marked (`lb_tci`): it still serves the stream it was fetched for, the next redirect drops it, and one whose stream was redirected during the fill is not kept at all | t_cache060 21-22 (bench `$F16A`: TCI on every fetch of a line) |
| P2 reset clears the caches | Reset no longer runs the invalidation sweeps; the valid and dirty bits take their power-up values from their initial blocks | t_halt 10 (a dirty line survives the timed reset; CPUSHA with the cache disabled writes it) |

Not changed: an inhibited instruction fetch leaves a matching I-cache line valid (WinUAE invalidates it; the manual's language concerns the data cache); CUBC clears supervisor branch-cache entries too; the half-cache geometry and clean-line DPI questions; the fill's critical-word latency.

## Findings

### P1 — Cache-inhibited data writes leave matching lines valid and can lose writes

The ordinary store branch in [ap68060_dmu.sv:1451](/home/adam/AP68060/rtl/ap68060_dmu.sv:1451) proceeds without pushing or invalidating a matching line when CM is cache inhibited. At commit, [E_W_PART:1601](/home/adam/AP68060/rtl/ap68060_dmu.sv:1601) handles locked writes and dirty writethrough hits, but has no equivalent treatment for an ordinary cache-inhibited write. The external write succeeds while the old cache line remains valid or dirty.

MC68060UM §5.1, p. 5-4, and §5.4.2, p. 5-8, require matching data-cache lines to be invalidated, with dirty lines pushed first. §5.7.2, p. 5-13, requires the push before the inhibited external access. WinUAE's [write_dcache040](/home/adam/WinUAE/newcpu.cpp:9921) follows this rule.

The reproducer uses user copyback and supervisor cache-inhibited TTR mappings of the same physical address:

1. Memory at `$9000` contains `$11112222`.
2. A copyback write leaves `$AAAABBBB` dirty in the cache.
3. A cache-inhibited write successfully places `$CCCCDDDD` in memory.
4. A cacheable read incorrectly returns `$AAAABBBB`. Alternatively, a subsequent `CPUSHA DC` overwrites memory with `$AAAABBBB`.

Both precise and imprecise inhibited modes fail. Matching clean lines also remain valid and return stale data. All cases reproduce with ordinary bus timing and with half-rate BCLK, waits, and burst inhibition. Explicitly pushing/invalidation before the inhibited store makes the controls pass.

The [bus trace](/home/adam/AP68060/obj/audit-cache-2026-10-12/dcache_ci_write_loss.log) shows the `$CCCCDDDD` write followed by a TM=0 cache push of `$AAAABBBB`. The extracted WinUAE function instead performs four push beats, invalidates the line, and then writes `$CCCCDDDD`.

Handle matching valid and dirty lines before committing an inhibited store to the bus, including both CM encodings and partial writes.

Evidence: [dirty-line probe](/home/adam/AP68060/obj/audit-cache-2026-10-12/dcache_ci_write_40.s), [clean-line probe](/home/adam/AP68060/obj/audit-cache-2026-10-12/dcache_ci_write_clean_40.s), [lost-write probe](/home/adam/AP68060/obj/audit-cache-2026-10-12/dcache_ci_write_loss.s), [control](/home/adam/AP68060/obj/audit-cache-2026-10-12/dcache_ci_write_40_control.s).

### P2 — Disabled-cache reads can push and invalidate retained data

The bypass-read branch in [ap68060_dmu.sv:1497](/home/adam/AP68060/rtl/ap68060_dmu.sv:1497) acts on `e_x.hit` without checking `dc_en`. Consequently, a cache-inhibited read can push and invalidate a matching dirty line even while CDIS is asserted or CACR.EDC is clear.

MC68060UM §5.3, pp. 5-6–5-7, requires CDIS accesses to bypass the caches and leave their contents unchanged. WinUAE's [read_dcache040](/home/adam/WinUAE/newcpu.cpp:9883) checks EDC before searching the data cache, so the EDC-clear case also disagrees with that reference.

With `$11112222` in memory and `$AAAABBBB` dirty in the cache, the probe disables caching, waits for CDIS to settle where applicable, and reads through a precise inhibited mapping. It should return `$11112222` without a push. Instead, the core pushes four longwords, invalidates the line, and returns `$AAAABBBB`. Both disable mechanisms reproduce under both bus configurations. Controls using a cacheable mapping during the disabled access preserve the line and pass.

Gate normal-access cache side effects by the effective cache enable. Explicit CINV/CPUSH and snoops must continue to operate while caches are disabled, as required by §5.3.

Evidence: [CDIS probe](/home/adam/AP68060/obj/audit-cache-2026-10-12/dcache_disabled_cdis.s), [CDIS bus trace](/home/adam/AP68060/obj/audit-cache-2026-10-12/dcache_disabled_cdis.log), [EDC probe](/home/adam/AP68060/obj/audit-cache-2026-10-12/dcache_disabled_cacr.s), [control](/home/adam/AP68060/obj/audit-cache-2026-10-12/dcache_disabled_cdis_control.s).

### P2 — A completed TCI instruction fill remains reusable after a redirect

[ap68060_fetch.sv:769](/home/adam/AP68060/rtl/ap68060_fetch.sv:769) prevents cache allocation when TCI is received, but completion unconditionally sets `lb_v` at [line 782](/home/adam/AP68060/rtl/ap68060_fetch.sv:782). The last-fill buffer subsequently satisfies matching fetches through `x_lb`, including a new nonsequential request after a redirect.

MC68060UM §5.7.1, p. 5-12, permits sequential instruction-buffer access during the burst, then requires the buffer to be discarded when the TCI fill completes. It must not supply a later redirected request.

An isolated fetch-unit probe completes a TCI burst containing opcode `$7001`, stops new fetches, changes the memory response to `$7002`, and redirects to the same address. The unit supplies `$7001` without another bus request. Explicit line invalidation causes a second request and returns `$7002`; the ordinary cacheable control correctly returns the cached `$7001`.

This is reproduced at the fetch interface. A separate complete-CPU loop passes because its speculative next-line fill replaces the buffer before the branch returns. Thus the evidence establishes the fetch-unit defect, not a demonstrated failure of that complete-CPU loop. WinUAE is not used as a physical TCI oracle.

Separate delivery of the outstanding instruction request from retention of the completed line. Simply suppressing `lb_v` without providing that delivery path would cause repeated misses in the present implementation.

Evidence: [fetch probe](/home/adam/AP68060/obj/audit-cache-2026-10-12/tb_fetch_cache.sv), [failure](/home/adam/AP68060/obj/audit-cache-2026-10-12/fetch-tci.log), [invalidation control](/home/adam/AP68060/obj/audit-cache-2026-10-12/fetch-tci-control.log), [cacheable control](/home/adam/AP68060/obj/audit-cache-2026-10-12/fetch-cacheable-control.log).

### P2 — External reset invalidates both caches and discards dirty data

The DMU reset path enters its all-lines invalidation sweep at [ap68060_dmu.sv:1086](/home/adam/AP68060/rtl/ap68060_dmu.sv:1086). The fetch reset path starts the equivalent instruction-cache sweep at [ap68060_fetch.sv:538](/home/adam/AP68060/rtl/ap68060_fetch.sv:538).

MC68060UM §§5.2–5.3, pp. 5-5–5-6, explicitly state that hardware reset clears CACR but does not change cache tags, state, or data. WinUAE's [m68k_reset2](/home/adam/WinUAE/newcpu.cpp:3617) clears CACR and calls `set_cpu_caches(false)`; its 040/060 cache-flush path does not invalidate entries for that call.

The complete-CPU probe leaves `$AAAABBBB` dirty at `$9000`, with `$11112222` still in memory, then pulses external reset. After restarting with caches disabled, it executes `CPUSHA DC`. Memory remains `$11112222`: the dirty line has been discarded. Pushing before reset makes the control pass. Both bus configurations reproduce the failure.

The probe does not depend on enabling uncleared caches after reset; it uses the explicitly supported operation of CPUSH while disabled. Preserve cache arrays and validity/dirty state across external reset, separately from transient pipeline state and cold-start initialization.

Evidence: [reset probe](/home/adam/AP68060/obj/audit-cache-2026-10-12/cache_reset.s), [result](/home/adam/AP68060/obj/audit-cache-2026-10-12/cache_reset.log), [control](/home/adam/AP68060/obj/audit-cache-2026-10-12/cache_reset_control.s).

## Reference differences and other conformance gaps

**Cache-inhibited instruction hits retain stale cache entries.** The [probe](/home/adam/AP68060/obj/audit-cache-2026-10-12/icache_ci.s) warms a routine returning 1, changes memory to return 2, enables an inhibited ITT mapping, then removes that mapping. Results are 1, 2, 1. Explicit I-cache invalidation makes the last result 2. [The core's CI path](/home/adam/AP68060/rtl/ap68060_fetch.sv:433) bypasses the old entry without invalidating it; WinUAE's [fill_icache040](/home/adam/WinUAE/newcpu.cpp:9484) invalidates a matching entry. This is a reproducible reference disagreement. The manual's explicit matching-entry invalidation language in §§5.1 and 5.4.2 refers to data-cache entries, so this is not counted among the four confirmed manual violations.

**The local WinUAE revision has two cache-maintenance discrepancies.** [flush_cpu_caches_040](/home/adam/WinUAE/newcpu.cpp:1779) tests bit 24 for DPI, although CACR.DPI is bit 28. Its [line/page push branch](/home/adam/WinUAE/newcpu.cpp:1759) calls the data-cache push function even when processing an instruction-cache match. An extracted-function probe confirms that `CPUSHL DC` with DPI=1 invalidates the data line, while `CPUSHL IC` leaves the instruction line valid and instead pushes/invalidates a data line. AP68060 passes the manual expectations for these cases. These WinUAE results should not become core test expectations.

The [WinUAE probe and results](/home/adam/AP68060/obj/audit-cache-2026-10-12/winuae-cache.log) use the original cache-function bodies extracted verbatim into a small memory/register adapter. They are not a run of the complete emulator; unused data-fill/update paths deliberately abort.

Two additional source-visible deviations affect performance or require clarification:

- **CUBC also clears supervisor branch entries.** [The implementation documents this](/home/adam/AP68060/rtl/ap68060_fetch.sv:233) and shares one clear signal for CABC/CUBC. CACR.CUBC in §5.2 requires supervisor entries to remain valid. This is a known prediction-retention gap; no incorrect retired result was demonstrated.
- **Half-cache geometry and clean-line DPI behavior need an authoritative tie-breaker.** AP68060 allocates two ways across 128 sets in half mode; WinUAE uses four ways across 64 sets. The manual's FOC/FIC descriptions do not resolve that implementation choice. Also, Table 5-3 describes clean CPUSH hits as invalidated, while the DPI and §5.3 prose broadly describe retaining pushed lines. The core retains clean lines with DPI set. These ambiguities are not promoted into confirmed defects. The matrix's DPI checks use dirty lines.

The fetch engine also waits for a full successful line fill before delivering instructions, whereas §5.7.1 describes critical-longword forwarding during the fill. That is a latency difference; this audit does not establish cycle equivalence to silicon.

## Validation

| Check | Result |
| --- | --- |
| Existing complete program suite, default parameters, ten bus configurations | **290/290 pass** — [log](/home/adam/AP68060/obj/audit-cache-2026-10-12/regression.log) |
| Twelve cache-related existing programs, FPU_PIPE/LOOP_PRED/EARLY_RES enabled, ten bus configurations | **120/120 pass** — [log](/home/adam/AP68060/obj/audit-cache-2026-10-12/regression-options.log) |
| Independent maintenance matrix, default parameters | **576/576 pass** — [log](/home/adam/AP68060/obj/audit-cache-2026-10-12/matrix-results.log) |
| Same maintenance matrix, all three optional parameters enabled | **576/576 pass** — [log](/home/adam/AP68060/obj/audit-cache-2026-10-12/matrix-results-options.log) |
| Focused reproductions and controls | **20 pass, 19 fail** — [results](/home/adam/AP68060/obj/audit-cache-2026-10-12/focused-results.log); 17 failures exercise the four findings, two exercise the instruction-CI reference disagreement |
| Dirty-victim restoration after TCI or a late fill error | Both pass in ordinary and slower bus configurations; included in focused results |

The maintenance matrix contains 288 programs, each run twice. It crosses CINV/CPUSH, line/page/all scopes, IC/DC/both selections, DPI clear/set, normal/disabled/CDIS/no-allocation conditions, and 4K/8K pages. It checks selected and unselected lines, memory writeback, subsequent I-cache execution, and D-cache refill counts. Low address bits are deliberately nonzero for line/page operations. Paging translation is disabled in this matrix; the existing MMU regressions supply separate translation coverage.

The slower matrix configuration includes half-rate BCLK, waits, burst inhibition, and native retries. The existing suite additionally covers snooping, snooped push retries, locked accesses, store buffering, split operands, late fill errors, and bus arbitration. Source review covered cache arrays and forwarding, replacement, cache-mode decisions, maintenance scans, branch-cache clearing, push/store buffers, and their BIU/snoop connections.

These checks do not exhaust all cache/MMU/snoop races or fault boundaries. No synthesis, timing analysis, or board validation was performed. Passing existing tests does not cover the newly reproduced cases.

## Captured inputs and reproduction

- AP68060 HEAD: `7e7651e9a55c97eb8b5d0346c003914682f2a7c9`, plus the [working-tree patch](/home/adam/AP68060/obj/audit-cache-2026-10-12/working-tree.patch).
- Source capture: **02:40:49 EDT, October 12, 2026**. [RTL manifest](/home/adam/AP68060/obj/audit-cache-2026-10-12/rtl-sha256.json). Production RTL still matched the capture at the final audit comparison.
- WinUAE HEAD: `7d84eae9aecaf98bcfee1e5cf2c765d2fbfe17b4`, clean working tree.
- [MC68060UM.pdf](/home/adam/Downloads/MC68060UM.pdf), SHA256 `313fd672dfac8f1d1bb580ad61e8e1583a38b07fb4e2891aff7f29ef49d3ef40`.

Rebuild captured RTL and rerun the focused probes:

```sh
python3 obj/audit-cache-2026-10-12/run_audit.py
```

The runner returns nonzero while the failures reproduce and checks printed simulator results. Use `--no-build` to reuse the binaries. The captured full-CPU bench adds only audit TCI stimulus and bus logging; the fetch probe drives the production fetch module directly.

Regenerate and run the maintenance matrix with the existing baseline binary:

```sh
python3 obj/audit-cache-2026-10-12/generate_cache_matrix.py
python3 obj/audit-cache-2026-10-12/run_matrix.py
```

The [artifact directory](/home/adam/AP68060/obj/audit-cache-2026-10-12) is ignored by Git and must be preserved separately when sharing this report.
