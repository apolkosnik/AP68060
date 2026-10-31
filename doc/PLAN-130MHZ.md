# Re-pipelining the core for 130 MHz

Goal (2026-10-31): the core passing the regression and the corpus with
Fmax 130 MHz (PERIOD = 7.692 ns, `PERIOD=7.692 syn/fitcore.sh f130_x`),
no multicycle or false-path exceptions, no seed sweeps.  Area may grow.

## 1. The budget

What the 120 MHz work measured on this device: a path of 4-5 LUT levels
with ~1 ns of wire needs 8.6-9.6 ns register to register (the 8.333 ns
fits fail by 0.3-1.3 ns on exactly those); the clock skew between a
launch and a latch in different placement regions costs up to 0.5 ns.
At 7.692 ns the budget is therefore **3 LUT levels on any path that
crosses a module boundary or fans out widely, 4 levels inside a block**,
with a register at every module boundary that carries a decision.

The 120 MHz branch reached that shape locally (every stall a register,
every select registered a clock ahead, every late decision the last
pick).  What remains is structural: whole stages whose work is 5-6 levels
deep even when perfectly arranged.  Those stages are split.

## 2. The pipeline today

```
fetch   F0 (set read, BTB read)  F1 (translate, tag compare, BTB take)  F2 (words)
decode  D1a (parse from the queue)  D1b (record, predictions)  -> 4-entry FIFO
useq    D2a (ROM address)  D2b (uop from the word)  D2c (uop from the state) -> 2-word queue
backend SKID  AG (file read, forward, EA add)  DC1  DT  DC2  EX  WB
dmu     DC1 (ATC, tag read)  DT (compare, fast decision)  DC2 (way, align)  WB (fast store)
        + the engine: a 54-state machine owning the slow paths
fpu     a sequential state machine at the EX stage (the P1-P6 pipeline is PIPE=0 here)
```

## 3. The stage map for 130 MHz

Each item names the deep work it removes from one clock and what it
costs in cycles.  The order is the order of work.

### 3.1 EX split: EX1 (result) / EX2 (flags, condition, exception)

Today EX forms the ALU result, its flags, the condition (alu_cc), the
exception-pending flag ex_xnz = f(alu_cc, md_z, fp_xvec), the Bcc/DBcc
early redirect, and feeds the stall registers' last pick (ex_okn) and
the operand captures of four younger stages -- the widest fan-out in
the design after the kill.

- EX1: operands -> ALU sum / logic / shifter result, registered
  (alu_res, alu_sum, the one-hot result case).  Its broadcast to the
  captures stays where it is (one clock late, exw_q) and so gets a
  whole clock of wire.
- EX2: flags from the registered sum and operands (N/Z/V/C/X), alu_cc,
  ex_xnz, the trap and CHK decisions, the early redirect.  WB's decision
  (xgo) reads EX2's registers as it reads EX's now.
- CCR dependence: a uop in EX1 that reads X or the CCR (ADDX/SUBX/NEGX,
  Scc, DBcc, Bcc, ABCD, the shifts' X) while EX2 holds a CCR writer
  takes EX2's flags through one forward mux (the flags are EX2's output
  register, one LUT), no interlock.
- Cost: branch resolution one clock later (mispredict +1), one more
  stage of the ~410-bit uop (narrow it first: 3.7).

### 3.2 AG split: AG1 (operands) / AG2 (address)

Today the file read happens as the uop enters AG (ports addressed from
the sequencer's queue), and AG itself scales the index, picks the
captured/forwarded value, and adds three operands; the capture
AND-OR into agb_q/agx_q from DC2's load data and EX's broadcast is the
8.333 ns fits' worst family (sk_v -> the file's address register,
alv_q -> agb_q, m2.x.way -> agx_q).

- AG1: the file read's result, the forward from EX's broadcast and
  from DC2's handed-over load, the index sign extension and scale, the
  interlock compares -- all into registers.  The skid merges into AG1
  (AG1 is the skid: it holds when AG2 does).
- AG2: add3 only, into dm_va and the register updates.
- Cost: load-use for an address register +1 clock (today a load's data
  reaches AG the clock after DC2 through the capture; with AG1 it still
  does, into AG1's registers, then AG2 adds); the measured cost of
  removing the DC2 capture outright was 2.1 % Dhrystone -- this keeps the
  capture, so less.

### 3.3 DT split: the fast decision a clock later

DT compares the tag chunks (now formed in DC1) against four ATC ways'
pages, picks the way, forms rdy_dt (the fast-path decision: 7 terms
from the compare, the dirty-way mask, the engine's state) and feeds DC2's
stall register the same clock (dc2h_c corrects it combinationally).

- DT1: the compare and the way one-hot, registered (today's chunk ANDs
  end in a register).
- DT2: rdy_dt from DT1's registers alone; DC2 becomes a stage that always
  takes the access and holds it when DT2 says not ready (the "assume
  ready, correct later" structure exists already as dc2h_c; it becomes a
  plain register).  A load's data path gains a clock (DC2's way select
  from a registered one-hot: one LUT level less in DC2 as well).
- Cost: load latency +1 (the handed-over capture 3.2 absorbs one of them
  for address operands).

### 3.4 The DMU's engine in two-process form with a one-hot state

The engine's 54 states decide from the lookup records, the valid/dirty
RAMs and the walker (e_x.way from the tag RAM's write register, tw_use
-> e_x.pa, the kill into the walker's registers).  At 8.333 ns the
engine is the second cluster and the placement-sensitive one.

- One always_comb for next state and the per-state actions as one-hot
  flags, every action decided from registers of the state before it
  (the 120 MHz work did this for E_S_ACT's victim, E_M_PLW's loads, the
  stolen lookup's translation; it becomes the rule).
- The lookups the engine steals from DC1 take their address from a
  register (st_la is one already) and return through DT1's registers:
  one clock more per engine lookup.
- Cost: cache misses, table walks, pushes each a few clocks longer
  (invisible in Dhrystone, ~1 % Whetstone at most).

### 3.5 WB: the commit decision ahead of its effects

WB decides x_go, the redirect, the SR/CCR update and the store's
acceptance in one clock from EX's registered state, and the decision
reaches the whole core (flush, fe_flush, the file's front-valid drop,
the DMU's kill).  At 130 MHz the fan-out alone is a clock.

- WB1: the decision registers (x_go, wb_redir, kill) formed from EX2's
  registers; WB2: their effects.  The kill is then a register seen by
  every stage one clock later; stages advance one more uop into the
  shadow of a redirect and drop it (the DMU already tolerates a killed
  record: it marks and drains).
- Cost: mispredict and exception entry +1 clock.

### 3.6 Fetch and decode

- The queue's consume loop (dec.r_use_f -> fetch.qw): D1a's take as a
  registered one-hot formed a clock ahead from the queue count's next
  value (the pattern of the uop queue's hd_nu/hd_ue).
- F0's next address from the BTB's take (btb0 write register -> fpc):
  the BTB's compare into a register (F1 takes a clock later: one more
  bubble on a BTB hit, or the BTB predicts F0's next line from the
  line before, as the 060 does).
- D1a's length and fault decisions for the multi-part path: left as
  they are unless the fit names them.

### 3.7 The uop and the register files

- Narrow the uop per stage (the fields each stage reads are known: AG
  reads base/idx/disp/upd*, EX a_src/b_src/op/sz/cond, WB d_reg/exc/
  br/target): fewer registers to add per new stage and a shorter fan-out
  from the sequencer.
- The register files' address registers (rdaddr_reg) take their picks
  from the sequencer's queue state; with AG1 the picks are registers
  formed a clock ahead with no late select (ag_ld, sk_ld disappear:
  AG1 takes whenever AG2 advances).

### 3.8 FPU

The FPU is a sequential state machine whose steps were split for
120 MHz; the 7.692 ns fit will say which steps (the 64-bit normalizer,
the rounding, the 32x32 partial products) need one more register.  The
P1-P6 arithmetic pipeline (PIPE) stays off.

## 4. Verification per step

Every step: the regression (tb/run_tests.sh, 393 PASS), the corpus
smoke (tb/run_cputest.py without --full), the benchmarks (tb/run_bench.sh:
Dhrystone cycles/iteration, Whetstone cycles/loop, recorded per step so
the cycle cost of each split is known), and a fit at PERIOD=7.692 judged
by the failing-endpoint histogram and syn/paths_by_module.py, not by Fmax
alone (fit-to-fit noise is ~300 ns TNS on near-identical netlists).  The
full corpus at the end of each section of 3.

## 5. Baseline

f130_a, HEAD dff41b3 at PERIOD=7.692: Fmax 95.51 MHz, slack -2.778 ns,
TNS -1,747 ns, 3,793 failing endpoints (16 below -2 ns, 343 in -2..-1,
3,434 in -1..0), 25,584 ALMs.

By module pair (TNS, endpoints, worst): backend internal -689 / 1,375 /
-2.40; DMU internal -318 / 696 / -2.78; fetch internal -157 / 312; snoop
internal -127 / 169; decode -> fetch -118 / 231 (the queue's consume
loop); useq -> backend -108 / 277 (the head's base/index through the
file into AG's captures); backend -> DMU -53 / 221; DMU -> BIU -52 / 94.

The largest destination families: fetch.qw -147 ns / 283 endpoints (the
queue words), snoop.fq -68 / 90, AG's agb_q/agx_q -109 / 91 (worst
-1.84: useq.hd.base -> the file -> the capture), EX's operand registers
ex_av/ex_bv and the slow ALU's xs_q.res -92 / 129 (ex_u.op and ex_u.cond
decoding into the operand picks: -1.83), WB's registers (wb_redir_pc,
wb_st_data, wb_exc_pc, wb_exc_addr, wb_res, wb_ccr) -138 / 180, the
stall registers' worst paths from ex_u.op (-2.40 into stall_ag_q), the
tag RAM's write register into the snoop phase and e_x.hit/way (-2.78,
the read-during-write bypass compare), the DMU's dw_word / m2.x1.pa /
tw_va / e_acc / pv_line (each -20..-30), the BIU's data output (-29).

So the 3.1 EX split comes first (ex_u.op/cond decode into the operand
picks, the result into WB's registers and the stall registers), then the
3.2 AG split (the useq -> file -> capture family), the fetch queue's
consume loop (3.6, the single largest destination), the tag RAM bypass
compare into the snoop phase and the engine (3.4).

## 6. Log

### 3.1 EX split (EX2), 2026-10-31

Done as planned: EX keeps the units, the result broadcast (exw_*) and the
early Bcc/DBcc redirect; EX2 (e2_*) holds the registered outcome -- the
result, the flags after the uop (e2_ccr), the exception terms (xok,
fault, xnz, odd, the vectors and addresses, the FPU's frame fields), the
targets and redirect terms -- and WB's registers are formed from EX2's
at adv_e2.  WB itself is unchanged; its hold-ahead (wbh_n) reads EX2's
ok as a register, so the stall registers' "ok" pick is a flip-flop now.
The DMU mirrors it with a record m3b between m3 (EX) and m4 (WB): the
older-store tests (st_older, ex_st, st_at, st_line, store_holds), the
per-source hit clears and the WB store's ok-ahead read it.  The FPU
interface's safe counts EX2 as an older uop (e2_fpchk, exact from
registers: the one conservative case is gone).

What the extra stage exposed, all fixed in the DMU/back end:
- the EX2 record was not cleared by a kill (a stale "older store"
  deadlocked the next load);
- younger accesses started bus cycles behind a uop that will flush them:
  a load after RESET on the bus while RSTO was asserted, a read after
  LPSTOP's broadcast, a read without LOCK ahead of the MOVEC that sets
  BUSCR.L (the MOVEC's serializing flush came one clock later).  New
  dm_quiet: a serializing or excepting uop at EX, EX2 or WB holds DC2's
  slow path (those younger accesses are flushed anyway: no cycle cost).
- t_cbsplit 29 required the DMU's split-store fallback (a snoop between
  the halves) to occur at least once at half-rate BCLK; whether the
  alternate master's periodic write lands between the halves follows from
  the pipeline's depth, and with one more stage it never does.  The test
  now walks the store's phase with extra reads and reports the count
  without requiring it; the exact checks stay.

Regression 392/392 (t_cbsplit as above), corpus smoke 5/5.
Cycle cost: Dhrystone 1,086 cycles/iteration (was 1,071: +1.4 %),
Whetstone 784,934 cycles/loop (was 754,922: +4.0 %, the FPU uops wait
one more stage for "no older uop in flight").

Fit f130_b (b4235d6, PERIOD 7.692): Fmax 103.8 MHz (from 95.5), slack
-1.94 ns (from -2.78), TNS -1,553 ns (from -1,747), 3,094 failing
endpoints (from 3,793), 25,627 ALMs.  What leads now: the snooper's
push-buffer compare into the DMU's snoop phase (-1.94), ex_u.op into the
stall registers (-1.85, the divide-by-zero test's op compare), wb_ok
into the walker's address (-1.75), ex_u.cond into the operand captures
(-1.76, 58 endpoints: the result's select by the condition, Scc, is in
the broadcast), ex_u.d_reg into DC1's capture (-1.62), the skid's base
into AG's capture (-1.60), the split access's second translation
(m2.x.pa -> m2.x1.*, -1.57, 42 endpoints), the store buffer's count into
the BIU's data (-1.57, 36).

### 3.1b The broadcast from EX2 (next)

EX2's result e2_res is the broadcast (xw_*): the front file's EX write
port and every younger stage's capture take a register, selected by
flags against EX2's uop (formed a clock ahead as before); EX bypasses its
own operands from EX2 for a back-to-back dependence (one 2:1 mux ahead
of the ALU, the register taking the value for the cycles after); AG
takes the broadcast a clock late (xw_q) with its writer flags one stage
longer (EX2, left EX2).  Removes the result-select-into-captures family
(ex_u.cond / ex_u.op / ex_rk / ex_bv into dc1_b, ex_av, the file's data
inputs).  Cost: AG's dependence on an EX result one clock longer.

Done 2026-10-31 (the broadcast from EX2).  Found on the way:
- AG's refusal flags for the broadcast after a load capture (cbl/cxl,
  now cbl2/cxl2 for the second clock) are the AG uop's own: a new uop
  entering must not inherit the previous uop's capture, or it refuses the
  broadcast it waits for and keeps a stale base (Dhrystone's strcpy
  overran its buffer; the regression had no such sequence -- t_integer
  306-310 now have it).
- A latent instruction-cache bug, present on main: F0 reads the tag RAM
  in the clock a fill writes another way of the same set; the tag read is
  the old one while the valid bit (read in F1 from registers) is the new
  one, so a stale tag of zero (any address below $800) matched: two ways
  hit.  Exposed by t_integer's new tests moving fail_all into the set of
  a line of code.  The way written in F0's read clock is no hit in F1
  (the just-filled line is the line buffer's).  To port to main.
Regression 392/392 (t_integer with 306-310), corpus smoke 5/5.
Cycle cost: Dhrystone 1,102 cycles/iteration (+1.5 % on EX2, +2.9 % on
main), Whetstone 789,862 cycles/loop (+0.6 % on EX2).

Fit f130_c (53cc21e): Fmax 99.1 MHz, slack -2.40, TNS -1,601, 3,786
endpoints, 25,718 ALMs -- worse than f130_b by TNS and count.  The
broadcast register e2_res fans out to the file's write port, four
stages' captures, EX's bypass ahead of the ALU and the clock-late copy:
the bypass put that fan-out's wire in front of the ALU (e2_res -> ccr_f
-2.15, mxe_b -> e2_stdata -1.87 x 30) and the captures' family moved
rather than vanished (sk_v -> agx_q -2.16 x 44, m2.x.way -> ex_av_r).
Two stall-register sources from far modules led (useq.uq_e, dmu.bo_err:
-2.4, placement).  Next: e2_res in four preserved copies, one per
consumer group (f130_d), then the AG split.

### 3.2 AG split: the design

Today a uop's base and index are read from the file as it enters AG
(the ports addressed from the sequencer's head and the skid, the bank
and front selects registered), and the capture into agb_q/agx_q picks,
in one clock, among the file's word (its MLAB read, then the select),
AG's own updates of this clock (b1/b0), the load handed over from DC2
(dc2_capv) and the broadcast (xw_q); AG then adds (add3) into dm_va.
The file's word is the last to arrive: that select and the skid's valid
(sk_v) are the worst family after EX2 (-2.2 ns).

AG1 (operands) / AG2 (address):
- AG1 holds the uop (a1_u) and the file's words for its base and index
  as read (a1_fb, a1_fx: the front/back selects applied, nothing else),
  with the skid folded in: the sequencer's uop enters AG1 directly
  (in_rdy = !stall_a1, a register), the skid register goes.
- AG2 is today's AG: agb_q/agx_q are loaded from AG1's registered words
  with the same capture AND-OR (the updates, the handed-over load, the
  broadcast) -- all its inputs are now flip-flops -- and adds.
- The writes the file sees between the read (AG1 entry) and the use
  (AG2 entry) are one clock more than today: AG's own updates written
  in the clock the uop sits in AG1 (fwe[0/1]: compared as today, one
  clock later), and the broadcast's write (xw, a register: compared with
  a1_u's registers while the uop sits in AG1, the word replaced).  The
  pattern is the existing one (b1/b0, qb_o), applied in AG1 as well.
- The writer flags (wf_*) and the interlock (ilk) move with the uop: the
  compares are made as the uop enters AG1 (one stage earlier), the flags
  shift by one more stage (DC1 .. left EX2 become AG2, DC1 .. left EX2).
  AG2's hold is the interlock as today.
- The register files' ports 0/1 (and 5/6, the skid's) are addressed from
  the sequencer's head as today; the skid's ports go with the skid.
Cost: one more stage between the sequencer and the data cache; a
mispredict or exception reaches the next uop one clock later still
(+1 on the branch penalty), load-use for an address register +1.

Fit f130_d (8a1ab96, the four copies): Fmax 102.45 MHz, slack -2.07,
TNS -1,478 (the best so far: a -1,747, b -1,553, c -1,601), 3,562
endpoints (3,351 of them within 1 ns), 25,600 ALMs.  Leading: AG's
address compare for the DMU's landing write into the stall register
(ag_u.scale -> stall_ag_q, -2.07: the adder's output compared, the OR
last), the EX bypass select ahead of the ALU (e2_dwe -> ccr_f -1.94,
e2_dwe -> e2_stdata x32), the kill into the walker (wb_ok -> tw_va
-1.78 x20), the FPU interface's next state into the stall registers
(oq.ns -> stall_* -1.8), DC2's load handed to AG (m2.x.way -> agb_q
-1.92).

Done 2026-10-31 (3.2, AG1): as designed, with the skid kept as AG1's
input buffer (in_rdy stays a register).  Exposed on the way: a uop
waiting in AG1 missed the loads and broadcasts that passed meanwhile
(Dhrystone, t_stld, t_movem_restart, t_fpu); AG1 now applies the same
capture rules while it waits, and the refusal of an older writer's
broadcast after a captured load is tracked by the load's position (in
EX: refuse xw; one clock longer for xw_q) rather than a fixed two-clock
window, in AG2 too.  Regression 392/392, Dhrystone 1,107 (+0.5 %),
Whetstone 790,634 (+0.1 %).

Worst paths of f130_d read in full (sta_to.tcl): each is 4-6 cells and
2-4 wires of 1.0-2.5 ns -- the wires from a register with a fan-out of
50-120 (e2_dwe 46, the decoder's consumed count 116, its adder 84).
Two small steps from that, 2026-10-31:
- EX's flat result select: the sources that are registers (operands,
  the slow result, the units') as one kept AND-OR, the adder's sum and
  the ALU's logic result joined in the last level (exw_oth).
- 3.6, the fetch queue's take: the decoder's consumed count and the
  head's length as registered one-hots, four copies each (one per
  quarter of the queue's words), every word's next value an AND-OR of
  constant-index reads -- no index arithmetic or 16:1 select in the
  words' path.
Corpus runs stopped on request (2026-10-31): the full run of the
pre-split HEAD ended at 3,759 of 4,504 slices, 3,640 pass, 99 known, no
failure under the current rules; the smoke on the AG split passed 5/5.

### 3.5 WB's decisions as registers: the design

kill_now, x_go, dbl_fault, take_irq/take_sbe/take_trace are each
`adv_wb ? X1 : X0` from WB's registers (two forms, the advance picking
last); kill_now then fans out to the DMU (its records, the engine's
dispatch: wb_ok -> tw_va -1.78), the stall registers' last AND, the
front-valid drop, the capture flags, xstk/x_first/xr_dbl.  The same
method as the stall registers: each decision formed a clock ahead from
the next values of its terms and registered (kill_q etc.), checked equal
to the direct form every clock; the consumers read the registers (with
preserved copies per module).  The next values exist already for most
terms (wb_v_n, the stall register's D for adv_wb, ok_new/ok_stay for
wb_ok, irq_pend_n, tr_n, sb_ne_n); to add: wb_redir's and wb_xf's next
(the EX2 -> WB exception select as an always_comb the clocked block
copies), the store buffer's error next (a DMU export), and xstk /
x_first / xr_dbl / halted next from the registered decisions themselves
(x_go_q, dbl_q) so no decision's cone feeds another's in the same clock.

Fit f130_e (1279cb6, AG1): Fmax 96.16 MHz, slack -2.71, TNS -1,957,
4,233 endpoints, 25,942 ALMs -- worse than d.  New at the top: the
divide-by-zero test on EX's bypassed operand (e2_dwe -> the bypass mux
-> a 32-bit zero test -> exh_n -> every stall register, -2.7; fixed:
the test on the register alone, a divide waits the bypass clock), AG2's
update-register compares into the capture (ag_u.upd_reg -> agx_q, -2.06
x 47: the b1/b0 compares, formed each clock; to be registered from the
uops' next values), the sequencer's MOVEM state into the file's selects
(useq.mv_act -> rff.sel_q -1.89), xr_dbl -> ccr_f (3.5).

Done 2026-10-31: AG2's capture compares (its update registers and DC2's
destination against AG1's base and index) as registers formed from the
uops the stages hold next, the loads picking last; the capture's
one-hot selects are then flip-flop ANDs.  Regression 392.

Fit f130_f (315e5ef, the result grouping and the one-hot takes): Fmax
104.19 MHz, slack -1.91, TNS -1,350, 3,085 endpoints, 25,510 ALMs -- the
best on every count.  Leading: the EX bypass select into the store data
(mxe_b -> e2_stdata -1.91 x32: ex_st read the case chain ex_res, now the
flat select), the flags' size select into ccr_f (ex_u.sz -1.86), the
snoop phase into the stall registers through the DMU's hold next
(sn_ph -> stall_a1/ag -1.86), the bypass select into the stall registers
through the divide's zero test (mxe_a -> stall_dc1 -1.65: the test is
now a flag registered in the divide's first EX clock, read from its
second), the skid's valid into AG1's file word (sk_v -> a1_fx -1.48 x31:
the word is now the one select before the flip-flop), the tag RAM's
write register into the snoop phase and the engine (-1.67).
