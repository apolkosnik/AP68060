# Registered per-stage stalls: design study (2026-10-30)

Status: NOT implemented.  The backend side is mechanical (section 2), but
the DMU's four hold D-values (hold1_n, dc2r_n, st_ok_n, hold_dt_n) are
formed inside its one clocked block from block-local decisions (m1s_n,
mts_n, m2ss_n -- the last one overridden by the records' hit clears,
which read the engine's state-machine decisions).  Exporting them exactly
means converting that block to the two-process form (next-state
always_comb + registers), with the MLAB arrays' writes kept clocked.
A conservative substitute for dc2r_n (dropping the simple-store term)
would stall every non-fast store a clock: rejected.

# Registered per-stage stalls for ap68060_backend.sv (HEAD 629723a)

Goal: stall_wb/ex/dc2/dt/dc1/ag become flip-flops stall_X_q whose D is the
next value of today's combinational chain, exactly (checked), so the chain
and its fan-out (every stage enable, the DMU's dm_adv copies, the fpif's
adv) start from registers.  Convention below: X_n = the D-input of
register X as the code already forms it; "late pick" = the "both
candidates, the late term selects last" pattern used throughout.

## 1. Terms and their next values (file:line of today's form)

Backend (rtl/ap68060_backend.sv):

| term | today | next value (hoist to a wire) |
|---|---|---|
| wb_v | 2576 `if (!stall_wb) wb_v <= adv_ex`, flush 2764, kill 2794, reset 2149 | `wb_v_n = !(flush \|\| kill_now \|\| !nreset) && (stall_wb ? wb_v : adv_ex)` |
| ex_v | 2372 | `ex_v_n = !(flush \|\| kill_now \|\| !nreset) && (stall_ex ? ex_v : adv_dc2)` (exists as v_n, 1955) |
| dc2_v | 2308 | same shape with stall_dc2 / adv_dt |
| dt_v | 2290 | same with stall_dt / adv_dc1 |
| dc1_v | 2264 | same with stall_dc1 / adv_ag |
| ag_v | 2769 (`!stall_ag`), flush/kill | `agv_n` as written at 2976: `!(flush\|\|kill_now\|\|!nreset) && (stall_ag ? ag_v : (ain_v && !stopped && !halted))` |
| wbh_q | 1925 `nreset && (adv_ex ? h_new : h_stay)` | `wbh_n` = that expression (h_new/h_stay 1900-1924) |
| wb_stv | 2416 (entering), 2726/2741 (cleared by a store or PLPA fault while staying) | `wb_stv_n = adv_ex ? ((ex_u.mem==M_ST\|\|ex_u.mem==M_RMW) && !ex_stkill && ok_new) : (wb_stv && !(wb_v && wb_st && dm_st_rdy && dm_st_fault && wb_exc==0) && !(wb_v && wb_mt && mt_done && wb_u.cond==4'd5 && mt_fault && wb_exc==0))` |
| exh_q | 1964 | `exh_n` = `v_n && ((mdw_n && !z_n && !fin_n) \|\| (slow_n && sp_n != np_n))` (1955-1964) |
| ex_fpw (twin ex_fpw_h) | 2361 at adv_dc2 | `ex_fpw_n = adv_dc2 ? ((dc2_u.op==OP_FPU) && !((dc2_u.mem!=M_NONE) && dm_fault) && dc2_u.exc==0) : ex_fpw` |
| fp_hold (fpif 1603) | `ex_v && (oq.hs \|\| (!safe_q && !oq.ns))` | `fph_n = ex_v_n && (hs_n \|\| (!safe_n && !ns_n))`, `{hs_n,ns_n} = (ex_v && !adv_ex) ? o0.{hs,ns} : o1.{hs,ns}` (fpif 1598; export o0/o1's hs, ns) |
| safe_q | 264-277 | `safe_n` = the block's `nv` |
| dc2_mx | 2297 at adv_dt | `dc2_mx_n = adv_dt ? ((dt_u.mem!=M_NONE) && dt_u.exc==0) : dc2_mx` |
| ilk_q | 2977 | `ilk_n = agv_n && ((nb\|nx\|nu\|nu2) != 0)` (nb.. use wf_next 2891, which reads adv_*/stall_* -- registers after the change) |
| flush, kill_now | flush is a register (2628/2642/2678); `kill_now = adv_wb ? (wb_is_exc\|\|wb_redir\|\|xgo1) : xgo0` (2048) | used as is (adv_wb = wb_v && !stall_wb_q) |

DMU (rtl/ap68060_dmu.sv) -- export the D of each hold as an output *_n:

| hold | today | export |
|---|---|---|
| hold1_q | 3030 `frz_n \|\| steal \|\| stl_set \|\| (m1v_n && m1s_n) \|\| (mtv_n && mts_n) \|\| busy_n` (locals 3005-3029) | `hold1_n = nreset ? <expr> : hold1_q` (no explicit reset today: the block is under `else`) |
| dc2r_q | 3038 `adv_dt ? r_new : (adv_dc2 \|\| r_stay)` | `dc2r_n`, same reset rule |
| st_ok_q | 3056 `adv_ex ? ok_new : ok_stay` | `st_ok_n`, same |
| hold_dt | 1467 `adv_dc1 ? h1 : (!adv_dt && ht)`, reset 1294 | `hold_dt_n = nreset ? (adv_dc1 ? h1 : (!adv_dt && ht)) : 1'b0` |

The DMU's adv_* are dm_adv[] (ap68060.sv 328-332), copies of the backend's;
after the change they are `X_v && !stall_X_q` -- one LUT from registers.

## 2. The six registers

    // next valids (one shape each)
    wire kf     = flush || kill_now || !nreset;
    wire wb_v_n  = !kf && (stall_wb  ? wb_v  : adv_ex);
    wire ex_v_n  = !kf && (stall_ex  ? ex_v  : adv_dc2);
    wire dc2_v_n = !kf && (stall_dc2 ? dc2_v : adv_dt);
    wire dt_v_n  = !kf && (stall_dt  ? dt_v  : adv_dc1);
    wire dc1_v_n = !kf && (stall_dc1 ? dc1_v : adv_ag);
    wire ag_v_n  = !kf && (stall_ag  ? ag_v  : (ain_v && !stopped && !halted));
    // holds next (wbh_n, exh_n, ilk_n, safe_n hoisted from their blocks;
    // hs0/ns0/hs1/ns1 new fpif outputs; *_n new DMU outputs)
    wire hs_n = (ex_v && !adv_ex) ? fp_hs0 : fp_hs1;
    wire ns_n = (ex_v && !adv_ex) ? fp_ns0 : fp_ns1;
    wire fph_n   = ex_v_n && (hs_n || (!safe_n && !ns_n));
    wire exho_n  = exh_n || (ex_v_n && ex_fpw_n && fph_n);
    wire dc2h_n  = dc2_v_n && dc2_mx_n && !dm_dc2r_n;
    wire swb_n   = wb_v_n  && (wbh_n || (wb_stv_n && !dm_st_ok_n));
    wire sex_n   = ex_v_n  && (exho_n || swb_n);
    wire sdc2_n  = dc2_v_n && (dc2h_n || sex_n);
    wire sdt_n   = dt_v_n  && (sdc2_n || dm_hold_dt_n);
    wire sdc1_n  = dc1_v_n && (sdt_n  || dm_hold1_n);
    wire sag_n   = ag_v_n  && (ilk_n  || sdc1_n);
    always_ff @(posedge clk) begin
        stall_wb_q <= swb_n; stall_ex_q <= sex_n; stall_dc2_q <= sdc2_n;
        stall_dt_q <= sdt_n; stall_dc1_q <= sdc1_n; stall_ag_q <= sag_n;
    end
    assign stall_wb = stall_wb_q; ... assign stall_ag = stall_ag_q;
    // (the old chain, for the check only)
    // synthesis translate_off
    wire c_wb  = wb_v && (wbh_q || (wb_stv && !dm_st_ok));
    wire c_ex  = ex_v && (ex_hold || c_wb);          // ex_hold as today
    wire c_dc2 = dc2_v && (dc2_hold || c_ex);
    wire c_dt  = dt_v && (c_dc2 || dm_hold_dt);
    wire c_dc1 = dc1_v && (c_dt || dm_hold1);
    wire c_ag  = ag_v && (ilk_q || c_dc1);
    always_ff @(posedge clk)
        if (nreset && {stall_wb,stall_ex,stall_dc2,stall_dt,stall_dc1,stall_ag}
                      !== {c_wb,c_ex,c_dc2,c_dt,c_dc1,c_ag}) begin
            $display("FAIL: registered stalls %b, the chain %b", ...); $finish;
        end
    // synthesis translate_on

Late picks (see section 4): write swb_n..sag_n not as the flat ORs above
but as `ok_new ? S_a : S_b` (two full copies of each, ok_new the last
2:1), and inside the adv_dt branch of dc2h_n as `tk_fast ? .. : ..`
(tk_fast exported from the DMU with dc2r_n split into its tk_fast=1/0
halves: dc2r_n1, dc2r_n0).  Synthesis will not do this by itself.

DMU exports (dmu block 3005-3067 and 1440-1467): turn the local
computations into module-level always_comb producing hold1_n, dc2r_n0/1,
st_ok_n (and keep `hold1_q <= hold1_n` etc.), add outputs, wire in
ap68060.sv, add backend inputs dm_hold1_n, dm_dc2r_n0/1, dm_hold_dt_n,
dm_st_ok_n, dm_tk_fast.  fpif: outputs hs0 = o0.hs, ns0 = o0.ns, hs1, ns1.

Hoists in the backend: wbh_n (block 1900-1925 -> always_comb + flop),
exh_n (1955-1964), safe_n (264-277), ilk_n (2935-2977: nb/nx/nu/nu2 and
agv_n are computed in a clocked block with blocking locals -- move the
flag arithmetic to an always_comb or duplicate agv_n/ilk expression as
wires).

## 3. Risks / open questions

- Exactness of every hoisted D: a missed in-place update of a term (as
  with ag_cancel for mxc_a) shows up as the check firing in a test that
  exercises it; the check is cheap, run the full regression and the cputest
  corpus sample.  wb_stv's two in-place clears (2726, 2741) are the
  easy-to-miss ones.
- kill_now is combinational and includes adv_wb, xgo1 (irq/trace/sb
  error terms, dbl1) -- 2-3 levels from registers; it gates every X_v_n.
  Acceptable in the D cone.
- DMU exports are combinational cones placed in the DMU feeding backend
  flops: the DMU->backend wire (1.5-2 ns) now sits inside a full-cycle D
  path instead of ahead of the chain.  Keep the preserved twins (hold*_b2)
  only for the check, or drop them.
- fpif o0/o1 (pred_f, out_f, 1566-1591) read the FPU's outputs of this
  cycle (f_done, f_excreq...): their depth is unknown; if hs_n/ns_n arrive
  late, fph_n could be formed as hs/ns-last picks too (two copies of
  sex_n..sag_n per candidate = 4 copies with ok_new; still small logic).
- Conservative substitutes are NOT needed: every term is a register's D;
  the only already-conservative term is safe_q (its D stays as is), so
  the registered stalls equal the chain by construction.
- dma_* kept copies (360-365) and dm_kill: become `X_v && !stall_X_q`;
  keep them (fan-out separation) or drop.  The fpif's adv = adv_ex is then
  a 1-LUT function of two registers.
- Data movement that reads adv_*/stall_* (every stage register's enable,
  wf_next, the capture selects cb_n/cx_n, mx*/mxt*/mxc*/mxg* D-inputs,
  rran selects ag_ld/sk_ld, the fv/lvt pending writes): unchanged
  semantically; all now see a flop one LUT away instead of the chain's end.
- The hold registers wbh_q/exh_q/ilk_q/st_ok_q/dc2r_q/hold1_q/hold_dt and
  the chain can be removed from synthesis once the stall registers exist
  (kept for the check); wb_hold (the un-predicted WB hold, 264) stays
  checked against wbh_q as today.

## 4. Critical D-input depth

- swb_n: wbh_n's h_new contains ok_new = ex_xok && !ex_fault && !ex_xnz
  && !ex_odd; ex_xnz has the ALU's condition (alu_cc) -- the EX critical
  path.  Today wbh_q's D is already alu_cc + ~2 levels and has not
  appeared among the failing endpoints.  With the OR chain swb->sag added
  naively the fan-in grows by ~6 terms: +1-2 LUT levels after ok_new.
  With the late pick (every S_x as `ok_new ? S_xa : S_xb`, S_xa/S_xb
  precomputed from registers and the other *_n), the depth after ok_new
  is one 2:1 mux, equal to today's wbh_q.  ok_new fans out to 6+2 muxes.
- sdc2_n: dc2r_n's r_new contains tk_fast (dmu 739: DT's tag compare /
  fast decision, the other deep path).  Same treatment: tk_fast last.
- Everything else (hold1_n ~3 levels of registers and e_st; hold_dt_n ~3;
  st_ok_n ~3 incl. sbn_n; exh_n ~3 incl. ex_div0's zero test; safe_n ~3
  incl. a 32-bit pc compare; ilk_n ~3; the valids 2) is within 4 levels,
  so each S_x's non-late part is ~5 levels + one DMU->backend wire:
  ~5-6 ns.  Expected worst D: alu_cc arrival + 1 mux, and DT tag compare +
  1 mux -- both already in the design.
- Area: ~6 flops + ~60-100 LUTs of duplicated candidate logic.
