//--------------------------------------------------------------------------//
// AP68060 - pipelined MC68040                                            //
//                                                                          //
// ap68060_backend.sv - AG, DC1, DT, DC2, EX and WB stages                   //
//                                                                          //
//   AG   read the front register file, effective address, (An)+/-(An)     //
//        and LEA-class register updates (written to the front file)       //
//   DC1  data ATC lookup and tag read (ap68060_dmu)                        //
//   DT   tag compare, translation and the fast-path decision (dmu)         //
//   DC2  way select and operand alignment (ap68060_dmu)                    //
//   EX   ALU, shifter, multiply/divide, branch resolution, front-file      //
//        result write                                                      //
//   WB   back (architectural) register file, CCR/SR, stores, exceptions,  //
//        interrupts, trace, serialization                                  //
//                                                                          //
// Operand values are read at AG.  A result produced by an older uop in EX  //
// is broadcast and captured by every younger uop in AG/DC1/DT/DC2 whose    //
// operand names that register, so EX never forwards; address operands     //
// (base, index, register updates) instead interlock at AG on any older    //
// uop that will write the register in EX (registered flags per stage).    //
//                                                                          //
// Every redirect (branch mispredict, exception, serialization) happens in  //
// WB, from state registered at the end of EX.                              //
//--------------------------------------------------------------------------//

module ap68060_backend
	import ap68060_pkg::*;
#(
	parameter bit         FPU_PIPE  = 1,
	parameter bit         EARLY_RES = 1,
	parameter bit         SKID      = 1,      // the skid register in front of AG
	parameter logic [7:0] PCR_REV   = 8'h06       // PCR revision number (bits 15-8)
)(
	input  logic        clk,
	input  logic        nreset,

	// uops from the sequencer
	input  logic        in_v,
	input  logic        in_eb,          // !in_v's twin register (AG's capture select)
	input  uop_t        in_u,
	output logic        in_rdy,
	input  logic  [1:0] uq_n,           // the sequencer's queue count, and the
	input  logic  [4:0] nx_b0, nx_i0,   // head's next base/index candidates:
	input  logic  [4:0] nx_b1, nx_i1,   // D2c's uop's, the queue's other word's

	// redirect of the front end (registered)
	output logic        redir_v,
	output logic [31:0] redir_pc,
	output logic        flush,          // kill everything younger than WB
	output logic        fe_flush,       // the front end's flush (D1, D2)
	output logic        ras_rv,         // D1's return stack to restore with it
	output logic  [2:0] ras_rtp,
	output logic  [3:0] ras_rn,

	// exception entry for the sequencer
	output logic        exc_go,         // pulse: run the exception routine
	output logic  [3:0] exc_kind,       // routine selector (see EK_*)
	output logic  [4:0] exc_ssp,        // supervisor stack register to use

	// micro-branch result back to the sequencer
	output logic        ucond_v,
	output logic        ucond,

	// architectural state the front end needs
	output logic [15:0] sr,
	output logic [31:0] vbr,
	output logic [31:0] cacr,
	output logic        bc_clr,         // MOVEC to CACR with CABC or CUBC: clear the branch cache
	output logic  [2:0] sfc,
	output logic  [2:0] dfc,

	// data memory unit (DC1/DC2 lookups; WB stores)
	output logic        dm_req,         // AG -> DC1: a memory uop advances
	output logic [31:0] dm_va,
	output logic  [1:0] dm_mem,
	output logic  [1:0] dm_msz,
	output logic  [2:0] dm_fc,
	output logic        dm_lock,
	output logic        dm_locke,
	output logic        dm_super,
	output logic        dm_older,       // uops older than DC2 are in EX or WB
	input  logic        sn_ihit,        // a snoop dropped valid instruction-cache data
	input  logic        tw_busy,        // the DMU runs a table search
	output logic  [4:0] pst5,           // PST4-PST0 this clock (MC68060UM table 2-7)
	output logic        irq_pending,    // IPEND: a request beats the mask
	output logic        bht_we,         // Bcc history training (decode's table)
	output logic  [7:0] bht_wa,
	output logic        bht_dis,        // ... the branch went against the static rule
	output logic        lp_we,          // loop exit training (decode's table)
	output logic [11:0] lp_wa,
	output logic        lp_tk,
	output logic        lp_mp,
	output logic        dm_noalloc,     // exception stacking / vector fetch
	output logic  [1:0] dm_ack,         // acknowledge cycle: 1 IACK, 2 BKPT, 3 LPSTOP
	input  logic        dm_hold1,       // the DMU holds DC1 (a register)
	input  logic        dm_hold1_b, dm_hold_dt_b, dm_dc2_rdy_b, dm_st_ok_b,   // the four holds' twins, for the stall chain alone
	input  logic        dm_hold1_n, dm_dc2r_n, dm_st_ok_n, dm_hold_dt_n,   // ... and their next values (dc2r conservative), for the registered stalls
	input  logic        dm_hold1_nk, dm_dc2r_n1, dm_dc2r_n0, dm_rdy_dt,   // hold1_n without the kill; dc2r_n for DT's late decision 1 / 0
	input  logic        dm_hold1_c,     // ... as its state gives it now (checks)
	input  logic        dm_hold_dt,     // ... and DT (a register)
	// WB maintenance (CINV/CPUSH/PFLUSH/PTEST) in the DMU
	output logic        mt_v,
	output logic  [2:0] mt_op,
	output logic  [1:0] mt_scope,
	output logic  [1:0] mt_caches,
	output logic [31:0] mt_addr,
	output logic  [2:0] mt_fc,
	output logic        mt_ng,
	output logic        mt_wr,
	input  logic        mt_done,
	input  logic [31:0] mt_pa,          // PLPA: the physical address
	input  logic        mt_fault,       // ... or its access error
	input  logic [31:0] mt_fslw,
	input  logic        dm_lkend,       // a LOCKE write completed (BUSCR sequences)
	input  logic        dm_iack_end,    // the interrupt acknowledge cycle ended
	input  logic        sb_empty,       // the DMU's store buffer is empty
	input  logic        sb_ne_n,        // ... and next cycle it holds a write
	input  logic        sb_err,         // ... and one of its writes, or a push, ended in TEA
	input  logic [31:0] sb_fslw,        // ... FSLW.SBE / FSLW.PBE
	output logic        sb_err_ack,     // the access error for it is being taken
	output logic        adv_dc1,        // DC1 -> DT
	output logic        adv_dt,         // DT -> DC2
	output logic        adv_dc2,        // DC2 -> EX
	output logic        adv_ex,         // EX -> WB
	output logic        adv_wb,         // WB completes
	input  logic        dm_dc2_rdy,     // DC2 memory op has its result (a register)
	input  logic        dm_dc2_rdy_c,   // ... as the DMU's state gives it now (checks)
	input  logic [31:0] dm_ldata,       // load data, right aligned
	input  logic [31:0] dm_ldata_al,    // the aligned operand (AG's capture)
	input  logic        dm_ldata_alv,   // ... is the load's data
	input  logic        dm_fault,       // DC2 access fault (with info below)
	input  logic  [7:0] dm_fvec,        // 2 access error, 3 address error
	input  logic [31:0] dm_faddr,
	input  logic [31:0] dm_fslw,        // ... its fault status long word
	input  logic [31:0] if_cause,       // the fetch's translation fault cause (FSLW)
	output logic        dm_st_v,        // WB store data valid this cycle
	output logic [31:0] dm_st_data,
	input  logic        dm_st_rdy,      // store accepted (WB may complete)
	input  logic        dm_st_fault,
	input  logic        dm_st_ok,       // ... without a fault (a register, a clock ahead)
	input  logic [31:0] dm_st_fslw,
	input  logic [31:0] dm_st_faddr,

	// interrupts (synchronized level, 0-7)
	input  logic  [2:0] ipl,
	output logic  [2:0] iack_lvl,
	output logic        rsto_req,
	input  logic        rsto_busy,

	output logic        kill_now,       // this cycle: younger uops are discarded
	output logic        lk_abandon,     // an open locked sequence is abandoned
	output logic        adv_ag,         // AG -> DC1
	output logic  [5:0] dm_adv,         // the DMU's copies: {AG, DC1, DT, DC2, EX, WB}
	output logic        dm_kill,        // ... and of kill_now
	output logic [31:0] dtt0,
	output logic [31:0] dtt1,
	output logic [31:0] itt0,
	output logic [31:0] itt1,
	output logic [31:0] tc,
	output logic [31:0] urp,
	output logic [31:0] srp,

	// debug
	output logic [31:0] dbg_pc,
	output logic        dbg_retire,
	output logic        halted,
	output logic        be_idle,        // no uop in the back end (registered state)
	output logic        fetch_stop      // no new instruction fetches: stopped
                                        // (STOP/LPSTOP) or halted
);

// the SR bits the 68060 keeps: T1 S M I2-I0 X N Z V C (no T0)
localparam logic [15:0] SR_MASK = 16'hB71F;

//--------------------------------------------------------------------------
// exception routine kinds (sequencer entry points)
//--------------------------------------------------------------------------
localparam logic [3:0] EK_FMT0  = 4'd0;   // format $0 (PC = exc_pc)
localparam logic [3:0] EK_FMT2  = 4'd1;   // format $2 / $3 (+ address)
localparam logic [3:0] EK_FMT4  = 4'd2;   // format $4: access error, FP disabled
localparam logic [3:0] EK_IRQ   = 4'd3;   // interrupt (IACK + format $0)
localparam logic [3:0] EK_RESET = 4'd4;   // reset: SSP/PC from 0/4

//--------------------------------------------------------------------------
// pipeline registers
//--------------------------------------------------------------------------
logic        ag_v, dc1_v, dt_v, dc2_v, ex_v, wb_v;
uop_t        ag_u, dc1_u, dt_u, dc2_u, ex_u, wb_u;
logic [31:0] ex_npc, wb_npc;  // EX's and WB's next instruction (pc + 2 ilen)

// operand values carried with the uop
logic [31:0] dc1_a, dc1_b, dc1_ea;
logic [31:0] dt_a, dt_b, dt_ea;
logic [31:0] dc2_a, dc2_b, dc2_ea;
logic [31:0] ex_a,  ex_b,  ex_ea,  ex_ld;
logic        ex_fault;
logic  [7:0] ex_fvec;
logic [31:0] ex_faddr;
logic [31:0] ex_fslw;

// results carried into WB
logic [31:0] wb_res;
logic  [4:0] wb_ccr;          // CCR after this uop
logic        wb_dwe;          // result register write (after EX kills)
logic [31:0] wb_upd_val, wb_upd2_val;
logic        wb_redir;        // the uop redirects the front end
logic [31:0] wb_redir_pc;
logic  [7:0] wb_exc;          // exception vector, 0 none
// (its cases the WB decisions use, registered with it: xflags)
localparam int XF_POST = 0, XF_RF = 1, XF_23 = 2, XF_2 = 3;
logic  [3:0] wb_xf;
function automatic logic [3:0] xflags(input logic [7:0] v);
	xflags = {v == 8'd2, v == 8'd2 || v == 8'd3, v == EXC_IFS || v == EXC_IFSA,
	          (v >= 8'd32 && v < 8'd48) || v == 8'd5 || v == 8'd6 || v == 8'd7};
endfunction
logic        wb_exc_pcv;      // the frame PC is wb_exc_pc (odd change of flow)
logic [31:0] wb_exc_pc;
logic        wb_oddrte;       // RTE to an odd PC: its SR is committed first
logic        wb_hodd;         // the handler address of an exception is odd
logic        xm_v, xm_m;      // the routine running is an interrupt's; its old M
logic        wb_exc_fmtv;     // the frame format is wb_exc_fmt (FPU exceptions)
logic  [3:0] wb_exc_fmt;
logic        wb_xcommit;      // the uop's address updates stand (FPU unimp/unsupp)
logic [31:0] wb_exc_addr;
logic [31:0] wb_exc_fslw;     // FSLW of an access error
logic        wb_st;           // store at WB
logic [31:0] wb_st_data;
logic [15:0] wb_sr_new;       // SR write
logic        wb_sr_we;

// AG register updates travel with the uop for the back file
logic [31:0] dc1_upd, dc1_upd2, dt_upd, dt_upd2, dc2_upd, dc2_upd2, ex_upd, ex_upd2;

//--------------------------------------------------------------------------
// register files
//--------------------------------------------------------------------------
// Register files (ap68060_rf, MLAB): the front file holds the newest
// values (AG address updates, EX results), the back file the architectural
// ones (WB).  fv[r] says the front copy of r is current; a redirect or an
// exception clears fv, so every register reads the back file again -- no
// copy.  Five read ports at AG: base, index, upd2 register, A, B.
logic [31:0] fv;
// an FPU uop may act once it is the oldest in flight -- or when the one at
// WB is an earlier uop of its own instruction that commits: no exception
// (a load's fault is known by now), not the last uop (no instruction
// boundary between them: no interrupt or trace), and not a store (a bus
// error on a store comes after WB)
// (ew_same: ex_u.pc == wb_u.pc, kept as the two load -- the compare
// stays off the FPU's hold, and so the stall chain)
logic ew_same;
logic wb_ok;                  // WB's uop has no exception (wb_exc == 0), registered
// the holds' next values (for the registered stalls, at the end of this
// module), and the same with the kill taken as 0; declared here, ahead of
// the blocks that form them
logic wbh_n, exh_n, safe_n, ilk_n;
logic exh_nk, safe_nk, ilk_nk;
logic  [2:0] ex_kx;           // EX's uop is FPU, DIV, TRAPcc (registered as it enters)
wire wb_fpchk = wb_v && !wb_u.last && wb_ok &&
                (wb_u.mem == M_NONE || wb_u.mem == M_LD) &&
                ex_v && ex_kx[2] && ew_same;     // (ex_kx[2]: EX's uop is FPU, registered)
// The FPU interface's safe (no older uop in flight: !wb_v || wb_fpchk) as
// a register formed a clock ahead, so its hold is one LUT of flip-flops
// and not the root of the stall chain.  Exact from the registers in every
// case but one: a uop leaving EX as the FPU's enters, where WB's next
// exception decision is EX's of this cycle -- there a sufficient condition
// (a plain uop, no unit, no branch) stands in, and safe may be a clock
// late (conservative: checked below as safe_q implies safe).
logic safe_q;
function automatic logic memok(input uop_t u);
	memok = (u.mem == M_NONE || u.mem == M_LD);
endfunction
wire ex_okc   = (ex_u.exc == 8'd0) && !ex_snw && !ex_fault && (ex_kx == 3'd0) && !ex_br;
wire dc2_fpu  = (dc2_u.op == OP_FPU);
always_comb begin
	logic nv;
	if (flush || kill_now || !nreset) nv = 1'b1;
	else if (stall_wb)
		nv = !wb_v || (!wb_u.last && wb_ok && memok(wb_u) &&
		               (ex_v ? (ex_kx[2] && ew_same)
		                     : (adv_dc2 && dc2_fpu && (dc2_u.pc == wb_u.pc))));
	else if (adv_ex)
		nv = !ex_u.last && ex_okc && memok(ex_u) && adv_dc2 && dc2_fpu && (dc2_u.pc == ex_u.pc);
	else nv = 1'b1;
	safe_n = nv;
end
// (the same with the kill taken as 0: a kill clears every stage's valid,
// so the stalls are zero whatever this is; its logic stays off their D)
always_comb begin
	logic nv;
	if (flush || 1'b0 || !nreset) nv = 1'b1;
	else if (stall_wb)
		nv = !wb_v || (!wb_u.last && wb_ok && memok(wb_u) &&
		               (ex_v ? (ex_kx[2] && ew_same)
		                     : (adv_dc2 && dc2_fpu && (dc2_u.pc == wb_u.pc))));
	else if (adv_ex)
		nv = !ex_u.last && ex_okc && memok(ex_u) && adv_dc2 && dc2_fpu && (dc2_u.pc == ex_u.pc);
	else nv = 1'b1;
	safe_nk = nv;
end
always_ff @(posedge clk) safe_q <= safe_n;
// synthesis translate_off
always_ff @(posedge clk)
	if (nreset && safe_q && !(!wb_v || wb_fpchk)) begin
		$display("FAIL: the FPU interface's safe register %b, the check %b (wb_v %b fpchk %b)", safe_q, !wb_v || wb_fpchk, wb_v, wb_fpchk);
		$finish;
	end
// synthesis translate_on
// FPU interface outputs (ap68060_fpif)
logic        fp_hold, fp_dkill, fp_taken, fp_xnext, fp_xcommit, fp_stkill;
logic        fp_hs0, fp_ns0, fp_hs1, fp_ns1;   // fpif's next-state candidates' hold flags
logic [31:0] fp_res, fp_xaddr;
logic  [7:0] fp_xvec;
logic  [3:0] fp_xfmt;
logic  [7:0] fp_mvslots;
logic  [2:0] fp_crslots;
logic  [2:0] fwe, bwe;
logic  [1:0] fwe_ag;           // fwe[0..1] without the advance (AG holds an update)
logic  [4:0] fwa [3], bwa [3];
logic [31:0] fwoh [3], bwoh [3];   // ... one-hot, from registers set as the uops enter
logic [31:0] ag_uoh, ag_u2oh;      // AG's update registers
logic [31:0] ex_doh;               // EX's destination
logic [31:0] wb_uoh, wb_u2oh, wb_doh;
logic [31:0] fwd [3], bwd [3];
logic  [4:0] rra [7];
logic [31:0] frd [7], brd [7], rrd [7];
logic  [4:0] ccr_f, ccr_b;

// special registers (architectural, written at WB)
logic [15:0] sr_r;            // T1 T0 S M 0 I2 I1 I0 (CCR lives in ccr_b)
logic [31:0] vbr_r, cacr_r;
logic  [2:0] sfc_r, dfc_r;
logic [31:0] tc_r, itt0_r, itt1_r, dtt0_r, dtt1_r, urp_r, srp_r;
// 68060: PCR (EDEBUG, DFP, ESS; the identification and revision are
// constants) and BUSCR (L, SL, LE, SLE)
logic        pcr_edebug, pcr_dfp, pcr_ess;
logic        bus_l, bus_sl, bus_le, bus_sle;

// exception information for the exception microroutine
logic [31:0] xi_pc, xi_addr, xi_fslw;
logic [15:0] xi_sr, xi_vecw;
logic [31:0] xi_vaddr;
logic  [1:0] xw_ph;           // exception entry: back-file writes of T8-T13

assign sr   = {sr_r[15:5], ccr_b};
assign dtt0 = dtt0_r;
assign dtt1 = dtt1_r;
assign itt0 = itt0_r;
assign itt1 = itt1_r;
assign tc   = tc_r;
assign urp  = urp_r;
assign srp  = srp_r;
assign vbr  = vbr_r;
assign cacr = cacr_r;
assign sfc  = sfc_r;
assign dfc  = dfc_r;

//--------------------------------------------------------------------------
// stall / advance (bubbles collapse: a stage moves into an empty one)
//--------------------------------------------------------------------------
logic wb_hold, ex_hold, dc2_hold, ag_hold;
logic stall_wb, stall_ex, stall_dc2, stall_dt, stall_dc1, stall_ag;
// Every hold the chain reads is a register, formed a clock ahead (or from
// registers in one level): WB's (wbh_q, and the DMU's store answer), EX's
// (from flags registered as the uop enters), DC2's and DC1's (the DMU's),
// DT's (the DMU's), AG's interlock (ilk_q).  A registered hold may be on a
// clock longer than the stage's state needs, never shorter (checked).
logic wbh_q;                  // WB holds but for its store's answer
// MOVEC's control register at WB, one-hot (decoded as the uop enters WB)
localparam int CR_SFC = 0, CR_DFC = 1, CR_CACR = 2, CR_TC = 3, CR_ITT0 = 4, CR_ITT1 = 5,
               CR_DTT0 = 6, CR_DTT1 = 7, CR_BUSCR = 8, CR_VBR = 9, CR_URP = 10, CR_SRP = 11,
               CR_PCR = 12;
logic [12:0] wb_crs;
function automatic logic [12:0] cr_sel(input uop_t u);
	logic [12:0] c;
	c = '0;
	if (u.op == OP_SPW)
		case (u.imm_b[7:0])
			8'h00: c[CR_SFC]   = 1'b1;
			8'h01: c[CR_DFC]   = 1'b1;
			8'h02: c[CR_CACR]  = 1'b1;
			8'h03: c[CR_TC]    = 1'b1;
			8'h04: c[CR_ITT0]  = 1'b1;
			8'h05: c[CR_ITT1]  = 1'b1;
			8'h06: c[CR_DTT0]  = 1'b1;
			8'h07: c[CR_DTT1]  = 1'b1;
			8'h08: c[CR_BUSCR] = 1'b1;
			8'h11: c[CR_VBR]   = 1'b1;
			8'h16: c[CR_URP]   = 1'b1;
			8'h17: c[CR_SRP]   = 1'b1;
			8'h18: c[CR_PCR]   = 1'b1;
			default: ;
		endcase
	cr_sel = c;
endfunction
logic dc2_mx;                 // DC2's uop accesses memory, no exception (registered)
logic ex_mdw, ex_fpw;         // EX's uop runs the multiply/divide unit, the FPU
logic ilk_q;                  // AG's interlock
wire  wb_hold_r = wb_v && (wbh_q || (wb_stv && !dm_st_ok_b));

// EX holds for a multicycle unit; DC2 for memory; WB for stores/serial ops
logic md_busy;                // multiply/divide running (register)
logic ex_md_start;
// DC2 holds until the data memory unit has its result
assign dc2_hold  = dc2_v && dc2_mx && !dm_dc2_rdy_b;
// Every stage's stall is a REGISTER, formed a clock ahead from the next
// values of the terms the chain below reads (at the end of this module:
// the holds' next values, hoisted from their prediction blocks, and the
// valids' next values).  The chain itself, c_*, remains for the check: the
// registers equal it, except that a DMU "not ready" may be a clock longer
// (dc2r_n is conservative) -- never shorter.
wire c_wb  = wb_hold_r;
wire c_ex  = ex_v  && (ex_hold  || c_wb);
wire c_dc2 = dc2_v && (dc2_hold || c_ex);
wire c_dt  = dt_v  && (c_dc2 || dm_hold_dt_b);
wire c_dc1 = dc1_v && (c_dt  || dm_hold1_b);
wire c_ag  = ag_v  && (ag_hold  || c_dc1);

assign adv_wb  = wb_v  && !stall_wb;
assign adv_ex  = ex_v  && !stall_ex;
assign adv_dc2 = dc2_v && !stall_dc2;
assign adv_dt  = dt_v  && !stall_dt;
assign adv_dc1 = dc1_v && !stall_dc1;
assign adv_ag  = ag_v  && !stall_ag;
// the DMU's copies of the advances and the kill, separate nodes (keep), so
// each drives its own module's registers
(* keep *) wire dma_ag  = ag_v  && !stall_ag;
(* keep *) wire dma_dc1 = dc1_v && !stall_dc1;
(* keep *) wire dma_dt  = dt_v  && !stall_dt;
(* keep *) wire dma_dc2 = dc2_v && !stall_dc2;
(* keep *) wire dma_ex  = ex_v  && !stall_ex;
(* keep *) wire dma_wb  = wb_v  && !stall_wb;
assign dm_adv = {dma_ag, dma_dc1, dma_dt, dma_dc2, dma_ex, dma_wb};

// AG's input: a skid register in front of it, so the sequencer's pop does
// not wait for the stall chain.  The sequencer may pop whenever the skid is
// empty; a uop it pops that AG cannot take waits in the skid, and AG takes
// the skid's first.  The register file is read, and the forwarding formed,
// for the uop entering AG (ain_u), wherever it comes from.
logic        sk_v;
// (a copy of sk_v for the register file's read addresses alone: sk_v
// selects the whole uop, and its fan-out would delay the reads)
(* preserve *) logic sk_vr;
(* preserve *) logic sk_va;          // ... and for AG's capture select
uop_t        sk_u;
wire         ain_v = sk_v || in_v;
uop_t        ain_u;
assign ain_u   = sk_v ? sk_u : in_u;
// (without the skid the sequencer's pop waits for AG: the stall chain)
assign in_rdy  = (SKID ? !sk_v : !stall_ag) && !flush && !stopped && !eredir_pend;
wire         ag_take = !stall_ag && !stopped && !halted;   // AG loads ain_u

//--------------------------------------------------------------------------
// AG
//--------------------------------------------------------------------------
// interlock: AG's base, index and update registers must not have an older
// writer in DC1, DT, DC2 or EX (its value only exists after EX), nor one
// that left EX last cycle (its value reaches AG a clock later, exw_q).  The
// AG uop carries a flag per register and stage (wf_*[k]: k = 0 DC1, 1 DT,
// 2 DC2, 3 EX, 4 left EX), registered: a uop entering AG compares its
// registers with the stages' writers as they will be next cycle (the stage
// advances pick at the last level), a uop staying in AG moves its flags
// with the stages.  The interlock is an OR of flags, off the compares.
// A base or index is written into agb_q/agx_q by EX's broadcast a clock
// late (exw_q: EX's result reaches the register file, WB and the operand
// forwards, nothing else in its own clock), and by a plain load leaving DC2
// (dc2_capd: that writer is then not waited for in EX); an update register
// waits for every older writer (else the EX write of the captured load
// could land after its own).
function automatic logic wmatch(input logic v, input uop_t u, input logic [4:0] r);
	wmatch = v && u.d_v && u.d_reg == r;
endfunction
logic [4:0] wf_b, wf_x, wf_u, wf_u2;
wire ag_interlock = ag_v && (
	(ag_u.base_v && (wf_b != 5'd0)) || (ag_u.idx_v  && (wf_x  != 5'd0)) ||
	(ag_u.upd_v  && (wf_u != 5'd0)) || (ag_u.upd2_v && (wf_u2 != 5'd0)));
// EX's broadcast, a clock late (AG's captures)
logic        exw_qv;
logic  [4:0] exw_qreg;
logic [31:0] exw_q;

// the load whose data can be handed over: its result is the data itself
// (a long MOVE, MOVEA.W sign-extended, a memory-indirect pointer)
// (captured as the load moves to EX: adv_dc2 selects at the last level)
// (the uop's part registered as it enters DC2: the flag is one AND of
// flip-flops, the DMU's aligned-data valid -- a register too -- among them)
function automatic logic capx(input uop_t u);
	capx = u.d_v && u.op == OP_MOV && u.cond == 4'd0 && u.mem == M_LD &&
	       u.a_src == OS_MEM && u.sz == SZ_L && u.exc == 8'd0;
endfunction
logic       dc2_capx;
always_ff @(posedge clk)
	if (adv_dt) dc2_capx <= capx(dt_u);
wire        dc2_capd = dc2_capx && dm_ldata_alv && dc2_v;
// synthesis translate_off
always_ff @(posedge clk)
	if (dc2_v && dc2_capx !== capx(dc2_u)) begin
		$display("FAIL: DC2's capture flag %b, its uop's %b (pc %h)", dc2_capx, capx(dc2_u), dc2_u.pc);
		$finish;
	end
// synthesis translate_on
// (from the DMU's aligned word, not its general byte gather: a short path;
// a misaligned load reaches AG a cycle later, through EX's write)
wire [31:0] dc2_capv = dc2_u.a_sxw ? {{16{dm_ldata_al[15]}}, dm_ldata_al[15:0]} : dm_ldata_al;

assign ag_hold = ilk_q;

// (ports 2-6 address registers whose next value is known: their bank
// selects, and their front-or-back selects fsel_q, are registered from
// next clock's address, rran; ports 0/1 address the sequencer's uop)
// (every port's address is a register whose next value is one of up to
// three candidates: ports 0/1 the sequencer's head -- held, loaded from
// D2c's uop, or from the queue's other word --, ports 2-4 AG's uop held
// or loaded, 5/6 the skid's; the load decisions come late and pick last)
logic [4:0] rran0 [7], rran1 [7], rran2 [7];
logic [6:0] rrsel1, rrsel2;
localparam logic [6:0] RSEL = 7'b1111111;
ap68060_rf #(.NR(7), .REGSEL(RSEL)) rff (.clk(clk), .we(fwe), .wa(fwa), .woh(fwoh), .wd(fwd), .ra(rra),
                                         .ran0(rran0), .ran1(rran1), .ran2(rran2), .rsel1(rrsel1), .rsel2(rrsel2), .rd(frd));
ap68060_rf #(.NR(7), .NW(2), .REGSEL(RSEL)) rfb (.clk(clk), .we(bwe), .wa(bwa), .woh(bwoh), .wd(bwd), .ra(rra),
                                                 .ran0(rran0), .ran1(rran1), .ran2(rran2), .rsel1(rrsel1), .rsel2(rrsel2), .rd(brd));
// AG loads ain_u when it takes it (not under a flush); the skid loads the
// sequencer's uop when AG does not take it; the sequencer's head takes
// D2c's uop when its queue is empty or pops its last word, the other
// queued word when it pops with two queued (uq_pop = in_rdy && in_v)
wire ag_ld = !flush && ag_take && ain_v;
wire sk_ld = !flush && !sk_v && SKID && in_v && in_rdy && !ag_take;
wire hd_nu = (uq_n == 2'd0) || (in_rdy && uq_n == 2'd1);
wire hd_ue = in_rdy && (uq_n == 2'd2);
assign rrsel1 = {sk_ld, sk_ld, ag_ld, ag_ld, ag_ld, hd_nu, hd_nu};
assign rrsel2 = {2'b00, 3'b000, hd_ue, hd_ue};
assign rran0[0] = in_u.base;        assign rran1[0] = nx_b0;            assign rran2[0] = nx_b1;
assign rran0[1] = in_u.idx;         assign rran1[1] = nx_i0;            assign rran2[1] = nx_i1;
assign rran0[2] = ag_u.upd2_reg;    assign rran1[2] = ain_u.upd2_reg;   assign rran2[2] = ag_u.upd2_reg;
assign rran0[3] = ag_u.a_reg;       assign rran1[3] = ain_u.a_reg;      assign rran2[3] = ag_u.a_reg;
assign rran0[4] = ag_u.b_reg;       assign rran1[4] = ain_u.b_reg;      assign rran2[4] = ag_u.b_reg;
assign rran0[5] = sk_u.base;        assign rran1[5] = in_u.base;        assign rran2[5] = sk_u.base;
assign rran0[6] = sk_u.idx;         assign rran1[6] = in_u.idx;         assign rran2[6] = sk_u.idx;
// the front-valid bits as they will be (fv's next value), and the ports'
// front-or-back selects from them
logic [31:0] fv_n;
logic  [6:0] fsel_q;
// front-valid bits: a front write makes the register's front copy
// current; a redirect, a refetch or an exception drops them all
// (the drop's term in four kept copies, a byte each: x_go comes late and
// one copy reached every bit)
(* keep *) wire [3:0] fv_clr = {4{(wb_commit && wb_redir) || wb_refetch || x_go || !nreset}};
// (fv is written a clock after the file, from the registered write fq_*,
// so its entries' enables come from flip-flops, not the late write
// enables; a select formed in between sees the pending write below.  A
// drop takes the pending write with it, as it took the write itself.)
logic  [2:0] fq_e;
logic  [4:0] fq_wa [3];
logic [31:0] fq_oh [3];
// (the drop too is applied a clock late, from its register fvc_q: fv's
// own enables then come from flip-flops alone; a select formed in the
// clock between sees the pending drop below, and a write in that clock
// -- after the drop -- survives it as a pending write)
logic fvc_q;
always_ff @(posedge clk) begin
	fvc_q <= fv_clr[0];
	for (int w = 0; w < 3; w++) begin
		fq_e[w]  <= fwe[w] && !fv_clr[0];
		fq_wa[w] <= fwa[w];
		fq_oh[w] <= fwoh[w];
	end
end
always_comb begin
	fv_n = fvc_q ? 32'd0 : fv;
	for (int i = 0; i < 3; i++) if (fq_e[i]) fv_n = fv_n | fq_oh[i];
end
// synthesis translate_off
// fv as written at once (the model the selects are checked against)
logic [31:0] fvm;
always_ff @(posedge clk) begin
	logic [31:0] m;
	m = fvm;
	for (int i = 0; i < 3; i++) if (fwe[i]) m = m | fwoh[i];
	if (fv_clr[0]) m = 32'd0;
	fvm <= m;
end
// synthesis translate_on
// (fv's own entry read at each candidate, this clock's writes -- their
// enables late -- and the drop applied after it, not through fv_n's 32:1)
function automatic logic fvn_at(input logic [4:0] a);
	logic v;
	v = fv[a] && !fvc_q;                                                   // (the pending drop)
	for (int w = 0; w < 3; w++) if (fq_e[w] && fq_wa[w] == a) v = 1'b1;   // pending writes
	for (int w = 0; w < 3; w++) if (fwe[w] && fwa[w] == a) v = 1'b1;     // this clock's
	fvn_at = v && !fv_clr[0];
endfunction
(* keep *) logic [6:0] fs0, fs1, fs2;   // (the candidates kept: the selects pick last)
always_comb
	for (int r = 0; r < 7; r++) begin
		fs0[r] = fvn_at(rran0[r]);
		fs1[r] = fvn_at(rran1[r]);
		fs2[r] = fvn_at(rran2[r]);
	end
always_ff @(posedge clk)
	for (int r = 0; r < 7; r++)
		fsel_q[r] <= (rrsel2[r] & fs2[r]) | (rrsel1[r] & !rrsel2[r] & fs1[r]) |
		             (!rrsel1[r] & !rrsel2[r] & fs0[r]);
// synthesis translate_off
always_ff @(posedge clk)
	for (int r = 0; r < 7; r++)
		if (fsel_q[r] !== fvm[rra[r]]) begin
			$display("FAIL: read port %0d's registered front select %b, fv's %b (register %0d)", r, fsel_q[r], fvm[rra[r]], rra[r]);
			$finish;
		end
// synthesis translate_on
// AG's base and index are read as the uop enters AG (ports 0/1 address
// the incoming uop) and kept in registers, so AG's address adders, and the
// data cache's set index after them, start from flip-flops
// (each source's own read port, the sequencer's 0/1 and the skid's 5/6,
// so the reads start from flip-flops and the skid's select comes after)
assign rra[0] = in_u.base;
assign rra[1] = in_u.idx;
// (the skid's base and index for the file's address ports from preserved
// twins, placed by the file; the uop's own copies stay with the skid)
(* preserve *) logic [4:0] sk_base_a, sk_idx_a;
assign rra[5] = sk_base_a;
assign rra[6] = sk_idx_a;
// AG's entering base and index from the file: front or back copy, the
// skid's or the sequencer's (= ain_u's)
wire [31:0] ain_fb = sk_vr ? (fsel_q[5] ? frd[5] : brd[5]) : (fsel_q[0] ? frd[0] : brd[0]);
wire [31:0] ain_fx = sk_vr ? (fsel_q[6] ? frd[6] : brd[6]) : (fsel_q[1] ? frd[1] : brd[1]);
// synthesis translate_off
always_ff @(posedge clk)
	if (nreset && wb_commit && wb_dwe && wb_u.upd2_v) begin
		$display("FAIL: a uop with both a register result and upd2 (pc %h): the back file has two write ports", wb_u.pc);
		$finish;
	end
always_ff @(posedge clk)
	if (nreset && (sk_vr != sk_v || sk_va != sk_v || in_eb == in_v)) begin
		$display("FAIL: the skid's copies sk_vr %b sk_va %b differ from sk_v %b, or in_eb %b from !in_v %b", sk_vr, sk_va, sk_v, in_eb, !in_v);
		$finish;
	end
// synthesis translate_on
assign rra[2] = ag_u.upd2_reg;
assign rra[3] = ag_u.a_reg;
assign rra[4] = ag_u.b_reg;
always_comb for (int i = 0; i < 7; i++) rrd[i] = fsel_q[i] ? frd[i] : brd[i];   // (every port's select registered)

logic [31:0] agb_q, agx_q;     // the base, and the index sign-extended
wire  [31:0] ag_base = agb_q;
// (the index is scaled here, a multiplexer before the carry-save step: the
// register file read as the uop enters AG has no room for it)
wire  [31:0] ag_ix   = agx_q << ag_u.scale;
function automatic logic [31:0] ixf(input logic [31:0] v, input uop_t u);
	ixf = u.idx_v ? (u.idx_l ? v : {{16{v[15]}}, v[15:0]}) : 32'd0;
endfunction
// three operands in one carry chain: a carry-save step (one LUT) and one
// adder, not two adders in a row
function automatic logic [31:0] add3(input logic [31:0] a, input logic [31:0] b,
                                     input logic [31:0] c);
	add3 = (a ^ b ^ c) + {((a & b) | (a & c) | (b & c)), 1'b0};
endfunction
wire [31:0] ag_ea   = add3(ag_base, ag_ix, ag_u.disp);
wire [31:0] ag_pinc = ag_base + {{24{ag_u.upd_amt[7]}}, ag_u.upd_amt};
wire [31:0] ag_updv = ag_u.pinc ? ag_pinc : ag_ea;
// (the file's write data from a second copy of the adders, kept: the one
// output's fanout to the banks, the forwarding and DC1 spread it too far)
(* keep *) wire [31:0] ag_updf = ag_u.pinc ? ag_base + {{24{ag_u.upd_amt[7]}}, ag_u.upd_amt}
                                           : add3(ag_base, ag_ix, ag_u.disp);
wire [31:0] ag_u2v  = rrd[2] + {{24{ag_u.upd2_amt[7]}}, ag_u.upd2_amt};
// bit field memory transfers sized by the field (see ucode.py _bf_mem_load)
// (the bit-field accesses' kinds registered as the uop enters AG, the
// field's byte count's cases as EX forms it: bf_n*)
logic       ag_bfh, ag_bft;
logic       bf_n1, bf_nle3, bf_n3, bf_n35;
// FPU transfers cancelled by run-time state: FMOVEM slots (dyn; the FPU
// interface latched the list in EX before these uops' addresses could be
// formed)
wire        ag_dyncan = (ag_u.dyn == 3'd1 && !fp_mvslots[ag_u.dynk[2:0]]);
wire        ag_cancel = (ag_bft && !bf_n35) || ag_dyncan;
wire  [1:0] ag_msz    = ag_bfh ? (bf_n1 ? SZ_B : bf_nle3 ? SZ_W : SZ_L) : ag_u.msz;
// the memory address: the cases are folded into the adder's operands
// (their selects are registers), so nothing follows the sum on its way
// to the data cache's set index
// (disp - 2 formed as the uop enters AG: ag_dm2)
logic [31:0] ag_dm2;
wire [31:0] ag_mdisp  = (ag_bft && bf_n3) ? ag_dm2 : ag_u.disp;
wire [31:0] ag_maddr  = add3(ag_base, ag_ix, ag_mdisp);

// operand read with the EX broadcast of this cycle (see the snoop below)
logic        exw_v;           // EX writes the front file this cycle
logic  [4:0] exw_reg;
logic [31:0] exw_val;

function automatic logic [31:0] opnd(input logic [1:0] src, input logic [4:0] r,
                                     input logic [31:0] rv,
                                     input logic [31:0] imm, input logic sxw);
	logic [31:0] v;
	case (src)
		OS_REG:  v = (exw_v && exw_reg == r) ? exw_val : rv;
		OS_IMM:  v = imm;
		default: v = 32'd0;
	endcase
	opnd = v;
endfunction

wire [31:0] ag_a = ag_u.a_upd ? ag_updv : opnd(ag_u.a_src, ag_u.a_reg, rrd[3], ag_u.imm, ag_u.a_sxw);
wire [31:0] ag_b = ag_u.b_upd ? ag_updv : opnd(ag_u.b_src, ag_u.b_reg, rrd[4], ag_u.imm_b, 1'b0);

// A snoop dropped instruction-cache data the front end may already have
// run ahead into (it predicts calls and returns, beyond the 68040's
// prefetch): the next instruction to reach AG is refetched, with it
// everything younger, its access not made (snp_pend).  Instructions
// already past AG may have been fetched from the dropped line as well: the
// next instruction to reach WB is refetched too (snp_wb, marked as it
// enters WB: no commit, no store).
// An exception routine's uops are not instructions (their pc is 0): the
// refetch waits for the handler's first instruction, which the routine's
// final jump fetches afresh anyway.
logic snp_pend, snp_wb;
wire  ag_snref = snp_pend && ag_v && ag_u.first && !ag_u.exr;
always_ff @(posedge clk) begin
	if (!nreset || flush) snp_pend <= 1'b0;
	else if (sn_ihit) snp_pend <= 1'b1;
	else if (adv_ag && ag_snref) snp_pend <= 1'b0;
end
wire  ex_snw = snp_wb && ex_u.first && !ex_u.exr;
always_ff @(posedge clk) begin
	if (!nreset || flush) snp_wb <= 1'b0;
	else if (sn_ihit) snp_wb <= 1'b1;
	else if (adv_ex && ex_snw) snp_wb <= 1'b0;
end

// memory request to the DMU, issued as the uop leaves AG
logic s_bit;
assign s_bit      = sr_r[13];
// CAS with a misaligned operand (a word at an odd address, a long not at a
// multiple of four) is the 68060's unimplemented integer instruction
// exception (vector 61, WinUAE gencpu): no access is made
wire        ag_casmis = (ag_u.op == OP_CAS) && (ag_u.mem != M_NONE) &&
                        ((ag_u.msz == SZ_W && ag_maddr[0]) ||
                         (ag_u.msz == SZ_L && ag_maddr[1:0] != 2'd0));
assign dm_req     = adv_ag && (ag_u.mem != M_NONE) && (ag_u.exc == 8'd0) && !ag_cancel && !ag_snref &&
                    !ag_casmis;
assign dm_va      = ag_maddr;
assign dm_mem     = ag_u.mem;
assign dm_msz     = ag_msz;
// BUSCR: L locks every operand access from the next one on, LE marks the
// next operand write as the end of the sequence; once that write is out
// LOCK/LOCKE stay negated until BUSCR is written again (MC68060UM 7.4)
logic        bus_lkdone;
wire         ag_wr      = (ag_u.mem == M_ST) || (ag_u.mem == M_RMW);
assign dm_lock    = ag_u.mlock || (bus_l && !bus_lkdone);
assign dm_locke   = ag_u.mlocke || (bus_le && !bus_lkdone && ag_wr);
// FC2 of the access selects the root and the supervisor checks: MOVES
// translates its SFC/DFC space (WinUAE: super = (sfc & 4) != 0)
assign dm_super   = dm_fc[2];
assign dm_older   = ex_v || wb_v;



// Bcc history: the counter counts disagreement with the static rule
// (saturating 0..3); trained as the branch leaves EX, written a cycle
// later unless that cycle flushes (the branch was on a wrong path: an
// older uop at WB redirected or trapped as it left EX)
wire        bht_agree = (ex_tkn == ex_u.bst);
logic       bht_we_q;
always_ff @(posedge clk) begin
	bht_we_q <= nreset && adv_ex && ex_u.op == OP_BCC && ex_u.br == BR_COND;
	bht_wa   <= ex_u.pc[8:1];
	bht_dis  <= !bht_agree;
end
assign bht_we = bht_we_q && !flush;
// loop exits: every backward Bcc/DBcc, the same way
logic       lp_we_q;
always_ff @(posedge clk) begin
	lp_we_q <= nreset && adv_ex && ex_u.bst &&
	           ((ex_u.op == OP_BCC && ex_u.br == BR_COND) || ex_u.op == OP_DBCC);
	lp_wa   <= ex_u.pc[12:1];
	lp_tk   <= ex_tkn;
	lp_mp   <= ex_mispred;
end
assign lp_we = lp_we_q && !flush;
assign dm_noalloc = (ag_u.mfc == MFC_SUP);
// acknowledge cycles (TT=3): 1 interrupt, 2 breakpoint, 3 LPSTOP broadcast
assign dm_ack     = (ag_u.mfc == MFC_IACK) ? 2'd1 : (ag_u.mfc == MFC_BKPT) ? 2'd2 :
                    (ag_u.mfc == MFC_LPS) ? 2'd3 : 2'd0;
always_comb begin
	case (ag_u.mfc)
		MFC_SFC: dm_fc = sfc_r;
		MFC_DFC: dm_fc = dfc_r;
		MFC_SUP: dm_fc = 3'd5;
		MFC_IACK: dm_fc = 3'd7;
		default: dm_fc = ag_u.mprog ? {s_bit, 2'b10} : {s_bit, 2'b01};
	endcase
end

//--------------------------------------------------------------------------
// EX
//--------------------------------------------------------------------------
// EX's operand values: the load data (A or B may be it) and A's word sign
// extension are merged in as the uop enters EX, so EX starts from
// flip-flops (ex_a/ex_b keep the register operands as they were)
logic [31:0] ex_av, ex_bv;

logic [31:0] alu_res, alu_sum, alu_sumo, alu_resf;
logic [15:0] ex_rk;            // the ALU result's case, one-hot (alu_rk), registered
function automatic logic [15:0] alu_rk(input logic [6:0] op);
	logic [15:0] r;
	r = '0;
	case (op)
		OP_MOV:  r[0] = 1'b1;   OP_AND:  r[1] = 1'b1;   OP_OR:   r[2] = 1'b1;
		OP_EOR:  r[3] = 1'b1;   OP_NOT:  r[4] = 1'b1;   OP_CLR:  r[5] = 1'b1;
		OP_EXT:  r[6] = 1'b1;   OP_EXTB: r[7] = 1'b1;   OP_SWAP: r[8] = 1'b1;
		OP_TAS:  r[9] = 1'b1;   OP_BCHG: r[10] = 1'b1;  OP_BCLR: r[11] = 1'b1;
		OP_BSET: r[12] = 1'b1;  OP_SCC:  r[13] = 1'b1;  OP_EA:   r[14] = 1'b1;
		OP_DBCC: r[15] = 1'b1;
		default: ;
	endcase
	alu_rk = r;
endfunction
logic  [4:0] alu_flags, alu_sumfl;
logic        alu_cc, alu_trap;
// the adder's controls, decoded as the uop enters EX: {CMP, X in, subtract,
// NEG(X)}; the adder ops' size mask for EX's result select (ex_sk: the sum's
// bits, the rest of the select takes the destination's upper bits); the
// ops whose flags are the adder's (ex_fs)
function automatic logic [3:0] alu_ctl(input logic [6:0] op);
	alu_ctl = {op == OP_CMP, op == OP_ADDX || op == OP_SUBX || op == OP_NEGX,
	           op == OP_SUB || op == OP_SUBX || op == OP_CMP || op == OP_NEG || op == OP_NEGX,
	           op == OP_NEG || op == OP_NEGX};
endfunction
logic  [3:0] ex_ac;
logic [31:0] ex_sk;
logic        ex_fs;

ap68060_alu alu (
	.op(ex_u.op), .sz(ex_u.sz), .cond(ex_u.cond),
	.a(ex_av), .b(ex_bv), .ea(ex_ea),
	.flags_in(ccr_f), .ac(ex_ac), .rk(ex_rk), .res_f(alu_resf),
	.res(alu_res), .res_sum(alu_sum), .sum_o(alu_sumo), .sum_fl(alu_sumfl), .flags_out(alu_flags),
	.cc_true(alu_cc), .trap(alu_trap)
);

// multiply / divide
logic        md_done, md_ovf;
logic [31:0] md_hi, md_lo;
logic [31:0] md_rem;          // remainder / product high, for OP_MDRES
logic [31:0] md_lol;          // quotient / product low, for OP_MDRES
logic        md_ovfl;         // the last divide overflowed
logic        md_go;           // this EX uop has started the unit
logic        md_fin;          // ... and the unit has finished (done is a pulse: EX may
                              // be stalled by WB in that cycle)
logic        md_z;            // ... is a divide by zero (from its second cycle)
wire         ex_is_md   = (ex_u.op == OP_MUL) || (ex_u.op == OP_DIV);
wire         ex_div0    = (ex_u.op == OP_DIV) &&
                          ((ex_u.sz == SZ_W) ? (ex_av[15:0] == 16'd0) : (ex_av == 32'd0));
assign ex_md_start = ex_v && ex_is_md && !md_go && !ex_div0 && !md_busy && !ex_fault;

// word forms: DIVU.W/DIVS.W divide the 32-bit Dn by a 16-bit source;
// MULU.W/MULS.W multiply the low words
// (the word forms' selects registered as the uop enters EX: ex_mw)
wire        md_sign = ex_u.cond[0];
logic [1:0] ex_mw;            // {the word form, a word multiply}
wire [31:0] md_a    = ex_mw[1] ?
                      (md_sign ? {{16{ex_av[15]}}, ex_av[15:0]} : {16'd0, ex_av[15:0]}) : ex_av;
wire [31:0] md_blo  = ex_mw[0] ?
                      (md_sign ? {{16{ex_bv[15]}}, ex_bv[15:0]} : {16'd0, ex_bv[15:0]}) : ex_bv;

ap68060_muldiv md (
	.clk(clk), .nreset(nreset), .kill(kill_now),
	.start(ex_md_start), .is_div(ex_u.op == OP_DIV), .sign_op(md_sign),
	.op_a(md_a), .op_lo(md_blo),
	.busy(md_busy), .done(md_done), .res_hi(md_hi), .res_lo(md_lo), .ovf(md_ovf)
);

// FPU: OP_FPU uops act once no older uop is in flight (WB empty)
wire         ex_is_fp = (ex_u.op == OP_FPU);
ap68060_fpif #(.FPU_PIPE(FPU_PIPE)) fpif (
	.clk(clk), .nreset(nreset),
	.ex_v(ex_v && ex_fpw), .sub(ex_u.cond),
	.fuse(ex_u.sz == SZ_W), .eal(ex_u.sz == SZ_B),
	.imm(ex_u.imm[7:0]), .immb(ex_u.imm_b[7:0]),
	.av(ex_av), .bv(ex_bv), .aw(ex_a[15:0]), .bw(ex_b[15:0]),
	.dw_a(dc2_a[15:0]), .dw_b(dc2_b[15:0]), .dw_ld(!stall_ex), .latch(ex_latch), .ea(ex_ea), .pc(ex_u.pc),
	.dw1_a(dt_a[15:0]), .dw1_b(dt_b[15:0]), .dw1_ld(adv_dt),
	.target(ex_u.target),
	.safe(safe_q), .adv(adv_ex), .kill(kill_now),
	.n_sub(dc2_u.cond), .n_fuse(dc2_u.sz == SZ_W), .n_eal(dc2_u.sz == SZ_B),
	.n_imm(dc2_u.imm[7:0]), .n_ea(dc2_ea), .n_pc(dc2_u.pc), .n_target(dc2_u.target),
	.hold(fp_hold), .hs0(fp_hs0), .ns0(fp_ns0), .hs1(fp_hs1), .ns1(fp_ns1), .res(fp_res), .dkill(fp_dkill), .taken(fp_taken),
	.xvec(fp_xvec), .xfmt(fp_xfmt), .xnext(fp_xnext), .xaddr(fp_xaddr),
	.xcommit(fp_xcommit), .st_kill(fp_stkill), .dfp(pcr_dfp),
	.mv_slots(fp_mvslots), .cr_slots(fp_crslots)
);

// EX holds while the unit runs, until the cycle its result is presented
// (a divide by zero holds its first cycle too: the zero test of the
// operand stays off the stall path, and the exception goes a cycle later)
// (multiply/divide: the result, flags and kill are registered from the
// unit's result in its done cycle, md_q_*, and presented the cycle after)
// (the multiply/divide unit's and the slow phases' a register formed a
// clock ahead, exh_q; the FPU's from fpif, its uop's flag ex_fpw registered
// as it enters)
logic exh_q;
// (ex_fpw's preserved twin here: the hold is the stall chain's root)
(* preserve *) logic ex_fpw_h;
assign ex_hold = exh_q || (ex_v && ex_fpw_h && fp_hold);

// multiply/divide result and flags
logic [31:0] md_res, md_q_res;
logic  [4:0] md_flags, md_q_flags;
logic        md_dkill, md_q_dkill;
always_comb begin
	md_res   = ex_bv;
	md_flags = ccr_f;
	md_dkill = 1'b0;
	if (ex_u.op == OP_MUL) begin
		if (ex_u.sz == SZ_W || !ex_u.cond[1]) begin
			// 32-bit product (MULx.W is 16x16 -> 32)
			md_res   = md_lo;
			md_flags = {ccr_f[4], md_lo[31], md_lo == 32'd0,
			            (ex_u.sz == SZ_L) &&
			            (md_sign ? (md_hi != {32{md_lo[31]}}) : (md_hi != 32'd0)),
			            1'b0};
		end
		else begin
			// 64-bit: this uop writes Dl; OP_MDRES writes Dh
			md_res   = md_lo;
			md_flags = {ccr_f[4], md_hi[31], {md_hi, md_lo} == 64'd0, 1'b0, 1'b0};
		end
	end
	else begin
		// divide
		if (ex_u.sz == SZ_W) begin
			// quotient must fit 16 bits
			logic ovf16;
			ovf16 = md_ovf ||
			        (md_sign ? (md_lo[31:15] != {17{md_lo[15]}}) : (md_lo[31:16] != 16'd0));
			if (ovf16) begin
				md_dkill = 1'b1;
				md_flags = {ccr_f[4], ccr_f[3], ccr_f[2], 1'b1, 1'b0};
			end
			else begin
				md_res   = {md_hi[15:0], md_lo[15:0]};
				md_flags = {ccr_f[4], md_lo[15], md_lo[15:0] == 16'd0, 1'b0, 1'b0};
			end
		end
		else begin
			if (md_ovf) begin
				md_dkill = 1'b1;
				md_flags = {ccr_f[4], ccr_f[3], ccr_f[2], 1'b1, 1'b0};
			end
			else begin
				md_res   = md_lo;
				md_flags = {ccr_f[4], md_lo[31], md_lo == 32'd0, 1'b0, 1'b0};
			end
		end
	end
end

// latch for CAS / bit fields
logic [31:0] ex_latch;

//--------------------------------------------------------------------------
// bit field unit.  BFSET latched the offset (bf_off) and width (bf_w,
// 1..32).  Register form (cond[3] = 0): the field is in B, numbered from
// bit 31, wrapping.  Memory form: B is the long word at EA + offset/8 and
// ex_latch[7:0] the byte after it; the field starts offset & 7 bits in.
// A is the BFINS source.  Flags come from the field, for BFINS from the
// inserted value.
//--------------------------------------------------------------------------
logic [31:0] bf_off;
logic  [5:0] bf_w;
// derived from the width when BFSET latches it (off the EX paths): 32 - width,
// the field mask at the top, BFEXTS's sign mask
logic  [5:0] bf_wn;
logic [31:0] bf_top, bf_sx;
logic  [2:0] bf_nb;             // memory form: bytes holding the field (1..5)

// leading zero count of a nonzero 32-bit value, as a balanced tree
function automatic logic [4:0] clz32(input logic [31:0] v);
	logic [4:0] n;
	logic [31:0] x;
	x = v;
	n[4] = (x[31:16] == 16'd0); if (n[4]) x = {x[15:0], 16'd0};
	n[3] = (x[31:24] == 8'd0);  if (n[3]) x = {x[23:0], 8'd0};
	n[2] = (x[31:28] == 4'd0);  if (n[2]) x = {x[27:0], 4'd0};
	n[1] = (x[31:30] == 2'd0);  if (n[1]) x = {x[29:0], 2'd0};
	n[0] = !x[31];
	clz32 = n;
endfunction
logic  [7:0] bf_lonew;          // memory form: the new byte after the long
logic [31:0] bf_res;
logic  [7:0] bf_lo_out;
// A bit field operation runs in four EX cycles, each from registers:
//   0  the operand rotated to the field (bf_rot_q) and the BFINS source
//      left aligned to the field width (bf_ins_q)
//   1  the field extracted, the changed operand, the leading zeros and the
//      flags (bf_*_q)
//   2  the changed operand rotated back, the result (xs, into xs_q)
//   3  the result
// (a rotate and a second variable shift in one cycle do not close at 65 MHz)
logic [31:0] bf_ins_q, bf_rot, bf_rot_q;
logic  [5:0] rox_nx, rox_nx_q;  // ROXL/ROXR: their count mod (size + 1), the same way
logic [31:0] bf_fld_q, bf_nrot_q;
logic [39:0] bf_nwin_q;
logic  [5:0] bf_lz_q;
logic        bf_nf_q, bf_zf_q, bf_r31_q;
logic [31:0] bf_fld, bf_nrot;
logic [39:0] bf_nwin;
logic  [5:0] bf_lz;
logic        bf_nf, bf_zf;
logic  [4:0] bf_flags;
// leading zero count of a nonzero 32-bit value: each nibble's count beside
// whether it is zero, then the first nonzero nibble picks (a tree, not a
// chain of shifts)
function automatic logic [4:0] lz32(input logic [31:0] v);
	logic [7:0] nz;
	logic [1:0] c [8];
	logic [2:0] k;
	for (int i = 0; i < 8; i++) begin
		logic [3:0] d;
		d = v[31 - 4*i -: 4];
		nz[i] = (d != 4'd0);
		c[i]  = d[3] ? 2'd0 : d[2] ? 2'd1 : d[1] ? 2'd2 : 2'd3;
	end
	k = 3'd7;
	for (int i = 7; i >= 0; i--) if (nz[i]) k = 3'(i);
	lz32 = {k, c[k]};
endfunction
always_comb begin
	logic [4:0]  o;
	logic [31:0] top, rot, fld, ins_top, nrot, nreg, zf;
	logic [39:0] win, mask40, ins40, nwin;
	logic [2:0]  t;
	t   = ex_u.cond[2:0];
	top = bf_top;
	ins_top = bf_ins_q;
	if (!ex_u.cond[3]) begin
		o   = bf_off[4:0];
		// rotate left by o: a funnel of B with itself
		rot = 32'(({ex_bv, ex_bv} << o) >> 32);
		win = '0; mask40 = '0; ins40 = '0;
	end
	else begin
		o   = {2'b00, bf_off[2:0]};
		// the bytes read, left justified: B holds the head transfer (byte,
		// word or long), the latch the tail byte
		case (bf_nb)
			3'd1:    win = {ex_bv[7:0], 32'd0};
			3'd2:    win = {ex_bv[15:0], 24'd0};
			3'd3:    win = {ex_bv[15:0], ex_latch[7:0], 16'd0};
			3'd4:    win = {ex_bv, 8'd0};
			default: win = {ex_bv, ex_latch[7:0]};
		endcase
		rot = 32'((win << o) >> 8);
		mask40 = {top, 8'h00} >> o;
		ins40  = {ins_top, 8'h00} >> o;
	end
	// cycle 0
	bf_rot = rot;
	// cycle 1, from bf_rot_q and bf_ins_q
	rot    = bf_rot_q;
	bf_fld = rot >> bf_wn;
	case (t)
		3'd2: bf_nrot = rot ^ top;                     // BFCHG
		3'd4: bf_nrot = rot & ~top;                    // BFCLR
		3'd6: bf_nrot = rot | top;                     // BFSET
		default: bf_nrot = (rot & ~top) | (ins_top & top);   // BFINS
	endcase
	case (t)
		3'd2: bf_nwin = win ^ mask40;
		3'd4: bf_nwin = win & ~mask40;
		3'd6: bf_nwin = win | mask40;
		default: bf_nwin = (win & ~mask40) | (ins40 & mask40);
	endcase
	// leading zeros of the field (FFO) over the top-aligned field; a zero
	// field gives the width
	zf    = rot & top;
	bf_lz = (zf == 32'd0) ? bf_w : {1'b0, lz32(zf)};
	if (t == 3'd7) begin
		bf_nf = ins_top[31];
		bf_zf = (ins_top & top) == 32'd0;
	end
	else begin
		bf_nf = rot[31];
		bf_zf = zf == 32'd0;
	end
	// cycle 2, from the bf_*_q registers
	nreg = 32'({bf_nrot_q, bf_nrot_q} >> o);
	bf_flags = {ccr_f[4], bf_nf_q, bf_zf_q, 1'b0, 1'b0};
	// the bytes written back: the head right justified, the tail byte
	bf_lo_out = (bf_nb == 3'd3) ? bf_nwin_q[23:16] : bf_nwin_q[7:0];
	case (t)
		3'd1: bf_res = bf_fld_q;                                         // BFEXTU
		3'd3: bf_res = bf_fld_q | (bf_r31_q ? bf_sx : 32'd0);              // BFEXTS
		3'd5: bf_res = bf_off + {26'd0, bf_lz_q};                         // BFFFO
		3'd0: bf_res = ex_bv;                                            // BFTST
		default: bf_res = !ex_u.cond[3] ? nreg :
		                  (bf_nb == 3'd1) ? {24'd0, bf_nwin_q[39:32]} :
		                  (bf_nb == 3'd2 || bf_nb == 3'd3) ? {16'd0, bf_nwin_q[39:24]} :
		                  bf_nwin_q[39:8];
	endcase
end


//--------------------------------------------------------------------------
// EX result selection
//--------------------------------------------------------------------------
logic [31:0] ex_res;
logic  [4:0] ex_flags;
logic        ex_dkill;        // suppress the register write
logic  [3:0] ex_kc;           // EX's uop is FPU, DIV, MDRES, DBcc (the ones that can)
logic        ex_dkill_chk;    // (simulation: the case form)
// synthesis translate_off
always_ff @(posedge clk)
	if (nreset && ex_v && ex_u.d_v && ex_dkill != ex_dkill_chk) begin
		$display("FAIL: ex_dkill %b, the case form %b (op %0d, pc %h)", ex_dkill, ex_dkill_chk, ex_u.op, ex_u.pc);
		$finish;
	end
// synthesis translate_on
logic        ex_taken;        // control transfer taken
logic [31:0] ex_target;
logic  [7:0] ex_xvec;         // exception raised in EX
// (... whether there is one, from flags registered as the uop enters EX
// and, for a slow op, as its result is: not through the vector's
// multiplexers and compare)
logic        ex_xnz, xs_xnz;
logic [31:0] ex_st;           // store data
logic        ex_sr_we;
logic [15:0] ex_sr_new;

// special register read: the MOVEC numbers as {ext[11], ext[3:0]}
// ($000-$008 -> $00-$08, $800-$808 -> $10-$18), then SR and CCR
logic [31:0] spr_q;           // MOVEC's register, read the clock before (its phase 0)
function automatic logic [31:0] spr_read(input logic [7:0] n);
	case (n)
		8'h00: spr_read = {29'd0, sfc_r};
		8'h01: spr_read = {29'd0, dfc_r};
		8'h02: spr_read = cacr_r;
		8'h03: spr_read = tc_r;
		8'h04: spr_read = itt0_r;
		8'h05: spr_read = itt1_r;
		8'h06: spr_read = dtt0_r;
		8'h07: spr_read = dtt1_r;
		8'h08: spr_read = {bus_l, bus_sl, bus_le, bus_sle, 28'd0};            // BUSCR
		8'h11: spr_read = vbr_r;
		8'h16: spr_read = urp_r;
		8'h17: spr_read = srp_r;
		8'h18: spr_read = {16'h0430, PCR_REV, pcr_edebug, 5'd0, pcr_dfp, pcr_ess}; // PCR
		8'h20: spr_read = {16'd0, sr_r[15:5], ccr_f};     // SR
		8'h21: spr_read = {27'd0, ccr_f};                   // CCR
		default: spr_read = 32'd0;
	endcase
endfunction

// slow (multi-cycle) operations: EX holds them for their phases; the
// result is formed into xs, registered (xs_q) in the next-to-last phase and
// presented in the last, so their logic never reaches the result path.
// The last phase: 2 for most (the shifter's first stage, then the shift;
// the others' xs just has the time), 3 for the bit fields (above) and for
// ROXL/ROXR (their count reduced mod size + 1 first).  CHK is one: its
// bound compare and the trap stay off the result and exception paths.
function automatic logic [1:0] nphase(input logic [6:0] op);
	case (op)
		OP_ROXL, OP_ROXR, OP_BF: nphase = 2'd3;
		OP_ASL, OP_ASR, OP_LSL, OP_LSR, OP_ROL, OP_ROR,
		OP_ABCD, OP_SBCD, OP_NBCD, OP_PACK, OP_UNPK, OP_CHK,
		OP_SPR, OP_SRLOG, OP_RTE, OP_RTEF, OP_IACKV, OP_MISC: nphase = 2'd2;
		default: nphase = 2'd0;
	endcase
endfunction

typedef struct packed {
	logic [31:0] res;
	logic  [4:0] flags;
	logic        dkill;
	logic        taken;
	logic [31:0] target;
	logic  [7:0] xvec;
	logic        sr_we;
	logic [15:0] sr_new;
} exo_t;

logic [1:0] ex_np;              // the EX uop's last phase (0: a fast op)
logic [1:0] ex_sp;              // ... and its phase now
logic       ex_slow;            // ex_np != 0, registered with it
logic       ex_shq;             // a shift or rotate, registered with it
// shifts and rotates: N and Z are the result's sign and zero test at the
// size, taken in the second cycle from the registered result (the barrel
// shift and a 32-bit zero test do not fit one cycle with the count logic)
function automatic logic is_shift(input logic [6:0] op);
	is_shift = (op == OP_ASL) || (op == OP_ASR) || (op == OP_LSL) || (op == OP_LSR) ||
	           (op == OP_ROL) || (op == OP_ROR) || (op == OP_ROXL) || (op == OP_ROXR);
endfunction
wire  ex_shift = ex_shq;
exo_t xf, xs, xs_q;

logic [31:0] sl_res;
logic  [4:0] sl_flags;
ap68060_alu_slow alu_s (
	.clk(clk), .op(ex_u.op), .sz(ex_u.sz), .a(ex_av), .b(ex_bv), .cnt(ex_a[5:0]), .flags_in(ccr_f),
	.nx_q(rox_nx_q), .nx(rox_nx),
	.res(sl_res), .flags_out(sl_flags)
);
// synthesis translate_off
always_ff @(posedge clk)
	if (nreset && ex_v && wb_v && ew_same != (ex_u.pc == wb_u.pc)) begin
		$display("FAIL: ew_same %b, the EX and WB PCs %h %h", ew_same, ex_u.pc, wb_u.pc);
		$finish;
	end
// synthesis translate_on
// (the shift count is never a memory operand: a count register or a
// constant, so it comes straight from the operand register)
// synthesis translate_off
always_ff @(posedge clk)
	if (ex_v && ex_u.a_src == OS_MEM &&
	    (ex_u.op == OP_ASL || ex_u.op == OP_ASR || ex_u.op == OP_LSL || ex_u.op == OP_LSR ||
	     ex_u.op == OP_ROL || ex_u.op == OP_ROR || ex_u.op == OP_ROXL || ex_u.op == OP_ROXR)) begin
		$display("FAIL: a shift with its count from memory (pc %h)", ex_u.pc);
		$finish;
	end
// synthesis translate_on

// fast operations
always_comb begin
	xf.res    = alu_res;
	xf.flags  = alu_flags;
	xf.dkill  = 1'b0;
	xf.taken  = 1'b0;
	xf.target = ex_u.target;
	xf.xvec   = 8'd0;
	xf.sr_we  = 1'b0;
	xf.sr_new = {sr_r[15:5], alu_flags};
	case (ex_u.op)
		OP_FPU: begin
			xf.res   = fp_res;
			xf.flags = ccr_f;
			xf.dkill = fp_dkill;
			xf.taken = fp_taken;
			xf.xvec  = fp_xvec;
		end
		OP_MUL, OP_DIV: begin
			xf.res   = md_q_res;
			xf.flags = md_q_flags;
			xf.dkill = md_q_dkill && (ex_u.op == OP_DIV);   // (a multiply never kills)
			if (ex_div0) begin
				xf.xvec  = 8'd5;
				xf.flags = {ccr_f[4:1], 1'b0};
			end
		end
		OP_MDRES: begin
			xf.res   = ex_u.cond[0] ? md_lol : md_rem;
			xf.dkill = md_ovfl;
		end
		OP_BCC:    xf.taken = alu_cc;
		OP_DBCC: begin
			// (the counter before the decrement: Dn.W - 1 == -1 exactly when
			// Dn.W is 0 -- the adder stays off the branch decision)
			xf.taken = !alu_cc && (ex_bv[15:0] != 16'd0);
			xf.dkill = alu_cc;
		end
		OP_TRAPCC: if (alu_cc) xf.xvec = 8'd7;     // (alu_trap: CHK's subtract too)
		OP_CCRLOG: xf.res = ex_bv;
		OP_SPW:    xf.res = ex_av;
		OP_LATCH:  xf.res = ex_av;
		OP_BFSET:  xf.res = ex_u.cond[0] ? {24'd0, bf_lonew} : {{3{ex_av[31]}}, ex_av[31:3]};
		OP_CAS: begin
			xf.flags = alu_flags;
			xf.res   = ex_bv;
		end
		default: ;
	endcase
	// control transfers
	case (ex_u.br)
		BR_IMM:  begin xf.taken = 1'b1; xf.target = ex_u.target; end
		BR_EA:   begin xf.taken = 1'b1; xf.target = ex_ea; end
		BR_A:    begin xf.taken = 1'b1; xf.target = ex_av; end
		BR_B:    begin xf.taken = 1'b1; xf.target = ex_bv; end
		default: ;
	endcase
end

// slow operations (first EX cycle)
always_comb begin
	xs.res    = sl_res;
	xs.flags  = ex_shift ? {sl_flags[4], 2'b00, sl_flags[1:0]} : sl_flags;
	xs.dkill  = 1'b0;
	xs.taken  = 1'b0;
	xs.target = ex_u.target;
	xs.xvec   = 8'd0;
	xs.sr_we  = 1'b0;
	xs.sr_new = {sr_r[15:5], ccr_f};
	case (ex_u.op)
		OP_SRLOG: begin
			logic [15:0] cur, n;
			cur = {sr_r[15:5], ccr_f};
			case (ex_u.cond[1:0])
				2'd0:    n = cur & ex_av[15:0];
				2'd1:    n = cur | ex_av[15:0];
				2'd2:    n = cur ^ ex_av[15:0];
				default: n = ex_av[15:0];
			endcase
			xs.sr_new = n & SR_MASK;
			xs.sr_we  = 1'b1;
			xs.flags  = xs.sr_new[4:0];
		end
		OP_SPR: begin
			// (the register read in the op's first clock: spr_q)
			logic [31:0] v, m;
			v = spr_q;
			m = (ex_u.sz == SZ_B) ? 32'h0000_00FF : (ex_u.sz == SZ_W) ? 32'h0000_FFFF : 32'hFFFF_FFFF;
			xs.res = (v & m) | (ex_bv & ~m);
			xs.flags = ccr_f;
		end
		OP_MISC: begin
			xs.flags = ccr_f;
			xs.res   = ex_av;              // maintenance: the address
			if (ex_u.cond == 4'd2 || ex_u.cond == 4'd8) begin
				// STOP / LPSTOP #imm: SR = imm, then wait for an interrupt
				xs.sr_new = ex_av[15:0] & SR_MASK;
				xs.sr_we  = 1'b1;
				xs.flags  = xs.sr_new[4:0];
			end
			// BKPT: after its acknowledge cycle, the illegal instruction
			// exception (format $0, the BKPT's PC)
			if (ex_u.cond == 4'd9) xs.xvec = 8'd4;
		end
		OP_IACKV: begin
			// IACK data: [7:0] vector, [8] AVEC, [9] TEA (spurious)
			logic [7:0] v;
			xs.flags = ccr_f;
			if (!ex_u.cond[0]) begin
				v = ex_av[9] ? 8'd24 : ex_av[8] ? (8'd24 + {5'd0, irq_lvl_r}) : ex_av[7:0];
				xs.res = {22'd0, v, 2'b00};                 // format $0 word
			end
			else begin
				v = ex_latch[9] ? 8'd24 : ex_latch[8] ? (8'd24 + {5'd0, irq_lvl_r}) : ex_latch[7:0];
				xs.res = vbr_r + {22'd0, v, 2'b00};         // vector address
			end
		end
		OP_RTEF: begin
			// 68060 frame formats (MC68060UM 8.4): $0 8, $2 and $3 12, $4
			// 16 bytes; any other is a format error
			xs.flags = ccr_f;
			if (ex_u.cond[0]) begin
				// FRESTORE header (FPU in reset state): only a NULL frame
				// (version byte $00) is compatible, anything else is a
				// format error at the FRESTORE
				xs.res = ex_av;
				if (ex_av[31:24] != 8'd0) xs.xvec = 8'd14;
			end
			else case (ex_av[15:12])
				4'h0:       xs.res = 32'd8;
				4'h2, 4'h3: xs.res = 32'd12;
				4'h4:       xs.res = 32'd16;
				default: begin xs.res = 32'd0; xs.xvec = 8'd14; end
			endcase
		end
		OP_RTE: begin
			xs.sr_new = ex_av[15:0] & SR_MASK;
			xs.sr_we  = 1'b1;
			xs.flags  = xs.sr_new[4:0];
			xs.taken  = 1'b1;
			xs.target = ex_bv;
		end
		OP_BF: begin
			xs.res   = bf_res;
			xs.flags = bf_flags;
		end
		OP_CHK: begin
			xs.res   = alu_res;
			xs.flags = alu_flags;
			if (alu_trap) xs.xvec = 8'd6;
		end
		default: ;
	endcase
end

// EX outputs: the registered slow result in a slow op's second cycle
always_comb begin
	exo_t x;
	x = ex_slow ? xs_q : xf;
	if (ex_slow && ex_shift) begin
		x.flags[3] = (ex_u.sz == SZ_B) ? xs_q.res[7] : (ex_u.sz == SZ_W) ? xs_q.res[15] : xs_q.res[31];
		x.flags[2] = (ex_u.sz == SZ_B) ? (xs_q.res[7:0] == 8'd0) :
		             (ex_u.sz == SZ_W) ? (xs_q.res[15:0] == 16'd0) : (xs_q.res == 32'd0);
	end
	ex_res    = x.res;
	ex_flags  = x.flags;
	// (the uops that can suppress their write, by flags registered as they
	// enter EX, so the kill is an AND-OR of its sources: no op decode on
	// the way to the forwarding enable; the others never kill, slow ones
	// included)
	ex_dkill  = (ex_kc[0] && fp_dkill) || (ex_kc[1] && md_q_dkill) ||
	            (ex_kc[2] && md_ovfl) || (ex_kc[3] && alu_cc);
	// synthesis translate_off
	ex_dkill_chk = x.dkill;
	// synthesis translate_on
	ex_taken  = x.taken;
	ex_target = x.target;
	ex_xvec   = x.xvec;
	ex_sr_we  = x.sr_we;
	ex_sr_new = x.sr_new;
end

// CAS: flags of (memory - Dc) and the equal decision
wire [31:0] cas_dc_m = ex_latch;
logic [31:0] cas_res;
logic  [4:0] cas_flags;
logic        cas_cc, cas_tr;
ap68060_alu cas_alu (
	.op(OP_CMP), .sz(ex_u.sz), .cond(4'd0),
	.a(cas_dc_m), .b(ex_bv), .ea(32'd0), .flags_in(ccr_f), .ac(alu_ctl(OP_CMP)),
	.rk(16'd0), .res_f(),
	.res(cas_res), .res_sum(), .sum_o(), .sum_fl(), .flags_out(cas_flags), .cc_true(cas_cc), .trap(cas_tr)
);
wire cas_eq = cas_flags[2];

logic [31:0] ex_szm;          // EX's size mask, registered as the uop enters

// store data, and stores the EX op cancels
logic ex_stkill;
always_comb begin
	ex_stkill = 1'b0;
	case (ex_u.op)
		OP_CAS:  ex_st = cas_eq ? ex_av : ex_bv;
		default: ex_st = ex_res;
	endcase
	// an FPU store whose result is not written (an enabled integer
	// SNAN/OPERR, an unimplemented / unsupported store): its stores go
	if (fp_stkill && ex_u.mem == M_ST && !ex_is_fp) ex_stkill = 1'b1;
end

// a store into the instruction stream just ahead (see EX -> WB).  Code
// the front end fetched elsewhere (a predicted target) is the program's
// to keep coherent: the 68040 prefetches both paths of a branch and
// requires CPUSHA before self-modified code runs (MC68040UM 4.5, 7.7).
wire ex_smc = (ex_u.mem == M_ST || ex_u.mem == M_RMW) &&
              ((ex_ea[31:6] == ex_npc[31:6]) ||
               (ex_ea[31:6] == ex_npc[31:6] + 26'd1));
logic smc_pend;
// mispredict: the actual path differs from the one the front end took
// (the target's source and the taken decision's, one-hot, registered as the
// uop enters EX: the target an AND-OR of flip-flops, its compare with the
// predicted target made per source in parallel; checked against EX's case)
logic        ex_br;
logic  [4:0] ex_ts;           // target: A, B, EA, the decoded one, the slow op's
logic  [4:0] ex_tk;           // taken: always, FPU, Bcc, DBcc, the slow op's
wire [31:0] ex_tgt     = ({32{ex_ts[0]}} & ex_av) | ({32{ex_ts[1]}} & ex_bv) |
                         ({32{ex_ts[2]}} & ex_ea) | ({32{ex_ts[3]}} & ex_u.target) |
                         ({32{ex_ts[4]}} & xs_q.target);
wire        ex_tne     = (ex_ts[0] && ex_av != ex_u.target) || (ex_ts[1] && ex_bv != ex_u.target) ||
                         (ex_ts[2] && ex_ea != ex_u.target) || (ex_ts[4] && xs_q.target != ex_u.target);
wire        ex_tkn     = ex_tk[0] || (ex_tk[1] && fp_taken) || (ex_tk[2] && alu_cc) ||
                         (ex_tk[3] && !alu_cc && (ex_bv[15:0] != 16'd0)) || (ex_tk[4] && xs_q.taken);
wire [31:0] ex_next    = ex_tkn ? ex_tgt : ex_npc;
wire        ex_mispred = ex_br && ((ex_tkn != ex_u.pred) || (ex_tkn && ex_u.pred && ex_tne));
// a mispredicted Bcc or DBcc that WB will redirect for no other reason
// turns the front end round as it leaves EX, a cycle early; the stages
// behind it still go at WB (the front end is not flushed again), and AG
// takes nothing in between.  Only those: their target is the decoded one,
// so whether they mispredicted is the condition alone (no target compare),
// and where they go is a choice of two registers.
logic       eredir_pend, wb_eredir;
wire        ex_bdb_tk = (ex_u.op == OP_BCC) ? alu_cc : (!alu_cc && (ex_bv[15:0] != 16'd0));
wire [31:0] ex_bdb_nx = ex_bdb_tk ? ex_u.target : ex_npc;
wire        ex_xok   = (ex_u.exc == 8'd0) && !ex_snw;   // no exception, no snoop refetch
assign      ex_xnz   = ex_slow ? xs_xnz :
                       ((ex_kx[2] && fp_xvec != 8'd0) || (ex_kx[1] && md_z) || (ex_kx[0] && alu_cc));
// (a divide by zero holds its first EX cycle, md_z set from the second: the
// uop leaves EX only with md_z, so the flag needs no zero test of its
// operand; its first cycle, held, is not compared)
wire        ex_early = ((ex_u.op == OP_BCC) || (ex_u.op == OP_DBCC)) && (ex_u.br != BR_NONE) && (ex_bdb_tk != ex_u.pred) &&
                       !ex_u.ser && !(ex_u.last && smc_pend) && ex_xok &&
                       !ex_fault && !ex_u.target[0] && !kill_now && !flush;
// synthesis translate_off
always_ff @(posedge clk)
	if (nreset && ex_v && ((ex_u.op == OP_BCC) || (ex_u.op == OP_DBCC)) && (ex_u.br != BR_NONE) &&
	    (ex_bdb_tk != ex_taken || (ex_early && !ex_mispred) || (ex_taken && ex_target != ex_u.target))) begin
		$display("FAIL: Bcc/DBcc early redirect: taken %b/%b mispred %b (pc %h)",
		         ex_bdb_tk, ex_taken, ex_mispred, ex_u.pc);
		$finish;
	end
// synthesis translate_on

// a change of flow to an odd address is an address error at the
// instruction (format $2, the address with A0 cleared), before any of its
// side effects -- the BSR/JSR push, the RTS/RTR pop -- except that RTE has
// committed its SR and RTR its CCR.  Bcc and DBcc validate their target
// taken or not (DBcc before its condition: Dn is left untouched).
// The frame PC follows gencpu, as the cputest 68040 AE group records it:
// the instruction, except JMP (opcode + 2; + 6 for the indexed modes), JSR
// (the odd target: the fault is the fetch there) and the handler fetch of
// an exception (the vector offset, without VBR).
// (Bcc/DBcc: the decoded target, so the check does not wait for the ALU
// result that decides taken -- that path ends in the register file write
// enable.  Every other change of flow is unconditional or an FPU one.)
// The unconditional transfers (and RTE) are always taken; the others'
// target is the decoded one, and only the FPU's (FBcc FDBcc) can be taken
// besides Bcc and DBcc: its own taken, not EX's selected one.
// (registered as the uop enters EX: the odd-address check and the
// exception address select start from a flip-flop, not the op decode)
logic       ex_bdb;
always_ff @(posedge clk)
	if (adv_dc2) ex_bdb <= (dc2_u.op == OP_BCC) || (dc2_u.op == OP_DBCC);
// synthesis translate_off
always_ff @(posedge clk)
	if (ex_v && ex_bdb !== ((ex_u.op == OP_BCC) || (ex_u.op == OP_DBCC))) begin
		$display("FAIL: EX's branch flag %b, its op's %b (pc %h)", ex_bdb, (ex_u.op == OP_BCC) || (ex_u.op == OP_DBCC), ex_u.pc);
		$finish;
	end
// synthesis translate_on
wire        ex_ubr  = (ex_u.br == BR_IMM) || (ex_u.br == BR_EA) ||
                      (ex_u.br == BR_A) || (ex_u.br == BR_B) || (ex_u.op == OP_RTE);
wire [31:0] ex_otgt = (!ex_bdb && ex_ubr) ? ex_tgt : ex_u.target;
wire        ex_odd  = ex_br && (ex_bdb ? ex_u.target[0] :
                                ex_ubr ? ex_tgt[0] :
                                ((ex_u.op == OP_FPU) && fp_taken && ex_u.target[0]));
logic [31:0] ex_odd_pc;
always_comb begin
	ex_odd_pc = ex_u.pc;
	if (ex_u.br == BR_EA)
		ex_odd_pc = ex_u.pc + (((ex_u.imm_b[5:3] == 3'd6) ||
		                        (ex_u.imm_b[5:0] == 6'o73)) ? 32'd6 : 32'd2);
	else if ((ex_u.br == BR_B && ex_u.op == OP_MOV) ||
	         (ex_u.br == BR_IMM && ex_u.op == OP_MOV && ex_u.imm_b[15:6] == 10'b0100_1110_10))
		ex_odd_pc = ex_otgt;                 // JSR (JSR_K: the opword in B)
	else if (ex_u.br == BR_A && ex_u.op == OP_MOV && ex_u.cond == 4'hF)
		ex_odd_pc = ex_ea - vbr_r;
end

// front-file write by EX, and its broadcast to younger operands
// (CAS writes Dc either way: on a match the value written -- the
// operand read, over Dc's bits above the size -- is Dc itself, so the
// compare stays off the write enable)
wire ex_d_eff = ex_u.d_v && !ex_dkill;
// (the odd-target check as it concerns a register write: only DBcc and
// FDBcc are branches that write one, so the ALU's taken stays off the
// operand forwarding -- the FPU's own taken, not EX's selected one)
wire ex_odd_w = ex_br && (ex_bdb ? ex_u.target[0] :
                          (ex_u.op == OP_FPU) && fp_taken && ex_tgt[0]);
// EX's result is final once its own unit is done (ex_hold, without WB's
// stall): the front file is written, and the younger uops capture it, from
// then on -- every cycle EX holds it, the same value -- so neither waits
// for the stall chain (adv_ex).  Without EX's exception decision: a uop
// that takes an exception, or whose instruction is refetched, flushes every
// younger uop, and the flush clears the front file (fv) in the same cycle.
wire        ex_fin  = ex_v && !ex_hold;
assign exw_v   = ex_fin && ex_d_eff;
wire        exw_ag  = exw_v;
// (EX's own operands' bypass, dc2_af/dc2_bf, from a kept copy as well: an
// unkept twin is merged into one of the kept ones and shares its fanout)
(* keep *) wire exw_f0 = ex_v && !ex_hold && ex_d_eff;
wire        exw_fw  = exw_f0;
// (the stages' captures of it from their own copies, kept: one net to the
// file, the FPU's hold inside, fanned out to three stages' operands too far)
(* keep *) wire exw_f1 = ex_v && !ex_hold && ex_d_eff;
(* keep *) wire exw_f2 = ex_v && !ex_hold && ex_d_eff;
(* keep *) wire exw_f3 = ex_v && !ex_hold && ex_d_eff;
assign exw_reg = ex_u.d_reg;
// DC2's operands as they enter EX: EX's broadcast of this cycle, then the
// load data and A's sign extension (ex_av, ex_bv)
// (whether EX's uop writes a DC2 operand's register is known before EX's
// result: EX's uop only changes when DC2 moves into it or leaves it empty,
// so the compare is made as the uop enters DC2 and registered, mx_a/mx_b)
logic       mx_a, mx_b;
// (the same for DT's uop against EX's: four compares from registers, DT's
// uop held or entering against EX's held or entering, the advances
// picking last; DT's captures of EX's result select by them)
logic       mxt_a, mxt_b;
function automatic logic omatch(input logic [1:0] src, input logic [4:0] r, input uop_t x);
	omatch = (src == OS_REG) && x.d_v && (x.d_reg == r);
endfunction
always_ff @(posedge clk) begin
	logic exv_n;
	exv_n = !(flush || kill_now || !nreset) && (stall_ex ? ex_v : adv_dc2);
	if (adv_dc1) begin
		mxt_a <= exv_n && (adv_dc2 ? omatch(dc1_u.a_src, dc1_u.a_reg, dc2_u) : omatch(dc1_u.a_src, dc1_u.a_reg, ex_u));
		mxt_b <= exv_n && (adv_dc2 ? omatch(dc1_u.b_src, dc1_u.b_reg, dc2_u) : omatch(dc1_u.b_src, dc1_u.b_reg, ex_u));
	end
	else begin
		mxt_a <= exv_n && (adv_dc2 ? omatch(dt_u.a_src, dt_u.a_reg, dc2_u) : omatch(dt_u.a_src, dt_u.a_reg, ex_u));
		mxt_b <= exv_n && (adv_dc2 ? omatch(dt_u.b_src, dt_u.b_reg, dc2_u) : omatch(dt_u.b_src, dt_u.b_reg, ex_u));
	end
end
// synthesis translate_off
always_ff @(posedge clk)
	if (dt_v && ex_v && (mxt_a !== omatch(dt_u.a_src, dt_u.a_reg, ex_u) ||
	                     mxt_b !== omatch(dt_u.b_src, dt_u.b_reg, ex_u))) begin
		$display("FAIL: DT's operand matches %b%b against EX's uop, the compares %b%b", mxt_a, mxt_b,
		         omatch(dt_u.a_src, dt_u.a_reg, ex_u), omatch(dt_u.b_src, dt_u.b_reg, ex_u));
		$finish;
	end
always_ff @(posedge clk)
	if (dc2_v && ex_v && (mx_a !== omatch(dc2_u.a_src, dc2_u.a_reg, ex_u) ||
	                      mx_b !== omatch(dc2_u.b_src, dc2_u.b_reg, ex_u))) begin
		$display("FAIL: DC2's operand matches %b%b against EX's uop, the compares %b%b", mx_a, mx_b,
		         omatch(dc2_u.a_src, dc2_u.a_reg, ex_u), omatch(dc2_u.b_src, dc2_u.b_reg, ex_u));
		$finish;
	end
// synthesis translate_on
// (and DC1's and AG's uops against EX's, the same way: their captures of
// EX's result select by registered flags)
logic       mxc_a, mxc_b, mxg_a, mxg_b;
always_ff @(posedge clk) begin
	logic exv_n;
	uop_t exn, c1n, agn;
	exv_n = !(flush || kill_now || !nreset) && (stall_ex ? ex_v : adv_dc2);
	exn   = adv_dc2 ? dc2_u : ex_u;
	c1n   = adv_ag ? ag_u : dc1_u;
	agn   = ag_ld ? ain_u : ag_u;
	// (a cancelled access's A becomes OS_ZERO as it enters DC1: ag_cancel)
	mxc_a <= exv_n && !(adv_ag && ag_cancel) && omatch(c1n.a_src, c1n.a_reg, exn);
	mxc_b <= exv_n && omatch(c1n.b_src, c1n.b_reg, exn);
	mxg_a <= exv_n && omatch(agn.a_src, agn.a_reg, exn);
	mxg_b <= exv_n && omatch(agn.b_src, agn.b_reg, exn);
end
// synthesis translate_off
always_ff @(posedge clk) begin
	if (dc1_v && ex_v && (mxc_a !== omatch(dc1_u.a_src, dc1_u.a_reg, ex_u) ||
	                      mxc_b !== omatch(dc1_u.b_src, dc1_u.b_reg, ex_u))) begin
		$display("FAIL: DC1's operand matches %b%b against EX's uop, the compares %b%b", mxc_a, mxc_b,
		         omatch(dc1_u.a_src, dc1_u.a_reg, ex_u), omatch(dc1_u.b_src, dc1_u.b_reg, ex_u));
		$finish;
	end
	if (ag_v && ex_v && (mxg_a !== omatch(ag_u.a_src, ag_u.a_reg, ex_u) ||
	                     mxg_b !== omatch(ag_u.b_src, ag_u.b_reg, ex_u))) begin
		$display("FAIL: AG's operand matches %b%b against EX's uop, the compares %b%b", mxg_a, mxg_b,
		         omatch(ag_u.a_src, ag_u.a_reg, ex_u), omatch(ag_u.b_src, ag_u.b_reg, ex_u));
		$finish;
	end
end
// synthesis translate_on
wire [31:0] dc2_af  = (exw_fw && mx_a) ? exw_val : dc2_a;
wire [31:0] dc2_bf  = (exw_fw && mx_b) ? exw_val : dc2_b;
always_ff @(posedge clk) begin
	if (adv_dt) begin
		// the uop EX holds next cycle: DC2's if it moves on, else EX's own
		// if it stays
		logic nv;
		uop_t nu;
		nv = adv_dc2 ? dc2_v : (ex_v && stall_ex);
		nu = adv_dc2 ? dc2_u : ex_u;
		mx_a <= dt_u.a_src == OS_REG && nv && nu.d_v && nu.d_reg == dt_u.a_reg;
		mx_b <= dt_u.b_src == OS_REG && nv && nu.d_v && nu.d_reg == dt_u.b_reg;
	end
	else if (adv_ex || !ex_v) begin
		mx_a <= 1'b0;
		mx_b <= 1'b0;
	end
end
// synthesis translate_off
always_ff @(posedge clk)
	if (nreset && dc2_v && !flush &&
	    (mx_a != (dc2_u.a_src == OS_REG && ex_v && ex_u.d_v && ex_u.d_reg == dc2_u.a_reg) ||
	     mx_b != (dc2_u.b_src == OS_REG && ex_v && ex_u.d_v && ex_u.d_reg == dc2_u.b_reg))) begin
		$display("FAIL: DC2's EX-writer flags %b%b disagree (pc %h)", mx_a, mx_b, dc2_u.pc);
		$finish;
	end
// synthesis translate_on
wire [31:0] dc2_av0 = (dc2_u.a_src == OS_MEM) ? dm_ldata : dc2_af;
// EX's result, as one flat select of its sources, the one-hot selects
// registered as the uop enters EX (ex_rs): the adder's sum, which comes
// last, passes one AND-OR, not the chain of the ALU's case, the op's case,
// the slow ops' and CAS's multiplexers (checked against that chain)
logic [10:0] ex_rs;
localparam int RS_SUM = 0, RS_ALU = 1, RS_SLOW = 2, RS_FP = 3, RS_MD = 4, RS_MDLO = 5,
               RS_MDREM = 6, RS_BV = 7, RS_AV = 8, RS_BFSET = 9, RS_CAS = 10;
function automatic logic [10:0] rsel(input uop_t u);
	logic [10:0] r;
	r = '0;
	if (nphase(u.op) != 2'd0) r[RS_SLOW] = 1'b1;
	else case (u.op)
		OP_ADD, OP_SUB, OP_ADDX, OP_SUBX, OP_NEGX, OP_NEG: r[RS_SUM] = 1'b1;
		OP_FPU:            r[RS_FP]    = 1'b1;
		OP_MUL, OP_DIV:    r[RS_MD]    = 1'b1;
		OP_MDRES:          if (u.cond[0]) r[RS_MDLO] = 1'b1; else r[RS_MDREM] = 1'b1;
		OP_CCRLOG:         r[RS_BV]    = 1'b1;
		OP_SPW, OP_LATCH:  r[RS_AV]    = 1'b1;
		OP_BFSET:          r[RS_BFSET] = 1'b1;
		OP_CAS:            r[RS_CAS]   = 1'b1;
		default:           r[RS_ALU]   = 1'b1;
	endcase
	rsel = r;
endfunction
always_comb begin
	logic [31:0] bfs;
	bfs = ex_u.cond[0] ? {24'd0, bf_lonew} : {{3{ex_av[31]}}, ex_av[31:3]};
	// (the sum at the size, ex_sk, in the last AND-OR; above it the
	// destination's bits, from flip-flops)
	exw_val = (ex_sk & alu_sumo) | ({32{ex_rs[RS_SUM]}} & ~ex_szm & ex_bv) |
	          ({32{ex_rs[RS_ALU]}}   & alu_resf) |
	          ({32{ex_rs[RS_SLOW]}}  & xs_q.res)  | ({32{ex_rs[RS_FP]}}    & fp_res)   |
	          ({32{ex_rs[RS_MD]}}    & md_q_res)  | ({32{ex_rs[RS_MDLO]}}  & md_lol)   |
	          ({32{ex_rs[RS_MDREM]}} & md_rem)    | ({32{ex_rs[RS_BV]}}    & ex_bv)    |
	          ({32{ex_rs[RS_AV]}}    & ex_av)     | ({32{ex_rs[RS_BFSET]}} & bfs)      |
	          ({32{ex_rs[RS_CAS]}}   & ((ex_bv & ex_szm) | (ex_latch & ~ex_szm)));
end
// synthesis translate_off
always_ff @(posedge clk)
	if (nreset && ex_fin && ex_u.d_v &&
	    exw_val != ((ex_u.op == OP_CAS) ? ((ex_bv & ex_szm) | (ex_latch & ~ex_szm)) : ex_res)) begin
		$display("FAIL: EX's flat result %h, the case chain gives %h (op %0d, pc %h)", exw_val,
		         (ex_u.op == OP_CAS) ? ((ex_bv & ex_szm) | (ex_latch & ~ex_szm)) : ex_res, ex_u.op, ex_u.pc);
		$finish;
	end
// the adder's registered controls and flags against the op's
always_ff @(posedge clk)
	if (nreset && ex_v) begin
		if (ex_ac != alu_ctl(ex_u.op)) begin
			$display("FAIL: EX's adder controls %b, the op's %b (pc %h)", ex_ac, alu_ctl(ex_u.op), ex_u.pc);
			$finish;
		end
		if (ex_fs && alu_sumfl != ex_flags) begin
			$display("FAIL: EX's adder flags %b, the op's %b (op %0d, pc %h)", alu_sumfl, ex_flags,
			         ex_u.op, ex_u.pc);
			$finish;
		end
	end
// synthesis translate_on

//--------------------------------------------------------------------------
// WB
//--------------------------------------------------------------------------
// interrupt recognition at instruction boundaries: a level above the mask,
// or a new level 7 (edge triggered, not masked)
logic  [2:0] ipl_q;
logic        nmi_edge;
logic  [2:0] irq_lvl_r;       // level being acknowledged
// the mask in force at the boundary: an instruction that writes SR (MOVE
// to SR, RTE, ANDI/ORI/EORI to SR) decides with its new mask, so a request
// it unmasks is taken at that very boundary (cputest irq/all).  A request
// that already beat the mask holds IPEND (ipend_r) and is taken at the next
// boundary although the instruction raises the mask; IPEND lasts only while
// the request is held at that level or above.
logic        ipend_r;
logic  [2:0] ipend_lvl;
// (registered: the instruction writing SR waits a clock in WB, wb_srw, so
// the mask its own boundary decides with is its new one)
logic  [2:0] irq_mask;
logic        wb_srw;          // WB's SR-writing uop has waited its clock
always_ff @(posedge clk)
	irq_mask <= (wb_v && wb_sr_we && wb_ok) ? wb_sr_new[10:8] : sr_r[10:8];
// level 7 is also recognised by level comparison whenever the mask is
// below 7 (MC68060UM 8.2.9, figure 8-3: RTE or MOVE to SR lowering the
// mask with IPL still at 7 takes it again); at mask 7 only its edge
wire         irq_pend_c = nmi_edge || (ipl_q > irq_mask) ||
                          (ipl_q != 3'd7 && ipend_r && ipl_q >= ipend_lvl);
// (a register: the same terms from the values their registers take,
// irq_pend_n below; checked against them)
logic        irq_pend_q, irq_pend_n;
wire         irq_pend = irq_pend_q;
wire  [2:0]  irq_lvl  = nmi_edge ? 3'd7 : ipl_q;

logic        stopped;         // STOP / LPSTOP: waiting for an interrupt
logic        lpstopped;       // ... LPSTOP (PST $16)
logic [31:0] stop_pc;         // the instruction after STOP
logic        rst_issued;      // RESET instruction: RSTO requested
logic        wb_stv;          // WB: a store, and no exception
logic        xstk;            // an exception routine runs (entry to its last uop)
logic        xr_dbl;          // ... and it is an access error's, an address
                              // error's or reset's (a fault in it: halt)
logic        x_first;         // the routine is done, its handler's first
                              // instruction has not completed yet
wire         xwin = xstk || x_first;   // interrupts are not sampled (8.3)
logic        wb_cof;          // the uop's control transfer was taken

wire wb_is_exc   = wb_v && !wb_ok;
wire wb_commit   = adv_wb && !wb_is_exc;
wire wb_bound    = wb_commit && wb_u.last;
wire [31:0] wb_next = wb_redir_pc;

// trace (68060: T1 only, no trace on change of flow) uses the T1 the
// instruction started with (sr_r before its own SR write); a traced STOP
// or LPSTOP loads SR, takes the trace and never stops (MC68060UM 8.2.6)
wire tr_now      = sr_r[15];
logic wb_is_stop;            // (decoded as the uop enters WB)
wire stop_traced = wb_is_stop && sr_r[15];
wire trace_due   = wb_bound && tr_now;
// 68060 priorities (MC68060UM 8.3): a trace goes before an interrupt due at
// the same boundary, and interrupts are not sampled from the start of an
// exception's processing until the first instruction of its handler has
// executed (the boundary at the end of an exception routine, xstk)
// a buffered write that ended in TEA: once the store buffer is empty, an
// access error (FSLW.SBE) at the next boundary, or instead of the
// exception the instruction at WB takes (MC68060UM 8.4.6); it goes before
// a trace or an interrupt.  (wb_refetch: below, not an exception.)
wire wb_refetch;
// Each decision in two forms from registers: if WB's uop advances this
// cycle (*1) and if it does not (*0, the stopped processor's); WB's advance,
// the end of the stall chain, picks last (checked against the direct forms)
wire wb_rf1  = wb_v && wb_xf[XF_RF];                                 // a refetch
wire wb_ex1  = wb_is_exc && !wb_rf1;                                 // an exception
wire bnd1    = wb_v && !wb_is_exc && wb_u.last;                      // a boundary
wire stp0    = stopped && !wb_v;
wire sbe1    = sb_err && sb_empty && (wb_ex1 || bnd1);
wire sbe0    = sb_err && sb_empty && stp0;
wire trc1    = bnd1 && tr_now && !sbe1;
wire irq1    = irq_pend && !(bnd1 && tr_now) && !sbe1 && bnd1 && !xstk;
wire irq0    = irq_pend && !sbe0 && stp0;
wire dbl1    = xr_dbl && !halted &&
               ((wb_ex1 && ((xstk && wb_xf[XF_23]) ||
                            (x_first && wb_xf[XF_2] && (wb_exc_fslw & FSLW_IO) != 32'd0))) ||
                (sbe1 && xstk && (sb_fslw & FSLW_SBE) != 32'd0));
wire dbl0    = xr_dbl && !halted && sbe0 && xstk && (sb_fslw & FSLW_SBE) != 32'd0;
wire xgo1    = (wb_ex1 || trc1 || irq1 || sbe1) && !dbl1 && !halted;
wire xgo0    = (irq0 || sbe0) && !dbl0 && !halted;
wire take_sbe    = adv_wb ? sbe1 : sbe0;
wire take_irq    = adv_wb ? irq1 : irq0;
// (at the first handler instruction's boundary irq_pend compares with the
// mask that instruction leaves: it can raise it to keep the interrupt
// out, 8.1, 8.2.9; no IPEND claim is formed inside xwin)
wire take_trace  = adv_wb && trc1;
assign sb_err_ack = take_sbe;
// a speculative fetch fault (code 1) is no exception: WB refetches the
// instruction once on demand, and only a second fault there is an access
// error (rt_armed/rt_pc)
// exceptions that stack the next instruction: the instruction completed
wire wb_post_trap_c = (wb_exc >= 8'd32 && wb_exc < 8'd48) ||
                    wb_exc == 8'd5 || wb_exc == 8'd6 || wb_exc == 8'd7;
wire wb_post_trap   = wb_xf[XF_POST];
assign wb_refetch = adv_wb && wb_rf1;
logic        rt_armed;
logic [31:0] rt_pc;
// a double bus fault (MC68060UM 7.9.3, 8.2.1): an access or
// address error while the routine of an access error, an address error or
// reset runs (stacking, the vector fetch, an odd handler address), or
// while the first instruction of its handler is fetched (figures 8-1,
// 8-5: the fetch of the first instruction is part of the exception's
// processing; a fault of that instruction's operands is a new access
// error), or a bus error on a buffered write of its stack frame: the
// processor halts (PST $1C) instead of taking it
wire dbl_fault   = adv_wb ? dbl1 : dbl0;
wire x_go        = adv_wb ? xgo1 : xgo0;
// synthesis translate_off
always_ff @(posedge clk)
	if (nreset) begin
		logic ts, ti, df, xg;
		ts = sb_err && sb_empty &&
		     ((adv_wb && wb_is_exc && !wb_refetch) || wb_bound || (stopped && !wb_v));
		ti = irq_pend && !trace_due && !ts && ((wb_bound && !xstk) || (stopped && !wb_v));
		df = xr_dbl && !halted &&
		     ((adv_wb && wb_is_exc && !wb_refetch &&
		       ((xstk && (wb_exc == 8'd2 || wb_exc == 8'd3)) ||
		        (x_first && wb_exc == 8'd2 && (wb_exc_fslw & FSLW_IO) != 32'd0))) ||
		      (ts && xstk && (sb_fslw & FSLW_SBE) != 32'd0));
		xg = ((adv_wb && wb_is_exc && !wb_refetch) || (trace_due && !ts) || ti || ts) && !df && !halted;
		if (ts != take_sbe || ti != take_irq || (trace_due && !ts) != take_trace ||
		    df != dbl_fault || xg != x_go) begin
			$display("FAIL: WB's two-form decisions disagree: sbe %b/%b irq %b/%b trace %b/%b dbl %b/%b go %b/%b",
			         take_sbe, ts, take_irq, ti, take_trace, trace_due && !ts, dbl_fault, df, x_go, xg);
			$finish;
		end
	end
// synthesis translate_on
// a locked sequence ends with its LOCKE write; before that only an
// exception (L and LE go to SL and SLE) or a MOVEC clearing BUSCR.L ends it
// (MC68060UM 7.4) -- not a pipeline flush, a NOP's included
assign lk_abandon = x_go || dbl_fault ||
                    (wb_commit && wb_crs[CR_BUSCR] && !wb_res[31]);

// register file write ports.  Front: the AG address updates and the EX
// result.  Back: the WB commit (a result beats an address update), the
// address update of a trap after its instruction, and at exception entry
// the routine's operands (T8-T13, three per cycle).
always_comb begin
	logic wcom, wpost;
	fwe[0] = adv_ag && ag_u.upd_v && !ag_dyncan;  fwa[0] = ag_u.upd_reg;  fwd[0] = ag_updf;
	fwe[1] = adv_ag && ag_u.upd2_v && !ag_dyncan; fwa[1] = ag_u.upd2_reg; fwd[1] = ag_u2v;
	fwoh[0] = ag_uoh; fwoh[1] = ag_u2oh; fwoh[2] = ex_doh;
	fwe_ag[0] = ag_v && ag_u.upd_v && !ag_dyncan;
	fwe_ag[1] = ag_v && ag_u.upd2_v && !ag_dyncan;
	fwe[2] = exw_v;                 fwa[2] = exw_reg;       fwd[2] = exw_val;
	wcom  = wb_commit;
	wpost = adv_wb && wb_is_exc && (wb_post_trap || wb_xcommit);
	// the back file: two write ports, the later winning on the same
	// register.  Port 0 the address update; port 1 the result, else the
	// second (deferred) update -- a uop with upd2 writes no register result
	// (its destination is memory), and its upd2 yields to its upd on the
	// same register.  So a result beats an update, and upd beats upd2.
	bwe[0] = (wcom || wpost) && wb_u.upd_v;  bwa[0] = wb_u.upd_reg;  bwd[0] = wb_upd_val;
	bwoh[0] = wb_uoh;
	if (wcom && wb_dwe) begin
		bwe[1] = 1'b1; bwa[1] = wb_u.d_reg; bwd[1] = wb_res; bwoh[1] = wb_doh;
	end
	else begin
		bwe[1] = (wcom || wpost) && wb_u.upd2_v && !(wb_u.upd_v && wb_u.upd_reg == wb_u.upd2_reg);
		bwa[1] = wb_u.upd2_reg; bwd[1] = wb_upd2_val; bwoh[1] = wb_u2oh;
	end
	bwe[2] = 1'b0; bwa[2] = '0; bwd[2] = '0; bwoh[2] = '0;
	case (xw_ph)
		2'd1: begin
			bwe = 3'b011;
			bwa[0] = R_T0 + 5'd8;  bwd[0] = xi_vaddr;
			bwa[1] = R_T0 + 5'd9;  bwd[1] = xi_pc;
		end
		2'd2: begin
			bwe = 3'b011;
			bwa[0] = R_T0 + 5'd10; bwd[0] = {16'd0, xi_sr};
			bwa[1] = R_T0 + 5'd11; bwd[1] = xi_addr;
		end
		2'd3: begin
			bwe = 3'b011;
			bwa[0] = R_T0 + 5'd12; bwd[0] = {16'd0, xi_vecw};
			bwa[1] = R_T0 + 5'd13; bwd[1] = xi_fslw;
		end
		default: ;
	endcase
	if (xw_ph != 2'd0) begin
		bwoh[0] = 32'd1 << bwa[0];      // (constants for each phase)
		bwoh[1] = 32'd1 << bwa[1];
	end
end


// WB holds: store not accepted yet; RESET until RSTO is done
// (decoded as the uop enters WB)
logic wb_reset, wb_mt;
logic mt_seen;                // the DMU finished this WB uop's maintenance
// the store buffer empties before exception processing and before a
// synchronizing instruction (MC68060UM 5.10, 8.4.6): WB waits with an
// exception, a serializing uop, or an instruction's last uop when a trace
// or an interrupt is due at its boundary
// (the last uop of an access error's, address error's or reset's routine
// waits for its frame's buffered writes: a bus error on one is a double
// bus fault, decided while the routine still runs)
wire sb_wait = !sb_empty && (wb_is_exc || wb_u.ser || (wb_u.last && (irq_pend || tr_now)) ||
                             (wb_u.last && xstk && xr_dbl));
assign wb_hold = wb_v && (halted || (!wb_is_exc &&
                 ((wb_st && !dm_st_rdy) || (wb_st && dm_st_fault) ||
                  (wb_reset && (!rst_issued || rsto_busy)) ||
                  (wb_mt && !mt_seen) || (wb_sr_we && !wb_srw))) || sb_wait);

// WB's hold a clock ahead (wb_hold_r: but for the store's answer, which the
// DMU forms), for the uop entering WB and for the one staying; what is not
// known early -- the interrupt level, the store buffer draining, RSTO --
// taken at its worst (a clock longer at most)
always_comb begin
	logic       irq_n, tr_n, hlt_n, ok_new, x_new, h_new, ok_stay, x_stay, h_stay;
	// an interrupt pending next cycle
	irq_n  = irq_pend_n;
	tr_n   = sr_r[15] || (wb_v && wb_sr_we && wb_sr_new[15]);
	hlt_n  = halted || dbl_fault || (wb_commit && wb_u.op == OP_MISC && wb_u.cond == 4'd10);
	// entering: RESET, maintenance and an SR write each wait at least a clock
	ok_new = ex_xok && !ex_fault && !ex_xnz && !ex_odd;
	x_new  = ((ex_u.op == OP_MISC) && (ex_u.cond == 4'd1 || (ex_u.cond >= 4'd3 && ex_u.cond <= 4'd5))) ||
	         ex_sr_we;
	h_new  = hlt_n || (ok_new && x_new) ||
	         (sb_ne_n && (!ok_new || ex_u.ser || (ex_u.last && (irq_n || tr_n || (xstk && xr_dbl)))));
	// staying: a store's or a PLPA's fault becomes its exception; RESET
	// until RSTO is done, maintenance until the DMU is (an SR write's clock
	// has passed)
	ok_stay = wb_ok && !(wb_st && dm_st_rdy && dm_st_fault) &&
	          !(wb_mt && mt_done && wb_u.cond == 4'd5 && mt_fault);
	x_stay  = (wb_reset && (!rst_issued || rsto_busy)) || (wb_mt && !(mt_seen || mt_done));
	h_stay  = hlt_n || (ok_stay && x_stay) ||
	          (sb_ne_n && (!ok_stay || wb_u.ser || (wb_u.last && (irq_n || tr_n || (xstk && xr_dbl)))));
	wbh_n = nreset && (adv_ex ? h_new : h_stay);
end
always_ff @(posedge clk) wbh_q <= wbh_n;
// irq_pend's registers as they will be (the main block's ipl_q, nmi_edge,
// ipend_r, ipend_lvl and irq_mask assignments, in their order)
always_comb begin
	logic       nm, ir;
	logic [2:0] il, mk;
	nm = nmi_edge;
	if (ipl == 3'd7 && ipl_q != 3'd7) nm = 1'b1;
	else if (ipl != 3'd7) nm = 1'b0;
	ir = ipend_r; il = ipend_lvl;
	if (ipl_q == 3'd0 || ipl_q == 3'd7 || (ipend_r && ipl_q < ipend_lvl) || x_go || xwin)
		ir = 1'b0;
	else if (ipl_q > sr_r[10:8] && (!ipend_r || ipl_q > ipend_lvl)) begin
		ir = 1'b1;
		il = ipl_q;
	end
	if (x_go && take_irq && !(adv_wb && wb_is_exc)) begin
		ir = 1'b0;
		if (irq_lvl == 3'd7) nm = 1'b0;
	end
	mk = (wb_v && wb_sr_we && wb_ok) ? wb_sr_new[10:8] : sr_r[10:8];
	irq_pend_n = nm || (ipl > mk) || (ipl != 3'd7 && ir && ipl >= il);
end
always_ff @(posedge clk) irq_pend_q <= nreset && irq_pend_n;
// EX's own hold as it will be: the values ex_v, the unit's flags and the
// phase counter take (their assignments below, in order)
always_comb begin
	logic       v_n, mdw_n, z_n, fin_n, slow_n;
	logic [1:0] np_n, sp_n;
	v_n    = !(flush || kill_now || !nreset) && (stall_ex ? ex_v : adv_dc2);
	mdw_n  = adv_dc2 ? ((dc2_u.op == OP_MUL || dc2_u.op == OP_DIV) &&
	                    !((dc2_u.mem != M_NONE) && dm_fault)) : ex_mdw;
	z_n    = (adv_dc2 ? 1'b0 : md_z) || (ex_v && ex_div0);
	fin_n  = adv_dc2 ? 1'b0 : (md_fin || (md_done && md_go));
	slow_n = adv_dc2 ? (nphase(dc2_u.op) != 2'd0) : ex_slow;
	np_n   = adv_dc2 ? nphase(dc2_u.op) : ex_np;
	sp_n   = (adv_ex || kill_now || !nreset) ? 2'd0 :
	         (ex_v && ex_slow && ex_sp != ex_np) ? ex_sp + 2'd1 : ex_sp;
	exh_n = v_n && ((mdw_n && !z_n && !fin_n) || (slow_n && sp_n != np_n));
end
// (the same with the kill taken as 0: a kill clears every stage's valid,
// so the stalls are zero whatever this is; its logic stays off their D)
always_comb begin
	logic       v_n, mdw_n, z_n, fin_n, slow_n;
	logic [1:0] np_n, sp_n;
	v_n    = !(flush || 1'b0 || !nreset) && (stall_ex ? ex_v : adv_dc2);
	mdw_n  = adv_dc2 ? ((dc2_u.op == OP_MUL || dc2_u.op == OP_DIV) &&
	                    !((dc2_u.mem != M_NONE) && dm_fault)) : ex_mdw;
	z_n    = (adv_dc2 ? 1'b0 : md_z) || (ex_v && ex_div0);
	fin_n  = adv_dc2 ? 1'b0 : (md_fin || (md_done && md_go));
	slow_n = adv_dc2 ? (nphase(dc2_u.op) != 2'd0) : ex_slow;
	np_n   = adv_dc2 ? nphase(dc2_u.op) : ex_np;
	sp_n   = (adv_ex || 1'b0 || !nreset) ? 2'd0 :
	         (ex_v && ex_slow && ex_sp != ex_np) ? ex_sp + 2'd1 : ex_sp;
	exh_nk = v_n && ((mdw_n && !z_n && !fin_n) || (slow_n && sp_n != np_n));
end
always_ff @(posedge clk) exh_q <= exh_n;
// synthesis translate_off
always_ff @(posedge clk)
	if (nreset) begin
		if (ex_v && (ex_szm != ((ex_u.sz == SZ_B) ? 32'h0000_00FF : (ex_u.sz == SZ_W) ? 32'h0000_FFFF : 32'hFFFF_FFFF) ||
		             ex_mw != {ex_u.sz == SZ_W, ex_u.op == OP_MUL && ex_u.sz == SZ_W})) begin
			$display("FAIL: EX's registered size selects disagree (pc %h)", ex_u.pc);
			$finish;
		end
		if (ag_v && (ag_bfh != ((ag_u.mem != M_NONE) && (ag_u.op == OP_MOV) && (ag_u.cond == 4'hE)) ||
		             ag_bft != ((ag_u.mem != M_NONE) && (ag_u.op == OP_MOV) && (ag_u.cond == 4'hD)) ||
		             ((ag_bfh || ag_bft) && (bf_n1 != (bf_nb == 3'd1) || bf_nle3 != (bf_nb <= 3'd3) ||
		                                     bf_n3 != (bf_nb == 3'd3) ||
		                                     bf_n35 != (bf_nb == 3'd3 || bf_nb == 3'd5))))) begin
			$display("FAIL: AG's registered bit-field selects disagree (pc %h)", ag_u.pc);
			$finish;
		end
		if (ex_v && (ex_br != ((ex_u.br != BR_NONE) || (ex_u.op == OP_RTE)) || ex_tgt != ex_target ||
		             ex_tkn != ex_taken || ex_tne != (ex_target != ex_u.target))) begin
			$display("FAIL: EX's registered target/taken selects disagree: %h/%h %b/%b (pc %h)",
			         ex_tgt, ex_target, ex_tkn, ex_taken, ex_u.pc);
			$finish;
		end
		if (ex_v && ex_u.op == OP_SPR && ex_sp == 2'd1 && spr_q != spr_read(ex_u.imm[7:0])) begin
			$display("FAIL: MOVEC's registered read %h, the register's %h (pc %h)", spr_q,
			         spr_read(ex_u.imm[7:0]), ex_u.pc);
			$finish;
		end
		if (wb_v && wb_xf != xflags(wb_exc)) begin
			$display("FAIL: WB's registered exception cases %b disagree with %0d (pc %h)", wb_xf, wb_exc, wb_u.pc);
			$finish;
		end
		if (wb_v && wb_crs != cr_sel(wb_u)) begin
			$display("FAIL: WB's control register select %b disagrees (pc %h)", wb_crs, wb_u.pc);
			$finish;
		end
		if (ex_v && !ex_hold && ex_xnz != (ex_xvec != 8'd0)) begin
			$display("FAIL: EX's exception flag %b, its vector %0d (pc %h)", ex_xnz, ex_xvec, ex_u.pc);
			$finish;
		end
		if (irq_pend_q != irq_pend_c) begin
			$display("FAIL: the registered interrupt pending %b, its terms' %b", irq_pend_q, irq_pend_c);
			$finish;
		end
		if (wb_v && wb_hold && !wb_hold_r) begin
			$display("FAIL: WB's registered hold is off while WB holds (pc %h)", wb_u.pc);
			$finish;
		end
		if (dc1_v && dm_hold1_c && !dm_hold1) begin
			$display("FAIL: DC1's registered hold is off while the DMU holds it (pc %h)", dc1_u.pc);
			$finish;
		end
		if (ex_v && ex_hold != ((ex_is_md && !md_z && !ex_fault && !md_fin) ||
		                        (ex_slow && ex_sp != ex_np) ||
		                        (ex_is_fp && !ex_fault && ex_u.exc == 8'd0 && fp_hold))) begin
			$display("FAIL: EX's hold from its entry flags disagrees (pc %h)", ex_u.pc);
			$finish;
		end
		if (dc2_v && dc2_mx != ((dc2_u.mem != M_NONE) && (dc2_u.exc == 8'd0))) begin
			$display("FAIL: DC2's memory flag disagrees (pc %h)", dc2_u.pc);
			$finish;
		end
		if (ilk_q != ag_interlock) begin
			$display("FAIL: AG's registered interlock %b, the flags' %b (pc %h)", ilk_q, ag_interlock, ag_u.pc);
			$finish;
		end
	end
// synthesis translate_on

assign mt_v      = wb_v && wb_mt && !wb_is_exc && !mt_seen;
assign mt_op     = (wb_u.cond == 4'd3) ? (wb_u.imm_b[5] ? 3'd2 : 3'd1) :
                   (wb_u.cond == 4'd4) ? 3'd3 : 3'd4;
assign mt_scope  = (wb_u.cond == 4'd3) ? wb_u.imm_b[4:3] :
                   (wb_u.cond == 4'd4) ? (wb_u.imm_b[4] ? 2'd3 : 2'd2) : 2'd2;
assign mt_caches = wb_u.imm_b[7:6];
assign mt_addr   = wb_res;
assign mt_fc     = dfc_r;
assign mt_ng     = (wb_u.cond == 4'd4) && !wb_u.imm_b[3];
assign mt_wr     = (wb_u.cond == 4'd5) && !wb_u.imm_b[6];   // PLPAW $F588, PLPAR $F5C8

logic        rst_seq;         // reset exception pending (first cycles)

// same-cycle kill of every younger uop: WB takes an exception or redirects
assign kill_now = adv_wb ? (wb_is_exc || wb_redir || xgo1) : xgo0;
(* keep *) wire dmk = dma_wb ? (wb_is_exc || wb_redir || xgo1) : xgo0;
assign dm_kill = dmk;

// exception entry values
logic  [7:0] x_vec;
logic  [3:0] x_fmt;
logic [31:0] x_pc, x_addr, x_fslw;
logic [15:0] x_osr, x_nsr;
logic  [3:0] x_kind;
logic  [4:0] x_ssp;
always_comb begin
	logic [15:0] cur;
	// the SR the exception sees: after this instruction when it completed
	cur = wb_commit ? (wb_sr_we ? (wb_sr_new & SR_MASK) : {sr_r[15:5], wb_ccr})
	                : {sr_r[15:5], wb_ccr};
	if (stopped && !wb_v) cur = {sr_r[15:5], ccr_b};
	// RTE to an odd PC (68060, WinUAE gencpu): the system byte stays, the
	// CCR is cleared except Z and N, set from the new SR word
	if (adv_wb && wb_is_exc && wb_oddrte)
		cur = {sr_r[15:8], 3'b000, 1'b0, wb_sr_new[15], wb_sr_new == 16'd0, 2'b00};
	// an interrupt whose handler address is odd: the address error comes
	// before the interrupt clears M (WinUAE cputest, 68060: M stays)
	if (adv_wb && wb_is_exc && wb_hodd && xm_v)
		cur[12] = xm_m;
	x_osr  = cur;
	x_fslw = 32'd0;
	x_addr = wb_u.pc;
	x_pc   = wb_u.pc;
	x_vec  = wb_exc;
	if (adv_wb && wb_is_exc) begin
		x_vec  = wb_exc;
		x_addr = wb_exc_addr;
		x_fslw = wb_exc_fslw;
		// TRAP #n, TRAPcc, CHK and divide-by-zero stack the next
		// instruction; faults and illegal opcodes the instruction
		x_pc   = ((wb_exc >= 8'd32 && wb_exc < 8'd48) ||
		          wb_exc == 8'd5 || wb_exc == 8'd6 || wb_exc == 8'd7) ? wb_npc : wb_u.pc;
		if (wb_exc_pcv) x_pc = wb_exc_pc;
	end
	else if (take_irq) begin
		x_vec  = 8'd24 + {5'd0, irq_lvl};   // replaced by the IACK result
		x_pc   = (stopped && !wb_v) ? stop_pc : wb_next;
	end
	else begin
		// trace: format $2, PC = where execution continues, address = the
		// traced instruction
		x_vec  = 8'd9;
		x_pc   = wb_next;
		x_addr = wb_u.pc;
	end
	// 68060 frames (MC68060UM table 8-1): $4 access error, $2 address
	// error, CHK, TRAPcc/TRAPV, divide by zero, trace; $0 the rest; the FPU
	// brings its own ($0, $2, $3, $4)
	x_fmt  = (x_vec == 8'd2) ? 4'h4 :
	         (x_vec == 8'd3 || x_vec == 8'd5 || x_vec == 8'd6 || x_vec == 8'd7 ||
	          x_vec == 8'd9) ? 4'h2 : 4'h0;
	if (adv_wb && wb_is_exc && wb_exc_fmtv) x_fmt = wb_exc_fmt;
	x_kind = take_irq && !(adv_wb && wb_is_exc) ? EK_IRQ :
	         (x_fmt == 4'h4) ? EK_FMT4 : (x_fmt == 4'h2 || x_fmt == 4'h3) ? EK_FMT2 : EK_FMT0;
	x_ssp  = R_ISP;
	// a store buffer bus error: an access error, the stacked address
	// meaningless; the PC is the one the exception due here would stack,
	// or where execution continues (8.4.6)
	if (take_sbe) begin
		x_vec  = 8'd2;
		x_fmt  = 4'h4;
		x_kind = EK_FMT4;
		x_fslw = sb_fslw;                // SBE and/or PBE
		x_addr = 32'd0;
		if (!(adv_wb && wb_is_exc)) x_pc = (stopped && !wb_v) ? stop_pc : wb_next;
	end

	// the SR inside the handler: S set, T cleared; an interrupt raises the
	// mask to its level and clears M (the 68060 keeps M only as a flag)
	x_nsr  = {2'b00, 1'b1, cur[12], cur[11:0]};
	if (take_irq && !(adv_wb && wb_is_exc)) begin
		x_nsr[12]   = 1'b0;
		x_nsr[10:8] = irq_lvl;
	end
end

// the return stack after the instruction at WB
function automatic logic [6:0] ras_after(input ras_snap_t r);
	case (r.k)
		2'd1:    ras_after = {r.tp + 3'd1, (r.n == 4'd8) ? 4'd8 : r.n + 4'd1};
		2'd2:    ras_after = {r.tp - 3'd1, r.n - 4'd1};
		default: ras_after = {r.tp, r.n};
	endcase
endfunction

always_ff @(posedge clk) begin
	redir_v <= 1'b0;
	ras_rv  <= 1'b0;
	exc_go  <= 1'b0;
	flush   <= 1'b0;
	fe_flush <= 1'b0;
	ucond_v <= 1'b0;
	dbg_retire <= 1'b0;

	if (!nreset) begin
		ag_v  <= 1'b0; dc1_v <= 1'b0; dt_v <= 1'b0; dc2_v <= 1'b0; ex_v <= 1'b0; wb_v <= 1'b0;
		sk_v  <= 1'b0;
		sk_vr <= 1'b0; sk_va <= 1'b0;
		eredir_pend <= 1'b0;
		wb_eredir   <= 1'b0;
		fv <= 32'd0;
		xw_ph <= 2'd0;
		ccr_f   <= 5'd0;
		ccr_b   <= 5'd0;
		sr_r    <= 16'h2700;
		xm_v    <= 1'b0;
		xm_m    <= 1'b0;
		vbr_r   <= 32'd0;
		cacr_r  <= 32'd0;
		bc_clr  <= 1'b0;
		sfc_r   <= 3'd0;
		dfc_r   <= 3'd0;
		tc_r    <= 32'd0;
		itt0_r  <= 32'd0; itt1_r <= 32'd0; dtt0_r <= 32'd0; dtt1_r <= 32'd0;
		urp_r  <= 32'd0; srp_r  <= 32'd0;
		pcr_edebug <= 1'b0; pcr_dfp <= 1'b0; pcr_ess <= 1'b0;
		bus_l <= 1'b0; bus_sl <= 1'b0; bus_le <= 1'b0; bus_sle <= 1'b0; bus_lkdone <= 1'b0;
		xi_pc   <= 32'd0; xi_addr <= 32'd0; xi_sr <= 16'd0; xi_vecw <= 16'd0; xi_fslw <= 32'd0;
		md_go   <= 1'b0;
		md_fin  <= 1'b0;
		md_z    <= 1'b0;
		ex_sp   <= 2'd0;
		xs_q    <= '0;
		md_rem  <= 32'd0;
		md_lol  <= 32'd0;
		md_ovfl <= 1'b0;
		ex_latch <= 32'd0;
		bf_off   <= 32'd0;
		bf_w     <= 6'd32;
		bf_wn    <= 6'd0;
		bf_top   <= 32'hFFFF_FFFF;
		bf_sx    <= 32'd0;
		bf_lonew <= 8'd0;
		ipl_q   <= 3'd0;
		nmi_edge <= 1'b0;
		irq_lvl_r <= 3'd0;
		ipend_r  <= 1'b0;
		ipend_lvl <= 3'd0;
		rt_armed <= 1'b0;
		rt_pc    <= 32'd0;
		smc_pend <= 1'b0;
		stopped <= 1'b0;
		lpstopped <= 1'b0;
		stop_pc <= 32'd0;
		rst_issued <= 1'b0;
		mt_seen <= 1'b0;
		wb_cof  <= 1'b0;
		halted  <= 1'b0;
		rst_seq <= 1'b1;
		exc_kind <= EK_RESET;
		exc_ssp <= R_ISP;
		redir_pc <= 32'd0;
		ras_rtp <= '0;
		ras_rn <= '0;
	end
	else begin
		//------------------------------------------------------------------
		// reset exception: the routine loads ISP and PC from 0 and 4
		//------------------------------------------------------------------
		if (rst_seq) begin
			rst_seq  <= 1'b0;
			exc_go   <= 1'b1;
			exc_kind <= EK_RESET;
			exc_ssp  <= R_ISP;
		end

		ipl_q <= ipl;
		// (no claim from the start of an exception's processing until its
		// handler's first instruction has completed, and none carried into
		// it: that instruction may raise the mask first, 8.1, 8.3)
		if (ipl_q == 3'd0 || ipl_q == 3'd7 || (ipend_r && ipl_q < ipend_lvl) || x_go || xwin)
			ipend_r <= 1'b0;
		else if (ipl_q > sr_r[10:8] && (!ipend_r || ipl_q > ipend_lvl)) begin
			ipend_r   <= 1'b1;
			ipend_lvl <= ipl_q;
		end
		if (ipl == 3'd7 && ipl_q != 3'd7) nmi_edge <= 1'b1;
		else if (ipl != 3'd7) nmi_edge <= 1'b0;

		//------------------------------------------------------------------
		// AG -> DC1
		//------------------------------------------------------------------
		if (adv_ag) begin
			dc1_u   <= ag_u;
			dc1_u.msz <= ag_msz;
			if (ag_snref) begin
				// refetched from here (see snp_pend): no access, no effect
				dc1_u.exc <= EXC_SNR;
				dc1_u.mem <= M_NONE;
			end
			else if (ag_casmis && ag_u.exc == 8'd0) begin
				dc1_u.exc <= 8'd61;
				dc1_u.mem <= M_NONE;
			end
			if (ag_cancel) begin
				// no tail byte: a load gives zero, a store nothing; a
				// cancelled FPU transfer also leaves its address registers
				dc1_u.mem   <= M_NONE;
				dc1_u.a_src <= OS_ZERO;
				if (ag_dyncan) begin
					dc1_u.upd_v  <= 1'b0;
					dc1_u.upd2_v <= 1'b0;
				end
			end
			dc1_a   <= ag_a;
			dc1_b   <= ag_b;
			dc1_ea  <= ag_u.ag ? ag_maddr : 32'd0;
			dc1_upd <= ag_updv;
			dc1_upd2 <= ag_u2v;
		end
		if (!stall_dc1) dc1_v <= adv_ag;
		else begin
			if (exw_f1 && mxc_a) dc1_a <= exw_val;
			if (exw_f1 && mxc_b) dc1_b <= exw_val;
		end
		// snoop into the uop entering DC1
		if (adv_ag && exw_f1) begin
			if (mxg_a) dc1_a <= exw_val;
			if (mxg_b) dc1_b <= exw_val;
		end

		//------------------------------------------------------------------
		// DC1 -> DT
		//------------------------------------------------------------------
		if (adv_dc1) begin
			dt_u    <= dc1_u;
			dt_a    <= (exw_f2 && mxc_a) ? exw_val : dc1_a;
			dt_b    <= (exw_f2 && mxc_b) ? exw_val : dc1_b;
			dt_ea   <= dc1_ea;
			dt_upd  <= dc1_upd;
			dt_upd2 <= dc1_upd2;
		end
		else if (dt_v && exw_f2) begin
			if (mxt_a) dt_a <= exw_val;
			if (mxt_b) dt_b <= exw_val;
		end
		if (!stall_dt) dt_v <= adv_dc1;

		//------------------------------------------------------------------
		// DT -> DC2
		//------------------------------------------------------------------
		if (adv_dt) begin
			dc2_u    <= dt_u;
			dc2_mx   <= (dt_u.mem != M_NONE) && (dt_u.exc == 8'd0);
			dc2_a    <= (exw_f3 && mxt_a) ? exw_val : dt_a;
			dc2_b    <= (exw_f3 && mxt_b) ? exw_val : dt_b;
			dc2_ea   <= dt_ea;
			dc2_upd  <= dt_upd;
			dc2_upd2 <= dt_upd2;
		end
		else if (dc2_v && exw_f3) begin
			if (mx_a) dc2_a <= exw_val;
			if (mx_b) dc2_b <= exw_val;
		end
		if (!stall_dc2) dc2_v <= adv_dt;

		//------------------------------------------------------------------
		// DC2 -> EX
		//------------------------------------------------------------------
		if (adv_dc2) begin
			ex_u     <= dc2_u;
			ex_npc   <= dc2_u.pc + {27'd0, dc2_u.ilen, 1'b0};
			ex_rs    <= rsel(dc2_u);
			ex_ac    <= alu_ctl(dc2_u.op);
			ex_rk    <= alu_rk(dc2_u.op);
			ex_doh   <= 32'd1 << dc2_u.d_reg;
			ex_br    <= (dc2_u.br != BR_NONE) || (dc2_u.op == OP_RTE);
			if (nphase(dc2_u.op) != 2'd0) begin
				ex_ts <= 5'b10000;
				ex_tk <= 5'b10000;
			end
			else begin
				ex_ts <= (dc2_u.br == BR_A) ? 5'b00001 : (dc2_u.br == BR_B) ? 5'b00010 :
				         (dc2_u.br == BR_EA) ? 5'b00100 : 5'b01000;
				ex_tk <= (dc2_u.br == BR_IMM || dc2_u.br == BR_EA || dc2_u.br == BR_A ||
				          dc2_u.br == BR_B)  ? 5'b00001 :
				         (dc2_u.op == OP_FPU)  ? 5'b00010 :
				         (dc2_u.op == OP_BCC)  ? 5'b00100 :
				         (dc2_u.op == OP_DBCC) ? 5'b01000 : 5'b00000;
			end
			ex_szm   <= (dc2_u.sz == SZ_B) ? 32'h0000_00FF :
			            (dc2_u.sz == SZ_W) ? 32'h0000_FFFF : 32'hFFFF_FFFF;
			ex_mw    <= {dc2_u.sz == SZ_W, dc2_u.op == OP_MUL && dc2_u.sz == SZ_W};
			ex_kx    <= {dc2_u.op == OP_FPU, dc2_u.op == OP_DIV, dc2_u.op == OP_TRAPCC};
			ex_sk    <= (rsel(dc2_u) & 11'd1 << RS_SUM) != 11'd0 ?
			            ((dc2_u.sz == SZ_B) ? 32'h0000_00FF : (dc2_u.sz == SZ_W) ? 32'h0000_FFFF : 32'hFFFF_FFFF) :
			            32'd0;
			ex_fs    <= nphase(dc2_u.op) == 2'd0 &&
			            (dc2_u.op == OP_ADD || dc2_u.op == OP_SUB || dc2_u.op == OP_ADDX ||
			             dc2_u.op == OP_SUBX || dc2_u.op == OP_NEG || dc2_u.op == OP_NEGX ||
			             dc2_u.op == OP_CMP);
			ex_kc    <= {dc2_u.op == OP_DBCC, dc2_u.op == OP_MDRES, dc2_u.op == OP_DIV,
			             dc2_u.op == OP_FPU};
			ex_a     <= dc2_af;
			ex_b     <= dc2_bf;
			ex_av    <= dc2_u.a_sxw ? {{16{dc2_av0[15]}}, dc2_av0[15:0]} : dc2_av0;
			ex_bv    <= (dc2_u.b_src == OS_MEM) ? dm_ldata : dc2_bf;
			ex_np    <= nphase(dc2_u.op);
			ex_slow  <= nphase(dc2_u.op) != 2'd0;
			ex_shq   <= is_shift(dc2_u.op);
			ex_ea    <= dc2_ea;
			ex_ld    <= dm_ldata;
			ex_upd   <= dc2_upd;
			ex_upd2  <= dc2_upd2;
			ex_fault <= (dc2_u.mem != M_NONE) && dm_fault;
			ex_mdw   <= (dc2_u.op == OP_MUL || dc2_u.op == OP_DIV) &&
			            !((dc2_u.mem != M_NONE) && dm_fault);
			ex_fpw   <= (dc2_u.op == OP_FPU) && !((dc2_u.mem != M_NONE) && dm_fault) &&
			            (dc2_u.exc == 8'd0);
			ex_fpw_h <= (dc2_u.op == OP_FPU) && !((dc2_u.mem != M_NONE) && dm_fault) &&
			            (dc2_u.exc == 8'd0);
			ex_fvec  <= dm_fvec;
			ex_faddr <= dm_faddr;
			ex_fslw  <= dm_fslw;
			md_go    <= 1'b0;
			md_fin   <= 1'b0;
			md_z     <= 1'b0;
		end
		if (!stall_ex) ex_v <= adv_dc2;
		// EX's next uop against WB's next: EX's own when it moves on
		if (adv_dc2) ew_same <= adv_ex ? (dc2_u.pc == ex_u.pc) : (dc2_u.pc == wb_u.pc);
		else if (adv_ex) ew_same <= 1'b1;
		if (ex_md_start) md_go <= 1'b1;
		if (ex_v && ex_div0) md_z <= 1'b1;
		// multi-cycle operations: phase 0's registers (the bit fields',
		// ROXL/ROXR's count), phase 1's (the bit fields'), the result in
		// the next-to-last phase
		if (ex_sp == 2'd0) begin
			bf_ins_q <= ex_av << bf_wn;
			bf_rot_q <= bf_rot;
			rox_nx_q <= rox_nx;
		end
		if (ex_sp == 2'd1) begin
			bf_fld_q  <= bf_fld;
			bf_nrot_q <= bf_nrot;
			bf_nwin_q <= bf_nwin;
			bf_lz_q   <= bf_lz;
			bf_nf_q   <= bf_nf;
			bf_zf_q   <= bf_zf;
			bf_r31_q  <= bf_rot_q[31];
		end
		spr_q <= spr_read(ex_u.imm[7:0]);
		if (ex_v && ex_slow && ex_sp == ex_np - 2'd1) begin
			xs_q   <= xs;
			xs_xnz <= xs.xvec != 8'd0;
		end
		if (ex_v && ex_slow && ex_sp != ex_np) ex_sp <= ex_sp + 2'd1;
		if (adv_ex || kill_now) ex_sp <= 2'd0;

		//------------------------------------------------------------------
		// EX -> WB
		//------------------------------------------------------------------
		if (adv_ex) begin
			wb_u        <= ex_u;
			wb_npc      <= ex_npc;
			wb_res      <= exw_val;
			wb_dwe      <= ex_d_eff;
			wb_upd_val  <= ex_upd;
			wb_upd2_val <= ex_upd2;
			wb_st       <= (ex_u.mem == M_ST || ex_u.mem == M_RMW) && !ex_stkill;
			// (the same, and no exception: the DMU's store valid from a
			// flip-flop -- wb_exc's compare sat on the stall chain)
			wb_stv      <= (ex_u.mem == M_ST || ex_u.mem == M_RMW) && !ex_stkill &&
			               ex_xok && !ex_fault && !ex_xnz && !ex_odd;
			wb_ok       <= ex_xok && !ex_fault && !ex_xnz && !ex_odd;
			wb_st_data  <= ex_st;
			// a store into the instruction stream just ahead (the fetch
			// queue and the instructions in flight) refetches after itself:
			// the 68040 does not promise this, the previous core's AmigaOS
			// boot depended on it (t_integer 192)
			// (a store from a uop before the last one -- MOVEM, a bit
			// field, CAS2 -- defers the refetch to the instruction's end:
			// redirecting there would drop the instruction's remaining uops)
			wb_redir    <= ex_mispred || ex_u.ser || ex_sr_we ||
			               (ex_u.last && (smc_pend || ex_smc));
			wb_eredir   <= ex_early;
			if (ex_early) begin
				fe_flush    <= 1'b1;
				redir_v     <= 1'b1;
				redir_pc    <= ex_bdb_nx;
				ras_rv      <= ex_u.ras.v;
				{ras_rtp, ras_rn} <= ras_after(ex_u.ras);
				eredir_pend <= 1'b1;
			end
			if (ex_u.last) smc_pend <= 1'b0;
			else if (ex_smc) smc_pend <= 1'b1;
			// where execution continues, predicted correctly or not
			wb_redir_pc <= ex_br ? ex_next : ex_npc;
			wb_sr_we    <= ex_sr_we;
			wb_crs      <= cr_sel(ex_u);
			wb_uoh      <= 32'd1 << ex_u.upd_reg;
			wb_u2oh     <= 32'd1 << ex_u.upd2_reg;
			wb_doh      <= 32'd1 << ex_u.d_reg;
			wb_reset    <= (ex_u.op == OP_MISC) && (ex_u.cond == 4'd1);
			wb_mt       <= (ex_u.op == OP_MISC) && (ex_u.cond >= 4'd3) && (ex_u.cond <= 4'd5);
			wb_is_stop  <= (ex_u.op == OP_MISC) && (ex_u.cond == 4'd2 || ex_u.cond == 4'd8);
			wb_cof      <= ex_br && ex_tkn;
			wb_sr_new   <= ex_sr_new;
			if (ex_u.exc == EXC_SNR || ex_snw) begin
				wb_exc      <= EXC_IFS;     // refetched by WB, never an error
				wb_xf       <= xflags(EXC_IFS);
				wb_exc_addr <= ex_u.pc;
				wb_exc_fslw <= 32'd0;
			end
			else if (ex_u.exc != 8'd0) begin
				logic ifs, ifa;
				// instruction fetch faults: a speculative one is refetched
				// once (wb_refetch) and becomes an access error when it
				// faults again at the same instruction
				ifs = (ex_u.exc == EXC_IFS || ex_u.exc == EXC_IFSA);
				ifa = (ex_u.exc == EXC_IFA || ex_u.exc == EXC_IFSA);
				if (ifs && !(rt_armed && rt_pc == ex_u.pc)) begin
					wb_exc <= ex_u.exc;
					wb_xf  <= xflags(ex_u.exc);
				end
				else if (ifs || ex_u.exc == EXC_IFB || ex_u.exc == EXC_IFA) begin
					wb_exc <= 8'd2;
					wb_xf  <= xflags(8'd2);
				end
				else begin
					wb_exc <= ex_u.exc;
					wb_xf  <= xflags(ex_u.exc);
				end
				// a faulted instruction fetch (MC68060UM 8.4.4): FA is the
				// instruction's opword address; FSLW IO, MA when the faulted
				// word was an extension word, read, long, TT normal, TM the
				// program space (FC 6 or 2), and the cause: the translation's
				// (PTA PTB IL PF SP, TWE LK for a bus error in the search) or a
				// bus error (RE)
				if (ifs || ex_u.exc == EXC_IFB || ex_u.exc == EXC_IFA) begin
					wb_exc_addr <= ex_u.pc;
					wb_exc_fslw <= FSLW_IO | ((ex_u.target != ex_u.pc) ? FSLW_MA : 32'd0) |
					               FSLW_R | {13'd0, sr_r[13], 2'b10, 16'd0} |
					               (ifa ? if_cause : (FSLW_RE | (if_cause & FSLW_TTR)));
				end
				else begin
					wb_exc_addr <= ex_u.pc;
					wb_exc_fslw <= 32'd0;
				end
			end
			else if (ex_fault) begin
				wb_exc      <= ex_fvec;
				wb_xf       <= xflags(ex_fvec);
				wb_exc_addr <= ex_faddr;
				wb_exc_fslw <= ex_fslw;
			end
			else if (!ex_xnz && ex_odd) begin
				wb_exc      <= 8'd3;
				wb_xf       <= xflags(8'd3);
				wb_exc_addr <= {ex_otgt[31:1], 1'b0};
				wb_exc_fslw <= 32'd0;
			end
			else if (ex_is_fp && ex_xnz) begin
				// FPU exceptions bring their frame format, PC and address;
				// the FPU-disabled frame ($4) has the instruction's PC at +$C
				wb_exc      <= ex_xvec;
				wb_xf       <= xflags(ex_xvec);
				wb_exc_addr <= fp_xaddr;
				wb_exc_fslw <= (fp_xfmt == 4'h4) ? ex_u.pc : 32'd0;
			end
			else begin
				wb_exc      <= ex_xvec;
				wb_xf       <= xflags(ex_xvec);
				wb_exc_addr <= ex_u.pc;
				wb_exc_fslw <= 32'd0;
			end
			wb_exc_pcv <= ex_xok && !ex_fault && !ex_xnz && ex_odd;
			wb_exc_pc  <= ex_odd_pc;
			wb_exc_fmtv <= 1'b0;
			wb_xcommit  <= 1'b0;
			if (ex_xok && !ex_fault && ex_is_fp && ex_xnz) begin
				wb_exc_pcv  <= 1'b1;
				wb_exc_pc   <= fp_xnext ? ex_npc : ex_u.pc;
				wb_exc_fmtv <= 1'b1;
				wb_exc_fmt  <= fp_xfmt;
				wb_xcommit  <= fp_xcommit;
			end
			wb_oddrte  <= ex_xok && !ex_fault && !ex_xnz && ex_odd &&
			              ex_u.op == OP_RTE;
			wb_hodd    <= ex_xok && !ex_fault && !ex_xnz && ex_odd &&
			              ex_u.br == BR_A && ex_u.op == OP_MOV && ex_u.cond == 4'hF;
			// CCR after this uop (CHK, CHK2, TRAPcc and divide-by-zero set
			// their flags and then trap: the stacked SR carries them)
			if (ex_xok && !ex_fault) begin
				logic [4:0] nf;
				// (the adder's flags, the late ones, picked last)
				nf = ex_fs ? alu_sumfl : (ex_u.op == OP_CAS) ? cas_flags : ex_flags;
				ccr_f  <= (nf & ex_u.ccr_we) | (ccr_f & ~ex_u.ccr_we);
				wb_ccr <= (nf & ex_u.ccr_we) | (ccr_f & ~ex_u.ccr_we);
			end
			else wb_ccr <= ccr_f;
			if (ex_u.op == OP_LATCH || (ex_u.op == OP_IACKV && !ex_u.cond[0])) ex_latch <= ex_av;
			if (ex_u.op == OP_BFSET && !ex_u.cond[0]) begin
				logic [5:0] w;
				logic [6:0] e;
				w = (ex_bv[4:0] == 5'd0) ? 6'd32 : {1'b0, ex_bv[4:0]};
				e = {4'd0, ex_av[2:0]} + {1'b0, w} + 7'd7;
				bf_off <= ex_av;
				bf_w   <= w;
				bf_wn  <= 6'd32 - w;
				bf_top <= ~(32'hFFFF_FFFF >> w);      // (a shift by 32 gives 0)
				bf_sx  <= 32'hFFFF_FFFF << w;
				bf_nb  <= e[5:3];
				bf_n1  <= e[5:3] == 3'd1;
				bf_nle3 <= e[5:3] <= 3'd3;
				bf_n3  <= e[5:3] == 3'd3;
				bf_n35 <= e[5:3] == 3'd3 || e[5:3] == 3'd5;
			end
			if (ex_u.op == OP_BF) bf_lonew <= bf_lo_out;
		end
		// (not when the next uop enters EX this cycle: the reset above is
		// for it, and the done belongs to the one leaving)
		if (md_done && md_go && !adv_dc2) md_fin <= 1'b1;
		if (md_done) begin
			md_q_res   <= md_res;
			md_q_flags <= md_flags;
			md_q_dkill <= md_dkill;
			md_rem  <= md_hi;
			md_lol  <= md_lo;
		end
		// overflow of the uop leaving EX (word forms check 16 bits)
		if (adv_ex && (ex_u.op == OP_DIV || ex_u.op == OP_MUL)) md_ovfl <= (ex_u.op == OP_DIV) && md_q_dkill;
		if (!stall_wb) wb_v <= adv_ex;
		// an SR-writing uop's first clock in WB has passed
		if (!stall_wb) wb_srw <= 1'b0;
		else if (wb_v && wb_sr_we) wb_srw <= 1'b1;

		//------------------------------------------------------------------
		// WB commit
		//------------------------------------------------------------------
		bc_clr <= 1'b0;
		if (wb_commit) begin
			dbg_retire <= wb_u.last;
			// architectural commit: the back file's write ports (bwe)
			ccr_b <= wb_ccr;
			// MOVEC to the 68060's control registers (masks: WinUAE
			// m68k_move2c; CABC and CUBC act and read as zero), the register
			// decoded as the uop enters WB (wb_crs)
			if (wb_crs[CR_SFC])  sfc_r  <= wb_res[2:0];
			if (wb_crs[CR_DFC])  dfc_r  <= wb_res[2:0];
			if (wb_crs[CR_CACR]) begin
				cacr_r <= wb_res & 32'hF880_E000;
				bc_clr <= wb_res[22] || wb_res[21];
			end
			if (wb_crs[CR_TC])   tc_r   <= wb_res & 32'h0000_FFFE;
			if (wb_crs[CR_ITT0]) itt0_r <= wb_res & 32'hFFFF_E364;
			if (wb_crs[CR_ITT1]) itt1_r <= wb_res & 32'hFFFF_E364;
			if (wb_crs[CR_DTT0]) dtt0_r <= wb_res & 32'hFFFF_E364;
			if (wb_crs[CR_DTT1]) dtt1_r <= wb_res & 32'hFFFF_E364;
			if (wb_crs[CR_BUSCR]) begin
				bus_l      <= wb_res[31];
				bus_le     <= wb_res[29];
				bus_lkdone <= 1'b0;
			end
			if (wb_crs[CR_VBR])  vbr_r  <= wb_res;
			if (wb_crs[CR_URP])  urp_r  <= wb_res & 32'hFFFF_FE00;
			if (wb_crs[CR_SRP])  srp_r  <= wb_res & 32'hFFFF_FE00;
			if (wb_crs[CR_PCR]) begin
				pcr_edebug <= wb_res[7];
				pcr_dfp    <= wb_res[1];
				pcr_ess    <= wb_res[0];
			end
			if (wb_sr_we) begin
				sr_r  <= {wb_sr_new[15:5], 5'd0};
				ccr_b <= wb_sr_new[4:0];
			end
			if (wb_is_stop && !stop_traced) begin
				stopped   <= 1'b1;
				lpstopped <= (wb_u.cond == 4'd8);
				stop_pc   <= wb_npc;
			end
			// HALT: the halted state until reset (no debug port to restart)
			if (wb_u.op == OP_MISC && wb_u.cond == 4'd10) halted <= 1'b1;
			if (wb_redir) begin
				flush    <= 1'b1;
				if (!wb_eredir) begin
					fe_flush <= 1'b1;
					redir_v  <= 1'b1;
					redir_pc <= wb_redir_pc;
					ras_rv   <= wb_u.ras.v;
					{ras_rtp, ras_rn} <= ras_after(wb_u.ras);
				end
				// the front file restarts from the back one (fv cleared below)
				ccr_f <= wb_sr_we ? wb_sr_new[4:0] : wb_ccr;
			end
			if (wb_bound) rt_armed <= 1'b0;
		end
		if (wb_refetch) begin
			flush    <= 1'b1;
			fe_flush <= 1'b1;
			redir_v  <= 1'b1;
			redir_pc <= wb_u.pc;
			ras_rv   <= wb_u.ras.v;
			{ras_rtp, ras_rn} <= {wb_u.ras.tp, wb_u.ras.n};
			ccr_f    <= ccr_b;
			rt_armed <= 1'b1;
			rt_pc    <= wb_u.pc;
		end

		//------------------------------------------------------------------
		// exception entry: a faulting/trapping uop (its effects discarded,
		// trap flags kept), a trace after an instruction, or an interrupt
		// at an instruction boundary.  The front file restarts from the
		// back file plus this uop's committed writes; T8..T13 carry the
		// routine's operands.
		//------------------------------------------------------------------
		if (x_go) begin
			logic [15:0] vw;
			rt_armed <= 1'b0;
			smc_pend <= 1'b0;
			// BUSCR: L and LE go to the shadow bits SL and SLE, LOCK and
			// LOCKE are negated (MC68060UM 7.4) -- L and LE as a MOVEC to
			// BUSCR committing now (a trace after it) leaves them
			if (wb_commit && wb_u.op == OP_SPW && wb_u.imm_b[7:0] == 8'h08) begin
				bus_sl  <= wb_res[31];
				bus_sle <= wb_res[29];
			end
			else begin
				bus_sl  <= bus_l;
				bus_sle <= bus_le;
			end
			bus_l    <= 1'b0;
			bus_le   <= 1'b0;
			vw = {x_fmt, 2'b00, x_vec, 2'b00};
			flush    <= 1'b1;
			fe_flush <= 1'b1;
			redir_v  <= 1'b0;
			ras_rv   <= 1'b0;
			xw_ph    <= 2'd1;          // T8-T13 first, then exc_go
			exc_kind <= x_kind;
			exc_ssp  <= x_ssp;
			xi_sr    <= x_osr;
			xi_vecw  <= vw;
			xi_pc    <= x_pc;
			xi_addr  <= x_addr;
			xi_fslw  <= x_fslw;
			sr_r     <= {x_nsr[15:5], 5'd0};
			xm_v     <= (x_kind == EK_IRQ);
			xm_m     <= x_osr[12];
			ccr_b    <= x_osr[4:0];
			ccr_f    <= x_osr[4:0];
			stopped  <= 1'b0;
			lpstopped <= 1'b0;
			if (take_irq && !(adv_wb && wb_is_exc)) begin
				irq_lvl_r <= irq_lvl;
				ipend_r   <= 1'b0;
				if (irq_lvl == 3'd7) nmi_edge <= 1'b0;
			end
			// the routine's operands, written into the back file by the
			// xw sequence: T8 vector address, T9 PC, T10 SR, T11 address
			// (format $2/$3: the EA or instruction; $4: FA or EA), T12 the
			// format/vector word, T13 the FSLW (or the faulted instruction's
			// PC: format $4 of the FPU)
			xi_vaddr <= vbr_r + {22'd0, x_vec, 2'b00};
		end

		// exception entry, continued: three cycles of back-file writes
		if (xw_ph != 2'd0) begin
			xw_ph <= xw_ph + 2'd1;
			if (xw_ph == 2'd3) begin
				xw_ph  <= 2'd0;
				exc_go <= 1'b1;
			end
		end

		// front-valid bits: a front write makes the register's front copy
		// current; a redirect, a refetch or an exception drops them all
		fv <= fv_n;

		// a store that took a bus error at WB becomes an access fault of
		// its instruction (WB holds one cycle, then takes it)
		if (wb_v && wb_st && dm_st_rdy && dm_st_fault && wb_exc == 8'd0) begin
			wb_stv      <= 1'b0;
			wb_ok       <= 1'b0;
			wb_exc      <= 8'd2;
			wb_xf       <= xflags(8'd2);
			wb_exc_addr <= dm_st_faddr;
			wb_exc_fslw <= dm_st_fslw;
		end
		// maintenance done: WB completes next cycle.  PLPA: the physical
		// address is its result, or its translation faulted (an access
		// error of the PLPA, format $4)
		if (wb_v && wb_mt && mt_done) begin
			mt_seen <= 1'b1;
			if (wb_u.cond == 4'd5) begin
				wb_res <= mt_pa;
				if (mt_fault && wb_exc == 8'd0) begin
					wb_stv      <= 1'b0;
					wb_ok       <= 1'b0;
					wb_exc      <= 8'd2;
					wb_xf       <= xflags(8'd2);
					wb_exc_addr <= wb_res;
					wb_exc_fslw <= mt_fslw;
				end
			end
		end
		// the locked sequence BUSCR opened ends with its LOCKE write
		if (dm_lkend) bus_lkdone <= 1'b1;
		if (adv_wb) mt_seen <= 1'b0;

		// RESET instruction: RSTO for 512 bus clocks, WB waits
		// (RSTO only once every bus cycle is done: the store buffer empty)
		if (wb_v && wb_reset && !rst_issued && !wb_is_exc && sb_empty) rst_issued <= 1'b1;
		if (dbl_fault) halted <= 1'b1;
		if (adv_wb) rst_issued <= 1'b0;

		//------------------------------------------------------------------
		// flush: kill every stage (the redirecting/excepting uop has left)
		//------------------------------------------------------------------
		if (flush) begin
			ag_v <= 1'b0; dc1_v <= 1'b0; dt_v <= 1'b0; dc2_v <= 1'b0; ex_v <= 1'b0; wb_v <= 1'b0;
			eredir_pend <= 1'b0;
		end
		else begin
			// D2 (or the skid) -> AG
			if (!stall_ag) ag_v <= ain_v && !stopped && !halted;
			if (ag_take && ain_v) begin
				ag_u      <= ain_u;
				ag_u.disp <= ain_u.dmv ? ain_u.disp + {{24{ain_u.dma[7]}}, ain_u.dma} : ain_u.disp;
				ag_bfh    <= (ain_u.mem != M_NONE) && (ain_u.op == OP_MOV) && (ain_u.cond == 4'hE);
				ag_uoh    <= 32'd1 << ain_u.upd_reg;
				ag_u2oh   <= 32'd1 << ain_u.upd2_reg;
				ag_bft    <= (ain_u.mem != M_NONE) && (ain_u.op == OP_MOV) && (ain_u.cond == 4'hD);
				ag_dm2    <= (ain_u.dmv ? ain_u.disp + {{24{ain_u.dma[7]}}, ain_u.dma} : ain_u.disp) - 32'd2;
			end
			// the skid: emptied when AG takes it; filled by a popped uop AG
			// did not take
			if (sk_v) begin
				if (ag_take) begin sk_v <= 1'b0; sk_vr <= 1'b0; sk_va <= 1'b0; end
			end
			else if (SKID && in_v && in_rdy && !ag_take) begin
				sk_v  <= 1'b1;
				sk_vr <= 1'b1; sk_va <= 1'b1;
				sk_u  <= in_u;
				sk_base_a <= in_u.base;
				sk_idx_a  <= in_u.idx;
			end
		end
		if (flush || kill_now) begin sk_v <= 1'b0; sk_vr <= 1'b0; sk_va <= 1'b0; end
		if (kill_now) begin
			ag_v <= 1'b0; dc1_v <= 1'b0; dt_v <= 1'b0; dc2_v <= 1'b0; ex_v <= 1'b0; wb_v <= 1'b0;
		end
	end
end

assign dm_st_v    = wb_v && wb_stv;
assign be_idle    = !(sk_v || ag_v || dc1_v || dt_v || dc2_v || ex_v || wb_v);
// synthesis translate_off
always_ff @(posedge clk)
	if (nreset && wb_v && (wb_stv != (wb_st && wb_exc == 8'd0) || wb_ok != (wb_exc == 8'd0))) begin
		$display("FAIL: wb_stv %b, the store %b with exception %0d (pc %h)", wb_stv, wb_st, wb_exc, wb_u.pc);
		$finish;
	end
// synthesis translate_on
assign iack_lvl   = irq_lvl_r;
assign rsto_req   = wb_v && wb_reset && !rst_issued && !wb_is_exc && sb_empty;
assign dm_st_data = wb_st_data;
assign dbg_pc     = wb_u.pc;
assign ucond      = 1'b0;

// processor status PST4-PST0 (MC68060UM table 2-7), a processor clock at
// a time: PST4 supervisor, PST3 branch, PST2 taken, PST1-0 instructions
// completed.  An instruction's completion shows for the clock it leaves
// WB; halted ($1C), low-power stopped ($16) and stopped ($17) persist;
// exception processing ($18) runs from the entry to the routine's handler
// fetch, which is the branch to the vector ($1B).  PULSE: $04/$14.
always_ff @(posedge clk) begin
	if (!nreset) xstk <= 1'b1;               // reset processing
	else if (x_go) xstk <= 1'b1;
	else if (adv_wb && wb_u.last) xstk <= 1'b0;
end
// the handler's first instruction: from the routine's end to that
// instruction's completion (a refetch of it is no completion)
always_ff @(posedge clk) begin
	if (!nreset || x_go || halted) x_first <= 1'b0;
	else if (adv_wb && wb_u.last && xstk) x_first <= 1'b1;
	else if (wb_bound || (adv_wb && wb_is_exc && !wb_refetch)) x_first <= 1'b0;
end
always_ff @(posedge clk)
	if (!nreset) xr_dbl <= 1'b1;             // (reset's routine)
	else if (x_go) xr_dbl <= (x_vec == 8'd2 || x_vec == 8'd3);
// IPEND: a request beats the mask, and an accepted interrupt keeps it
// asserted until the terminal response of its acknowledge cycle (7.8.1),
// retries included.  (iack_hold is status only: whether to take another
// interrupt is decided by irq_pend alone.)
logic iack_hold;
always_ff @(posedge clk)
	if (!nreset || halted) iack_hold <= 1'b0;
	else if (x_go) iack_hold <= take_irq && !(adv_wb && wb_is_exc);
	else if (dm_iack_end) iack_hold <= 1'b0;
assign irq_pending = irq_pend || iack_hold;
// (and from the moment STOP, LPSTOP or RESET is at WB: no fetch cycle may
// follow the LPSTOP broadcast or run with RSTO asserted; the broadcast is
// issued from AG, so its access holds the fetch from there until LPSTOP
// reaches WB and the stop takes over)
// The interrupt that ends LPSTOP brings the bus back with its acknowledge
// cycle: the fetch stays held until that exception's sequence is done
logic lps_out, lps_wake;
always_ff @(posedge clk)
	if (!nreset || flush || stopped) lps_out <= 1'b0;
	else if (ag_v && ag_u.mfc == MFC_LPS) lps_out <= 1'b1;
always_ff @(posedge clk)
	if (!nreset) lps_wake <= 1'b0;
	else if (x_go) lps_wake <= stopped && lpstopped;
	else if (!xstk) lps_wake <= 1'b0;
// (registered: the BIU's arbitration starts from a flip-flop.  A clock
// later is still well before what it holds the fetch for: the LPSTOP
// broadcast leaves from AG through the DMU, RSTO waits for the transfer in
// progress)
always_ff @(posedge clk)
	fetch_stop <= stopped || halted || lps_out || lps_wake || (ag_v && ag_u.mfc == MFC_LPS) ||
	              (wb_v && (wb_is_stop || wb_reset));
always_comb begin
	logic s, br, tk;
	s  = sr_r[13];
	br = (wb_u.op == OP_BCC || wb_u.op == OP_DBCC) || wb_cof || (wb_u.br != BR_NONE);
	tk = wb_cof || (wb_redir && wb_u.br != BR_NONE);
	if (halted)                    pst5 = 5'h1C;
	else if (stopped && lpstopped) pst5 = 5'h16;
	else if (stopped)              pst5 = 5'h17;
	else if (adv_wb && wb_u.last && xstk)
		pst5 = 5'h1B;
	else if (xstk)                 pst5 = 5'h18;
	else if (adv_wb && wb_u.last && !wb_is_exc) begin
		if (wb_u.op == OP_RTE)                              pst5 = 5'h15;
		else if (wb_u.op == OP_MISC && wb_u.cond == 4'd11)  pst5 = {s, 4'h4};
		else                                                pst5 = {s, br, br && tk, 2'b01};
	end
	else                           pst5 = {s, 4'h0};
end

// AG's base and index registers and its writer flags: loaded with the
// incoming uop (with the front-file writes of that cycle bypassed in), then
// following EX's broadcast and the captured loads while AG holds.
// The flags of the stages as they will be next cycle: a stage takes its
// predecessor's uop when that advances, keeps its own when it stalls, is
// empty otherwise.  m* are the writers there now of the register asked.
function automatic logic [4:0] wf_next(input logic [3:0] m, input logic m_ag, input logic cap);
	// m: {EX, DC2, DT, DC1} now; m_ag: the uop leaving AG (entering DC1)
	wf_next[0] = adv_ag ? m_ag : (stall_dc1 && m[0]);
	wf_next[1] = adv_dc1 ? m[0] : (stall_dt  && m[1]);
	wf_next[2] = adv_dt  ? m[1] : (stall_dc2 && m[2]);
	// (a load captured as it leaves DC2 is not waited for in EX)
	wf_next[3] = adv_dc2 ? (m[2] && !cap) : (stall_ex && m[3]);
	// (its result reaches AG the clock after it leaves EX)
	wf_next[4] = adv_ex && m[3];
endfunction
function automatic logic [3:0] wm(input logic [4:0] r);
	wm = {wmatch(ex_v, ex_u, r), wmatch(dc2_v, dc2_u, r), wmatch(dt_v, dt_u, r),
	      wmatch(dc1_v, dc1_u, r)};
endfunction
// (a load captured last clock is younger than the writer whose result
// exw_q brings now: that one is not taken, cbl/cxl)
logic cbl, cxl;
always_ff @(posedge clk) begin
	exw_q    <= exw_val;
	exw_qreg <= exw_reg;
	exw_qv   <= exw_v && !kill_now && !flush && nreset;
end
always_ff @(posedge clk) begin
	logic ag_new, cb_n, cx_n, cb_o, cx_o, agv_n, qb_o, qx_o;
	logic [4:0] nb, nx, nu, nu2;
	logic [31:0] b, x, bh, xh;
	logic        b1, b0, x1, x0;
	// (ain_v from the twins sk_va and in_eb: the select's own flip-flops)
	ag_new = !flush && !stall_ag && (sk_va || !in_eb) && !stopped;
	// a plain load leaving DC2 whose data a base or index takes
	cb_n = adv_dc2 && dc2_capd && dc2_u.d_reg == ain_u.base;
	cx_n = adv_dc2 && dc2_capd && dc2_u.d_reg == ain_u.idx;
	cb_o = adv_dc2 && dc2_capd && dc2_u.d_reg == ag_u.base;
	cx_o = adv_dc2 && dc2_capd && dc2_u.d_reg == ag_u.idx;
	// the value entering: the register file (front or back copy), AG's own
	// updates of this cycle, the load leaving DC2 (younger than any writer
	// still in EX: it wins), youngest first.  One-hot selects from register
	// compares over all but the load, which picks last (adv_dc2 comes
	// late); the index sign-extended on each source in parallel.  (EX's
	// result of this cycle is written into the file at this edge: the flags
	// wait for it, and exw_q brings it next clock.)
	b1 = fwe_ag[1] && fwa[1] == ain_u.base;
	b0 = fwe_ag[0] && fwa[0] == ain_u.base;
	x1 = fwe_ag[1] && fwa[1] == ain_u.idx;
	x0 = fwe_ag[0] && fwa[0] == ain_u.idx;
	b = ({32{ain_u.base_v && b1}}                  & fwd[1])  |
	    ({32{ain_u.base_v && !b1 && b0}}           & ag_updv)  |
	    ({32{ain_u.base_v && !b1 && !b0}} & ain_fb);
	x = ({32{x1}}                  & ixf(fwd[1], ain_u))  |
	    ({32{!x1 && x0}}           & ixf(ag_updv, ain_u))  |
	    ({32{!x1 && !x0}} & ixf(ain_fx, ain_u));
	// the value staying: EX's result of last clock (exw_q), else its own
	qb_o = exw_qv && exw_qreg == ag_u.base && !cbl;
	qx_o = exw_qv && exw_qreg == ag_u.idx  && !cxl;
	bh = (ag_v && ag_u.base_v && qb_o) ? exw_q : agb_q;
	xh = (ag_v && ag_u.idx_v && qx_o) ? ixf(exw_q, ag_u) : agx_q;
	// the load's capture, then the advance, select last
	agb_q <= ag_new ? ((cb_n && ain_u.base_v) ? dc2_capv : b)
	                : ((cb_o && ag_v && ag_u.base_v) ? dc2_capv : bh);
	agx_q <= ag_new ? (cx_n ? ixf(dc2_capv, ain_u) : x)
	                : ((cx_o && ag_v && ag_u.idx_v) ? ixf(dc2_capv, ag_u) : xh);
	if (ag_new) begin
		cbl   <= cb_n;
		cxl   <= cx_n;
		nb  = ain_u.base_v ? wf_next(wm(ain_u.base), wmatch(ag_v, ag_u, ain_u.base), cb_n) : 4'd0;
		nx  = ain_u.idx_v  ? wf_next(wm(ain_u.idx),  wmatch(ag_v, ag_u, ain_u.idx),  cx_n) : 4'd0;
		nu  = ain_u.upd_v  ? wf_next(wm(ain_u.upd_reg),  wmatch(ag_v, ag_u, ain_u.upd_reg),  1'b0) : 4'd0;
		nu2 = ain_u.upd2_v ? wf_next(wm(ain_u.upd2_reg), wmatch(ag_v, ag_u, ain_u.upd2_reg), 1'b0) : 4'd0;
	end
	else begin
		// AG holds (or is empty): the flags follow their stages, nothing
		// enters DC1 from AG
		cbl <= cb_o;
		cxl <= cx_o;
		nb  = wf_next(wf_b[3:0],  1'b0, cb_o);
		nx  = wf_next(wf_x[3:0],  1'b0, cx_o);
		nu  = wf_next(wf_u[3:0],  1'b0, 1'b0);
		nu2 = wf_next(wf_u2[3:0], 1'b0, 1'b0);
	end
	if (flush || kill_now || !nreset) begin
		nb = 5'd0; nx = 5'd0; nu = 5'd0; nu2 = 5'd0;
	end
	wf_b <= nb; wf_x <= nx; wf_u <= nu; wf_u2 <= nu2;
	// the interlock as it will be: AG's next uop (ag_v as it is loaded) and
	// its flags
	agv_n = !(flush || kill_now || !nreset) && (stall_ag ? ag_v : (ain_v && !stopped && !halted));
	ilk_q <= agv_n && ((nb | nx | nu | nu2) != 5'd0);
end
// synthesis translate_off
// the flags against the compares they replace
always_ff @(posedge clk)
	if (nreset && ag_v && !flush) begin
		logic [3:0] mb, mx, mu, mu2;
		mb = wm(ag_u.base); mx = wm(ag_u.idx); mu = wm(ag_u.upd_reg); mu2 = wm(ag_u.upd2_reg);
		if ((ag_u.upd_v && wf_u[3:0] != mu) || (ag_u.upd2_v && wf_u2[3:0] != mu2) ||
		    (ag_u.base_v && wf_b[2:0] != mb[2:0]) || (ag_u.idx_v && wf_x[2:0] != mx[2:0]) ||
		    (ag_u.base_v && wf_b[3] && !mb[3]) || (ag_u.idx_v && wf_x[3] && !mx[3]) ||
		    ((wf_b[4] || wf_x[4] || wf_u[4] || wf_u2[4]) && !wb_v)) begin
			$display("FAIL: AG writer flags b %b/%b x %b/%b u %b/%b u2 %b/%b (pc %h)",
			         wf_b, mb, wf_x, mx, wf_u, mu, wf_u2, mu2, ag_u.pc);
			$finish;
		end
	end
// synthesis translate_on


//--------------------------------------------------------------------------
// The registered stalls (see the chain c_* above).  AG's interlock next
// (ilk_n) is the capture block's arithmetic formed again from the same
// registers; the valids' next values are their updates' shapes; the other
// holds' next values come hoisted from their prediction blocks (wbh_n,
// exh_n, safe_n) and from the DMU (dm_*_n).
//--------------------------------------------------------------------------
always_comb begin
	logic agn, cbn, cxn, cbo, cxo, agvn;
	logic [4:0] nb, nx, nu, nu2;
	agn = !flush && !stall_ag && (sk_va || !in_eb) && !stopped;
	cbn = adv_dc2 && dc2_capd && dc2_u.d_reg == ain_u.base;
	cxn = adv_dc2 && dc2_capd && dc2_u.d_reg == ain_u.idx;
	cbo = adv_dc2 && dc2_capd && dc2_u.d_reg == ag_u.base;
	cxo = adv_dc2 && dc2_capd && dc2_u.d_reg == ag_u.idx;
	if (agn) begin
		nb  = ain_u.base_v ? wf_next(wm(ain_u.base), wmatch(ag_v, ag_u, ain_u.base), cbn) : 4'd0;
		nx  = ain_u.idx_v  ? wf_next(wm(ain_u.idx),  wmatch(ag_v, ag_u, ain_u.idx),  cxn) : 4'd0;
		nu  = ain_u.upd_v  ? wf_next(wm(ain_u.upd_reg),  wmatch(ag_v, ag_u, ain_u.upd_reg),  1'b0) : 4'd0;
		nu2 = ain_u.upd2_v ? wf_next(wm(ain_u.upd2_reg), wmatch(ag_v, ag_u, ain_u.upd2_reg), 1'b0) : 4'd0;
	end
	else begin
		nb  = wf_next(wf_b[3:0],  1'b0, cbo);
		nx  = wf_next(wf_x[3:0],  1'b0, cxo);
		nu  = wf_next(wf_u[3:0],  1'b0, 1'b0);
		nu2 = wf_next(wf_u2[3:0], 1'b0, 1'b0);
	end
	if (flush || kill_now || !nreset) begin
		nb = 5'd0; nx = 5'd0; nu = 5'd0; nu2 = 5'd0;
	end
	agvn  = !(flush || kill_now || !nreset) && (stall_ag ? ag_v : (ain_v && !stopped && !halted));
	ilk_n = agvn && ((nb | nx | nu | nu2) != 5'd0);
end
always_comb begin
	logic agn, cbn, cxn, cbo, cxo, agvn;
	logic [4:0] nb, nx, nu, nu2;
	agn = !flush && !stall_ag && (sk_va || !in_eb) && !stopped;
	cbn = adv_dc2 && dc2_capd && dc2_u.d_reg == ain_u.base;
	cxn = adv_dc2 && dc2_capd && dc2_u.d_reg == ain_u.idx;
	cbo = adv_dc2 && dc2_capd && dc2_u.d_reg == ag_u.base;
	cxo = adv_dc2 && dc2_capd && dc2_u.d_reg == ag_u.idx;
	if (agn) begin
		nb  = ain_u.base_v ? wf_next(wm(ain_u.base), wmatch(ag_v, ag_u, ain_u.base), cbn) : 4'd0;
		nx  = ain_u.idx_v  ? wf_next(wm(ain_u.idx),  wmatch(ag_v, ag_u, ain_u.idx),  cxn) : 4'd0;
		nu  = ain_u.upd_v  ? wf_next(wm(ain_u.upd_reg),  wmatch(ag_v, ag_u, ain_u.upd_reg),  1'b0) : 4'd0;
		nu2 = ain_u.upd2_v ? wf_next(wm(ain_u.upd2_reg), wmatch(ag_v, ag_u, ain_u.upd2_reg), 1'b0) : 4'd0;
	end
	else begin
		nb  = wf_next(wf_b[3:0],  1'b0, cbo);
		nx  = wf_next(wf_x[3:0],  1'b0, cxo);
		nu  = wf_next(wf_u[3:0],  1'b0, 1'b0);
		nu2 = wf_next(wf_u2[3:0], 1'b0, 1'b0);
	end
	if (flush || !nreset) begin
		nb = 5'd0; nx = 5'd0; nu = 5'd0; nu2 = 5'd0;
	end
	agvn   = !(flush || !nreset) && (stall_ag ? ag_v : (ain_v && !stopped && !halted));
	ilk_nk = agvn && ((nb | nx | nu | nu2) != 5'd0);
end
// (the next values without the kill -- kill_now is late and zeroes every
// valid, so it is the last AND -- and with DT's late decision rdy_dt
// (the DMU's dc2r_n1/n0) picked last; both picks one LUT before the flop)
wire kf       = flush || !nreset;
wire wb_v_n   = !kf && (stall_wb  ? wb_v  : adv_ex);
wire ex_v_n   = !kf && (stall_ex  ? ex_v  : adv_dc2);
wire dc2_v_n  = !kf && (stall_dc2 ? dc2_v : adv_dt);
wire dt_v_n   = !kf && (stall_dt  ? dt_v  : adv_dc1);
wire dc1_v_n  = !kf && (stall_dc1 ? dc1_v : adv_ag);
wire ag_v_n   = !kf && (stall_ag  ? ag_v  : (ain_v && !stopped && !halted));
// WB's store valid as it will be: loaded as the uop enters, cleared by a
// store's or a PLPA's fault (the clears last, as in the block)
wire wb_stv_n = (adv_ex ? ((ex_u.mem == M_ST || ex_u.mem == M_RMW) && !ex_stkill &&
                           ex_xok && !ex_fault && !ex_xnz && !ex_odd)
                        : wb_stv) &&
                !(wb_v && wb_st && dm_st_rdy && dm_st_fault && wb_exc == 8'd0) &&
                !(wb_v && wb_mt && mt_done && wb_u.cond == 4'd5 && mt_fault && wb_exc == 8'd0);
wire dc2_mx_n = adv_dt ? ((dt_u.mem != M_NONE) && (dt_u.exc == 8'd0)) : dc2_mx;
wire ex_fpw_n = adv_dc2 ? ((dc2_u.op == OP_FPU) && !((dc2_u.mem != M_NONE) && dm_fault) && (dc2_u.exc == 8'd0))
                        : ex_fpw;
wire hs_n     = (ex_v && !adv_ex) ? fp_hs0 : fp_hs1;
wire ns_n     = (ex_v && !adv_ex) ? fp_ns0 : fp_ns1;
wire fph_n    = ex_v_n && (hs_n || (!safe_nk && !ns_n));
wire exho_n   = exh_nk || (ex_v_n && ex_fpw_n && fph_n);
wire dc2h_n1  = dc2_v_n && dc2_mx_n && !dm_dc2r_n1;
wire dc2h_n0  = dc2_v_n && dc2_mx_n && !dm_dc2r_n0;
wire swb_n    = wb_v_n  && (wbh_n || (wb_stv_n && !dm_st_ok_n));
wire sex_n    = ex_v_n  && (exho_n || swb_n);
wire sdc2_n1  = dc2_v_n && (dc2h_n1 || sex_n);
wire sdc2_n0  = dc2_v_n && (dc2h_n0 || sex_n);
wire sdt_n1   = dt_v_n  && (sdc2_n1 || dm_hold_dt_n);
wire sdt_n0   = dt_v_n  && (sdc2_n0 || dm_hold_dt_n);
wire sdc1_n1  = dc1_v_n && (sdt_n1  || dm_hold1_nk);
wire sdc1_n0  = dc1_v_n && (sdt_n0  || dm_hold1_nk);
wire sag_n1   = ag_v_n  && (ilk_nk  || sdc1_n1);
wire sag_n0   = ag_v_n  && (ilk_nk  || sdc1_n0);
always_ff @(posedge clk) begin
	stall_wb  <= !kill_now && swb_n;
	stall_ex  <= !kill_now && sex_n;
	stall_dc2 <= !kill_now && (dm_rdy_dt ? sdc2_n1 : sdc2_n0);
	stall_dt  <= !kill_now && (dm_rdy_dt ? sdt_n1  : sdt_n0);
	stall_dc1 <= !kill_now && (dm_rdy_dt ? sdc1_n1 : sdc1_n0);
	stall_ag  <= !kill_now && (dm_rdy_dt ? sag_n1  : sag_n0);
end
// synthesis translate_off
// the registered stalls against the chain: equal, but for a DMU "not
// ready" a clock longer than the block's (dc2r_n conservative), which
// holds DC2 and the stages before it
logic ilk_nq, dc2r_nq;
always_ff @(posedge clk) begin
	logic cons;
	ilk_nq  <= ilk_n;
	dc2r_nq <= dm_dc2r_n;
	cons = dc2_v && dc2_mx && dm_dc2_rdy_b && !dc2r_nq;
	if (nreset && ilk_nq !== ilk_q) begin
		$display("FAIL: AG's interlock next value %b, the register %b", ilk_nq, ilk_q);
		$finish;
	end
	if (nreset && (stall_wb !== c_wb || stall_ex !== c_ex ||
	               (c_dc2 && !stall_dc2) || (c_dt && !stall_dt) || (c_dc1 && !stall_dc1) || (c_ag && !stall_ag) ||
	               (!cons && (stall_dc2 !== c_dc2 || stall_dt !== c_dt || stall_dc1 !== c_dc1 || stall_ag !== c_ag)))) begin
		$display("FAIL: the registered stalls %b%b%b%b%b%b (wb ex dc2 dt dc1 ag), the chain %b%b%b%b%b%b, conservative %b",
		         stall_wb, stall_ex, stall_dc2, stall_dt, stall_dc1, stall_ag, c_wb, c_ex, c_dc2, c_dt, c_dc1, c_ag, cons);
		$finish;
	end
end
// synthesis translate_on

endmodule
