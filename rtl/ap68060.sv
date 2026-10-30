//--------------------------------------------------------------------------//
// AP68060 - pipelined MC68040                                            //
//                                                                          //
// ap68060.sv - top level: the 68060 bus (MC68060UM sections 2, 7)          //
//                                                                          //
// clk is the processor clock (CLK); bclk_en is CLKEN, marking the CLK      //
// edges that are bus clock (BCLK) edges.  Bidirectional pins are split     //
// into _i/_o/_oe: a_i, ts_n_i and tt_i are the bus as another master      //
// drives it (snooped with SNOOP), d_i the data bus in either direction;   //
// a_oe covers the address, the transfer attributes, TS and BS3-BS0;      //
// lock_oe LOCK and LOCKE (driven negated a bus clock past a_oe when the   //
// bus is released with either asserted, 2.3.7); sas_oe SAS (driven        //
// negated a bus clock past a_oe, 2.4.3).                                  //
// At reset (the last bus clock with RSTI asserted) IPL2-IPL0 and D15-D0   //
// select the special modes (7.14): IPL0 the ignore states (D15-D0 their   //
// counts), IPL1 the native acknowledge termination (TRA), IPL2 the extra  //
// data write hold.                                                        //
//--------------------------------------------------------------------------//

module ap68060
	import ap68060_pkg::*;
#(
	// area options (doc/STATUS.md has what each costs in ALMs and cycles)
	parameter bit         FPU_PIPE     = 0,      // the FPU's arithmetic pipeline (0: the state machine alone)
	parameter bit         LOOP_PRED    = 0,      // D1's loop exit predictor
	parameter bit         EARLY_RES    = 0,      // (unused here: AG's early results are not built)
	parameter bit         DC1_ATC      = 1,      // (unused here: the DMU's DT stage compares the tags)
	parameter bit         SKID         = 1,      // the skid register in front of AG (65 MHz timing: +400 ALMs)
	parameter logic [7:0] PCR_REV      = 8'h06   // PCR revision number
)(

	input  logic        clk,
	input  logic        bclk_en,
	input  logic        rsti_n,

	output logic [31:0] a_o,
	output logic        a_oe,
	input  logic [31:0] a_i,
	input  logic        ts_n_i,
	input  logic  [1:0] tt_i,
	input  logic        snoop_n,
	input  logic [31:0] d_i,
	output logic [31:0] d_o,
	output logic        d_oe,
	output logic        rw_n,
	output logic  [1:0] siz,
	output logic  [3:0] bs_n,         // BS0-BS3 (bs_n[0] = BS0, D31-D24)
	output logic  [1:0] tt,
	output logic  [2:0] tm,
	output logic  [1:0] tln,
	output logic  [1:0] upa,
	output logic        ciout_n,
	output logic        lock_n,
	output logic        locke_n,
	output logic        lock_oe,        // LOCK and LOCKE driven
	output logic        sas_oe,         // SAS driven
	output logic        ts_n,
	output logic        tip_n,
	output logic        tip_oe,
	output logic        sas_n,
	input  logic        ta_n,
	input  logic        tea_n,
	input  logic        tra_n,
	input  logic        tci_n,
	input  logic        tbi_n,
	input  logic        cla_n,
	input  logic  [2:0] ipl_n,
	input  logic        avec_n,
	output logic        br_n,
	input  logic        bg_n,
	input  logic        bgr_n,
	input  logic        bb_n_i,
	output logic        bb_n_o,
	output logic        bb_oe,
	input  logic        btt_n_i,
	output logic        btt_n_o,
	output logic        btt_oe,
	output logic        rsto_n,
	input  logic        cdis_n,       // CDIS: disable the caches (contents kept)
	input  logic        mdis_n,       // MDIS: disable page translation
	output logic        ipend_n,      // IPEND
	output logic  [4:0] pst,          // PST4-PST0 (MC68060UM table 2-7), every CLK

	// debug / test
	output logic [31:0] dbg_pc,
	output logic        dbg_retire,
	output logic        dbg_halted
);

// reset synchronizer: two bus clocks (RSTI is synchronized to the clock-
// enabled edges, 2.8.3, 7.13), so the reset -- and the release of the bus
// pins it begins -- always starts on a bus clock edge.  Once RSTI negates
// the processor stays in reset for another 27 CLK cycles (7.13).
logic [1:0] rst_q = 2'b00;
logic [4:0] rst_hold = 5'd27;
// The reset itself is a register, from the values the synchronizer and the
// counter take (the same clocks as their compare), with a copy for the
// back end and one for the DMU: each drives its own registers' resets.
(* preserve *) logic nreset = 1'b0, nrst_be = 1'b0, nrst_dm = 1'b0;
always_ff @(posedge clk) begin
	logic [1:0] q;
	logic [4:0] h;
	q = bclk_en ? {rst_q[0], rsti_n} : rst_q;
	h = !rst_q[1] ? 5'd27 : (rst_hold != 5'd0) ? rst_hold - 5'd1 : 5'd0;
	rst_q    <= q;
	rst_hold <= h;
	nreset   <= q[1] && (h == 5'd0);
	nrst_be  <= q[1] && (h == 5'd0);
	nrst_dm  <= q[1] && (h == 5'd0);
end

// the special modes (7.14): IPL2-IPL0 and D15-D0 registered on the bus
// clock edges while RSTI is asserted; the last one stands
logic        m_ign, m_native, m_hold;
logic [15:0] m_cnt;
always_ff @(posedge clk)
	if (bclk_en && !rsti_n) begin
		m_ign    <= !ipl_n[0];
		m_native <= !ipl_n[1];
		m_hold   <= !ipl_n[2];
		m_cnt    <= d_i[15:0];
	end

// CDIS and MDIS act at the next internal boundary: two bus-clock samples
// (the multiplexed bus mode CDIS selects at reset is not provided)
logic [1:0] cdis_s, mdis_s;
always_ff @(posedge clk)
	if (bclk_en) begin
		cdis_s <= {cdis_s[0], !cdis_n};
		mdis_s <= {mdis_s[0], !mdis_n};
	end
wire cdis = cdis_s[1];
wire mdis = mdis_s[1];

// IPEND and PST change on any processor clock, whatever CLKEN says
// (MC68060UM 7.1); PST is $10 in reset (table 2-8)
logic        irq_pending;
logic  [4:0] pst5;
always_ff @(posedge clk) begin
	if (!nreset) begin
		ipend_n <= 1'b1;
		pst     <= 5'h10;
	end
	else begin
		ipend_n <= !irq_pending;
		pst     <= pst5;
	end
end

// interrupt level synchronizer: IPL2-IPL0 are synchronized to CLK, and a
// level is valid once it has been held for two consecutive CLK periods
// (7.8.1, before figure 7-25).  The three pins settle at
// different times when the level changes, so for a clock the synchronized
// value can be one no device raised (0 to 3 passing through 1); the two
// samples must agree before the core sees a new level.
logic [2:0] ipl_s1 = 3'd0, ipl_s2 = 3'd0, ipl_s3 = 3'd0, ipl_v = 3'd0;
always_ff @(posedge clk) begin
	ipl_s1 <= ~ipl_n;
	ipl_s2 <= ipl_s1;
	ipl_s3 <= ipl_s2;
	if (ipl_s2 == ipl_s3) ipl_v <= ipl_s3;
end

//--------------------------------------------------------------------------
// BIU: client 0 data (DMU), client 1 instruction fetch
//--------------------------------------------------------------------------
localparam int NC = 2;
logic [NC-1:0] b_req, b_gnt, b_done, b_err;
logic          b_own;
logic          sn_ovf, sn_busy, sn_act;
logic          sb_empty, sb_ne_n, sb_err, sb_err_ack;     // the DMU's store buffer
logic  [31:0]  sb_fslw;
busreq_t       b_breq [NC];
logic [127:0]  b_wdata [NC];
logic          b_rvalid;
logic          b_rclient;
logic [31:0]   b_rdata;
logic  [1:0]   b_rbeat;
logic          b_ravec, b_rtci;
logic  [1:0]   b_errbeat;
logic          b_idle;
logic          rsto_req, rsto_busy;
logic          be_fstop, be_idle;
logic          b_drop, b_more;
// the fetch's hold at the BIU, its own flip-flop (placed by the BIU's
// arbitration): a clock after the back end's, still well before what it
// holds the fetch for (the LPSTOP broadcast, RSTO waiting for any transfer
// starting)
logic          bstop_q;
always_ff @(posedge clk) bstop_q <= be_fstop || rsto_busy;

ap68060_biu #(.NC(NC)) biu (
	.clk(clk), .nreset(nreset), .bclk_en(bclk_en),
	.m_ign(m_ign), .m_native(m_native), .m_hold(m_hold), .m_cnt(m_cnt),
	// (client 1, the fetch, is held while stopped, halted or resetting)
	.req(b_req & ~{bstop_q, 1'b0}), .breq(b_breq), .wdata(b_wdata),
	.gnt(b_gnt), .done(b_done), .err(b_err),
	.rvalid(b_rvalid), .rclient(b_rclient), .rdata(b_rdata), .rbeat(b_rbeat),
	.ravec(b_ravec), .rtci(b_rtci), .errbeat(b_errbeat), .idle(b_idle), .own(b_own),
	.unlock(lk_abandon), .sn_busy(sn_busy || sn_act), .halt(dbg_halted), .drop({1'b0, b_drop}), .more({1'b0, b_more}),   // (the DMU engine sees no grant while frozen)
	.rsto_req(rsto_req), .rsto_busy(rsto_busy),
	.a_o(a_o), .a_oe(a_oe), .d_i(d_i), .d_o(d_o), .d_oe(d_oe),
	.rw_n(rw_n), .siz(siz), .bs_n(bs_n), .tt(tt), .tm(tm), .tln(tln), .upa(upa),
	.ciout_n(ciout_n), .lock_n(lock_n), .locke_n(locke_n), .lk_oe(lock_oe),
	.ts_n(ts_n), .tip_n(tip_n), .tip_oe(tip_oe), .sas_n(sas_n), .sas_oe(sas_oe),
	.ta_n(ta_n), .tea_n(tea_n), .tra_n(tra_n),
	.tci_n(tci_n), .tbi_n(tbi_n), .avec_n(avec_n), .cla_n(cla_n),
	.br_n(br_n), .bg_n(bg_n), .bgr_n(bgr_n), .ts_n_i(ts_n_i),
	.bb_n_i(bb_n_i), .bb_n_o(bb_n_o), .bb_oe(bb_oe),
	.btt_n_i(btt_n_i), .btt_n_o(btt_n_o), .btt_oe(btt_oe),
	.rsto_n(rsto_n)
);

//--------------------------------------------------------------------------
// bus snooper
//--------------------------------------------------------------------------
logic        sn_req, sn_look, sn_hit, sn_dirty, sn_inv, sn_wr;
logic [31:0] sn_pa;
logic [127:0] sn_line;
logic [31:0] sn_wword;
logic [15:0] sn_wbe;
logic        sn_ic, sn_ic_all, sn_ic_done, sn_ihit;
logic        sf_req, sf_more, sf_ack, dt_we, dv_we, dv_lv, pq0_v, pq1_v;
logic  [6:0] sf_set, dt_set, dv_set;
logic  [1:0] sf_way, dt_way;
logic [20:0] dt_tag;
logic  [3:0] dv_wm;
logic [27:0] pq0_l, pq1_l;
logic [31:0] sn_ic_pa;

ap68060_snoop snoop (
	.clk(clk), .nreset(nreset), .bclk_en(bclk_en), .own(a_oe),
	.a_i(a_i), .ts_n_i(ts_n_i), .tt_i(tt_i), .snoop_n(snoop_n),
	.dc_req(sn_req), .dc_pa(sn_pa), .dc_look(sn_look), .dc_hit(sn_hit),
	.dc_inv(sn_inv), .dc_ovf(sn_ovf), .busy(sn_busy),
	.sf_req(sf_req), .sf_set(sf_set), .sf_way(sf_way), .sf_more(sf_more), .sf_ack(sf_ack),
	.dt_we(dt_we), .dt_set(dt_set), .dt_way(dt_way), .dt_tag(dt_tag),
	.dv_we(dv_we), .dv_set(dv_set), .dv_wm(dv_wm), .dv_lv(dv_lv),
	.pq0_v(pq0_v), .pq0_l(pq0_l), .pq1_v(pq1_v), .pq1_l(pq1_l),
	.ic_req(sn_ic), .ic_pa(sn_ic_pa), .ic_all(sn_ic_all), .ic_done(sn_ic_done)
);
assign sn_wr    = 1'b0;          // (the 68060 never takes snooped data)
assign sn_wbe   = '0;
assign sn_wword = '0;

//--------------------------------------------------------------------------
// back end and front end
//--------------------------------------------------------------------------
logic        redir_v, flush, fe_flush;
logic [31:0] redir_pc;
logic        exc_go;
logic  [3:0] exc_kind;
logic  [4:0] exc_ssp;
logic        ucond_v, ucond;
logic [15:0] sr;
logic [31:0] vbr, cacr;
logic        bc_clr;
logic  [2:0] sfc, dfc;
logic        kill_now, adv_ag, lk_abandon;
logic  [5:0] dm_adv;                // the DMU's copies of the advances
logic        dm_kill;               // ... and of the kill
logic [31:0] dtt0, dtt1, itt0, itt1, tc;
logic        q_odd;
logic        dm_hold1, dm_hold1_c, dm_dc2_rdy_c, dm_st_ok, mt_v, mt_ng, mt_wr, mt_done;
logic  [2:0] mt_op, mt_fc;
logic  [1:0] mt_scope, mt_caches;
logic [31:0] mt_addr, mt_pa, mt_fslw, urp, srp, dm_st_faddr;
logic        mt_fault, dm_lkend, dm_iack_end;
logic  [2:0] iack_lvl;
logic  [1:0] dm_ack;

logic        dm_req, adv_dc1, adv_dt, adv_dc2, adv_ex, adv_wb, dm_hold_dt;
logic [31:0] dm_va;
logic  [1:0] dm_mem, dm_msz;
logic  [2:0] dm_fc;
logic        dm_lock, dm_locke, dm_super, dm_noalloc, dm_older;
logic        bht_we, tw_busy;
logic  [7:0] bht_wa;
logic        bht_dis;
logic        lp_we, lp_tk, lp_mp;
logic [11:0] lp_wa;
logic        dm_dc2_rdy, dm_fault, dm_st_v, dm_st_rdy, dm_st_fault;
logic [31:0] dm_ldata, dm_faddr, dm_st_data, dm_ldata_al;
logic        dm_ldata_alv;
logic  [7:0] dm_fvec;
logic [31:0] dm_fslw, dm_st_fslw;

logic  [1:0] uq_n;
logic        rq_e, uq_e;      // the record / uop FIFO is empty (registers)
uop_t        uq0;
logic        uo_rdy;

ap68060_backend #(.FPU_PIPE(FPU_PIPE), .EARLY_RES(EARLY_RES), .SKID(SKID),
                  .PCR_REV(PCR_REV)) be (
	.clk(clk), .nreset(nrst_be),
	.in_v(uq_n != 2'd0), .in_u(uq0), .in_rdy(uo_rdy),
	.redir_v(redir_v), .redir_pc(redir_pc), .flush(flush), .fe_flush(fe_flush),
	.ras_rv(ras_rv), .ras_rtp(ras_rtp), .ras_rn(ras_rn),
	.exc_go(exc_go), .exc_kind(exc_kind), .exc_ssp(exc_ssp),
	.ucond_v(ucond_v), .ucond(ucond),
	.sr(sr), .vbr(vbr), .cacr(cacr), .bc_clr(bc_clr), .sfc(sfc), .dfc(dfc),
	.dm_req(dm_req), .dm_va(dm_va), .dm_mem(dm_mem), .dm_msz(dm_msz),
	.dm_fc(dm_fc), .dm_lock(dm_lock), .dm_locke(dm_locke), .dm_super(dm_super), .dm_noalloc(dm_noalloc), .dm_ack(dm_ack), .dm_older(dm_older),
	.bht_we(bht_we), .bht_wa(bht_wa), .bht_dis(bht_dis),
	.lp_we(lp_we), .lp_wa(lp_wa), .lp_tk(lp_tk), .lp_mp(lp_mp), .sn_ihit(sn_ihit),
	.tw_busy(tw_busy), .pst5(pst5),
	.irq_pending(irq_pending),
	.adv_dc1(adv_dc1), .adv_dt(adv_dt), .adv_dc2(adv_dc2), .adv_ex(adv_ex), .adv_wb(adv_wb),
	.dm_dc2_rdy(dm_dc2_rdy), .dm_dc2_rdy_c(dm_dc2_rdy_c), .dm_ldata(dm_ldata), .dm_ldata_al(dm_ldata_al), .dm_ldata_alv(dm_ldata_alv), .dm_fault(dm_fault),
	.dm_fvec(dm_fvec), .dm_faddr(dm_faddr), .dm_fslw(dm_fslw), .if_cause(if_cause),
	.dm_st_v(dm_st_v), .dm_st_data(dm_st_data), .dm_st_rdy(dm_st_rdy), .dm_st_ok(dm_st_ok),
	.dm_st_fault(dm_st_fault), .dm_st_fslw(dm_st_fslw), .dm_lkend(dm_lkend), .dm_iack_end(dm_iack_end),
	.sb_empty(sb_empty), .sb_ne_n(sb_ne_n), .sb_err(sb_err), .sb_fslw(sb_fslw), .sb_err_ack(sb_err_ack),
	.dm_hold1(dm_hold1), .dm_hold1_c(dm_hold1_c), .dm_hold_dt(dm_hold_dt), .mt_v(mt_v), .mt_op(mt_op), .mt_scope(mt_scope),
	.mt_caches(mt_caches), .mt_addr(mt_addr), .mt_fc(mt_fc), .mt_ng(mt_ng),
	.mt_wr(mt_wr), .mt_done(mt_done), .mt_pa(mt_pa), .mt_fault(mt_fault), .mt_fslw(mt_fslw),
	.dm_st_faddr(dm_st_faddr),
	.urp(urp), .srp(srp),
	.ipl(ipl_v), .iack_lvl(iack_lvl), .rsto_req(rsto_req), .rsto_busy(rsto_busy),
	.kill_now(kill_now), .lk_abandon(lk_abandon), .adv_ag(adv_ag), .dm_adv(dm_adv), .dm_kill(dm_kill),
	.dtt0(dtt0), .dtt1(dtt1), .itt0(itt0), .itt1(itt1), .tc(tc),
	.dbg_pc(dbg_pc), .dbg_retire(dbg_retire), .halted(dbg_halted), .be_idle(be_idle),
	.fetch_stop(be_fstop)
);

ap68060_dmu #(.DC1_ATC(DC1_ATC)) dmu (
	.clk(clk), .nreset(nrst_dm),
	.adv_ag(dm_adv[5]), .dm_req(dm_req), .dm_va(dm_va), .dm_mem(dm_mem),
	.dm_msz(dm_msz), .dm_fc(dm_fc), .dm_lock(dm_lock), .dm_locke(dm_locke),
	.dm_super(dm_super), .dm_noalloc(dm_noalloc), .dm_ack(dm_ack), .iack_lvl(iack_lvl),
	.dm_older(dm_older),
	.adv_dc1(dm_adv[4]), .adv_dt(dm_adv[3]), .adv_dc2(dm_adv[2]), .adv_ex(dm_adv[1]), .adv_wb(dm_adv[0]),
	.kill_now(dm_kill), .dm_hold1(dm_hold1), .dm_hold1_c(dm_hold1_c), .hold_dt(dm_hold_dt),
	.dc2_rdy(dm_dc2_rdy), .dc2_rdy_c(dm_dc2_rdy_c), .ldata(dm_ldata), .ldata_al(dm_ldata_al), .ldata_alv(dm_ldata_alv), .fault(dm_fault), .fvec(dm_fvec),
	.faddr(dm_faddr), .fslw(dm_fslw),
	.st_v(dm_st_v), .st_data(dm_st_data), .st_rdy(dm_st_rdy), .st_ok(dm_st_ok),
	.st_fault(dm_st_fault), .st_fslw(dm_st_fslw), .st_faddr(dm_st_faddr), .lkend(dm_lkend), .iack_end(dm_iack_end),
	.mt_v(mt_v), .mt_op(mt_op), .mt_scope(mt_scope), .mt_caches(mt_caches),
	.mt_addr(mt_addr), .mt_fc(mt_fc), .mt_ng(mt_ng), .mt_wr(mt_wr),
	.mt_done(mt_done), .mt_pa(mt_pa), .mt_fault(mt_fault), .mt_fslw(mt_fslw),
	.cacr(cacr), .tc(tc), .urp(urp), .srp(srp),
	.dtt0(dtt0), .dtt1(dtt1), .itt0(itt0), .itt1(itt1),
	.cdis(cdis), .mdis(mdis), .tw_busy(tw_busy),
	// (an instruction-side table search waits for every older instruction:
	// the back end and the decoded/sequenced queues empty)
	.iw_req(iw_req), .iw_ok(be_idle && rq_e && uq_e),
	.iw_va(iw_va), .iw_fc2(iw_fc2), .iw_done(iw_done), .iw_ent(iw_ent),
	.iw_cause(iw_cause),
	.ic_inv(ic_inv), .ic_inv_scope(ic_inv_scope), .ic_inv_pa(ic_inv_pa),
	.ic_inv_done(ic_inv_done),
	.iatc_flush_all(iatc_flush_all), .iatc_flush_page(iatc_flush_page),
	.iatc_flush_ng(iatc_flush_ng), .iatc_flush_la(iatc_flush_la),
	.iatc_flush_fc2(iatc_flush_fc2), .iatc_wr(iatc_wr), .iatc_wla(iatc_wla),
	.iatc_wfc2(iatc_wfc2), .iatc_went(iatc_went),
	.bi_req(b_req[0]), .bi_breq(b_breq[0]), .bi_wdata(b_wdata[0]),
	.bi_gnt(b_gnt[0]), .bi_done(b_done[0]), .bi_err(b_err[0]), .bi_drop(b_drop), .bi_more(b_more),
	.bi_rvalid(b_rvalid && b_rclient == 1'b0), .b_rdata(b_rdata), .b_rbeat(b_rbeat),
	.sb_empty(sb_empty), .sb_ne_n(sb_ne_n), .sb_err(sb_err), .sb_fslw(sb_fslw), .sb_err_ack(sb_err_ack),
	.b_ravec(b_ravec), .b_rtci(b_rtci),
	.sn_req(sn_req), .sn_pa(sn_pa), .sn_look(sn_look), .sn_hit(sn_hit),
	.sn_dirty(sn_dirty), .sn_line(sn_line), .sn_inv(sn_inv), .sn_act(sn_act), .sn_wr(sn_wr),
	.sn_wbe(sn_wbe), .sn_wword(sn_wword),
	.sf_req(sf_req), .sf_set(sf_set), .sf_way(sf_way), .sf_more(sf_more), .sf_ack(sf_ack),
	.dt_we(dt_we), .dt_set(dt_set), .dt_way(dt_way), .dt_tag(dt_tag),
	.dv_we(dv_we), .dv_set(dv_set), .dv_wm(dv_wm), .dv_lv(dv_lv),
	.pq0_v(pq0_v), .pq0_l(pq0_l), .pq1_v(pq1_v), .pq1_l(pq1_l)
);

// instruction side <-> DMU
logic        iw_req, iw_fc2, iw_done;
atce_t       iw_ent;
logic [31:0] iw_cause, if_cause;
logic [31:0] iw_va;
logic        ic_inv, ic_inv_done;
logic  [1:0] ic_inv_scope;
logic [31:0] ic_inv_pa;
logic        iatc_flush_all, iatc_flush_page, iatc_flush_ng, iatc_flush_fc2;
logic [31:0] iatc_flush_la;
logic        iatc_wr, iatc_wfc2;
logic [31:0] iatc_wla;
atce_t       iatc_went;

// front end
logic [15:0] win [8];
logic  [7:0] win_flt, win_fdem, win_fatc, win_bt;
logic [31:0] bt_tgt;
logic        btb_we, btb_wv, btb_wslot;
logic [BTB_AW-1:0] btb_wi;
logic [BTB_TW-1:0] btb_wtag;
logic [30:0] btb_wtgt;
logic  [1:0] btb_wkind;
logic [30:0] d_ras;
logic        ras_rv;
logic  [2:0] ras_rtp;
logic  [3:0] ras_rn;
logic  [2:0] d_ras_tp;
logic  [3:0] d_ras_n;
pd_t         pd0;
logic  [3:0] qcnt;
logic [31:0] qpc;
logic        cs_fire, cr_fire, q_simple;
logic  [2:0] cr_use;
logic        d_redir_v;
logic [31:0] d_redir_pc;
logic  [2:0] rq_n;
dinst_t      rq0, rq1;
logic        rq_pop;

ap68060_fetch fetch (
	.clk(clk), .nreset(nreset),
	.redir_v(redir_v), .redir_pc(redir_pc),
	.d_redir_v(d_redir_v), .d_redir_pc(d_redir_pc),
	// no fetches while stopped or halted, nor while RSTO is asserted
	// (MC68060UM 7.8.2.1: no bus activity after the LPSTOP broadcast; 2.8.4:
	// all bus cycles are suspended before RSTO is asserted)
	.stop(be_fstop || rsto_busy), .smode(sr[13]),
	.cacr(cacr), .tc(tc), .itt0(itt0), .itt1(itt1), .cdis(cdis), .mdis(mdis),
	.win(win), .win_flt(win_flt), .win_fdem(win_fdem), .win_fatc(win_fatc),
	.win_bt(win_bt), .bt_tgt(bt_tgt),
	.pd0(pd0), .q_simple(q_simple), .qcnt(qcnt), .qpc(qpc),
	.cs_fire(cs_fire), .cr_fire(cr_fire), .cr_use(cr_use),
	.q_odd(q_odd),
	.bc_clr(bc_clr),
	.btb_we(btb_we), .btb_wi(btb_wi), .btb_wv(btb_wv), .btb_wtag(btb_wtag),
	.btb_wslot(btb_wslot), .btb_wkind(btb_wkind), .btb_wtgt(btb_wtgt),
	.d_ras(d_ras), .d_ras_tp(d_ras_tp), .d_ras_n(d_ras_n),
	.iw_req(iw_req), .iw_va(iw_va), .iw_fc2(iw_fc2), .iw_done(iw_done),
	.iw_ent(iw_ent), .iw_cause(iw_cause), .if_cause(if_cause),
	.iatc_wr(iatc_wr), .iatc_wla(iatc_wla), .iatc_wfc2(iatc_wfc2), .iatc_went(iatc_went),
	.iatc_flush_all(iatc_flush_all), .iatc_flush_page(iatc_flush_page),
	.iatc_flush_ng(iatc_flush_ng), .iatc_flush_la(iatc_flush_la),
	.iatc_flush_fc2(iatc_flush_fc2),
	.ic_inv(ic_inv), .ic_inv_scope(ic_inv_scope), .ic_inv_pa(ic_inv_pa),
	.ic_inv_done(ic_inv_done),
	.sn_inv(sn_ic), .sn_inv_pa(sn_ic_pa), .sn_inv_all(sn_ic_all), .sn_inv_done(sn_ic_done),
	.sn_ihit(sn_ihit),
	.b_req(b_req[1]), .b_breq(b_breq[1]),
	.b_gnt(b_gnt[1]), .b_done(b_done[1]), .b_err(b_err[1]),
	.b_rvalid(b_rvalid && b_rclient == 1'b1), .b_rdata(b_rdata), .b_rbeat(b_rbeat),
	.b_rtci(b_rtci)
);
assign b_wdata[1] = '0;

ap68060_decode #(.LOOP_PRED(LOOP_PRED)) dec (
	.clk(clk), .nreset(nreset), .flush(fe_flush),
	.ras_rv(ras_rv), .ras_rtp(ras_rtp), .ras_rn(ras_rn),
	.win(win), .win_flt(win_flt), .win_fdem(win_fdem), .win_fatc(win_fatc),
	.win_bt(win_bt), .bt_tgt(bt_tgt),
	.pd0(pd0), .q_simple(q_simple), .qcnt(qcnt), .qpc(qpc), .q_odd(q_odd),
	.smode(sr[13]),
	.cs_fire(cs_fire), .cr_fire(cr_fire), .cr_use(cr_use),
	.d_redir_v(d_redir_v), .d_redir_pc(d_redir_pc),
	.rq_n(rq_n), .rq_e(rq_e), .rq0(rq0), .rq1(rq1), .rq_pop(rq_pop),
	.bht_we(bht_we), .bht_wa(bht_wa), .bht_dis(bht_dis),
	.lp_we(lp_we), .lp_wa(lp_wa), .lp_tk(lp_tk), .lp_mp(lp_mp),
	.btb_we(btb_we), .btb_wi(btb_wi), .btb_wv(btb_wv), .btb_wtag(btb_wtag),
	.btb_wslot(btb_wslot), .btb_wkind(btb_wkind), .btb_wtgt(btb_wtgt),
	.ras_o(d_ras), .ras_tp_o(d_ras_tp), .ras_n_o(d_ras_n)
);

ap68060_useq useq (
	.clk(clk), .nreset(nreset), .flush(fe_flush),
	.rq_n(rq_n), .rq0(rq0), .rq1(rq1), .rq_pop(rq_pop),
	.smode(sr[13]),
	.exc_go(exc_go), .exc_kind(exc_kind), .exc_ssp(exc_ssp),
	.uq_n(uq_n), .uq_e(uq_e), .uq0(uq0), .uq_pop(uo_rdy && uq_n != 2'd0)
);

endmodule
