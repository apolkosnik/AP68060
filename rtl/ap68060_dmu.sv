//--------------------------------------------------------------------------//
// AP68060 - pipelined MC68040                                            //
//                                                                          //
// ap68060_dmu.sv - data memory unit: D-ATC, data cache, table walker       //
//                                                                          //
// Mirrors the back end's DC1, DT, DC2, EX and WB stages for memory uops.   //
//                                                                          //
// Data cache (MC68060UM section 5): 8 Kbytes, four ways of 128 sets of    //
// 16-byte lines, physically tagged (PA31-11; the set is PA10-4 = LA10-4). //
// A valid and a dirty bit per line; replacement takes the first invalid   //
// way, else a 2-bit counter (CACR.FOC, the 1/2-cache mode: ways 0 and 1). //
// Copyback and write-through cachable, precise and imprecise cache-       //
// inhibited modes.  CACR.NAD: misses do not allocate.  A write-through    //
// access that hits a dirty line pushes it first (5.5.3, 5.5.4), the line  //
// staying valid; CPUSH with CACR.DPI leaves the lines it pushes valid.    //
//                                                                          //
// DC1  D-ATC lookup (its second port) and the tag read, registered       //
// DT   tag compare against every ATC way's page, the translation and the  //
//      fast-path decision, from those registers; the data RAM reads DT's  //
//      set, and a load waits in DT until that read is fresh (no older    //
//      store to its line in flight, no engine activity): no merging      //
// DC2  fast path: a load (or unlocked read-modify-write) that hits a      //
//      cachable line, within one line, is answered from the two long      //
//      words of each way taken as it left DT (way and alignment picked    //
//      here).  Everything else stalls DC2 and runs in the engine.         //
// WB   a copyback store that hit writes the data RAM in its WB cycle;     //
//      other stores go through the engine.                                 //
//                                                                          //
// The engine runs one job at a time: the DC2 slow path (table walk, line  //
// fill -- the dirty victim pushed afterwards --, cache-inhibited accesses //
// with a matching line pushed and invalidated first, operands crossing a  //
// line in two parts), WB stores, WB maintenance (CINV, CPUSH, PFLUSH,     //
// PTEST) and the instruction side's table walks.  While it uses the       //
// lookup ports it holds DC1 (dm_hold1), one cycle longer than it uses     //
// them so DC1 re-reads its own address.                                   //
//                                                                          //
// Bus snooping (MC68040UM 4.4, 4.7.2, 7.9) through the sn_* port: the    //
// snooper (ap68060_snoop.sv) asks for the cache; the DMU freezes the      //
// engine, holds DC1 and blocks the WB fast store, looks the physical      //
// line up in the tags, the push buffer and a push waiting for the bus,    //
// and applies the snooper's invalidate or sink write to what it found.    //
// An invalidated line also loses its hit in the DC2/EX/WB records and     //
// the engine's current part, so no store lands in a line that left.      //
// One cycle after the port is released re-reads the lookup address of    //
// DC1 or the engine before they run again.                                //
//                                                                          //
// Table walks (MC68060UM 4.2): root, pointer and page descriptors with    //
// indirect page descriptors, 4K/8K pages, accumulated W, U/M history per  //
// table 4-1 (a locked read-modify-write sets U of a table descriptor and  //
// of a page read; a page write's history is a plain write).  The search   //
// bypasses the data cache, reads and writes alike (4.2.5).  A search that //
// fails -- an invalid descriptor, a bus error, a write to a write-        //
// protected page, a user access to a supervisor-only page -- creates no  //
// ATC entry (4.3), and a write that meets write protection updates no     //
// history; the walk's result goes straight to the access that asked for   //
// it, which also covers the no-allocate modes (TCR NAD/NAI, 4.1.2).       //
//--------------------------------------------------------------------------//

module ap68060_dmu
	import ap68060_pkg::*;
#(
	// (unused: DT always compares the tags against every ATC way's page)
	parameter bit DC1_ATC = 1
)
(
	input  logic        clk,
	input  logic        nreset,

	// back end
	input  logic        adv_ag,
	input  logic        dm_req,
	input  logic [31:0] dm_va,
	input  logic  [1:0] dm_mem,
	input  logic  [1:0] dm_msz,
	input  logic  [2:0] dm_fc,
	input  logic        dm_lock,
	input  logic        dm_locke,
	input  logic        dm_super,
	input  logic        dm_noalloc,
	input  logic  [1:0] dm_ack,         // acknowledge cycle: 1 IACK, 2 BKPT, 3 LPSTOP
	input  logic  [2:0] iack_lvl,
	input  logic        dm_older,       // uops older than DC2 still in EX or WB
	input  logic        adv_dc1,
	input  logic        adv_dt,
	input  logic        adv_dc2,
	input  logic        adv_ex,
	input  logic        adv_wb,
	input  logic        kill_now,
	output logic        dm_hold1,       // DC1 must hold (the engine owns its ports): a
	                                    // register, formed a clock ahead, conservative
	output logic        dm_hold1_c,     // ... as the stage's own state gives it (checks)
	output logic        hold_dt,        // DT must hold (a register): its lookup or
	                                    // data read is stale
	output logic        dc2_rdy,        // DC2's access is done: a register, conservative
	output logic        dc2_rdy_c,      // ... as the stage's own state gives it (checks)
	output logic [31:0] ldata,
	output logic [31:0] ldata_al,       // the aligned long (or word, low half)
	output logic        ldata_alv,      // ... is the load's data
	output logic        fault,
	output logic  [7:0] fvec,
	output logic [31:0] faddr,
	output logic [31:0] fslw,
	input  logic        st_v,
	input  logic [31:0] st_data,
	output logic        st_rdy,
	output logic        st_fault,
	output logic        st_ok,          // WB's store answered without a fault: a
	                                    // register, formed a clock ahead, conservative
	output logic [31:0] st_fslw,
	output logic        lkend,          // a LOCKE write completed
	output logic        iack_end,       // an interrupt acknowledge cycle ended (its
	                                    // terminal response, after any retries)
	output logic [31:0] st_faddr,

	// WB maintenance: CINV/CPUSH/PFLUSH/PLPA
	input  logic        mt_v,
	input  logic  [2:0] mt_op,          // MT_*
	input  logic  [1:0] mt_scope,       // 1 line, 2 page, 3 all
	input  logic  [1:0] mt_caches,      // bit 0 DC, bit 1 IC
	input  logic [31:0] mt_addr,
	input  logic  [2:0] mt_fc,          // DFC (PTEST/PFLUSH)
	input  logic        mt_ng,          // PFLUSHN/PFLUSHAN: spare global entries
	input  logic        mt_wr,          // PLPAW
	output logic        mt_done,
	output logic [31:0] mt_pa,          // PLPA: the physical address
	output logic        mt_fault,       // ... or an access error, with
	output logic [31:0] mt_fslw,        // ... its FSLW

	// control registers
	input  logic [31:0] cacr,
	input  logic [31:0] tc,
	input  logic [31:0] urp,
	input  logic [31:0] srp,
	input  logic [31:0] dtt0,
	input  logic [31:0] dtt1,
	input  logic [31:0] itt0,
	input  logic [31:0] itt1,
	input  logic        cdis,           // CDIS: caches disabled (contents kept)
	input  logic        mdis,           // MDIS: no page translation (TTRs still apply)
	output logic        tw_busy,        // a table search is running (PST)

	// instruction side: table walks for the I-ATC, ATC/cache maintenance
	input  logic        iw_req,
	input  logic        iw_ok,          // nothing older than the fetch is in flight
	input  logic [31:0] iw_va,
	input  logic        iw_fc2,
	output logic        iw_done,
	output atce_t       iw_ent,
	output logic [31:0] iw_cause,       // FSLW cause of a failed search (iw_ent.r clear)
	output logic        ic_inv,         // invalidate instruction cache lines
	output logic  [1:0] ic_inv_scope,   // 1 line, 2 page, 3 all
	output logic [31:0] ic_inv_pa,
	input  logic        ic_inv_done,
	output logic        iatc_flush_all,
	output logic        iatc_flush_page,
	output logic        iatc_flush_ng,
	output logic [31:0] iatc_flush_la,
	output logic        iatc_flush_fc2,
	output logic        iatc_wr,        // table walk result into the I-ATC
	output logic [31:0] iatc_wla,
	output logic        iatc_wfc2,
	output atce_t       iatc_went,

	// BIU client (the store buffer ahead of the engine)
	output logic        bi_req,
	output busreq_t     bi_breq,
	output logic [127:0] bi_wdata,
	input  logic        bi_gnt,
	input  logic        bi_done,
	output logic        bi_drop,        // the push the BIU holds for a retry: dropped
	output logic        bi_more,        // another transfer waits behind the one on the bus
	input  logic        bi_err,
	input  logic        bi_rvalid,
	input  logic [31:0] b_rdata,
	input  logic  [1:0] b_rbeat,
	input  logic        b_ravec,
	input  logic        b_rtci,

	// store buffer (CACR.ESB, MC68060UM 5.9, 5.10, 8.4.6)
	output logic        sb_empty,       // no buffered write left
	output logic        sb_ne_n,        // ... and next cycle, one is left
	output logic        sb_err,         // a buffered write or a push ended in TEA
	output logic [31:0] sb_fslw,        // ... FSLW.SBE and/or FSLW.PBE
	input  logic        sb_err_ack,     // ... the access error for it is taken

	// bus snooper
	input  logic        sn_req,         // level: the snooper wants the cache
	input  logic [31:0] sn_pa,
	output logic        sn_look,        // pulse: sn_hit/sn_dirty/sn_line valid
	output logic        sn_hit,
	output logic        sn_dirty,
	output logic [127:0] sn_line,        // with sn_look: the line found
	input  logic        sn_inv,         // pulse: invalidate what was found
	output logic        sn_act,         // the snoop port holds the cache (the engine waits)
	input  logic        sn_wr,          // pulse: write into what was found
	input  logic [15:0] sn_wbe,         // the bytes, of...
	input  logic [31:0] sn_wword,       // ...this long word on every long word
	// ... its fast invalidations: a line its copy of the tags found, by set
	// and way, invalidated in one frozen clock (sf_ack), one per clock while
	// sf_more says another follows
	input  logic        sf_req,
	input  logic  [6:0] sf_set,
	input  logic  [1:0] sf_way,
	input  logic        sf_more,
	output logic        sf_ack,
	// ... and what it keeps that copy with: the tag writes (as the RAM
	// takes them), the valid writes (a clock after they land), and the
	// dirty lines waiting for a push, which only the full lookup handles
	output logic        dt_we,
	output logic  [6:0] dt_set,
	output logic  [1:0] dt_way,
	output logic [20:0] dt_tag,
	output logic        dv_we,
	output logic  [6:0] dv_set,
	output logic  [3:0] dv_wm,
	output logic        dv_lv,
	output logic        pq0_v,
	output logic [27:0] pq0_l,
	output logic        pq1_v,
	output logic [27:0] pq1_l
);

localparam logic [2:0] MT_CINV = 3'd1, MT_CPUSH = 3'd2, MT_PFLUSH = 3'd3, MT_PLPA = 3'd4;

// MDIS, sampled once here: DC1's registered translation decision (m1_tp)
// is formed from the pin a clock ahead, so the two always agree
logic mdis_q = 1'b0;
always_ff @(posedge clk) mdis_q <= mdis;
wire tc_e   = tc[15] && !mdis_q;
wire tc_e_n = tc[15] && !mdis;      // (tc_e at the next clock)
wire tc_p  = tc[14];
wire dc_en = cacr[31] && !cdis;
wire dc_nad = cacr[30];           // no allocate
wire dc_dpi = cacr[28];           // CPUSH leaves lines valid
wire dc_foc = cacr[27];           // 1/2-cache mode

//--------------------------------------------------------------------------
// stage records
//--------------------------------------------------------------------------
typedef struct packed {
	logic        v;
	logic [31:0] a;        // logical address
	logic  [1:0] mem;
	logic  [1:0] msz;
	logic  [2:0] fc;
	logic        lock;
	logic        locke;
	logic        smode;    // FC2
	logic        noalloc;
	logic  [1:0] ack;      // acknowledge cycle (TT=3): 1 IACK, 2 BKPT, 3 LPSTOP
} req_t;

// translation and lookup result of one access part
typedef struct packed {
	logic        ok;       // translation known
	logic        flt;      // access fault (ATC/TTR)
	logic        walk;     // needs a table search (ATC miss or M update)
	logic [31:0] pa;
	logic  [1:0] cm;
	logic  [1:0] upa;
	logic        hit;      // the line is in the cache
	logic  [1:0] way;
	logic [31:0] why;      // flt: its FSLW cause bits (TTR SP WP PF TWE)
} xres_t;

typedef struct packed {
	req_t        r;
	xres_t       x;        // part 0 (the only part unless split)
	logic        split;    // crosses a line: part 1 at the next line
	xres_t       x1;
	logic        fast;     // DC2 can answer from the RAM output
	logic        sfast;    // a load across a line whose first part hit:
	                       // DC2's copy holds it, one lookup for the rest
} mrec_t;

req_t  m1;
mrec_t m2, m3, m4;

function automatic logic [2:0] nbytes(input logic [1:0] msz);
	case (msz)
		SZ_B:    nbytes = 3'd1;
		SZ_W:    nbytes = 3'd2;
		SZ_L:    nbytes = 3'd4;
		default: nbytes = 3'd0;     // line
	endcase
endfunction

// MOVES to the instruction spaces is a data reference (MC68040UM table 3-2)
function automatic logic [1:0] fc_tt(input logic [2:0] fc);
	fc_tt = (fc == 3'd1 || fc == 3'd2 || fc == 3'd5 || fc == 3'd6) ? TT_NORMAL : TT_ALT;
endfunction
function automatic logic [2:0] fc_tm(input logic [2:0] fc);
	fc_tm = (fc == 3'd2) ? 3'd1 : (fc == 3'd6) ? 3'd5 : fc;
endfunction

// TTR match on A31-24 with mask, E, and the S field
function automatic logic ttr_hit(input logic [31:0] t, input logic [31:0] a,
                                 input logic s);
	ttr_hit = t[15] && (((a[31:24] ^ t[31:24]) & ~t[23:16]) == 8'd0) &&
	          (t[14] || (t[13] == s));
endfunction

// n bytes at offset o of a line, right aligned
function automatic logic [31:0] take(input logic [127:0] line, input logic [3:0] o,
                                     input logic [2:0] n);
	logic [31:0] v;
	v = '0;
	for (int i = 0; i < 4; i++)
		if (3'(i) < n) v = {v[23:0], line[127 - 8*(o + i) -: 8]};
	take = v;
endfunction

// A write of n bytes from the top of d (left aligned) at offset o of a
// line: the operand rotated onto the byte lanes, as the bus would carry
// it, is replicated over the line's four long words, and the byte enables
// pick the bytes (byte o+i is on lane (o+i) mod 4, which holds byte i).
function automatic logic [31:0] lanes32(input logic [31:0] d, input logic [1:0] o);
	case (o)
		2'd0:    lanes32 = d;
		2'd1:    lanes32 = {d[7:0], d[31:8]};
		2'd2:    lanes32 = {d[15:0], d[31:16]};
		default: lanes32 = {d[23:0], d[31:24]};
	endcase
endfunction
function automatic logic [15:0] bmask(input logic [3:0] o, input logic [2:0] n);
	logic [3:0] m;
	m = 4'b1111 << (3'd4 - n);
	bmask = {m, 12'd0} >> o;
endfunction

function automatic logic [3:0] dmask(input logic [15:0] be);
	dmask = {|be[15:12], |be[11:8], |be[7:4], |be[3:0]};
endfunction

// left-align an operand of msz
function automatic logic [31:0] lalign(input logic [31:0] d, input logic [1:0] msz);
	case (msz)
		SZ_B:    lalign = {d[7:0], 24'd0};
		SZ_W:    lalign = {d[15:0], 16'd0};
		default: lalign = d;
	endcase
endfunction

//--------------------------------------------------------------------------
// storage
//--------------------------------------------------------------------------
// line valid and dirty, per set and way: one word per set, {dirty[3:0],
// valid[3:0]}, in LUT RAMs written as flip-flops were (at the edge) and
// read without a clock.  A LUT RAM is 32 deep whatever its width up to 20,
// so the ways share a word: copies at DC1's set (valid and dirty), at the
// engine's or the snoop's (valid; a snoop freezes the engine), at the
// scan's (dirty), and at the write's set, which forms the word written
// (the ways not written keep their bits).  A clear of every line (CINVA,
// reset) sweeps the 64 sets (E_M_CLR)
(* ramstyle = "MLAB, no_rw_check" *) logic [7:0] lsd [128];   // DC1's set
(* ramstyle = "MLAB, no_rw_check" *) logic [7:0] lse [128];   // lv_ra
(* ramstyle = "MLAB, no_rw_check" *) logic [7:0] lsw [128];   // the write's set
logic  [6:0] lv_ra;                // the engine copy's read: see below
wire   [3:0] lve_q;                // the engine's (or the snoop's) set
logic  [6:0] clr_i;                // the clear sweep's set
logic        clr_rst;              // ... after reset (else CINVA: then E_M_IC)
// synthesis translate_off
logic [3:0]  lv  [128];             // (simulation: the flip-flop model, checked)
// synthesis translate_off
initial for (int i = 0; i < 128; i++) lv[i] = 4'd0;
// synthesis translate_on
// synthesis translate_on
// dirty [set][way] (the push writes the whole line): read at the scan's
// set, the snooped set, else the fill/push set (they never read at once: a
// snoop freezes the engine), and at DC1's (a write-through hit on a dirty
// line leaves the fast path: the line is pushed first).  No clear: every
// read is of a valid line, and whatever makes a line valid writes its
// dirty bit
logic  [6:0] ld_ra;
// The write port's read-modify-write runs a clock after its request, from
// flip-flops (vq_*: the request's set, ways and bits); the copies' readers
// in that clock see it through a bypass, as if written at the request's
// edge
logic        vq_v, vq_lwe, vq_lv, vq_dwe, vq_dv;
logic  [6:0] vq_set;
logic  [3:0] vq_wm;
function automatic logic [7:0] vmerge(input logic [7:0] wo);
	vmerge[7:4] = vq_dwe ? ((wo[7:4] & ~vq_wm) | (vq_wm & {4{vq_dv}})) : wo[7:4];
	vmerge[3:0] = vq_lwe ? ((wo[3:0] & ~vq_wm) | (vq_wm & {4{vq_lv}})) : wo[3:0];
endfunction
wire   [7:0] lsm_r = lse[lv_ra];
wire   [7:0] lsm_q = (vq_v && vq_set == lv_ra) ? vmerge(lsm_r) : lsm_r;
wire   [3:0] ld_q  = lsm_q[7:4];
// the maintenance scan's set, its valid and dirty bits: a copy the write
// port keeps current (the scan's decision reads 4 bits, not 256)
logic  [6:0] sc_set;
logic  [3:0] sc_lv, sc_ld;
logic  [3:0] sc_m;                // the scan's tag matches, the set's four ways
                                  // (E_M_SCANW2: the tags hold through the set)
logic [1:0]  rrc;                  // replacement counter

// lookup ports: DC1, or the engine while it holds DC1
logic        steal, steal_q;
logic [31:0] st_la;                // engine lookup address (logical, for the ATC)
logic        st_fc2;
logic  [6:0] st_set;               // engine data/tag set

// snoop port phases: A presents the snooped set, B compares, H holds the
// cache for the snooper's action, R re-reads the owner's address; F
// invalidates a way the snooper's tag copy found (no RAM read: DC1's valid
// bits are read asynchronously, so nothing needs reading again)
typedef enum logic [2:0] { SN_IDLE, SN_A, SN_B, SN_H, SN_R, SN_F } snph_t;
snph_t       sn_ph;
wire         sn_frz  = (sn_ph != SN_IDLE);
assign       sn_act  = sn_frz;
assign       sf_ack  = (sn_ph == SN_F);
assign       bi_drop = bo_cancel && bo_ingnt && (e_st == E_BUS_W);
wire         sn_rset = (sn_ph == SN_A) || (sn_ph == SN_B);   // B: the line stays readable in H
logic  [1:0] sn_src;               // found in: 0 a cache way, 1 push buffer, 2 queued push
logic        sn_pvm, sn_bom;       // ... and (whatever sn_src says) the push buffer's
                                   // line, the queued push's line: both go with it
logic  [1:0] sn_way;
logic        bo_cancel;            // the queued push was invalidated by a snoop
logic        bo_ingnt;             // the engine's transfer is the BIU's (granted)
logic  [3:0] bo_got;               // a line read: the long words that arrived
logic        bo_later;             // a piece of this transfer has completed: a
                                  // bus error now is on a later access (FSLW.MA)

// The tag read set: AG's address (late: register read, address adders)
// passes one LUT; the other choices are registers, merged beforehand.
// Each way's RAMs get their own copy of that LUT (fanout 18, not 72).
// DT's own set while it looks its access up again (tk_relook).
logic        tk_relook;            // DT re-reads its tags and ATC entries
req_t        mt;                   // DT's access
wire  [6:0] la_alt   = sn_rset ? sn_pa[10:4] : steal ? st_set : tk_relook ? mt.a[10:4] :
                       m1.a[10:4];
wire        la_ag    = adv_ag && !sn_rset && !steal && !tk_relook;
// The data read set: the access DT holds next clock (DC1's when it moves
// on, else DT's own: a held DT reads its set every clock), or the
// engine's or the snoop's
wire  [6:0] dr_alt   = sn_rset ? sn_pa[10:4] : steal ? st_set : mt.a[10:4];
wire        dr_m1    = adv_dc1 && !sn_rset && !steal;

// tags: port A for lookups (DC1 or stolen), port B for engine reads/writes
logic [20:0] tq_a [4];
logic [20:0] tq_b [4];
logic        tw_b;
logic  [6:0] tset_b;
logic  [1:0] tway_b;
logic [20:0] twd_b;

// data: write port for stores and fills, read port for DT / engine
logic [127:0] dq_rn [4];           // unregistered output
logic  [31:0] dqa [4];             // DC2: every way's operand bytes (taken as
                                   // the access left DT)
// synthesis translate_off
logic  [63:0] dq2w [4];            // (simulation: the two long words, the check's)
// synthesis translate_on
logic         dw;
logic  [6:0]  dw_set;
logic  [1:0]  dw_way;
logic [15:0]  dw_be;
logic  [1:0]  dw_src;              // 0 dw_word on every long word, 1 bo_line, 2 pv_line
logic [31:0]  dw_word;
logic [127:0] dw_wdata;
// bytes of a long word (on every long word of a line) merged into a line
function automatic logic [127:0] mrg(input logic [127:0] l, input logic [15:0] be,
                                     input logic [31:0] w);
	for (int i = 0; i < 16; i++) if (be[i]) l[8*i +: 8] = w[8*(i % 4) +: 8];
	mrg = l;
endfunction
// the two long words of a line from long word k (the second wraps): every
// byte DC2 reads -- the operand (at most four bytes from its address) and
// AG's aligned long word -- is in them
function automatic logic [63:0] pair(input logic [127:0] l, input logic [1:0] k);
	pair = {l[127 - 32 * k -: 32], l[127 - 32 * 2'(k + 2'd1) -: 32]};
endfunction

// the WB fast store of this cycle (its RAM write is dw next cycle)
logic         wfm;
logic  [6:0]  wfm_set;
logic  [1:0]  wfm_way;
logic [15:0]  wfm_be;
logic [31:0]  wfm_word;

genvar gw;
generate
	for (gw = 0; gw < 4; gw++) begin : g_way
		(* keep *) wire [6:0] la_set_n = la_ag ? dm_va[10:4] : la_alt;
		(* keep *) wire [6:0] dr_set_n = dr_m1 ? m1.a[10:4] : dr_alt;
		ap68060_tdp #(.AW(7), .DW(21)) tag (
			.clk(clk),
			.addr_a(la_set_n), .we_a(1'b0), .wd_a(21'd0), .q_a(tq_a[gw]),
			.qa_raw(), .qa_bv(), .qa_bd(),
			.addr_b(tset_b), .we_b(tw_b && tway_b == 2'(gw)), .wd_b(twd_b), .q_b(tq_b[gw])
		);
		ap68060_sdp_be #(.AW(7), .NB(16), .OREG(0)) dat (
			.clk(clk),
			.we(dw && dw_way == 2'(gw)), .waddr(dw_set), .wbe(dw_be), .wdata(dw_wdata),
			.raddr(dr_set_n), .oce(1'b1), .q(dq_rn[gw])
		);
	end
endgenerate

// DC2's copy: every way's operand at its address, as the access leaves DT
// (DT held until that read was fresh): byte p of the two long words at the
// address by its index tk_bsel[p] (none: zero); the way is picked in DC2
always_ff @(posedge clk)
	if (adv_dt) for (int j = 0; j < 4; j++) begin
		logic [63:0] pr;
		pr = pair(dq_rn[j], mt.a[3:2]);
		for (int p = 0; p < 4; p++)
			dqa[j][8*p +: 8] <= (mt_bsel[p] == 3'd7) ? 8'd0 : pr[63 - 8*mt_bsel[p] -: 8];
		// synthesis translate_off
		dq2w[j] <= pr;
		// synthesis translate_on
	end

//--------------------------------------------------------------------------
// D-ATC
//--------------------------------------------------------------------------
logic        atc_hit;
atce_t       atc_e;
logic  [3:0] atc_hv, atc_hvd;
logic [19:0] atc_wpa [4], atc_wpad [4];
atce_t       atc_entd [4];
logic  [3:0] m1_aset;            // DC1's ATC set, registered with its address
logic  [3:0] mt_aset;            // ... and DT's
logic        atc_wr;
logic [31:0] atc_wla;
logic        atc_wfc2;
atce_t       atc_went;
logic        atc_fall, atc_fpage, atc_fng;

logic        tw_upd;              // the search updates an entry (first write: M)
logic  [3:0] tw_hw;               // ... in this way (the ATC's hit vector then)
ap68060_atc #(.SAME_SEARCH(0)) datc (
	.clk(clk), .nreset(nreset), .p8k(tc_p), .half(tc[11]),
	// (the first port is the engine's alone: its stolen address; DC1 and
	// DT look up through the second)
	.la(st_la), .fc2(st_fc2),
	.hit(atc_hit), .ent(atc_e), .hv(atc_hv), .wpa(atc_wpa),
	// (the second port: DC1's lookup, or DT's while it looks up again)
	.la_d(tk_relook ? mt.a : m1.a), .fc2_d(tk_relook ? mt.smode : m1.smode),
	.set_d(tk_relook ? mt_aset : m1_aset), .hv_d(atc_hvd), .wpa_d(atc_wpad), .ent_d(atc_entd),
	.wr(atc_wr), .wla(atc_wla), .wfc2(atc_wfc2), .went(atc_went),
	.wsame(tw_upd), .wsame_hv(tw_hw),
	.flush_all(atc_fall), .flush_page(atc_fpage), .flush_nonglobal(atc_fng)
);

//--------------------------------------------------------------------------
// translation and cache tag compare of a lookup
//--------------------------------------------------------------------------
// (xlate_t: the transparent translation registers' matches given, formed a
// clock ahead by the engine's lookup)
function automatic xres_t xlate_t(input logic [31:0] la, input logic s, input logic wr,
                                  input logic alt, input logic tt0, input logic tt1,
                                  input logic hitv, input atce_t e,
                                  input logic [20:0] t0, input logic [20:0] t1,
                                  input logic [20:0] t2, input logic [20:0] t3,
                                  input logic [3:0] valid);
	xres_t x;
	logic [20:0] tg;
	x = '0;
	if (alt) begin
		// an alternate address space (MOVES with SFC/DFC 0, 3, 4, 7): the
		// address is physical, untranslated (MC68060UM 4.2.5), and the
		// access implicitly noncachable (7.7.1), UPA 0 (table 7-2)
		x.ok  = 1'b1;
		x.pa  = la;
		x.cm  = 2'b10;
		x.upa = 2'd0;
	end
	else if (tt0 || tt1) begin
		x.ok  = 1'b1;
		x.pa  = la;
		x.cm  = tt0 ? dtt0[6:5] : dtt1[6:5];
		x.upa = tt0 ? dtt0[9:8] : dtt1[9:8];
		x.flt = wr && (tt0 ? dtt0[2] : dtt1[2]);
		x.why = FSLW_TTR | FSLW_WP;
	end
	else if (!tc_e) begin
		// the default transparent translation (MC68060UM 4.1.2): cache
		// mode DCO, UPA DUO, write protection DWO of the TCR
		x.ok  = 1'b1;
		x.pa  = la;
		x.cm  = tc[9:8];
		x.upa = tc[7:6];
		x.flt = wr && tc[5];
		x.why = FSLW_WP;
	end
	else if (hitv) begin
		x.pa  = {e.pa[19:1], tc_p ? la[12] : e.pa[0], la[11:0]};
		x.cm  = e.cm;
		x.upa = e.upa;
		if (!e.r || (e.s && !s) || (wr && e.w)) begin
			x.flt = 1'b1;      // nonresident, supervisor only, write protected
			x.ok  = 1'b1;
			x.why = e.b ? (FSLW_TWE | FSLW_LK) : !e.r ? FSLW_PF :
			        (e.s && !s) ? FSLW_SP : FSLW_WP;
		end
		else if (wr && !e.m) x.walk = 1'b1;   // first write: set M (3.3)
		else x.ok = 1'b1;
	end
	else x.walk = 1'b1;
	tg = x.pa[31:11];
	if (x.ok && !x.flt) begin
		if (valid[0] && t0 == tg) begin x.hit = 1'b1; x.way = 2'd0; end
		if (valid[1] && t1 == tg) begin x.hit = 1'b1; x.way = 2'd1; end
		if (valid[2] && t2 == tg) begin x.hit = 1'b1; x.way = 2'd2; end
		if (valid[3] && t3 == tg) begin x.hit = 1'b1; x.way = 2'd3; end
	end
	xlate_t = x;
endfunction
function automatic xres_t xlate(input logic [31:0] la, input logic s, input logic wr,
                                input logic alt,
                                input logic hitv, input atce_t e,
                                input logic [20:0] t0, input logic [20:0] t1,
                                input logic [20:0] t2, input logic [20:0] t3,
                                input logic [3:0] valid);
	xlate = xlate_t(la, s, wr, alt, ttr_hit(dtt0, la, s), ttr_hit(dtt1, la, s),
	                hitv, e, t0, t1, t2, t3, valid);
endfunction

// the FSLW RW field of the DC2 access: read, write, read-modify-write
wire  [1:0] m2_rw  = (m2.r.mem == M_LD) ? 2'b10 : (m2.r.mem == M_ST) ? 2'b01 : 2'b11;

// whether an address is used untranslated (alternate space, a TTR,
// translation off, CPU space): registered with DC1's address and with
// DT's, and again every clock they hold, so DT's compare select starts
// from a flip-flop
function automatic logic transp_of(input logic [31:0] a, input logic s, input logic [2:0] fc,
                                   input logic [1:0] ack);
	transp_of = (fc_tt(fc) == TT_ALT) || ttr_hit(dtt0, a, s) || ttr_hit(dtt1, a, s) ||
	            !tc_e_n || (ack != 2'd0);
endfunction
logic       m1_tp;

//--------------------------------------------------------------------------
// DT: DC1's lookup, registered -- every ATC way's page and entry with the
// hit vector, the set's four tags -- compared and decided here
//--------------------------------------------------------------------------
logic  [3:0] mt_hv;
logic [19:0] mt_wpa [4];
atce_t       mt_ent [4];
logic [20:0] mt_tq [4];
logic        mt_tp;
logic        mt_tt0, mt_tt1;      // DT's address's TTR matches, registered the same way
logic        mt_stale;            // the registered lookup may be stale
logic        tk_rdok;             // the ports read DT's address at the last edge, quiet
logic        tk_dok;              // the data RAM's output this clock is DT's set, fresh
assign tk_relook = mt.v && mt_stale;
// the valid and dirty bits of DT's set, read every clock (always current)
wire  [7:0] lsd_r = lsd[mt.a[10:4]];
wire  [7:0] lsd_q = (vq_v && vq_set == mt.a[10:4]) ? vmerge(lsd_r) : lsd_r;
wire  [3:0] lvd_q = lsd_q[3:0];
wire  [3:0] ldd_q = lsd_q[7:4];
wire        tk_wr = (mt.mem == M_ST) || (mt.mem == M_RMW);
// the hit way's entry (at most one way hits: AND-OR)
logic        tk_ahit;
atce_t       tk_ae;
always_comb begin
	tk_ahit = |mt_hv;
	tk_ae   = '0;
	for (int i = 0; i < 4; i++) tk_ae = tk_ae | (mt_hv[i] ? mt_ent[i] : '0);
end
// the translation (xlate: its attributes; its hit is replaced by the
// parallel compare below)
xres_t      x_tkr;
always_comb x_tkr = xlate_t(mt.a, mt.smode, tk_wr, fc_tt(mt.fc) == TT_ALT, mt_tt0, mt_tt1,
                            tk_ahit, tk_ae, mt_tq[0], mt_tq[1], mt_tq[2], mt_tq[3], lvd_q);
// the cache tag compare in parallel with the translation: each way's tag
// against the untranslated address and against the physical page of each
// of the ATC's four ways; the ATC's hit vector, not its selected entry,
// then picks.  Whether the access may use the hit (a translation that is
// valid and does not fault) still comes from xlate.
logic       tk_hit;
logic [1:0] tk_way;
logic [3:0] tk_mj;                // the matching way, one-hot
always_comb begin
	for (int j = 0; j < 4; j++) begin
		logic ma;
		ma = 1'b0;
		for (int i = 0; i < 4; i++)
			if (mt_hv[i] && mt_tq[j] == {mt_wpa[i][19:1], tc_p ? mt.a[12] : mt_wpa[i][0], mt.a[11]})
				ma = 1'b1;
		tk_mj[j] = lvd_q[j] && (mt_tp ? (mt_tq[j] == mt.a[31:11]) : ma);
	end
	tk_hit = |tk_mj;
	tk_way = tk_mj[3] ? 2'd3 : tk_mj[2] ? 2'd2 : tk_mj[1] ? 2'd1 : 2'd0;
end
xres_t      x_tk;                 // DT's translation and lookup
always_comb begin
	x_tk     = x_tkr;
	x_tk.hit = tk_hit && x_tkr.ok && !x_tkr.flt;
	x_tk.way = tk_way;
	if (mt.ack != 2'd0) begin
		// CPU space (interrupt acknowledge) is never translated nor cached
		x_tk     = '0;
		x_tk.ok  = 1'b1;
		x_tk.pa  = mt.a;
		x_tk.cm  = 2'b10;
	end
end
// synthesis translate_off
// (the engine's state for the bench's profile: a parameterized module's
// enum items are not reachable through the hierarchy in Verilator)
wire sim_es_s  = (e_st == E_S_START);
wire sim_es_w  = (e_st == E_W_START);
wire sim_es_m  = (e_st == E_M_START);
wire sim_es_tw = (e_st == E_TW_START);
wire sim_es_i  = (e_st == E_IDLE);
logic sim_xchk = 1'b0;        // (the bench fails the run on it)
always_ff @(posedge clk)
	if (nreset && mt.v && !mt_stale && mt.ack == 2'd0 &&
	    (x_tkr.hit != x_tk.hit || (x_tkr.hit && x_tkr.way != x_tk.way))) begin
		$display("FAIL: DMU: the parallel tag compare says hit=%b way=%0d, xlate hit=%b way=%0d",
		         x_tk.hit, x_tk.way, x_tkr.hit, x_tkr.way);
		sim_xchk <= 1'b1;
	end
// synthesis translate_on
wire  [4:0] mt_end   = {1'b0, mt.a[3:0]} + {2'b00, nbytes(mt.msz)};
wire        tk_split = (mt.msz != SZ_Q) && (mt_end > 5'd16);
// a write-through access to a dirty line goes to the engine (it is pushed)
// (the dirty bits picked by the one-hot match, not the encoded way)
wire        tk_wtd   = (x_tk.cm == 2'b00) && |(ldd_q & tk_mj);
wire        tk_sfast = ((mt.mem == M_LD) || (mt.mem == M_ST && x_tk.cm == 2'b01)) &&
                       tk_split && x_tk.ok && !x_tk.flt && x_tk.hit && !tk_wtd &&
                       !x_tk.cm[1] && dc_en && (mt.ack == 2'd0) && !mt.lock;
// (locked accesses -- TAS, CAS, BUSCR.L sequences -- bypass the data cache:
// MC68060UM 5.2; the slow path pushes and invalidates a hit first)
wire        tk_fast  = (mt.mem == M_LD || mt.mem == M_RMW) && !mt.lock &&
                       x_tk.ok && !x_tk.flt && x_tk.hit && !tk_wtd &&
                       !x_tk.cm[1] && dc_en && (mt.ack == 2'd0) &&
                       (mt.msz != SZ_Q) && !tk_split;
// DC2's operand bytes: output byte p (0 the least significant) of an
// n-byte operand at byte o of the two long words is byte o + n - 1 - p
// (byte 0 the first): its index, 7 for none, registered with the access
logic [2:0] tk_bsel [4];
logic [2:0] mt_bsel [4];        // (registered as the access enters DT, from DC1's)
always_comb
	for (int p = 0; p < 4; p++)
		tk_bsel[p] = (3'(p) < nbytes(mt.msz)) ? 3'({1'b0, mt.a[1:0]} + nbytes(mt.msz) - 3'd1 - 3'(p)) : 3'd7;

// a store older than an access, by its line within the page (PA11-4 =
// LA11-4: a store to the same offset of another page counts too)
function automatic logic st_at(input req_t r, input logic sp, input logic [7:0] l);
	st_at = r.v && (r.mem == M_ST || r.mem == M_RMW) &&
	        (r.a[11:4] == l || (sp && 8'(r.a[11:4] + 8'd1) == l));
endfunction

//--------------------------------------------------------------------------
// DC2
//--------------------------------------------------------------------------
// a store older than the DC2 load and not yet in the data RAM (EX, WB)
wire  [27:0] m2_line = m2.x.pa[31:4];
// (by the line within the page only, PA11-4: a store to the same offset of
// another page is taken for the line too -- the load waits for it, or takes
// the slow path; a smaller, shorter compare on DC2's hold)
function automatic logic st_line(input mrec_t m, input logic [27:0] l);
	st_line = m.r.v && (m.r.mem == M_ST || m.r.mem == M_RMW) &&
	          (m.x.pa[11:4] == l[7:0] || (m.split && m.x1.pa[11:4] == l[7:0]));
endfunction
wire hz = st_line(m3, m2_line) || st_line(m4, m2_line);
// any store older than the DC2 uop not yet performed: a table walk for the
// DC2 uop must see it (the program may just have written a descriptor)
wire st_older = (m3.r.v && (m3.r.mem == M_ST || m3.r.mem == M_RMW)) ||
                (m4.r.v && (m4.r.mem == M_ST || m4.r.mem == M_RMW));
// a RAM write to the load's set, or any engine activity, after its lookup
// read the RAM makes the captured line stale: the engine reads it again
logic m1_stale;

// (DT let the access go only with its data read fresh: no older store to
// its line was still in flight then, and every older one is ahead of it)
wire fast_now = m2.r.v && m2.fast;
// the hit way's operand (the way registered: a 4-input multiplexer per bit;
// a split load's bytes past the line are masked by its user)
logic [31:0] fast_data;
assign fast_data = dqa[m2.x.way];
// synthesis translate_off
logic [63:0] dq2;
assign dq2 = dq2w[m2.x.way];
always_ff @(posedge clk)
	if (nreset && m2.r.v && m2.fast &&
	    fast_data != take({dq2, 64'd0}, {2'd0, m2.r.a[1:0]}, nbytes(m2.r.msz))) begin
		$display("FAIL: DMU: DC2's byte selects give %h, the operand is %h", fast_data,
		         take({dq2, 64'd0}, {2'd0, m2.r.a[1:0]}, nbytes(m2.r.msz)));
		$finish;
	end
// synthesis translate_on

// a store with its translation known needs nothing from DC2 unless it must
// allocate (copyback miss), is split, or is a MOVE16 destination
// (decided in DT, registered with the access: m2_ss, cleared with its hit)
function automatic logic st_simp(input req_t r, input xres_t x, input logic sp);
	st_simp = (r.mem == M_ST) && x.ok && !x.flt && !sp && (r.msz != SZ_Q) &&
	          !(dc_en && x.cm == 2'b01 && !x.hit && !r.noalloc && !r.lock && !dc_nad);
endfunction
logic m2_ss;
logic [3:0] dt_inv;            // ways invalidated in the set of the access entering DC2
wire st_simple = m2.r.v && m2_ss;
// synthesis translate_off
always_ff @(posedge clk)
	if (nreset && m2.r.v && !e_job && !e_dc2_done && m2_ss != st_simp(m2.r, m2.x, m2.split)) begin
		$display("FAIL: DMU: DC2's simple-store decision %b disagrees", m2_ss);
		$finish;
	end
// synthesis translate_on

//--------------------------------------------------------------------------
// engine
//--------------------------------------------------------------------------
typedef enum logic [5:0] {
	E_IDLE,
	E_S_START, E_S_XL, E_S_XLW, E_S_XL2, E_S_ACT, E_S_RD, E_S_RDW, E_S_RD2, E_S_RD3, E_S_BUSD,
	E_S_NEXT, E_S_DONE, E_SP_W, E_SP_2, E_SP_3, E_SP_4,
	E_W_START, E_W_DEC, E_W_PART, E_W_DONE, E_SB,
	E_M_START, E_M_PG, E_M_SCAN, E_M_SCANW, E_M_SCANW2, E_M_SCAN2, E_M_IC, E_M_DONE, E_M_CLR,
	E_M_PLW, E_M_PL2, E_M_PLD,
	E_FILL, E_FILLW, E_FILL_W, E_FILL_INS, E_PUSHV, E_PUSHVW, E_PUSHV_W, E_PUSHV_B,
	E_BUS, E_BUS_W,
	E_TW_START, E_TW_DESC, E_TW_DBUS, E_TW_EVAL, E_TW_UPD, E_TW_UPD_W, E_TW_LKW,
	E_TW_DONE,
	E_I_DONE
} est_t;

est_t        e_st, e_ret, e_sret, e_wret;
logic        e_dc2_done;          // DC2 has its answer (until it advances)
logic        e_wst_done;          // the WB store is done (until WB advances)
logic [31:0] e_ldata;
logic        e_flt;
logic [31:0] e_faddr;
logic [31:0] e_fslw;
logic        e_part;              // working on part 1 of a split access
xres_t       e_x;                 // translation of the current part
logic [31:0] e_va;                // logical address of the current part
logic  [2:0] e_n;                 // bytes in the current part
logic [31:0] e_acc;               // load: assembled operand; store: left aligned data
logic        e_fromline;          // the operand comes from bo_line (inhibited fill)
logic        e_fillhit;           // ... from the line a load's own fill just installed
logic        wd_push, wd_wtp, wd_upd, wd_cb, wd_sb;   // E_W_DEC's decisions
logic        fv_lv, fv_ld, fv_bad;   // the fill's victim / the pushed line: valid, dirty
logic [15:0] wd_be;
logic        e_falloc;            // a store's part has been allocated once: if a snoop
                                  // takes the line before the lookup after the fill
                                  // finds it, the part stays a miss (WB writes
                                  // memory), never a second fill: a stream of snoops
                                  // could close that window every time
logic        e_kill;              // the DC2 job's uop was discarded
logic        sp_job;              // the DC2 job is a fast split read
logic        sp_dw;               // a write to the next set while its
                                  // stolen read was in flight (E_SP_W)
// engine states that write no data, tag or valid bit: the RAM outputs a
// lookup captured stay good (a fast split read; the RAM re-reads the held
// DC1 set after its stolen read, steal_q)
wire  e_quiet = (e_st == E_IDLE) || (e_st == E_S_START) || (e_st == E_SP_W) ||
                (e_st == E_SP_2) || (e_st == E_SP_3) || (e_st == E_SP_4) ||
                (e_st == E_S_DONE && sp_job);
// engine states that hold DC1 (all but the idle engine and its WB store's)
// (compares of the state register with constants only: it stays a state
// machine, one-hot)
wire  e_busy = (e_st != E_IDLE) && (e_st != E_W_START) && (e_st != E_W_DEC) &&
               (e_st != E_W_PART) && (e_st != E_W_DONE) && (e_st != E_SB);
// the engine's own lookup through the stolen ports: the ATC's part (an
// asynchronous read) registered in the clock the tag RAM takes the stolen
// set (E_S_XLW, E_SP_W), the tags compared from the RAM the clock after
logic        lk_hit;
logic        lk_tt0, lk_tt1;       // ... and the address's TTR matches
atce_t       lk_e;
logic  [3:0] lk_hv;
logic        e_job;               // the engine owns the DC2 uop until it is done

// bus operation built by a job, run by E_BUS
typedef struct packed {
	logic        line;            // a line transfer
	logic        rd;
	logic  [1:0] tt;
	logic  [2:0] tm;
	logic  [1:0] upa;
	logic        ci;
	logic        lock;
	logic        locke;
	logic  [1:0] ack;             // acknowledge cycle (see req_t)
	logic  [1:0] tln;
} bop_t;
bop_t        bo;
logic [31:0] bo_wd;               // write data (left aligned)
logic [127:0] bo_line;            // line write data / line read result
logic        bo_err;
logic [31:0] bo_rd;               // read data (right aligned)
logic        bo_avec;
logic        bo_tci;
logic  [2:0] bo_left;             // bytes still to do
logic  [2:0] bo_pn;               // bytes in the piece on the bus
logic [31:0] bo_pa;               // address of the piece on the bus

// fill/push state
logic  [6:0] f_set;
// the dirty RAM's one read: the snoop's compare, the scan's set, else the
// fill/push set
// (the dirty bits share the valid copy's read: the states that read
// one and not the other are given the other's set)
always_comb ld_ra = lv_ra;
logic  [1:0] f_way;
logic [20:0] f_tag;
logic        pv_v;                // push buffer holds a dirty line
logic [127:0] pv_line;
logic [27:0] pv_lpa;              // its line address
logic        pv_dirty;
logic  [1:0] pv_tln;
logic        pv_keep;             // E_PUSHV: the pushed line stays valid (clean)
logic [127:0] mv_line;            // MOVE16 line buffer

// the RAM write data: a fill or a restored victim (the buffers hold until
// the write cycle), else a word on every long word with byte enables
assign dw_wdata = (dw_src == 2'd1) ? bo_line : (dw_src == 2'd2) ? pv_line : {4{dw_word}};

// maintenance scan
logic  [8:0] ms_i;                // set*4 + way

// table walk state
typedef enum logic [1:0] { TW_ROOT, TW_PTR, TW_PAGE, TW_IND } twl_t;
twl_t        tw_lvl;
logic [31:0] tw_va;
logic        tw_s, tw_wr, tw_pt, tw_i;   // FC2, write access, PLPA, I-side
logic [31:0] tw_cause;            // FSLW bits of a failed search (PTA PTB IL PF TWE LK)
logic [31:0] tw_da;               // descriptor address
logic [31:0] tw_d;                // descriptor
logic        tw_wp;               // accumulated write protect
logic        tw_fail, tw_berr;
logic [31:0] tw_pd;               // final page descriptor
logic        tw_use;              // the data access's search is done: its
atce_t       tw_e;                // ... entry (or failure) for the access,
                                  // whether or not the ATC took it

wire [6:0] tw_ri = tw_va[31:25];
wire [6:0] tw_pi = tw_va[24:18];
wire [5:0] tw_gi = tc_p ? {1'b0, tw_va[17:13]} : tw_va[17:12];

// next aligned piece of an operand
function automatic void piece(input logic [31:0] a, input logic [2:0] left,
                              output logic [1:0] bsiz, output logic [2:0] n);
	if (a[1:0] == 2'b00 && left >= 3'd4) begin bsiz = SIZ_L; n = 3'd4; end
	else if (a[0] == 1'b0 && left >= 3'd2) begin bsiz = SIZ_W; n = 3'd2; end
	else begin bsiz = SIZ_B; n = 3'd1; end
endfunction

logic [1:0] bo_siz;
logic [2:0] bo_np;
always_comb piece(bo_pa, bo_left, bo_siz, bo_np);

// bytes of a read piece from the lanes, right aligned
function automatic logic [31:0] grab(input logic [31:0] d, input logic [2:0] n,
                                     input logic [1:0] a10);
	case (n)
		3'd1:    grab = {24'd0, d[31 - 8*a10 -: 8]};
		3'd2:    grab = {16'd0, a10[1] ? d[15:0] : d[31:16]};
		default: grab = d;
	endcase
endfunction

// the victim way of a set: the first invalid way, else the counter
function automatic logic [1:0] victim(input logic [3:0] valid, input logic [1:0] rr,
                                      input logic half);
	logic [1:0] v;
	v = half ? {1'b0, rr[0]} : rr;
	for (int i = 3; i >= 0; i--) if (!valid[i] && !(half && i >= 2)) v = 2'(i);
	victim = v;
endfunction

// a line targeted by a store still in EX/WB is never evicted
function automatic logic store_holds(input logic [6:0] set, input logic [1:0] way,
                                     input mrec_t a, input mrec_t b);
	store_holds = (a.r.v && (a.r.mem == M_ST || a.r.mem == M_RMW) && a.x.hit &&
	               a.x.pa[10:4] == set && a.x.way == way) ||
	              (b.r.v && (b.r.mem == M_ST || b.r.mem == M_RMW) && b.x.hit &&
	               b.x.pa[10:4] == set && b.x.way == way);
endfunction

// FSLW of an access error (MC68060UM figure 8-6; the SIZE coding is the
// SIZ pins', see the package): the cause bits, LK, RW (2'b10 read, 2'b01
// write, 2'b11 read-modify-write), the size, TT (MOVE16 01, else normal:
// the 68060 reports MOVES as a normal access of its function code) and TM,
// the function code
function automatic logic [31:0] mk_fslw(input logic [31:0] cause, input logic lk,
                                        input logic [1:0] rw, input logic [1:0] msz,
                                        input logic [2:0] fc);
	logic [1:0] sz;
	case (msz)
		SZ_B: sz = 2'b01;
		SZ_W: sz = 2'b10;
		SZ_L: sz = 2'b00;
		default: sz = 2'b11;
	endcase
	// TT and TM as the access drives them (MOVES: table 4-2)
	mk_fslw = cause | (lk ? FSLW_LK : 32'd0) |
	          {7'd0, rw, sz, (msz == SZ_Q) ? TT_MOVE16 : fc_tt(fc), fc_tm(fc), 16'd0};
endfunction

//--------------------------------------------------------------------------
// WB fast store: a copyback store that hit, within one line
//--------------------------------------------------------------------------
wire wb_st   = m4.r.v && st_v && (m4.r.mem == M_ST || m4.r.mem == M_RMW);
// (whether WB's access can be a fast store at all: registered as it
// enters WB, cleared with its hit)
logic m4_fq;
wire wb_fast = wb_st && m4_fq && dc_en && (e_st == E_IDLE) && !e_wst_done;
// WB's store straight into the store buffer: a write-through or noncachable
// imprecise store (CACR.ESB) in one aligned bus piece that has no line in
// the cache, not locked, no acknowledge cycle -- the engine's E_SB path in
// its own WB clock.  Qualified as it enters WB (m4_fsb); without room it
// takes the engine's path (which keeps a younger access's bus cycles behind
// it)
function automatic logic fsb_of(input mrec_t m);
	fsb_of = !m.split && m.x.ok && !m.x.flt && !m.r.lock && (m.r.msz != SZ_Q) &&
	         (m.r.ack == 2'd0) && (m.x.cm == 2'b00 || m.x.cm == 2'b11) && cacr[29] &&
	         !(m.x.hit && dc_en) &&
	         ((m.r.msz == SZ_B) || (m.r.msz == SZ_W && !m.r.a[0]) ||
	          (m.r.msz == SZ_L && m.r.a[1:0] == 2'b00));
endfunction
logic m4_fsb;
// synthesis translate_off
always_ff @(posedge clk)
	if (nreset && m4.r.v && m4_fq != (!m4.split && m4.x.ok && !m4.x.flt && m4.x.hit &&
	                                 m4.x.cm == 2'b01 && !m4.r.lock && (m4.r.msz != SZ_Q))) begin
		$display("FAIL: DMU: WB's fast-store qualification %b disagrees", m4_fq);
		$finish;
	end
// synthesis translate_on
wire  [31:0] wbf_word = lanes32(lalign(st_data, m4.r.msz), m4.r.a[1:0]);
wire  [15:0] wbf_be   = bmask(m4.r.a[3:0], nbytes(m4.r.msz));
assign wfm      = wb_fast && !sn_frz;
assign wfm_set  = m4.x.pa[10:4];
assign wfm_way  = m4.x.way;
assign wfm_be   = wbf_be;
assign wfm_word = wbf_word;
// a copyback store across a line, both parts hits: the first part this
// cycle, the second the next (wfs_p1), without the engine
logic        wfs_p1;
wire wb_fs = wb_st && m4.split && m4.x.ok && !m4.x.flt && m4.x.hit && m4.x.cm == 2'b01 &&
             m4.x1.ok && !m4.x1.flt && m4.x1.hit && m4.x1.cm == 2'b01 && dc_en &&
             !m4.r.lock && (m4.r.msz != SZ_Q) && (e_st == E_IDLE) && !e_wst_done && !wfs_p1;
wire   [2:0] wfs_n0  = 3'(5'd16 - {1'b0, m4.r.a[3:0]});
wire  [31:0] wfs_acc = lalign(st_data, m4.r.msz);

// (not while an older store is in EX: it reaches WB, which has the engine
// first, before DC2's job -- the bus sees a store before a younger load's
// access, whatever the timing)
wire ex_st    = m3.r.v && (m3.r.mem != M_LD);
wire dc2_slow = m2.r.v && !fast_now && !(st_simple && !e_job) && !e_dc2_done && !e_job &&
                !ex_st;
logic wb_fsb, wb_slow;            // (below, with the store buffer's room)

// store ready: the fast store answers in its own WB cycle (the engine's
// stores through the registered pulse)
logic st_rdy_r, st_fault_r;
assign st_rdy   = st_rdy_r || (wb_fast && !sn_frz) || (wfs_p1 && m4.x1.hit && !sn_frz) || wb_fsb;
assign st_fault = st_rdy_r && st_fault_r;

// The holds the pipeline's stall chain reads, as registers: each formed a
// clock ahead, for the access entering the stage and for the one staying
// (the advance picks last), from the state as it will be.  Where the next
// state is not known early -- a snoop starting, the engine leaving a state
// -- the hold is assumed (a clock longer at worst).  Checked: a registered
// hold is never off while the stage's own state holds it.
//   hold1_q   DC1: the snoop port, a stolen port, a stale lookup, the engine
//   dc2r_q    DC2: the access done (the fast path, a simple store, the engine)
//   st_ok_q   WB:  the store answered without a fault
logic hold1_q, dc2r_q, st_ok_q;
assign dm_hold1 = hold1_q;
assign dc2_rdy  = dc2r_q;
assign st_ok    = st_ok_q;

//--------------------------------------------------------------------------
// store buffer: a four-entry FIFO of bus write cycles (one entry per cycle,
// up to four bytes) for writes to write-through and noncachable imprecise
// pages while CACR.ESB is set.  The WB store completes once its pieces
// are in; the buffer then writes them out ahead of every other bus cycle
// of the DMU, and so ahead of the fetch's too (it is the BIU's first
// client), as 5.10 requires: no other bus cycle passes a buffered write.
// A TEA on one is recorded (sb_err) and the rest still go out (8.4.6).
//--------------------------------------------------------------------------
typedef struct packed {
	logic [31:0] pa;
	logic  [2:0] n;           // bytes: 1, 2 or 4 (an aligned bus cycle)
	logic [31:0] wd;          // left aligned
	logic  [1:0] tt;
	logic  [2:0] tm;
	logic  [1:0] upa;
	logic        ci;
} sbent_t;
localparam int SBN = 2;           // store buffer entries (the 68060 has 4: 150 ALMs more)
sbent_t      sb [SBN];
logic  [2:0] sb_n;
logic        sel_q;               // the BIU saw the store buffer's request last cycle
logic        own_sb;              // the transaction in progress is the buffer's
logic        sb_fly;              // the head is on the bus (granted, not done)
wire         sb_room = (sb_n < 3'(SBN));
wire         sb_push = (e_st == E_SB) && sb_room;
assign wb_fsb  = wb_st && m4_fsb && (e_st == E_IDLE) && !e_wst_done && !wfs_p1 && sb_room;
assign wb_slow = wb_st && !wb_fast && !wb_fs && !wfs_p1 && !e_wst_done && !wb_fsb;
// present the head while it is not on the bus already (the BIU may start
// again in the clock its done comes); the engine's request waits while the
// buffer holds anything, so no bus cycle of the engine passes a write
wire         sb_sel  = (sb_n != 3'd0) && !sb_fly;
// (a buffered write on the bus: more writes behind it, or the engine's)
assign       bi_more = sb_fly && (sb_n > 3'd1 || b_req);
wire         sb_pop  = own_sb && (bi_done || bi_err);
logic        sb_mt;               // (sb_n == 0, a register)
assign       sb_empty = sb_mt;
assign       sb_ne_n  = (sb_n > 3'd1) || (sb_n == 3'd1 && !sb_pop) || sb_push || wb_fsb;
// a TEA on a buffered write (SBE) or on a push (PBE): recorded, reported
// as an access error once the buffers are empty (MC68060UM 8.4.6)
logic        sbe_r, pbe_r;
assign       sb_err  = sbe_r || pbe_r;
assign       sb_fslw = (sbe_r ? FSLW_SBE : 32'd0) | (pbe_r ? FSLW_PBE : 32'd0);

// the engine's view of the BIU port
logic        b_req;
busreq_t     b_breq;
logic [127:0] b_wdata;
wire         b_gnt    = bi_gnt && !sel_q;
wire         b_done   = bi_done && !own_sb;
wire         b_err    = bi_err && !own_sb;
wire         b_rvalid = bi_rvalid && !own_sb;

always_comb begin
	bi_req   = sb_sel || (b_req && sb_n == 3'd0);
	bi_breq  = b_breq;
	bi_wdata = b_wdata;
	// (the head stays presented while it is on the bus: the BIU reads its
	// data again for a retry or the extra write hold)
	if (sb_sel || sb_fly) begin
		bi_breq       = '0;
		bi_breq.addr  = sb[0].pa;
		bi_breq.siz   = (sb[0].n == 3'd4) ? SIZ_L : (sb[0].n == 3'd2) ? SIZ_W : SIZ_B;
		bi_breq.rd    = 1'b0;
		bi_breq.tt    = sb[0].tt;
		bi_breq.tm    = sb[0].tm;
		bi_breq.upa   = sb[0].upa;
		bi_breq.ci    = sb[0].ci;
		bi_wdata      = {96'd0, (sb[0].n == 3'd1) ? {4{sb[0].wd[31:24]}} :
		                        (sb[0].n == 3'd2) ? {2{sb[0].wd[31:16]}} : sb[0].wd};
	end
end

always_ff @(posedge clk) begin
	if (!nreset) begin
		sb_n <= '0; sb_mt <= 1'b1; sel_q <= 1'b0; own_sb <= 1'b0; sb_fly <= 1'b0; sbe_r <= 1'b0;
		for (int i = 0; i < SBN; i++) sb[i] <= '0;
	end
	else begin
		logic [2:0] n;
		n = sb_n;
		sel_q <= sb_sel;
		if (bi_gnt) begin
			own_sb <= sel_q;
			sb_fly <= sel_q;
		end
		if (sb_pop) sb_fly <= 1'b0;
		if (sb_pop) begin
			for (int i = 0; i < SBN - 1; i++) sb[i] <= sb[i + 1];
			n = n - 3'd1;
			if (bi_err) sbe_r <= 1'b1;
		end
		if (sb_push || wb_fsb) begin
			sbent_t e;
			if (wb_fsb) begin
				e.pa  = m4.x.pa;
				e.n   = nbytes(m4.r.msz);
				e.wd  = lalign(st_data, m4.r.msz);
				e.tt  = fc_tt(m4.r.fc);
				e.tm  = fc_tm(m4.r.fc);
				e.upa = m4.x.upa;
				e.ci  = m4.x.cm[1];
			end
			else begin
				e.pa  = bo_pa;
				e.n   = bo_np;
				e.wd  = bo_wd;
				e.tt  = bo.tt;
				e.tm  = bo.tm;
				e.upa = bo.upa;
				e.ci  = bo.ci;
			end
			sb[n[$clog2(SBN)-1:0]] <= e;
			n = n + 3'd1;
		end
		sb_n  <= n;
		sb_mt <= (n == 3'd0);
		if (sb_err_ack) sbe_r <= 1'b0;
	end
end

// a failed line fill: the long words of the line the access needs (this
// part's bytes e_va[3:0] .. +e_n-1) all arrived -- a load can complete
// from them; a store needs none from memory, only that the fill started
wire  [3:0] fill_last  = {1'b0, e_va[3:0]} + {1'b0, e_n} - 4'd1;
wire  [3:0] fill_need  = ((4'b1111 << e_va[3:2]) & (4'b1111 >> (2'd3 - fill_last[3:2])));
wire        fill_late_ok = (m2.r.mem == M_ST) ? (bo_got != 4'd0)
                                              : ((fill_need & ~bo_got) == 4'd0);

//--------------------------------------------------------------------------
// BIU request from the engine's bus step
//--------------------------------------------------------------------------
logic e_breq;
always_comb begin
	b_req  = e_breq;
	b_breq = '0;
	// acknowledge cycles (TT=3): IACK at $FFFFFFFF, a byte, TM = the level;
	// the breakpoint acknowledge (a byte read at 0) and the LPSTOP
	// broadcast (a word write at $FFFFFFFE) have TM = 0
	b_breq.addr  = (bo.ack == 2'd1) ? 32'hFFFF_FFFF : bo_pa;
	b_breq.siz   = (bo.ack == 2'd1) ? SIZ_B : bo.line ? SIZ_LINE : bo_siz;
	b_breq.rd    = bo.rd;
	b_breq.tt    = (bo.ack != 2'd0) ? TT_ACK : bo.tt;
	b_breq.tm    = (bo.ack == 2'd1) ? iack_lvl : (bo.ack != 2'd0) ? 3'd0 : bo.tm;
	b_breq.tln   = bo.tln;
	b_breq.upa   = bo.upa;
	b_breq.ci    = bo.ci;
	b_breq.lock  = bo.lock;
	b_breq.locke = bo.locke && !bo.rd && (bo.line || bo_left == bo_np);
	// (the LPSTOP broadcast drives D31-D16 high, the new SR on D15-D0)
	b_wdata      = bo.line ? bo_line :
	               {96'd0, (bo.ack == 2'd3) ? {16'hFFFF, bo_wd[31:16]} :
	                       (bo_np == 3'd1) ? {4{bo_wd[31:24]}} :
	                       (bo_np == 3'd2) ? {2{bo_wd[31:16]}} : bo_wd};
end

// (a stale lookup waits the clock its set is read again, rather than
// sending its uop to the engine's general path: after a miss the loads
// behind it would otherwise each take that path, every one keeping the
// engine busy for the next)
assign dm_hold1_c = sn_frz || steal || steal_q || (m1.v && m1_stale) || tk_relook ||
                  (e_st != E_IDLE && e_st != E_W_START &&
                   e_st != E_W_DEC && e_st != E_W_PART && e_st != E_W_DONE && e_st != E_SB);

// the engine's operand bytes: E_SP_3 the rest of a split load from the
// stolen read, E_S_RD2 from the read way (both the RAM's: registered in
// etk_q before the merge); E_S_ACT from the line a fill delivered (bo_line,
// its own extractor: the RAM is off that merge's path)
wire  [2:0]  sp_n0    = 3'(5'd16 - {1'b0, m2.r.a[3:0]});
wire         etk_sp   = (e_st == E_SP_3);
wire  [3:0]  etk_o    = etk_sp ? 4'd0 : e_va[3:0];
wire  [2:0]  etk_n    = etk_sp ? 3'(nbytes(m2.r.msz) - sp_n0) : e_n;
wire [31:0]  etk_b    = take(bo_line, e_va[3:0], e_n);
// (the way's line from the RAM's output, registered with the offset and
// count before the bytes are taken and merged: E_S_RD3, E_SP_4)
logic [127:0] etk_l;
logic   [3:0] etk_oq;
logic   [2:0] etk_nq;
wire   [31:0] etk_q    = take(etk_l, etk_oq, etk_nq);

always_ff @(posedge clk) begin
	// a store's answer is kept until WB takes it: WB can be held in the
	// cycle it comes (sb_wait: an interrupt due while older stores are
	// buffered), and e_wst_done keeps the engine from doing the store again
	// -- a pulse there was lost and WB waited for it forever
	if (adv_wb || !wb_st) st_rdy_r <= 1'b0;
	mt_done   <= 1'b0;
	lkend     <= 1'b0;
	iack_end  <= 1'b0;
	iw_done   <= 1'b0;
	atc_wr    <= 1'b0;
	atc_fall  <= 1'b0;
	atc_fpage <= 1'b0;
	iatc_flush_all  <= 1'b0;
	iatc_flush_page <= 1'b0;
	iatc_wr   <= 1'b0;
	tw_b      <= 1'b0;
	dw        <= 1'b0;
	steal_q   <= steal;

	if (!nreset) begin
		m1 <= '0; mt <= '0; m2 <= '0; m3 <= '0; m4 <= '0;
		mt_stale <= 1'b0; tk_rdok <= 1'b0; tk_dok <= 1'b0; hold_dt <= 1'b0;
		// (reset leaves the cache alone -- tags, valid and dirty state, data,
		// 5.3: its valid and dirty bits have their power-up values from
		// their initial blocks; CINVA clears it when the software asks)
		e_st <= E_IDLE; e_ret <= E_IDLE; e_sret <= E_IDLE; e_wret <= E_IDLE;
		e_dc2_done <= 1'b0; e_wst_done <= 1'b0;
		e_ldata <= '0; e_flt <= 1'b0; e_faddr <= '0; e_fslw <= '0;
		e_part <= 1'b0; e_x <= '0; e_va <= '0; e_n <= '0; e_acc <= '0; e_fromline <= 1'b0; e_fillhit <= 1'b0; e_falloc <= 1'b0;
		e_breq <= 1'b0;
		bo <= '0; bo_wd <= '0; bo_line <= '0; bo_err <= 1'b0; bo_rd <= '0;
		bo_avec <= 1'b0; bo_tci <= 1'b0; bo_left <= '0; bo_pn <= '0; bo_pa <= '0;
		steal <= 1'b0; st_la <= '0; st_fc2 <= 1'b0; st_set <= '0;
		f_set <= '0; f_way <= '0; f_tag <= '0;
		pv_v <= 1'b0; pv_line <= '0; pv_lpa <= '0; pv_dirty <= 1'b0; pv_tln <= '0; pv_keep <= 1'b0;
		mv_line <= '0;
		ms_i <= '0;
		sc_set <= '0; sc_lv <= '0; sc_ld <= '0;
		vq_v   <= 1'b0;
		tw_lvl <= TW_ROOT; tw_va <= '0; tw_s <= 1'b0; tw_wr <= 1'b0; tw_pt <= 1'b0; tw_cause <= '0;
		tw_i <= 1'b0; tw_da <= '0; tw_d <= '0; tw_wp <= 1'b0; tw_fail <= 1'b0;
		tw_berr <= 1'b0; tw_pd <= '0; tw_upd <= 1'b0; tw_hw <= 4'd0; tw_use <= 1'b0; tw_e <= '0;
		iw_cause <= '0;
		tset_b <= '0; tway_b <= '0; twd_b <= '0;
		dw_set <= '0; dw_way <= '0; dw_be <= '0; dw_src <= '0; dw_word <= '0;
		ic_inv <= 1'b0; ic_inv_scope <= '0; ic_inv_pa <= '0;
		rrc <= 2'd0;
		st_fault_r <= 1'b0; st_fslw <= '0; st_faddr <= '0;
		mt_pa <= '0; mt_fault <= 1'b0; mt_fslw <= '0;
		lkend <= 1'b0;
		iack_end <= 1'b0;
		atc_wla <= '0; atc_wfc2 <= 1'b0; atc_went <= '0; atc_fng <= 1'b0;
		iatc_flush_ng <= 1'b0; iatc_flush_la <= '0; iatc_flush_fc2 <= 1'b0;
		iatc_wla <= '0; iatc_wfc2 <= 1'b0; iatc_went <= '0;
		iw_ent <= '0;
		e_kill <= 1'b0;
		sp_dw  <= 1'b0;
		sp_job <= 1'b0;
		wfs_p1 <= 1'b0;
		e_job <= 1'b0;
		m1_stale <= 1'b0;
		sn_ph <= SN_IDLE; sn_src <= '0; sn_way <= '0; bo_cancel <= 1'b0; bo_later <= 1'b0; pbe_r <= 1'b0;
		bo_ingnt <= 1'b0; sn_pvm <= 1'b0; sn_bom <= 1'b0;
		sn_look <= 1'b0; sn_hit <= 1'b0; sn_dirty <= 1'b0;
		clr_i   <= '0;
		clr_rst <= 1'b0;
	end
	else begin
		// the valid/dirty write port: at most one writer per cycle (the
		// WB fast store runs only with the engine idle, the snoop port
		// only with it frozen), applied at the end of this block
		logic [6:0] vp_set;
		logic [3:0] vp_wm;               // ways written
		logic       vp_lwe, vp_lv;       // valid: write, value
		logic       vp_dwe, vp_dv;       // dirty: write, value
		logic       m1s_n, mts_n, m2ss_n;   // m1_stale, mt_stale, m2_ss as they will be
		// (its three writers, each with its own request: the WB store's
		// fast path vf_*, the engine ve_*, the snoop port vs_*; combined
		// below, never two at once)
		logic [6:0] vf_set, ve_set, vs_set;
		logic [3:0] vf_wm, ve_wm, vs_wm;
		logic       vf_lwe, vf_lv, vf_dwe, vf_dv, ve_lwe, ve_lv, ve_dwe, ve_dv;
		logic       vs_lwe, vs_lv, vs_dwe, vs_dv;
		vp_set = '0; vp_wm = '0; vp_lwe = 1'b0; vp_lv = 1'b0;
		vp_dwe = 1'b0; vp_dv = 1'b0;
		vf_set = '0; vf_wm = '0; vf_lwe = 1'b0; vf_lv = 1'b0; vf_dwe = 1'b0; vf_dv = 1'b0;
		ve_set = '0; ve_wm = '0; ve_lwe = 1'b0; ve_lv = 1'b0; ve_dwe = 1'b0; ve_dv = 1'b0;
		vs_set = '0; vs_wm = '0; vs_lwe = 1'b0; vs_lv = 1'b0; vs_dwe = 1'b0; vs_dv = 1'b0;
		m1s_n = m1_stale; mts_n = mt_stale; m2ss_n = m2_ss;

		//--------------------------------------------------------------
		// stage records
		//--------------------------------------------------------------
		if (adv_ag) begin
			m1.v       <= dm_req;
			m1.a       <= dm_va;
			m1.mem     <= dm_mem;
			m1.msz     <= dm_msz;
			m1.fc      <= dm_fc;
			m1.lock    <= dm_lock;
			m1.locke   <= dm_locke;
			m1.smode   <= dm_super;
			m1.noalloc <= dm_noalloc;
			m1.ack     <= dm_ack;
			m1_tp      <= transp_of(dm_va, dm_super, dm_fc, dm_ack);
			m1_aset    <= tc_p ? dm_va[16:13] : dm_va[15:12];
			// (the tag read of this edge is DT's when it looks up again)
			m1s_n      = (dw && dw_src != 2'd0 && dw_set == dm_va[10:4]) || !e_quiet || tk_relook;
		end
		else begin
			if (adv_dc1) m1.v <= 1'b0;
			m1_tp <= transp_of(m1.a, m1.smode, m1.fc, m1.ack);
			m1_aset <= tc_p ? m1.a[16:13] : m1.a[15:12];
			if ((dw && dw_src != 2'd0 && dw_set == m1.a[10:4]) || !e_quiet || tk_relook)
				m1s_n = 1'b1;
			// held, the RAMs and the ATC read DC1's own set again every
			// clock (la_alt): a clock with the engine quiet and no write to
			// that set makes the lookup good again from the next one
			else if (!steal && !sn_rset) m1s_n = 1'b0;
		end

		// DC1 -> DT: the lookup registered.  DT's lookup is stale after an
		// engine write (tags, the ATC) or a fill of its set; it then reads
		// its own address again (tk_relook: the tag RAM and the ATC's
		// second port, DC1 waiting) until a quiet clock has read it fresh
		begin
			logic stl;
			if (adv_dc1) begin
				mt      <= m1;
				mt_hv   <= atc_hvd;
				mt_wpa  <= atc_wpad;
				mt_ent  <= atc_entd;
				mt_tq   <= tq_a;
				// (formed again, from the pins and registers as they will be
				// in DT's clock: m1_tp is DC1's)
				mt_tp   <= transp_of(m1.a, m1.smode, m1.fc, m1.ack);
				mt_tt0  <= ttr_hit(dtt0, m1.a, m1.smode);
				for (int p = 0; p < 4; p++)
					mt_bsel[p] <= (3'(p) < nbytes(m1.msz)) ?
					              3'({1'b0, m1.a[1:0]} + nbytes(m1.msz) - 3'd1 - 3'(p)) : 3'd7;
				mt_tt1  <= ttr_hit(dtt1, m1.a, m1.smode);
				mt_aset <= m1_aset;
				stl = (dw && dw_src != 2'd0 && dw_set == m1.a[10:4]) || !e_quiet;
			end
			else begin
				if (adv_dt) mt.v <= 1'b0;
				mt_tp   <= transp_of(mt.a, mt.smode, mt.fc, mt.ack);
				mt_tt0  <= ttr_hit(dtt0, mt.a, mt.smode);
				mt_tt1  <= ttr_hit(dtt1, mt.a, mt.smode);
				mt_aset <= tc_p ? mt.a[16:13] : mt.a[15:12];
				stl = mt_stale;
				if ((dw && dw_src != 2'd0 && dw_set == mt.a[10:4]) || !e_quiet) stl = 1'b1;
				else if (tk_rdok) begin
					mt_hv  <= atc_hvd;
					mt_wpa <= atc_wpad;
					mt_ent <= atc_entd;
					mt_tq  <= tq_a;
					stl    = 1'b0;
				end
			end
			mts_n = stl;
			tk_rdok  <= tk_relook && !steal && !sn_rset && e_quiet &&
			            !(dw && dw_src != 2'd0 && dw_set == mt.a[10:4]);
		end

		// DT's data read: the data RAM reads, at the next edge, the set of
		// the access DT holds then.  That read is good when the pipeline,
		// not the engine or the snoop, owns the port, the engine is quiet,
		// no write lands at that edge, and no older store to the line is
		// still in flight (DT's, if a new access enters, DC2's, EX's, WB's).
		// A load waits in DT (hold_dt, a register) until it has such a read.
		// (formed for both candidates, DC1's access entering and DT's own
		// staying, from registers; the advance picks last)
		begin
			logic hz1, hzt, dk1, dkt, h1, ht;
			hz1 = (dw && dw_set == m1.a[10:4]) || !e_quiet || sn_frz || st_at(mt, tk_split, m1.a[11:4]) ||
			      st_at(m2.r, m2.split, m1.a[11:4]) || st_at(m3.r, m3.split, m1.a[11:4]) ||
			      st_at(m4.r, m4.split, m1.a[11:4]);
			hzt = (dw && dw_set == mt.a[10:4]) || !e_quiet || sn_frz ||
			      st_at(m2.r, m2.split, mt.a[11:4]) || st_at(m3.r, m3.split, mt.a[11:4]) ||
			      st_at(m4.r, m4.split, mt.a[11:4]);
			dk1 = !sn_rset && !steal && !hz1;
			dkt = !sn_rset && !steal && !hzt;
			h1  = m1.v && (((m1.mem == M_LD || m1.mem == M_RMW) && !dk1) ||
			               (dw && dw_src != 2'd0 && dw_set == m1.a[10:4]) || !e_quiet);
			ht  = mt.v && (((mt.mem == M_LD || mt.mem == M_RMW) && !dkt) ||
			               (dw && dw_src != 2'd0 && dw_set == mt.a[10:4]) || !e_quiet ||
			               (mt_stale && !tk_rdok));
			tk_dok  <= adv_dc1 ? dk1 : dkt;
			hold_dt <= adv_dc1 ? h1 : (!adv_dt && ht);
		end

		// DT -> DC2
		if (adv_dt) begin
			m2.r     <= mt;
			m2.x     <= x_tk;
			m2.split <= tk_split;
			m2.x1    <= '0;
			m2.fast  <= tk_fast;
			m2.sfast <= tk_sfast;
			m2ss_n   = st_simp(mt, x_tk, tk_split);
		end
		else begin
			if (adv_dc2) m2.r.v <= 1'b0;
		end

		if (adv_dc2) begin
			m3 <= m2;
			e_dc2_done <= 1'b0;
		end
		else if (adv_ex) m3.r.v <= 1'b0;

		if (adv_ex) begin
			m4    <= m3;
			m4_fq <= !m3.split && m3.x.ok && !m3.x.flt && m3.x.hit && m3.x.cm == 2'b01 &&
			         !m3.r.lock && (m3.r.msz != SZ_Q);
			m4_fsb <= fsb_of(m3);
		end
		else if (adv_wb) m4.r.v <= 1'b0;

		if (adv_wb) e_wst_done <= 1'b0;

		//--------------------------------------------------------------
		// the WB fast store
		//--------------------------------------------------------------
		if (wb_fast && !sn_frz) begin
			dw      <= 1'b1;
			dw_set  <= m4.x.pa[10:4];
			dw_way  <= m4.x.way;
			dw_be   <= wbf_be;
			dw_src  <= 2'd0;
			dw_word <= wbf_word;
			vf_set = m4.x.pa[10:4]; vf_wm[m4.x.way] = 1'b1; vf_dwe = 1'b1; vf_dv = 1'b1;
			// answered this cycle: kept only if WB does not move
			st_rdy_r   <= !adv_wb;
			st_fault_r <= 1'b0;
			e_wst_done <= !adv_wb;
		end
		// (the store into the store buffer: the buffer's push, below)
		if (wb_fsb) begin
			st_rdy_r   <= !adv_wb;
			st_fault_r <= 1'b0;
			e_wst_done <= !adv_wb;
		end
		if (wb_fs && !sn_frz) begin
			dw      <= 1'b1;
			dw_set  <= m4.x.pa[10:4];
			dw_way  <= m4.x.way;
			dw_be   <= bmask(m4.r.a[3:0], wfs_n0);
			dw_src  <= 2'd0;
			dw_word <= lanes32(wfs_acc, m4.r.a[1:0]);
			vf_set = m4.x.pa[10:4]; vf_wm[m4.x.way] = 1'b1; vf_dwe = 1'b1; vf_dv = 1'b1;
			wfs_p1  <= 1'b1;
		end
		if (wfs_p1 && !sn_frz) begin
			wfs_p1 <= 1'b0;
			if (m4.x1.hit) begin
				dw      <= 1'b1;
				dw_set  <= m4.x1.pa[10:4];
				dw_way  <= m4.x1.way;
				dw_be   <= bmask(4'd0, nbytes(m4.r.msz) - wfs_n0);
				dw_src  <= 2'd0;
				dw_word <= lanes32(wfs_acc << (8 * wfs_n0), 2'd0);
				vf_set = m4.x1.pa[10:4]; vf_wm[m4.x1.way] = 1'b1; vf_dwe = 1'b1; vf_dv = 1'b1;
				st_rdy_r   <= !adv_wb;
				st_fault_r <= 1'b0;
				e_wst_done <= !adv_wb;
			end
			else begin
				// a snoop took the second line in between: the engine
				// writes the second part (as its own access would)
				e_part <= 1'b1;
				e_va   <= {m4.r.a[31:4] + 28'd1, 4'd0};
				e_x    <= m4.x1;
				e_n    <= nbytes(m4.r.msz) - wfs_n0;
				e_acc  <= wfs_acc << (8 * wfs_n0);
				bo_err <= 1'b0;
				e_st   <= E_W_DEC;
			end
		end

		//--------------------------------------------------------------
		// engine (frozen while the snooper has the cache)
		//--------------------------------------------------------------
		if (!sn_frz)
		case (e_st)
		E_IDLE: begin
			steal <= 1'b0;
			if (wfs_p1)
				;                       // a split WB store's second part next
			else if (mt_v && !mt_done)
				e_st <= E_M_START;
			else if (wb_slow)
				e_st <= E_W_START;
			else if ((dc2_slow || (e_job && m2.r.v && !e_dc2_done)) && !kill_now)
				e_st <= E_S_START;
			// (an instruction-side search only once every instruction before
			// the fetch is done and its stores are out: the fetch runs ahead,
			// and a descriptor an older store changes must be read as written)
			else if (iw_req && !iw_done && iw_ok && sb_empty) begin
				tw_va <= iw_va; tw_s <= iw_fc2; tw_wr <= 1'b0; tw_pt <= 1'b0; tw_i <= 1'b1;
				e_ret <= E_I_DONE;
				e_st  <= E_TW_START;
			end
		end

		//==============================================================
		// DC2 slow path
		//==============================================================
		E_S_START: begin
			e_kill <= 1'b0;
			e_job  <= 1'b1;
			e_part <= 1'b0;
			e_va   <= m2.r.a;
			e_x    <= m2.x;
			e_acc  <= '0;
			e_flt  <= 1'b0;
			e_fromline <= 1'b0;
			e_fillhit  <= 1'b0;
			e_falloc   <= 1'b0;
			e_n    <= m2.split ? 3'(5'd16 - {1'b0, m2.r.a[3:0]}) : nbytes(m2.r.msz);
			sp_dw  <= 1'b0;
			sp_job <= m2.sfast;
			if (m2.sfast) begin
				// a load across a line, its first part a hit held in DC2's
				// copy: look the next line up (translation, tags and data
				// at once through the stolen ports)
				steal  <= 1'b1;
				st_la  <= {m2.r.a[31:4] + 28'd1, 4'd0};
				st_fc2 <= m2.r.smode;
				st_set <= m2.r.a[10:4] + 7'd1;
				e_st   <= E_SP_W;
			end
			else
				e_st   <= m2.x.ok ? E_S_ACT : E_S_XL;
		end
		E_SP_W: begin                   // the RAMs register the stolen address
			lk_hit <= atc_hit; lk_e <= atc_e; lk_hv <= atc_hv;
			lk_tt0 <= ttr_hit(dtt0, {m2.r.a[31:4] + 28'd1, 4'd0}, m2.r.smode);
			lk_tt1 <= ttr_hit(dtt1, {m2.r.a[31:4] + 28'd1, 4'd0}, m2.r.smode);
			e_st   <= E_SP_2;
		end
		E_SP_3: begin
			logic [2:0] n0;
			n0 = 3'(5'd16 - {1'b0, m2.r.a[3:0]});
			// (the first part's translation back, whichever way: the
			// second's goes to m2.x1 or is looked up again, and its
			// register's enable stays off the decision below)
			e_x <= m2.x;
			if (!e_kill && m2.sfast && m2.r.mem == M_ST && e_x.ok && !e_x.flt && !e_x.walk &&
			    e_x.hit && e_x.cm == 2'b01) begin
				// a copyback store across a line: both parts hit, nothing
				// to read; WB writes them
				m2.x1 <= e_x;
				e_st  <= E_S_DONE;
			end
			else if (!e_kill && m2.sfast && m2.r.mem == M_LD && e_x.ok && !e_x.flt && !e_x.walk &&
			    e_x.hit && !e_x.cm[1] &&
			    !hz && !st_line(m3, e_x.pa[31:4]) && !st_line(m4, e_x.pa[31:4]) &&
			    !sp_dw && !(dw && (dw_set == m2.r.a[10:4] || dw_set == st_set))) begin
				// the first part from DC2's copy, the rest from the stolen read
				// (registered here, merged in E_SP_4)
				etk_l <= dq_rn[e_x.way]; etk_oq <= etk_o; etk_nq <= etk_n;
				m2.x1 <= e_x;
				e_st  <= E_SP_4;
			end
			else begin
				steal  <= 1'b0;
				sp_job <= 1'b0;
				e_st   <= m2.x.ok ? E_S_ACT : E_S_XL;
			end
		end
		E_SP_4: begin
			logic [2:0] n0;
			n0 = 3'(5'd16 - {1'b0, m2.r.a[3:0]});
			// (DC2's bytes are fast_data's: its low bytes, past the line,
			// give way to the stolen read's)
			e_acc <= (fast_data & ~((32'd1 << (8 * (nbytes(m2.r.msz) - n0))) - 32'd1)) | etk_q;
			e_st  <= E_S_DONE;
		end
		E_SP_2: begin
			xres_t x;
			x  = xlate_t({m2.r.a[31:4] + 28'd1, 4'd0}, m2.r.smode, (m2.r.mem != M_LD),
			           fc_tt(m2.r.fc) == TT_ALT, lk_tt0, lk_tt1, lk_hit, lk_e,
			           tq_a[0], tq_a[1], tq_a[2], tq_a[3], lve_q);
			// the second part's translation and lookup, registered (the
			// decision and the data next cycle; the RAM keeps reading the
			// stolen set)
			e_x  <= x;
			e_st <= E_SP_3;
		end
		E_S_XL: begin
			// look the current part up (translation and tags) via the
			// stolen ports; one cycle for the RAM and ATC reads
			steal  <= 1'b1;
			st_la  <= e_va;
			st_fc2 <= m2.r.smode;
			st_set <= e_va[10:4];
			e_st   <= E_S_XLW;
		end
		E_S_XLW: begin                 // the RAMs register the stolen address
			lk_hit <= atc_hit; lk_e <= atc_e; lk_hv <= atc_hv;
			lk_tt0 <= ttr_hit(dtt0, e_va, m2.r.smode);
			lk_tt1 <= ttr_hit(dtt1, e_va, m2.r.smode);
			e_st   <= E_S_XL2;
		end
		E_S_XL2: begin
			xres_t x;
			// after a search: its result, not the ATC's (a failed search
			// leaves no entry, nor does one in the no-allocate mode)
			x = xlate_t(e_va, m2.r.smode, (m2.r.mem != M_LD), fc_tt(m2.r.fc) == TT_ALT,
			          lk_tt0, lk_tt1, lk_hit || tw_use,
			          tw_use ? tw_e : lk_e, tq_a[0], tq_a[1], tq_a[2], tq_a[3], lve_q);
			if (tw_use && tw_fail) begin
				x.ok  = 1'b1;
				x.flt = 1'b1;
				x.hit = 1'b0;
				x.why = tw_cause;
			end
			tw_use <= 1'b0;
			if (m2.r.ack != 2'd0) begin
				x    = '0;
				x.ok = 1'b1;
				x.pa = e_va;
				x.cm = 2'b10;
			end
			if (x.walk && e_kill) begin
				// the uop was discarded: no search for it
				steal  <= 1'b0;
				e_job  <= 1'b0;
				tw_use <= 1'b0;
				e_st   <= E_IDLE;
			end
			else if (x.walk && (st_older || dm_older)) begin
				// a table search only for the oldest instruction: older
				// stores first (they need the engine at WB), and nothing
				// older may discard the access after its search has set
				// history bits or met a bus error: retry
				steal <= 1'b0;
				e_st  <= E_IDLE;
			end
			else if (x.walk) begin
				tw_va <= e_va; tw_s <= m2.r.smode; tw_wr <= (m2.r.mem != M_LD);
				tw_pt <= 1'b0; tw_i <= 1'b0;
				tw_upd <= lk_hit;           // a first write: the entry's M
				tw_hw  <= lk_hv;            // (its way: the install rewrites it)
				e_ret <= E_S_XL;            // look up again after the walk
				e_st  <= E_TW_START;
			end
			else begin
				e_x  <= x;
				e_st <= E_S_ACT;
			end
		end
		E_S_ACT: if (e_kill) begin
			// the uop was discarded (no bus operation is in flight here)
			steal <= 1'b0;
			e_job <= 1'b0;
			e_st  <= E_IDLE;
		end
		else begin
			logic cach, alloc, rd, move16;
			move16 = (m2.r.msz == SZ_Q);
			cach   = dc_en && !e_x.cm[1] && !m2.r.lock && (m2.r.ack == 2'd0) && !move16;
			alloc  = cach && !m2.r.noalloc && !dc_nad;
			rd     = (m2.r.mem == M_LD) || (m2.r.mem == M_RMW);
			if (e_x.flt) begin
				e_flt   <= 1'b1;
				// FA is the operand's first byte even when a later part
				// faulted; MA marks an ATC fault on the second page
				e_faddr <= m2.r.a;
				e_fslw  <= mk_fslw(e_x.why, m2.r.lock, m2_rw, m2.r.msz, m2.r.fc) |
				           (e_part ? FSLW_MA : 32'd0);
				e_st    <= E_S_DONE;
			end
			else if (e_fillhit) begin
				// the fill installed the line (f_way) and it is in bo_line:
				// the operand from it, the part now a hit in that way
				e_acc     <= (e_acc << (8 * e_n)) | etk_b;
				e_fillhit <= 1'b0;
				e_x.hit   <= 1'b1;
				e_x.way   <= f_way;
				e_st      <= E_S_NEXT;
			end
			else if (e_fromline) begin
				// an inhibited fill delivered the line: the operand from it
				e_acc <= (e_acc << (8 * e_n)) | etk_b;
				e_fromline <= 1'b0;
				e_x.hit <= 1'b0;
				e_x.cm  <= 2'b11;
				e_st    <= E_S_NEXT;
			end
			else if (rd && !e_part && (m2.r.ack == 2'd0) && (e_x.cm[1] || m2.r.lock) && dm_older) begin
				// a serialized read (a noncachable serialized page, or a
				// locked access) waits until every earlier instruction has
				// completed: pending writes are on the bus first, and
				// nothing older can abort the instruction after the read
				// (MC68040UM 4.3.2, 7.7).  So does any cache-inhibited
				// read: the 68040 never reads a device on a path an older
				// instruction then discards (a mispredicted branch, a
				// serializing MOVEC) -- such a read cannot be taken back.
				// Retry from the start.
				steal <= 1'b0;
				e_st  <= E_IDLE;
			end
			else if (move16 && m2.r.mem == M_LD && e_x.hit && dc_en) begin
				// MOVE16 source hit: read from the cache, no allocation
				e_st <= E_S_RD;
			end
			else if (move16 && m2.r.mem == M_ST) begin
				// MOVE16 destination: a write hit invalidates the line (the
				// write replaces all of it) -- unless the data cache is
				// disabled, which leaves its contents alone (5.3)
				if (e_x.hit && dc_en) begin
					ve_set = e_va[10:4]; ve_wm[e_x.way] = 1'b1;
					ve_lwe = 1'b1; ve_lv = 1'b0; ve_dwe = 1'b1; ve_dv = 1'b0;
					e_x.hit <= 1'b0;
				end
				e_st <= E_S_NEXT;
			end
			else if (m2.r.mem == M_ST) begin
				// a store: a copyback miss allocates the line for WB
				if (alloc && e_x.cm == 2'b01 && !e_x.hit && !e_falloc) begin
					f_set  <= e_va[10:4];
					f_tag  <= e_x.pa[31:11];
					f_way  <= victim(lve_q, rrc, dc_foc);
					e_sret <= E_S_XL;
					e_st   <= E_FILL;
				end
				else e_st <= E_S_NEXT;
			end
			else if (cach && e_x.hit && rd) begin
				// older stores to this line must reach the RAM first: give
				// the engine back to them and retry (the line of the part
				// being read: a split's second part has its own)
				if (e_part ? (st_line(m3, e_x.pa[31:4]) || st_line(m4, e_x.pa[31:4])) : hz) begin
					steal <= 1'b0; e_st <= E_IDLE;
				end
				else if (e_x.cm == 2'b00 && ld_q[e_x.way]) begin
					// a write-through read hit on a dirty line: the line is
					// pushed and stays valid (MC68060UM 5.5.3), then read
					f_set   <= e_va[10:4];
					f_way   <= e_x.way;
					pv_keep <= 1'b1;
					e_sret  <= E_S_ACT;
					e_st    <= E_PUSHV;
				end
				else e_st <= E_S_RD;
			end
			else if (alloc && rd && !e_x.hit &&
			         (e_part ? (st_line(m3, e_x.pa[31:4]) || st_line(m4, e_x.pa[31:4])) : hz)) begin
				// an older store to this line has not reached memory yet
				steal <= 1'b0;
				e_st  <= E_IDLE;
			end
			else if (alloc && rd && !e_x.hit) begin
				f_set  <= e_va[10:4];
				f_tag  <= e_x.pa[31:11];
				f_way  <= victim(lve_q, rrc, dc_foc);
				e_sret <= E_S_XL;
				e_st   <= E_FILL;
			end
			else if (rd) begin
				// cache inhibited, no-allocate miss, locked, MOVE16 miss,
				// IACK: a matching line is pushed (if dirty) and invalidated
				// first (5.4.2, 5.7.2); the bus access goes in program order.
				// With the data cache disabled (CACR.EDC clear, CDIS) the
				// cache's contents are left alone (5.3)
				if (e_x.hit && dc_en && (m2.r.lock || e_x.cm[1] || move16)) begin
					f_set  <= e_va[10:4];
					f_way  <= e_x.way;
					pv_keep <= 1'b0;
					e_sret <= E_S_ACT;
					e_x.hit <= 1'b0;
					e_st   <= E_PUSHV;
				end
				else if ((m3.r.v && m3.r.mem != M_LD) || (m4.r.v && m4.r.mem != M_LD)) begin
					// older stores go to the bus first (program order): the
					// engine is theirs until they are done, then retry
					steal <= 1'b0;
					e_st  <= E_IDLE;
				end
				else begin
					bo.line  <= move16;
					bo.rd    <= 1'b1;
					bo.tt    <= move16 ? TT_MOVE16 : fc_tt(m2.r.fc);
					bo.tm    <= fc_tm(m2.r.fc);
					bo.upa   <= e_x.upa;
					bo.ci    <= e_x.cm[1];
					bo.lock  <= m2.r.lock;
					bo.locke <= 1'b0;
					bo.ack   <= m2.r.ack;
					bo.tln   <= 2'd0;
					bo_pa    <= move16 ? {e_x.pa[31:4], 4'd0} : e_x.pa;
					bo_left  <= move16 ? 3'd4 : e_n;
					e_ret    <= E_S_BUSD;
					e_st     <= E_BUS;
				end
			end
			else e_st <= E_S_NEXT;
		end
		E_S_RD: begin
			// read the line through the stolen data port
			steal  <= 1'b1;
			st_set <= e_va[10:4];
			e_st   <= E_S_RDW;
		end
		E_S_RDW: e_st <= E_S_RD2;
		E_S_RD2: begin
			// (the operand's bytes registered here, merged in E_S_RD3)
			if (m2.r.msz == SZ_Q) begin
				mv_line <= dq_rn[e_x.way];
				e_st    <= E_S_NEXT;
			end
			else begin
				etk_l <= dq_rn[e_x.way]; etk_oq <= etk_o; etk_nq <= etk_n;
				e_st  <= E_S_RD3;
			end
		end
		E_S_RD3: begin
			e_acc <= (e_acc << (8 * e_n)) | etk_q;
			e_st  <= E_S_NEXT;
		end
		E_S_BUSD: begin
			// the bus read finished
			// (an acknowledge cycle's TEA is no access error: IACK takes the
			// spurious vector, BKPT the illegal instruction exception)
			if (bo_err && (m2.r.ack == 2'd0)) begin
				// MA: the fault on the second or later access of a
				// misaligned operand (8.4.4.3), the next line's part or a
				// later bus cycle of this one; FA stays the first byte
				e_flt   <= 1'b1;
				e_faddr <= m2.r.a;
				// (TTR: the access is mapped by a TTR, 8.4.4 -- a bus
				// error too; the translation's why carries it; issue 2)
				e_fslw  <= mk_fslw(FSLW_RE, m2.r.lock, m2_rw, m2.r.msz, m2.r.fc) |
				           ((e_part || bo_later) ? FSLW_MA : 32'd0) | (e_x.why & FSLW_TTR);
				e_st    <= E_S_DONE;
			end
			else begin
				if (m2.r.ack == 2'd1)
					// IACK: [7:0] vector, [8] AVEC, [9] TEA = spurious
					e_acc <= {22'd0, bo_err, bo_avec && !bo_err, bo_rd[7:0]};
				else if (m2.r.msz == SZ_Q)
					mv_line <= bo_line;
				else
					e_acc <= (e_acc << (8 * e_n)) | bo_rd;
				e_st <= E_S_NEXT;
			end
		end
		E_S_NEXT: begin
			if (m2.split && !e_part) begin
				m2.x   <= e_x;                 // part 0, for WB
				e_part <= 1'b1;
				e_falloc <= 1'b0;
				e_va   <= {m2.r.a[31:4] + 28'd1, 4'd0};
				e_n    <= nbytes(m2.r.msz) - e_n;
				e_st   <= E_S_XL;
			end
			else begin
				if (m2.split) m2.x1 <= e_x; else m2.x <= e_x;
				e_st <= E_S_DONE;
			end
		end
		E_S_DONE: begin
			e_job      <= 1'b0;
			e_dc2_done <= !e_kill;
			e_ldata    <= e_acc;
			steal      <= 1'b0;
			e_st       <= E_IDLE;
		end

		//==============================================================
		// WB store through the engine (write-through, cache-inhibited,
		// locked, split, MOVE16)
		//==============================================================
		E_W_START: begin
			e_part <= 1'b0;
			e_acc  <= lalign(st_data, m4.r.msz);
			e_va   <= m4.r.a;
			e_x    <= m4.x;
			e_n    <= m4.split ? 3'(5'd16 - {1'b0, m4.r.a[3:0]}) : nbytes(m4.r.msz);
			bo_err <= 1'b0;
			e_st   <= E_W_DEC;
		end
		E_W_DEC: begin
			// the part's decisions, registered for E_W_PART: push the line
			// it hits first (locked, cache-inhibited: invalidated; a dirty
			// line under write-through: kept), update the line, a copyback
			// write (no bus write), through the store buffer
			logic upd;
			upd     = e_x.hit && dc_en && !e_x.cm[1] && !m4.r.lock && (m4.r.msz != SZ_Q);
			wd_push <= e_x.hit && dc_en && (m4.r.lock || e_x.cm[1]);
			wd_wtp  <= upd && e_x.cm == 2'b00 && ld_q[e_x.way];
			wd_upd  <= upd;
			wd_cb   <= upd && e_x.cm == 2'b01;
			wd_sb   <= cacr[29] && !m4.r.lock && (m4.r.msz != SZ_Q) && (m4.r.ack == 2'd0) &&
			           (e_x.cm == 2'b00 || e_x.cm == 2'b11);
			wd_be   <= bmask(e_va[3:0], e_n);
			e_st    <= E_W_PART;
		end
		E_W_PART: begin
			logic [15:0] be; logic upd;
			be  = wd_be;
			upd = wd_upd;
			if (wd_push) begin
				// a locked write bypasses the cache (5.2), a cache-inhibited
				// one too (5.4.2, 5.7.2): a line it hits is pushed if dirty
				// and invalidated before the bus write -- unless the data
				// cache is disabled, which leaves its contents alone (5.3)
				f_set   <= e_va[10:4];
				f_way   <= e_x.way;
				pv_keep <= 1'b0;
				e_x.hit <= 1'b0;
				e_sret  <= E_W_DEC;
				e_st    <= E_PUSHV;
			end
			else if (wd_wtp) begin
				// a write-through write hit on a dirty line: the line is
				// pushed first, staying valid (MC68060UM 5.5.4)
				f_set   <= e_va[10:4];
				f_way   <= e_x.way;
				pv_keep <= 1'b1;
				e_sret  <= E_W_DEC;
				e_st    <= E_PUSHV;
			end
			else begin
			if (upd) begin
				dw      <= 1'b1;
				dw_set  <= e_va[10:4];
				dw_way  <= e_x.way;
				dw_be   <= be;
				dw_src  <= 2'd0;
				dw_word <= lanes32(e_acc, e_va[1:0]);
				if (e_x.cm == 2'b01)
					if (be != 16'd0) begin
						ve_set = e_va[10:4]; ve_wm[e_x.way] = 1'b1; ve_dwe = 1'b1; ve_dv = 1'b1;
					end
			end
			if (wd_cb) begin
				e_st <= E_W_DONE;            // copyback: no bus write
			end
			else begin
				bo.line  <= (m4.r.msz == SZ_Q);
				bo.rd    <= 1'b0;
				bo.tt    <= (m4.r.msz == SZ_Q) ? TT_MOVE16 : fc_tt(m4.r.fc);
				bo.tm    <= fc_tm(m4.r.fc);
				bo.upa   <= e_x.upa;
				bo.ci    <= e_x.cm[1];
				bo.lock  <= m4.r.lock;
				bo.locke <= m4.r.locke && (!m4.split || e_part);
				bo.ack   <= m4.r.ack;
				bo.tln   <= 2'd0;
				bo_pa    <= (m4.r.msz == SZ_Q) ? {e_x.pa[31:4], 4'd0} : e_x.pa;
				bo_left  <= (m4.r.msz == SZ_Q) ? 3'd4 : e_n;
				bo_wd    <= e_acc;
				bo_line  <= mv_line;
				e_ret    <= E_W_DONE;
				// write-through and noncachable imprecise writes go through
				// the store buffer when it is enabled (5.9); locked ones,
				// MOVE16 and acknowledge cycles never do
				e_st     <= wd_sb ? E_SB : E_BUS;
			end
			end
		end
		E_SB: begin
			// one bus cycle's piece a clock into the store buffer
			if (sb_room) begin
				bo_wd   <= bo_wd << (8 * bo_np);
				bo_pa   <= bo_pa + {29'd0, bo_np};
				bo_left <= bo_left - bo_np;
				if (bo_left == bo_np) e_st <= E_W_DONE;
			end
		end
		E_W_DONE: begin
			// (the LPSTOP broadcast completes with TA or TEA alike: the
			// processor enters the low-power stop either way, 7.8.2.1)
			if (bo_err && m4.r.ack != 2'd3) begin
				st_rdy_r     <= 1'b1;
				st_fault_r   <= 1'b1;
				st_fslw    <= mk_fslw(FSLW_WE, m4.r.lock, (m4.r.mem == M_RMW) ? 2'b11 : 2'b01,
				                      m4.r.msz, m4.r.fc) |
				              (((m4.split && e_part) || bo_later) ? FSLW_MA : 32'd0) |
				              (e_x.why & FSLW_TTR);
				st_faddr   <= m4.r.a;
				e_wst_done <= 1'b1;
				e_st       <= E_IDLE;
			end
			else if (m4.split && !e_part) begin
				e_part <= 1'b1;
				e_acc  <= e_acc << (8 * e_n);
				e_va   <= {m4.r.a[31:4] + 28'd1, 4'd0};
				e_x    <= m4.x1;
				e_n    <= nbytes(m4.r.msz) - e_n;
				e_st   <= E_W_DEC;
			end
			else begin
				st_rdy_r     <= 1'b1;
				st_fault_r   <= 1'b0;
				e_wst_done <= 1'b1;
				e_st       <= E_IDLE;
			end
		end

		//==============================================================
		// maintenance at WB
		//==============================================================
		E_M_START: begin
			case (mt_op)
				MT_PLPA: begin
					// PLPA: look the address up (the D-ATC, through the
					// stolen port), in the space of the DFC
					steal  <= 1'b1;
					st_la  <= mt_addr;
					st_fc2 <= mt_fc[2];
					mt_fault <= 1'b0;
					e_st   <= E_M_PLW;
				end
				MT_PFLUSH: begin
					// both ATCs; by page, or all (N: spare global entries)
					atc_fng       <= mt_ng;
					iatc_flush_ng <= mt_ng;
					if (mt_scope == 2'd3) begin
						atc_fall       <= 1'b1;
						iatc_flush_all <= 1'b1;
						e_st <= E_M_DONE;
					end
					else begin
						steal  <= 1'b1;
						st_la  <= mt_addr;
						st_fc2 <= mt_fc[2];
						iatc_flush_la  <= mt_addr;
						iatc_flush_fc2 <= mt_fc[2];
						e_st <= E_M_PG;
					end
				end
				default: begin
					// CINV / CPUSH
					if (!mt_caches[0]) e_st <= E_M_IC;
					else if (mt_op == MT_CINV && mt_scope == 2'd3) begin
						clr_i <= '0;
						e_st  <= E_M_CLR;
					end
					else begin
						// scan one set (line), or every set (page, all)
						ms_i <= (mt_scope == 2'd1) ? {mt_addr[10:4], 2'b00} : 9'd0;
						e_st <= E_M_SCAN;
					end
				end
			endcase
		end
		E_M_CLR: begin
			// every line invalid, a set a cycle (CINVA; and after reset)
			ve_set = clr_i; ve_wm = 4'hF;
			ve_lwe = 1'b1; ve_lv = 1'b0;
			clr_i <= clr_i + 7'd1;
			if (clr_i == 7'd127) begin
				clr_rst <= 1'b0;
				e_st    <= clr_rst ? E_IDLE : E_M_IC;
			end
		end
		E_M_PG: begin
			// the stolen ATC read of the page is valid now: flush it.  The
			// flush pulse compares the lookup port in the NEXT cycle, so the
			// port stays stolen (E_M_DONE / the walk's end release it)
			atc_fpage       <= 1'b1;
			iatc_flush_page <= 1'b1;
			e_st <= E_M_DONE;
		end
		E_M_PLW: begin                  // the ATC reads the stolen address
			lk_hit <= atc_hit; lk_e <= atc_e; lk_hv <= atc_hv;
			e_st   <= E_M_PL2;
		end
		E_M_PL2: begin
			// PLPA (MC68060UM 4.7.3): a transparent translation (a TTR of
			// the space, or no paged translation: the TCR's defaults)
			// answers the address itself; a D-ATC hit with the access's
			// rights its translation; anything else a table search, which
			// loads the ATC (program space: the I-ATC) and gives the page.
			// The faults are the access's: TTR/default write protection,
			// supervisor or write protection, invalid descriptors.
			// PLPA works on the data side whatever the DFC (4.7.3, appendix
			// D: "Data TTR hit ... Data ATC hit ... update Data ATC"); the
			// DFC gives the space's S bit.  (WinUAE uses the instruction
			// TTRs and ATC for DFC 2 and 6.)
			logic t0, t1, it0, it1, hitt, wpt, isp;
			isp = 1'b0;
			t0  = ttr_hit(dtt0, mt_addr, mt_fc[2]);
			t1  = ttr_hit(dtt1, mt_addr, mt_fc[2]);
			it0 = ttr_hit(itt0, mt_addr, mt_fc[2]);
			it1 = ttr_hit(itt1, mt_addr, mt_fc[2]);
			hitt = isp ? (it0 || it1) : (t0 || t1);
			wpt  = isp ? (it0 ? itt0[2] : itt1[2]) : (t0 ? dtt0[2] : dtt1[2]);
			mt_pa   <= mt_addr;
			mt_fslw <= mk_fslw(32'd0, 1'b0, mt_wr ? 2'b01 : 2'b10, SZ_L, mt_fc);
			if (hitt) begin
				if (mt_wr && wpt) begin
					mt_fault <= 1'b1;
					mt_fslw  <= mk_fslw(FSLW_TTR | FSLW_WP, 1'b0, 2'b01, SZ_L, mt_fc);
				end
				e_st <= E_M_DONE;
			end
			else if (!tc_e) begin
				if (mt_wr && !isp && tc[5]) begin
					mt_fault <= 1'b1;
					mt_fslw  <= mk_fslw(FSLW_WP, 1'b0, 2'b01, SZ_L, mt_fc);
				end
				e_st <= E_M_DONE;
			end
			else if (!isp && lk_hit && lk_e.r && !lk_e.b && !(mt_wr && !lk_e.m)) begin
				mt_pa <= {lk_e.pa[19:1], tc_p ? mt_addr[12] : lk_e.pa[0], mt_addr[11:0]};
				if ((lk_e.s && !mt_fc[2]) || (mt_wr && lk_e.w)) begin
					mt_fault <= 1'b1;
					mt_fslw  <= mk_fslw((lk_e.s && !mt_fc[2]) ? FSLW_SP : FSLW_WP, 1'b0,
					                    mt_wr ? 2'b01 : 2'b10, SZ_L, mt_fc);
				end
				e_st <= E_M_DONE;
			end
			else begin
				tw_va <= mt_addr; tw_s <= mt_fc[2]; tw_wr <= mt_wr;
				tw_pt <= 1'b1; tw_i <= isp; tw_upd <= !isp && lk_hit; tw_hw <= lk_hv;
				e_ret <= E_M_PLD;
				e_st  <= E_TW_START;
			end
		end
		E_M_PLD: begin
			// the search's outcome: a failed search faults with its cause,
			// else the page's protection, else the physical address
			mt_pa <= {(tc_p ? {tw_pd[31:13], mt_addr[12]} : tw_pd[31:12]), mt_addr[11:0]};
			if (tw_fail) begin
				mt_fault <= 1'b1;
				mt_fslw  <= mk_fslw(tw_cause, 1'b0, mt_wr ? 2'b01 : 2'b10, SZ_L, mt_fc);
			end
			tw_use <= 1'b0;
			e_st <= E_M_DONE;
		end
		E_M_SCAN: begin
			tset_b <= ms_i[8:2];
			steal  <= 1'b1;
			st_set <= ms_i[8:2];
			sc_set <= ms_i[8:2];
			e_st   <= E_M_SCANW;
		end
		E_M_SCANW: begin
			// (a write this cycle reaches the copy at the write port)
			sc_lv <= lve_q;
			sc_ld <= ld_q;                 // (ld_ra: sc_set)
			e_st <= E_M_SCANW2;
		end
		E_M_SCANW2: begin
			// the set's tags are out: each way's match, registered
			for (int j = 0; j < 4; j++)
				sc_m[j] <= (mt_scope == 2'd3) ||
				           (mt_scope == 2'd1 && tq_b[j] == mt_addr[31:11]) ||
				           (mt_scope == 2'd2 && (tc_p ? tq_b[j][20:2] == mt_addr[31:13]
				                                      : tq_b[j][20:1] == mt_addr[31:12]));
			e_st <= E_M_SCAN2;
		end
		E_M_SCAN2: begin
			logic [6:0] s; logic [1:0] w; logic m;
			s = ms_i[8:2]; w = ms_i[1:0];
			m = sc_lv[w] && sc_m[w];
			// synthesis translate_off
			if (sc_m[w] != ((mt_scope == 2'd3) ||
			                (mt_scope == 2'd1 && tq_b[w] == mt_addr[31:11]) ||
			                (mt_scope == 2'd2 && (tc_p ? tq_b[w][20:2] == mt_addr[31:13]
			                                           : tq_b[w][20:1] == mt_addr[31:12])))) begin
				$display("FAIL: DMU: the scan's registered tag match disagrees (set %0d way %0d)", s, w);
				$finish;
			end
			// synthesis translate_on
			if (m && mt_op == MT_CPUSH && sc_ld[w]) begin
				// push (invalidated with the push, unless CACR.DPI)
				f_set   <= s;
				f_way   <= w;
				pv_keep <= dc_dpi;
				e_sret  <= E_M_SCAN;         // reread the set's tags after
				e_st    <= E_PUSHV;
			end
			else begin
				if (m && !(mt_op == MT_CPUSH && dc_dpi)) begin
					ve_set = s; ve_wm[w] = 1'b1;
					ve_lwe = 1'b1; ve_lv = 1'b0; ve_dwe = 1'b1; ve_dv = 1'b0;
				end
				if ((mt_scope == 2'd1 && w == 2'd3) || ms_i == 9'h1FF) e_st <= E_M_IC;
				else begin
					ms_i <= ms_i + 9'd1;
					e_st <= (w == 2'd3) ? E_M_SCAN : E_M_SCAN2;
				end
			end
		end
		E_M_IC: begin
			// the instruction cache part of CINV/CPUSH
			steal <= 1'b0;
			if (mt_caches[1]) begin
				ic_inv       <= 1'b1;
				ic_inv_scope <= mt_scope;
				ic_inv_pa    <= mt_addr;
				if (ic_inv_done) begin
					ic_inv <= 1'b0;
					e_st   <= E_M_DONE;
				end
			end
			else e_st <= E_M_DONE;
		end
		E_M_DONE: begin
			mt_done <= 1'b1;
			steal   <= 1'b0;
			e_st    <= E_IDLE;
		end

		//==============================================================
		// sub-step: line fill of (f_set, f_way) with tag f_tag, critical
		// long word first.  The victim goes to the push buffer if dirty
		// and is pushed after the fill (4.6.2).  Returns to e_sret.
		//==============================================================
		E_FILL: begin
			if (store_holds(f_set, f_way, m3, m4))
				f_way <= dc_foc ? {1'b0, !f_way[0]} : f_way + 2'd1;   // a store's line
			else begin
				steal  <= 1'b1;
				st_set <= f_set;
				tset_b <= f_set;
				e_st   <= E_FILLW;
			end
		end
		E_FILLW: begin
			// (the victim's valid and dirty bits, registered for E_FILL_W)
			fv_lv <= lve_q[f_way];
			fv_ld <= ld_q[f_way];
			fv_bad <= 1'b0;
			e_st  <= E_FILL_W;
		end
		E_FILL_W: if (fv_bad) e_st <= E_FILLW;   // a snoop came between: read them again
		else begin
			if (fv_lv && fv_ld) begin
				pv_v     <= 1'b1;
				pv_line  <= dq_rn[f_way];
				pv_lpa   <= {tq_b[f_way], f_set};
				pv_dirty <= 1'b1;
				pv_tln   <= f_way;
			end
			ve_set = f_set; ve_wm[f_way] = 1'b1;
			ve_lwe = 1'b1; ve_lv = 1'b0; ve_dwe = 1'b1; ve_dv = 1'b0;
			bo.line <= 1'b1; bo.rd <= 1'b1; bo.tt <= TT_NORMAL;
			bo.tm <= m2.r.smode ? TM_SDATA : TM_UDATA;
			bo.upa <= e_x.upa; bo.ci <= 1'b0; bo.lock <= 1'b0; bo.locke <= 1'b0;
			bo.ack <= 2'd0; bo.tln <= f_way;
			bo_pa   <= {f_tag, f_set, e_va[3:2], 2'b00};
			bo_left <= 3'd4;
			e_ret   <= E_FILL_INS;
			e_st    <= E_BUS;
		end
		E_FILL_INS: begin
			if (!bo_err && !bo_tci) begin
				dw      <= 1'b1;
				dw_set  <= f_set;
				dw_way  <= f_way;
				dw_be   <= 16'hFFFF;
				dw_src  <= 2'd1;
				tw_b    <= 1'b1;
				tset_b  <= f_set;
				tway_b  <= f_way;
				twd_b   <= f_tag;
				ve_set = f_set; ve_wm[f_way] = 1'b1;
				ve_lwe = 1'b1; ve_lv = 1'b1; ve_dwe = 1'b1; ve_dv = 1'b0;
				rrc <= rrc + 2'd1;
			end
			else if (pv_v) begin
				// failed or inhibited fill: the dirty victim is restored
				dw      <= 1'b1;
				dw_set  <= f_set;
				dw_way  <= f_way;
				dw_be   <= 16'hFFFF;
				dw_src  <= 2'd2;
				ve_set = f_set; ve_wm[f_way] = 1'b1;
				ve_lwe = 1'b1; ve_lv = 1'b1; ve_dwe = 1'b1; ve_dv = pv_dirty;
				pv_v <= 1'b0;
			end
			if (bo_err && fill_late_ok) begin
				// a later beat of the line failed, but what the access needs
				// arrived (MC68060UM 7.9.1, 5.7.1): no exception, the line not
				// allocated; a load takes its operand from what was read, a
				// store goes to memory (as for TCI)
				e_fromline <= (m2.r.mem != M_ST);
				e_x.cm     <= 2'b11;
				e_st       <= (m2.r.mem != M_ST) ? E_S_ACT : E_S_NEXT;
			end
			else if (bo_err) begin
				// the operand's long word did not arrive: an access fault --
				// on a later access of a misaligned operand (MA, 8.4.4.3) in
				// the next line's part, or when its first long word arrived
				// and a later one of the same line did not
				e_flt   <= 1'b1;
				e_faddr <= m2.r.a;
				e_fslw  <= mk_fslw(FSLW_RE, 1'b0, m2_rw, m2.r.msz, m2.r.fc) |
				           ((e_part || bo_got[e_va[3:2]]) ? FSLW_MA : 32'd0) | (e_x.why & FSLW_TTR);
				e_st    <= E_S_DONE;
			end
			else if (bo_tci) begin
				// not cachable after all: the operand from the line read,
				// a store of this page writes through
				e_fromline <= (m2.r.mem != M_ST);
				e_x.cm     <= 2'b11;
				e_st       <= (m2.r.mem != M_ST) ? E_S_ACT : E_S_NEXT;
			end
			else if (pv_v) e_st <= E_PUSHV_B;
			else if (m2.r.mem == M_LD && e_sret == E_S_XL) begin
				// a load's own fill: its operand from the line just read
				// (still in bo_line), not a second lookup of that line
				e_fillhit <= 1'b1;
				e_st      <= E_S_ACT;
			end
			else e_st <= e_sret;
			if (!bo_err && !bo_tci && m2.r.mem == M_ST && e_sret == E_S_XL) e_falloc <= 1'b1;
		end

		//==============================================================
		// sub-step: push (f_set, f_way) if dirty and invalidate it
		// (E_PUSHV), or push the buffered victim (E_PUSHV_B).  Returns
		// to e_sret.
		//==============================================================
		E_PUSHV: begin
			steal  <= 1'b1;
			st_set <= f_set;
			tset_b <= f_set;
			e_st   <= E_PUSHVW;
		end
		E_PUSHVW: begin
			// (the line's valid and dirty bits, registered for E_PUSHV_W)
			fv_lv  <= lve_q[f_way];
			fv_ld  <= ld_q[f_way];
			fv_bad <= 1'b0;
			e_st   <= E_PUSHV_W;
		end
		E_PUSHV_W: if (fv_bad) e_st <= E_PUSHVW;  // a snoop came between: read them again
		else begin
			if (fv_lv && fv_ld) begin
				pv_v     <= 1'b1;
				pv_line  <= dq_rn[f_way];
				pv_lpa   <= {tq_b[f_way], f_set};
				pv_dirty <= 1'b1;
				pv_tln   <= f_way;
				e_st     <= E_PUSHV_B;
			end
			else e_st <= e_sret;
			// invalidated, or (pv_keep) left valid and clean
			ve_set = f_set; ve_wm[f_way] = 1'b1;
			ve_lwe = 1'b1; ve_lv = pv_keep && fv_lv; ve_dwe = 1'b1; ve_dv = 1'b0;
		end
		E_PUSHV_B: begin
			bo.line <= 1'b1; bo.rd <= 1'b0; bo.tt <= TT_NORMAL; bo.tm <= TM_PUSH;
			bo.upa <= 2'd0; bo.ci <= 1'b0; bo.lock <= 1'b0; bo.locke <= 1'b0;
			bo.ack <= 2'd0; bo.tln <= pv_tln;
			bo_pa   <= {pv_lpa, 4'd0};
			bo_left <= 3'd4;
			bo_line <= pv_line;
			pv_v    <= 1'b0;
			e_ret   <= e_sret;
			e_st    <= E_BUS;
		end

		//==============================================================
		// sub-step: bus operation (pieces / line / IACK); returns to e_ret
		//==============================================================
		E_BUS: begin
			bo_err <= 1'b0;
			bo_later <= 1'b0;
			bo_got <= 4'd0;
			bo_tci <= 1'b0;
			bo_avec <= 1'b0;
			bo_rd  <= '0;
			bo_ingnt <= 1'b0;
			if (bo_cancel) begin
				// a snoop invalidated the line this push carries
				bo_cancel <= 1'b0;
				e_st      <= e_ret;
			end
			else begin
				e_breq <= 1'b1;
				e_st   <= E_BUS_W;
			end
		end
		E_BUS_W: begin
			if (bo_cancel && !b_gnt && !bo_ingnt) begin
				// not on the bus yet (the snooped master has it): dropped
				bo_cancel <= 1'b0;
				e_breq    <= 1'b0;
				e_st      <= e_ret;
			end
			else if (b_gnt) begin
				e_breq <= 1'b0;
				bo_pn  <= bo_np;
				bo_ingnt <= 1'b1;
			end
			// (granted, then retried with the bus given up: the BIU drops
			// it on bi_drop and reports it done)
			if (b_err || b_done) begin
				bo_ingnt  <= 1'b0;
				bo_cancel <= 1'b0;
			end
			if (b_rvalid) begin
				if (bo.line) begin
					bo_line[127 - 32*b_rbeat -: 32] <= b_rdata;
					bo_got[b_rbeat] <= 1'b1;
					if (b_rbeat == bo_pa[3:2]) bo_tci <= b_rtci;
				end
				else begin
					bo_rd   <= (bo_rd << (8 * bo_pn)) | grab(b_rdata, bo_pn, bo_pa[1:0]);
					bo_avec <= b_ravec;
				end
			end
			// the last write of a locked sequence (LOCKE) is out
			if ((b_err || b_done) && bo.locke && !bo.rd && (bo.line || bo_left == bo_pn))
				lkend <= 1'b1;
			if ((b_err || b_done) && bo.ack == 2'd1) iack_end <= 1'b1;
			if (b_err) begin
				// a push that ends in TEA: aborted at once, recorded for the
				// access error (FSLW.PBE, 8.4.6); the line is gone
				if (bo.line && !bo.rd && bo.tm == TM_PUSH) pbe_r <= 1'b1;
				else bo_err <= 1'b1;
				e_st   <= e_ret;
			end
			else if (b_done) begin
				if (bo.line || bo.ack != 2'd0) e_st <= e_ret;
				else begin
					if (!bo.rd) bo_wd <= bo_wd << (8 * bo_pn);
					bo_pa   <= bo_pa + {29'd0, bo_pn};
					bo_left <= bo_left - bo_pn;
					if (bo_left == bo_pn) e_st <= e_ret;
					else begin e_breq <= 1'b1; bo_later <= 1'b1; end
				end
			end
		end

		//==============================================================
		// table walk (MC68040UM 3.2); the entry goes into the ATC that
		// tw_i selects; returns to e_ret
		//==============================================================
		E_TW_START: begin
			tw_lvl  <= TW_ROOT;
			tw_wp   <= 1'b0;
			tw_fail <= 1'b0;
			tw_berr <= 1'b0;
			tw_cause <= 32'd0;
			tw_da   <= {(tw_s ? srp[31:9] : urp[31:9]), 9'd0} + {23'd0, tw_ri, 2'b00};
			e_st    <= E_TW_DESC;
		end
		E_TW_DESC: begin
			// descriptor read: from memory, the data cache bypassed (4.2.5)
			steal   <= 1'b1;
			bo.line <= 1'b0; bo.rd <= 1'b1; bo.tt <= TT_NORMAL;
			bo.tm   <= tw_i ? TM_TBL_CODE : TM_TBL_DATA;
			bo.upa  <= 2'd0; bo.ci <= 1'b0; bo.lock <= 1'b0; bo.locke <= 1'b0;
			bo.ack  <= 2'd0; bo.tln <= 2'd0;
			bo_pa   <= tw_da;
			bo_left <= 3'd4;
			e_wret  <= e_ret;
			e_ret   <= E_TW_DBUS;
			e_st    <= E_BUS;
		end
		E_TW_DBUS: begin
			e_ret <= e_wret;
			if (bo_err) begin
				tw_fail  <= 1'b1;
				tw_berr  <= 1'b1;
				tw_cause <= FSLW_TWE | FSLW_LK;
				e_st     <= E_TW_DONE;
			end
			else begin
				tw_d <= bo_rd;
				e_st <= E_TW_EVAL;
			end
		end
		E_TW_EVAL: begin
			case (tw_lvl)
				TW_ROOT, TW_PTR: begin
					if (!tw_d[1]) begin
						tw_fail  <= 1'b1;          // invalid table descriptor
						tw_cause <= (tw_lvl == TW_ROOT) ? FSLW_PTA : FSLW_PTB;
						e_st     <= E_TW_DONE;
					end
					else if (tw_wr && tw_d[2]) begin
						// a write (or read-modify-write) meets a write-
						// protected table descriptor: the search ends here,
						// WP, the descriptor's history untouched (4.2.6.3).
						// (WinUAE walks on and reports what it finds below.)
						tw_fail  <= 1'b1;
						tw_cause <= FSLW_WP;
						e_st     <= E_TW_DONE;
					end
					else begin
						tw_wp <= tw_wp | tw_d[2];
						if (!tw_d[3]) e_st <= E_TW_UPD;   // set U
						else begin
							if (tw_lvl == TW_ROOT) begin
								tw_lvl <= TW_PTR;
								tw_da  <= {tw_d[31:9], 9'd0} + {23'd0, tw_pi, 2'b00};
							end
							else begin
								tw_lvl <= TW_PAGE;
								tw_da  <= (tc_p ? {tw_d[31:7], 7'd0} : {tw_d[31:8], 8'd0}) +
								          {24'd0, tw_gi, 2'b00};
							end
							e_st <= E_TW_DESC;
						end
					end
				end
				default: begin
					// page descriptor (or the one an indirect points to)
					if (tw_d[1:0] == 2'b00 || (tw_lvl == TW_IND && tw_d[1:0] == 2'b10)) begin
						tw_fail  <= 1'b1;
						tw_cause <= (tw_d[1:0] == 2'b10) ? FSLW_IL : FSLW_PF;
						e_st     <= E_TW_DONE;
					end
					else if (tw_d[1:0] == 2'b10) begin
						tw_lvl <= TW_IND;
						tw_da  <= {tw_d[31:2], 2'b00};
						e_st   <= E_TW_DESC;
					end
					else begin
						logic wp, sv, setm;
						wp   = tw_wp | tw_d[2];
						sv   = tw_d[7] && !tw_s;
						setm = tw_wr && !wp && !sv && !tw_d[4];
						tw_pd <= tw_d;
						tw_wp <= wp;
						if (tw_wr && wp) begin
							// a write meets write protection: the search
							// ends, no entry, no history (4.2.6.3)
							tw_fail  <= 1'b1;
							tw_cause <= FSLW_WP;
							e_st     <= E_TW_DONE;
						end
						else begin
							// a user access to a supervisor-only page fails
							// too, its U set as for the access (no M: 4.2.2.3)
							if (sv) begin
								tw_fail  <= 1'b1;
								tw_cause <= FSLW_SP;
							end
							if (!tw_d[3] || setm) e_st <= E_TW_UPD;
							else e_st <= E_TW_DONE;
						end
					end
				end
			endcase
		end
		E_TW_UPD: begin
			// descriptor update, MC68060UM 4.2.5 and table 4-1: a table
			// descriptor's U and a page read's U by a locked read-modify-
			// write; a page write's U and M by a plain write.  The data
			// cache is not searched.
			logic page, sv, setm, needu, lk;
			logic [31:0] nd;
			page  = (tw_lvl == TW_PAGE || tw_lvl == TW_IND);
			sv    = page && tw_d[7] && !tw_s;
			setm  = page && tw_wr && !tw_wp && !sv && !tw_d[4];
			needu = !tw_d[3];
			lk    = needu && !(page && tw_wr);
			nd    = tw_d | 32'h8 | (setm ? 32'h10 : 32'h0);
			bo.line <= 1'b0; bo.tt <= TT_NORMAL;
			bo.tm   <= tw_i ? TM_TBL_CODE : TM_TBL_DATA;
			bo.upa  <= 2'd0; bo.ci <= 1'b0;      // CIOUT negated (table 7-2)
			bo.ack <= 2'd0; bo.tln <= 2'd0;
			bo_pa   <= tw_da;
			bo_left <= 3'd4;
			e_wret <= e_ret;
			e_st   <= E_BUS;
			if (lk) begin
				// the locked read-modify-write: the descriptor read again
				// with LOCK, U set in what it reads, written back with LOCK
				// and LOCKE (E_TW_LKW; MC68060UM 4.2.5, the 10.3 timings'
				// extra read)
				bo.rd   <= 1'b1;
				bo.lock <= 1'b1; bo.locke <= 1'b0;
				e_ret   <= E_TW_LKW;
			end
			else begin
				tw_d  <= nd;
				if (page) tw_pd <= nd;
				bo.rd   <= 1'b0;
				bo.lock <= 1'b0; bo.locke <= 1'b0;
				bo_wd   <= nd;
				e_ret   <= E_TW_UPD_W;
			end
		end
		E_TW_LKW: begin
			// the locked read is in: U into it, the locked write ends it
			logic [31:0] nd;
			nd = bo_rd | 32'h8;
			if (bo_err) begin
				e_ret    <= e_wret;
				tw_fail  <= 1'b1;
				tw_berr  <= 1'b1;
				tw_cause <= FSLW_TWE | FSLW_LK;
				e_st     <= E_TW_DONE;
			end
			else begin
				tw_d <= nd;
				if (tw_lvl == TW_PAGE || tw_lvl == TW_IND) tw_pd <= nd;
				bo.rd   <= 1'b0;
				bo.lock <= 1'b1; bo.locke <= 1'b1;
				bo_pa   <= tw_da;
				bo_left <= 3'd4;
				bo_wd   <= nd;
				e_ret   <= E_TW_UPD_W;
				e_st    <= E_BUS;
			end
		end
		E_TW_UPD_W: begin
			e_ret <= e_wret;
			if (bo_err) begin
				tw_fail  <= 1'b1;
				tw_berr  <= 1'b1;
				tw_cause <= FSLW_TWE | FSLW_LK;
				e_st     <= E_TW_DONE;
			end
			else if (tw_lvl == TW_ROOT) begin
				tw_lvl <= TW_PTR;
				tw_da  <= {tw_d[31:9], 9'd0} + {23'd0, tw_pi, 2'b00};
				e_st   <= E_TW_DESC;
			end
			else if (tw_lvl == TW_PTR) begin
				tw_lvl <= TW_PAGE;
				tw_da  <= (tc_p ? {tw_d[31:7], 7'd0} : {tw_d[31:8], 8'd0}) +
				          {24'd0, tw_gi, 2'b00};
				e_st   <= E_TW_DESC;
			end
			else e_st <= E_TW_DONE;
		end
		E_TW_DONE: begin
			atce_t e;
			e = '0;
			if (!tw_fail) begin
				e.pa  = tc_p ? {tw_pd[31:13], 1'b0} : tw_pd[31:12];
				e.g   = tw_pd[10];
				e.upa = tw_pd[9:8];
				e.s   = tw_pd[7];
				e.cm  = tw_pd[6:5];
				e.m   = tw_pd[4];
				e.w   = tw_wp;
				e.r   = 1'b1;
			end
			e.b = tw_berr;
			// install a successful search, unless the ATC is in its
			// no-allocate mode (TCR NAD, NAI) -- an entry whose M a write
			// sets is still updated (4.1.2)
			if (!tw_fail) begin
				if (tw_i) begin
					if (!tc[12]) begin
						iatc_wr   <= 1'b1;
						iatc_wla  <= tw_va;
						iatc_wfc2 <= tw_s;
						iatc_went <= e;
					end
				end
				else if (!tc[13] || tw_upd) begin
					atc_wr   <= 1'b1;
					atc_wla  <= tw_va;
					atc_wfc2 <= tw_s;
					atc_went <= e;
				end
			end
			iw_ent   <= e;
			iw_cause <= tw_cause;
			tw_e     <= e;
			tw_use   <= !tw_i;
			steal <= 1'b0;
			e_st  <= e_ret;
		end
		E_I_DONE: begin
			iw_done <= 1'b1;
			e_st    <= E_IDLE;
		end

		default: e_st <= E_IDLE;
		endcase

		if ((e_st == E_SP_W || e_st == E_SP_2) && dw && dw_set == st_set) sp_dw <= 1'b1;
		// a snoop (the engine frozen) between the victim's bits and their use
		if (sn_frz && (e_st == E_FILL_W || e_st == E_PUSHV_W)) fv_bad <= 1'b1;

		if (kill_now) begin
			// a kill comes with the WB uop leaving (or WB empty): every
			// record goes, including one EX moved into WB this cycle
			m1.v   <= 1'b0;
			mt.v   <= 1'b0;
			m2.r.v <= 1'b0;
			m3.r.v <= 1'b0;
			m4.r.v <= 1'b0;
			e_dc2_done <= 1'b0;
			if (e_st != E_IDLE) e_kill <= 1'b1;
			if (e_st == E_IDLE) e_job <= 1'b0;
		end

		//--------------------------------------------------------------
		// snoop port
		//--------------------------------------------------------------
		if (sb_err_ack) pbe_r <= 1'b0;
		sn_look <= 1'b0;
		case (sn_ph)
		SN_IDLE: if (sn_req) sn_ph <= SN_A;
		         else if (sf_req) sn_ph <= SN_F;
		SN_F:    if (!sf_more) sn_ph <= SN_IDLE;
		// the RAMs register the snooped set; a data or tag write issued
		// just before the freeze lands in this cycle, and a read of the
		// same set would miss it: present the set once more
		SN_A:    if (!dw && !tw_b) sn_ph <= SN_B;
		SN_B: begin
			logic [6:0] s;
			logic h; logic [1:0] w;
			s = sn_pa[10:4];
			h = 1'b0; w = 2'd0;
			for (int i = 0; i < 4; i++)
				if (lve_q[i] && tq_a[i] == sn_pa[31:11]) begin h = 1'b1; w = 2'(i); end
			sn_way <= w;
			sn_look <= 1'b1;
			// (CACR.DPI keeps a pushed line valid: a way and a push can
			// both hold it)
			sn_pvm <= pv_v && pv_lpa == sn_pa[31:4];
			sn_bom <= (e_st == E_BUS || e_st == E_BUS_W) && bo.line && !bo.rd &&
			          bo.tm == TM_PUSH && !bo_cancel && bo_pa[31:4] == sn_pa[31:4];
			if (h) begin
				sn_src   <= 2'd0;
				sn_hit   <= 1'b1;
				sn_dirty <= ld_q[w];          // (ld_ra: the snooped set)
			end
			else if (pv_v && pv_lpa == sn_pa[31:4]) begin
				// the dirty victim waiting for its push (4.7.2)
				sn_src   <= 2'd1;
				sn_hit   <= 1'b1;
				sn_dirty <= 1'b1;
			end
			else if ((e_st == E_BUS || e_st == E_BUS_W) && bo.line && !bo.rd &&
			         bo.tm == TM_PUSH && !bo_cancel && bo_pa[31:4] == sn_pa[31:4]) begin
				// a push queued for the bus
				sn_src   <= 2'd2;
				sn_hit   <= 1'b1;
				sn_dirty <= 1'b1;
			end
			else begin
				sn_hit   <= 1'b0;
				sn_dirty <= 1'b0;
			end
			sn_ph <= SN_H;
		end
		SN_H: begin
			logic [6:0] s;
			s = sn_pa[10:4];
			// (a line found in a way is invalidated below)
			if (sn_inv && sn_pvm) pv_v <= 1'b0;
			if (sn_inv && sn_bom) bo_cancel <= 1'b1;
			if (sn_wr) begin
				case (sn_src)
				2'd0: begin
					dw      <= 1'b1;
					dw_set  <= s;
					dw_way  <= sn_way;
					dw_be   <= sn_wbe;
					dw_src  <= 2'd0;
					dw_word <= sn_wword;
					vs_set = s; vs_wm[sn_way] = 1'b1; vs_dwe = 1'b1; vs_dv = 1'b1;
				end
				2'd1: for (int i = 0; i < 16; i++)
					if (sn_wbe[i]) pv_line[8*i +: 8] <= sn_wword[8*(i % 4) +: 8];
				default: for (int i = 0; i < 16; i++)
					if (sn_wbe[i]) bo_line[8*i +: 8] <= sn_wword[8*(i % 4) +: 8];
				endcase
			end
			if (!sn_req) sn_ph <= SN_R;
		end
		default: sn_ph <= SN_IDLE;   // SN_R: the owner's address is read again
		endcase
		// a snooped line leaves the cache: the full lookup's (H, found in a
		// way) or a fast invalidation (F)
		begin
			logic       iv;
			logic [6:0] iv_s;
			logic [1:0] iv_w;
			iv   = (sn_ph == SN_H && sn_inv && sn_src == 2'd0) || sn_ph == SN_F;
			iv_s = (sn_ph == SN_F) ? sf_set : sn_pa[10:4];
			iv_w = (sn_ph == SN_F) ? sf_way : sn_way;
			if (iv) begin
				vs_set = iv_s; vs_wm[iv_w] = 1'b1;
				vs_lwe = 1'b1; vs_lv = 1'b0; vs_dwe = 1'b1; vs_dv = 1'b0;
				// (the records' hits on it: at the valid/dirty write port)
			end
		end

		//--------------------------------------------------------------
		// a line leaving the cache -- whoever invalidates it: a snoop, an
		// inhibited or locked access's push, CINV/CPUSH -- leaves no record
		// with a hit on it: the stage records, those loaded this clock from
		// the stage before (DC1's lookup read the valid bits before this
		// write lands), both parts of a split, and the engine's own
		//--------------------------------------------------------------
		// (the record DC2 takes now loses its hit if its way is invalidated
		// in this cycle: the way is late in DT, so the ways invalidated in
		// its set are registered (dt_inv) and the hit cleared a cycle later,
		// in DC2 or, if it moved on, in EX.  Its data was read before: a
		// fast load completes with it, as if before the invalidation; a store
		// must lose the hit, or WB would write the line it no longer has)
		// the valid/dirty port's request: the snoop port's, else the
		// engine's, else the WB store's (the engine is frozen while the
		// snoop port holds the cache; the WB store writes with the engine
		// idle and no snoop)
		if (vs_lwe || vs_dwe) begin
			vp_set = vs_set; vp_wm = vs_wm; vp_lwe = vs_lwe; vp_lv = vs_lv; vp_dwe = vs_dwe; vp_dv = vs_dv;
		end
		else if (ve_lwe || ve_dwe) begin
			vp_set = ve_set; vp_wm = ve_wm; vp_lwe = ve_lwe; vp_lv = ve_lv; vp_dwe = ve_dwe; vp_dv = ve_dv;
		end
		else begin
			vp_set = vf_set; vp_wm = vf_wm; vp_lwe = vf_lwe; vp_lv = vf_lv; vp_dwe = vf_dwe; vp_dv = vf_dv;
		end
		// synthesis translate_off
		if (32'(vs_lwe || vs_dwe) + 32'(ve_lwe || ve_dwe) + 32'(vf_lwe || vf_dwe) > 1) begin
			$display("FAIL: DMU: two writers of the valid/dirty port at once (snoop %b engine %b store %b)",
			         vs_lwe || vs_dwe, ve_lwe || ve_dwe, vf_lwe || vf_dwe);
			$finish;
		end
		// synthesis translate_on
		// (the invalidations alone, for the records' hits: the snoop port's,
		// else the engine's -- the WB store's writes never invalidate)
		begin
		logic       iv_v;
		logic [6:0] iv_set;
		logic [3:0] iv_wm;
		iv_v   = (vs_lwe && !vs_lv) || (ve_lwe && !ve_lv);
		iv_set = (vs_lwe || vs_dwe) ? vs_set : ve_set;
		iv_wm  = (vs_lwe || vs_dwe) ? vs_wm : ve_wm;
		// synthesis translate_off
		if (iv_v != (vp_lwe && !vp_lv) || (iv_v && (iv_set != vp_set || iv_wm != vp_wm))) begin
			$display("FAIL: DMU: the invalidation select disagrees with the port's request");
			$finish;
		end
		// synthesis translate_on
		dt_inv <= (iv_v && adv_dt && mt.a[10:4] == iv_set) ? iv_wm : 4'd0;
		if (dt_inv[m2.x.way] && m2.x.hit) begin
			if (!adv_dc2) begin
				xres_t xm;
				xm = m2.x;
				xm.hit = 1'b0;
				m2.x.hit <= 1'b0;
				m2ss_n   = st_simp(m2.r, xm, m2.split);
			end
			else m3.x.hit <= 1'b0;
			// (the engine taking the access this cycle copies its record)
			if (e_st == E_S_START && !sn_frz) e_x.hit <= 1'b0;
		end
		if (iv_v) begin
			for (int j = 0; j < 4; j++) if (iv_wm[j]) begin
				if (!adv_dt) begin
					if (m2.x.hit && m2.x.pa[10:4] == iv_set && m2.x.way == 2'(j)) begin
						xres_t xm;
						xm = m2.x;
						xm.hit = 1'b0;
						m2.x.hit <= 1'b0;
						m2ss_n   = st_simp(m2.r, xm, m2.split);
					end
					if (m2.x1.hit && m2.x1.pa[10:4] == iv_set && m2.x1.way == 2'(j)) m2.x1.hit <= 1'b0;
				end
				if (adv_dc2) begin
					if (m2.x.hit && m2.x.pa[10:4] == iv_set && m2.x.way == 2'(j)) m3.x.hit <= 1'b0;
					if (m2.x1.hit && m2.x1.pa[10:4] == iv_set && m2.x1.way == 2'(j)) m3.x1.hit <= 1'b0;
				end
				else begin
					if (m3.x.hit && m3.x.pa[10:4] == iv_set && m3.x.way == 2'(j)) m3.x.hit <= 1'b0;
					if (m3.x1.hit && m3.x1.pa[10:4] == iv_set && m3.x1.way == 2'(j)) m3.x1.hit <= 1'b0;
				end
				if (adv_ex) begin
					if (m3.x.hit && m3.x.pa[10:4] == iv_set && m3.x.way == 2'(j)) begin
						m4.x.hit <= 1'b0;
						m4_fq    <= 1'b0;
					end
					if (m3.x1.hit && m3.x1.pa[10:4] == iv_set && m3.x1.way == 2'(j)) m4.x1.hit <= 1'b0;
				end
				else begin
					if (m4.x.hit && m4.x.pa[10:4] == iv_set && m4.x.way == 2'(j)) begin
						m4.x.hit <= 1'b0;
						m4_fq    <= 1'b0;
					end
					if (m4.x1.hit && m4.x1.pa[10:4] == iv_set && m4.x1.way == 2'(j)) m4.x1.hit <= 1'b0;
				end
				if (e_x.hit && e_x.pa[10:4] == iv_set && e_x.way == 2'(j)) e_x.hit <= 1'b0;
			end
		end
		end

		//--------------------------------------------------------------
		// the valid/dirty write port
		//--------------------------------------------------------------
		begin
			vq_v   <= (vp_dwe || vp_lwe) && vp_wm != 4'd0;
			vq_set <= vp_set;
			vq_wm  <= vp_wm;
			vq_lwe <= vp_lwe; vq_lv <= vp_lv;
			vq_dwe <= vp_dwe; vq_dv <= vp_dv;
			if (vq_v) begin
				logic [7:0] wn;
				wn = vmerge(lsw[vq_set]);
				lsd[vq_set] <= wn;
				lse[vq_set] <= wn;
				lsw[vq_set] <= wn;
			end
			for (int j = 0; j < 4; j++)
				if (vp_wm[j]) begin
					// synthesis translate_off
					if (vp_lwe) lv[vp_set][j] <= vp_lv;
					// synthesis translate_on
					if (vp_lwe && vp_set == sc_set) sc_lv[j] <= vp_lv;
					if (vp_dwe && vp_set == sc_set) sc_ld[j] <= vp_dv;
				end
			// (the snooper's copy follows a clock later)
			dv_we  <= vp_lwe;
			dv_set <= vp_set;
			dv_wm  <= vp_wm;
			dv_lv  <= vp_lv;
		end
		m1_stale <= m1s_n;
		mt_stale <= mts_n;
		m2_ss    <= m2ss_n;

		//--------------------------------------------------------------
		// the holds a clock ahead (hold1_q, dc2r_q, st_ok_q)
		//--------------------------------------------------------------
		begin
			logic frz_n, nofrz, m1v_n, mtv_n, busy_n, stl_set, r_new, r_stay, ok_new, ok_stay;
			logic [2:0] sbn_n;
			// the snoop port may hold the cache next cycle
			frz_n = sn_frz || sn_req || sf_req;
			nofrz = !frz_n;
			// DC1: the snoop port, a port stolen now or next cycle (steal_q is
			// steal now), the records' stale lookups, the engine's state next
			// cycle (a state left for IDLE unconditionally is not held for)
			m1v_n   = !kill_now && (adv_ag ? dm_req : (!adv_dc1 && m1.v));
			mtv_n   = !kill_now && (adv_dc1 ? m1.v : (!adv_dt && mt.v));
			stl_set = !sn_frz && (e_st == E_S_START || e_st == E_S_XL || e_st == E_S_RD ||
			                      e_st == E_M_START || e_st == E_M_SCAN || e_st == E_FILL ||
			                      e_st == E_PUSHV || e_st == E_TW_DESC);
			if (sn_frz)
				busy_n = e_busy;
			else
				busy_n = (e_busy && e_st != E_S_DONE && e_st != E_I_DONE) ||
				         (e_st == E_IDLE && !wfs_p1 &&
				          ((mt_v && !mt_done) ||
				           (!wb_slow && (dc2_slow || (e_job && m2.r.v && !e_dc2_done) ||
				                         (iw_req && !iw_done && iw_ok && sb_empty))))) ||
				         (e_st == E_W_PART && (wd_push || wd_wtp || (!wd_cb && !wd_sb)));
			hold1_q <= frz_n || steal || stl_set || (m1v_n && m1s_n) || (mtv_n && mts_n) || busy_n;
			// DC2: an access entering (its fast and simple-store decisions,
			// unless the engine may own DC2); one staying (the engine
			// finishing it, or its own decisions while the engine does not
			// own it)
			r_new  = !mt.v || (!e_job && e_st != E_S_START && (tk_fast || m2ss_n));
			r_stay = !m2.r.v || e_dc2_done || (e_st == E_S_DONE && !sn_frz && !e_kill) ||
			         (!e_job && !(e_st == E_S_START && !sn_frz) && (m2.fast || m2ss_n));
			dc2r_q <= adv_dt ? r_new : (adv_dc2 || r_stay);
			// WB: a store entering (the fast store: the engine idle, or
			// finishing a job, and no snoop starting); one staying (answered
			// now, the engine's store finishing, or the fast store as the
			// engine finishes its job)
			// (into the store buffer: the engine idle, the buffer's count as
			// it will be below its size)
			sbn_n   = sb_n - 3'(sb_pop) + 3'(sb_push || wb_fsb);
			ok_new  = (!m3.split && m3.x.ok && !m3.x.flt && m3.x.hit && m3.x.cm == 2'b01 &&
			           !m3.r.lock && (m3.r.msz != SZ_Q) && dc_en && nofrz &&
			           ((e_st == E_IDLE && !(e_job && m2.r.v && !e_dc2_done)) || e_st == E_S_DONE)) ||
			          (fsb_of(m3) && (sbn_n < 3'(SBN)) &&
			           ((e_st == E_IDLE && !(e_job && m2.r.v && !e_dc2_done)) ||
			            (e_st == E_S_DONE && !sn_frz)));
			ok_stay = (st_rdy_r && !st_fault_r) || (!sn_frz && (wb_fast || (wfs_p1 && m4.x1.hit))) || wb_fsb ||
			          (!sn_frz && e_st == E_W_DONE && !(bo_err && m4.r.ack != 2'd3) &&
			           !(m4.split && !e_part)) ||
			          (nofrz && m4_fq && dc_en && !e_wst_done && e_st == E_S_DONE);
			st_ok_q <= adv_ex ? ok_new : ok_stay;
		end
	end
end

// synthesis translate_off
always_ff @(posedge clk)
	if (nreset) begin
		if (mt.v && (mt_bsel[0] != tk_bsel[0] || mt_bsel[1] != tk_bsel[1] ||
		             mt_bsel[2] != tk_bsel[2] || mt_bsel[3] != tk_bsel[3])) begin
			$display("FAIL: DMU: DT's registered byte selects disagree");
			$finish;
		end
		if (mt.v && (mt_tt0 != ttr_hit(dtt0, mt.a, mt.smode) || mt_tt1 != ttr_hit(dtt1, mt.a, mt.smode)) &&
		    !kill_now) begin
			$display("FAIL: DMU: DT's registered TTR matches disagree");
			$finish;
		end
		if (!e_kill && ((e_st == E_S_XL2 && (lk_tt0 != ttr_hit(dtt0, e_va, m2.r.smode) ||
		                                     lk_tt1 != ttr_hit(dtt1, e_va, m2.r.smode))) ||
		                (e_st == E_SP_2 &&
		                 (lk_tt0 != ttr_hit(dtt0, {m2.r.a[31:4] + 28'd1, 4'd0}, m2.r.smode) ||
		                  lk_tt1 != ttr_hit(dtt1, {m2.r.a[31:4] + 28'd1, 4'd0}, m2.r.smode))))) begin
			$display("FAIL: DMU: the engine's registered TTR matches disagree");
			$finish;
		end
		if (dc2r_q && !dc2_rdy_c) begin
			$display("FAIL: DMU: DC2's registered ready is ahead of its state");
			$finish;
		end
		if (wb_st && st_ok_q && !(st_rdy && !st_fault)) begin
			$display("FAIL: DMU: WB's registered store answer is ahead of its state");
			$finish;
		end
	end
// synthesis translate_on

// synthesis translate_off
always_ff @(posedge clk)
	if (nreset && e_st == E_M_SCAN2 &&
	    (sc_lv != lv[sc_set] || (!sn_frz && (sc_lv & sc_ld) != (sc_lv & ld_q)))) begin
		$display("FAIL: the scan's valid/dirty copy %b %b, the set's %b", sc_lv, sc_ld, lv[sc_set]);
		$finish;
	end
// synthesis translate_on

// snoop: the line found, valid with sn_look (the RAM still reads the
// snooped set in the first H cycle)
assign sn_line = (sn_src == 2'd0) ? dq_rn[sn_way] : (sn_src == 2'd1) ? pv_line : bo_line;

assign tw_busy = (e_st == E_TW_START) || (e_st == E_TW_DESC) || (e_st == E_TW_DBUS) ||
                 (e_st == E_TW_EVAL) || (e_st == E_TW_UPD) || (e_st == E_TW_UPD_W) ||
                 (e_st == E_TW_DONE) ||
                 (e_st == E_BUS && (e_ret == E_TW_DBUS || e_ret == E_TW_UPD_W));

// DC2 answer
// once the engine has taken the DC2 uop, only its completion releases it
assign dc2_rdy_c = !m2.r.v || e_dc2_done || (!e_job && (fast_now || st_simple));
assign ldata   = e_dc2_done ? e_ldata : fast_data;
// for AG's capture of a load into a base or index: the operand, offered for
// an aligned long or an even word
assign ldata_al  = ldata;
assign ldata_alv = e_dc2_done ||
                   (m2.r.msz == SZ_L && m2.r.a[1:0] == 2'd0) ||
                   (m2.r.msz == SZ_W && !m2.r.a[0]);
assign fault   = e_dc2_done && e_flt;
assign fvec    = 8'd2;
assign faddr   = e_faddr;
assign fslw    = e_fslw;

// the snooper's tag copy: the tag RAM's writes; the lines a snoop must
// find through the full lookup (a dirty line on its way out, 4.7.2)
assign dt_we  = tw_b;
assign dt_set = tset_b;
assign dt_way = tway_b;
assign dt_tag = twd_b;
assign pq0_v  = pv_v;
assign pq0_l  = pv_lpa;
assign pq1_v  = (e_st == E_BUS || e_st == E_BUS_W) && bo.line && !bo.rd &&
                bo.tm == TM_PUSH && !bo_cancel;
assign pq1_l  = bo_pa[31:4];

// the valid copies' reads: DC1's lookup; the engine state's set (a
// split's next line, the slow path's, the scan's, else the fill/push set), or the snooped set while the engine is frozen
wire  [7:0] lse_q = lsm_q;              // (with the pending write's bypass)
assign lve_q = lse_q[3:0];
always_comb
	lv_ra = (sn_ph == SN_B)                            ? sn_pa[10:4] :
	        (e_st == E_SP_2)                           ? st_set :     // (the next set)
	        (e_st == E_S_XL2 || e_st == E_S_ACT ||
	         e_st == E_W_DEC)                          ? e_va[10:4] :
	        (e_st == E_M_SCANW || e_st == E_M_SCANW2 ||
	         e_st == E_M_SCAN2)                        ? sc_set : f_set;
initial for (int i = 0; i < 128; i++) begin
	lsd[i] = 8'd0; lse[i] = 8'd0; lsw[i] = 8'd0;
end
// synthesis translate_off
// the copies against the flip-flop model (after reset's sweep)
always_ff @(posedge clk)
	if (nreset && !clr_rst && (lvd_q != lv[mt.a[10:4]] || lve_q != lv[lv_ra])) begin
		$display("FAIL: dmu: valid RAM %b/%b, model %b/%b (sets %0d %0d)",
		         lvd_q, lve_q, lv[mt.a[10:4]], lv[lv_ra], mt.a[10:4], lv_ra);
		$finish;
	end
// synthesis translate_on

endmodule
