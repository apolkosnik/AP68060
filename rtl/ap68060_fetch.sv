//--------------------------------------------------------------------------//
// AP68060 - pipelined MC68040                                            //
//                                                                          //
// ap68060_fetch.sv - instruction fetch: I-ATC, instruction cache and the   //
// instruction queue                                                        //
//                                                                          //
// Instruction cache (MC68060UM section 5): 8 Kbytes, four ways of 128     //
// sets of 16-byte lines, physically tagged (PA31-11; the set is LA10-4),  //
// a valid bit per line, replacement by the first invalid way, else a      //
// 2-bit counter (CACR.FIC, the 1/2-cache mode: ways 0 and 1).  Lines are   //
// filled by a burst (critical long word first, TBI handled by the BIU,    //
// TCI, or CACR.NAI: the line is not allocated).  Cache-inhibited pages    //
// and CACR.EIC = 0 fetch long words.                                       //
//                                                                          //
// I-ATC (MC68060UM 4.3): 64 entries like the D-ATC, translating          //
// supervisor and user program space; ITT0/ITT1 transparent translation,  //
// the TCR's DCI/DUI without paging.  An ATC miss asks the DMU's engine   //
// for a table walk; the walk installs the entry here (iatc_wr), and its   //
// result -- the entry, or the failure: no entry is made for one -- is     //
// kept for the page (wf_*) until the next redirect, so the fetch that     //
// looks again uses it (also in the no-allocate mode, TCR NAI).           //
//                                                                          //
// A stream D1 or the BTB redirected to is a prediction (fspec): the back  //
// end has not asked for it, and older stores (a descriptor the program   //
// just wrote) may not be performed yet.  Its fetches do not search the   //
// tables: an ATC miss there is a speculative ATC fault, which the back   //
// end refetches on demand (then the search sees every older store), and  //
// none of its faults is a demand fetch's.                                  //
//                                                                          //
// Pipeline, one aligned long word (two words) per cycle -- D1 decodes at  //
// most one instruction a cycle, and the queue absorbs long ones:         //
//   F0  fpc: I-ATC set read and per-way compare (its pipelined port, at  //
//       a registered set index), ITT match, cache tag and data RAM read, //
//       BTB read                                                           //
//   F1  translation (the ATC's hit way), tag compare (the hit, one-hot    //
//       per way, registered                                                //
//       with each way's words); the BTB's compare, and a taken branch     //
//       sends F0 to its target (one bubble)                                //
//   F2  the hit's words (the way's, or the line buffer's); anything else  //
//       rolls fpc back and runs in the miss engine (walk, line fill,      //
//       cache-inhibited stream, fault)                                     //
//   F3-F5  predecode (ap68060_predec: P1-P3, from F3's registered words)  //
//   F6  append to the queue                                                //
//                                                                          //
// The queue holds QN words: slot 0 is always the next word D1 decodes,   //
// and D1 sees the first eight.  D1 takes the head instruction (its       //
// predecoded length, cs_fire: one LUT from registers, fanned out over the //
// queue) or a length it decided the clock before (cr_fire, cr_use): the //
// queue's next state is formed for both, and for neither, from registers //
// (kept nets), and the takes pick one last.  The BTB targets of flagged  //
// words are a ring.  A redirect (D1 branch or WB) empties the queue and  //
// pipeline.  A faulted fetch marks its words; D1 raises the fault only if //
// it uses them (8.2.1).  Only the first fetch of a stream the back end   //
// redirected is a demand fetch; a fault on the lookahead after it (or in //
// a predicted stream) is speculative, and the back end refetches it on   //
// demand before it becomes an access error.                              //
//                                                                          //
// Branch target buffer: 256 entries (BTB_AW) looked up with the F0       //
// address; a hit (the long word holds the last word of a branch D1        //
// predicted taken) ends the chunk at that word, flags it, and sends F0 to //
// the target in F1 (one bubble): the entry's, or for a return the top of //
// the fetch's return stack (pushed by the calls it takes, resynchronized //
// from D1's on every redirect, and moved back when the chunk misses).    //
// The flagged word's target travels in a FIFO; D1 checks every flagged   //
// word against its own decode.                                            //
//                                                                          //
// CINV/CPUSH of the instruction cache arrive from the DMU (ic_inv): all   //
// in one cycle, a line in two, a page by a scan of the 64 sets.  Bus     //
// snooping (sn_inv) invalidates a line the same way (MC68040UM table   //
// 4-3, V5/V6).  The valid bits are written a clock after the tags are    //
// compared.                                                               //
//--------------------------------------------------------------------------//

module ap68060_fetch
	import ap68060_pkg::*;
(
	input  logic        clk,
	input  logic        nreset,

	input  logic        redir_v,      // highest priority
	input  logic [31:0] redir_pc,
	input  logic        d_redir_v,    // decoder (predicted branch)
	input  logic [31:0] d_redir_pc,
	input  logic        stop,         // do not start new fetches

	input  logic        smode,        // S bit: program space and TM 6/2
	input  logic [31:0] cacr,
	input  logic [31:0] tc,
	input  logic [31:0] itt0,
	input  logic [31:0] itt1,
	input  logic        cdis,         // CDIS: the cache is disabled (contents kept)
	input  logic        mdis,         // MDIS: no page translation

	// decoder side
	output logic [15:0] win [8],
	output logic  [7:0] win_flt,      // the word came from a faulted fetch
	output logic  [7:0] win_fdem,     // ... the first of a redirected stream
	output logic  [7:0] win_fatc,     // ... an ATC (MMU) fault, else bus error
	output logic  [7:0] win_bt,       // ... ends a branch the BTB took
	output logic [31:0] bt_tgt,       // the target of the first such word
	output pd_t         pd0,          // predecode of win[0]
	output logic        q_simple,     // ... D1 can take it in one pass now
	output logic  [3:0] qcnt,         // words available in win (0..8)
	output logic [31:0] qpc,
	// what D1 takes: the head instruction (pd0.tot words), or cr_use words
	input  logic        cs_fire,
	input  logic        cr_fire,
	input  logic  [2:0] cr_use,
	output logic        q_odd,        // the stream starts at an odd address
	input  logic        bc_clr,       // CACR.CABC/CUBC: clear the branch cache
	input  logic        btb_we,       // D1 writes a BTB entry
	input  logic [BTB_AW-1:0] btb_wi,
	input  logic        btb_wv,       // valid (0 drops the entry)
	input  logic [BTB_TW-1:0] btb_wtag,
	input  logic        btb_wslot,    // the branch ends at word 1 of the long word
	input  logic  [1:0] btb_wkind,    // 0 branch, 1 call, 2 return (target from the stack)
	input  logic [30:0] btb_wtgt,     // target[31:1]
	input  logic [30:0] d_ras,        // D1's return stack, its top entry: the
	input  logic  [2:0] d_ras_tp,     // ... fetch's pointer and top entry are
	input  logic  [3:0] d_ras_n,      // ... resynchronized on every redirect

	// DMU: table walks for the I-ATC, I-ATC maintenance, cache invalidation
	output logic        iw_req,
	output logic [31:0] iw_va,
	output logic        iw_fc2,
	input  logic        iw_done,
	input  atce_t       iw_ent,       // the walk's entry (r clear: it failed)
	input  logic [31:0] iw_cause,     // ... and the failure's FSLW cause
	output logic [31:0] if_cause,     // FSLW cause of the last faulted fetch (its translation
	                                  // fault; a bus error: TTR when an ITT maps it)
	input  logic        iatc_wr,
	input  logic [31:0] iatc_wla,
	input  logic        iatc_wfc2,
	input  atce_t       iatc_went,
	input  logic        iatc_flush_all,
	input  logic        iatc_flush_page,
	input  logic        iatc_flush_ng,
	input  logic [31:0] iatc_flush_la,
	input  logic        iatc_flush_fc2,
	input  logic        ic_inv,
	input  logic  [1:0] ic_inv_scope,  // 1 line, 2 page, 3 all
	input  logic [31:0] ic_inv_pa,
	output logic        ic_inv_done,
	input  logic        sn_inv,        // snooper: invalidate the line at sn_inv_pa
	input  logic [31:0] sn_inv_pa,
	input  logic        sn_inv_all,    // ... or every line
	output logic        sn_inv_done,
	output logic        sn_ihit,       // ... and removed a valid line (or the line buffer)

	// BIU client
	output logic        b_req,
	output busreq_t     b_breq,
	input  logic        b_gnt,
	input  logic        b_done,
	input  logic        b_err,
	input  logic        b_rvalid,
	input  logic [31:0] b_rdata,
	input  logic  [1:0] b_rbeat,
	input  logic        b_rtci
);

localparam int QN  = 16;          // queue words (D1 sees the first eight)
localparam int TQN = 8;           // targets of BTB-flagged words

// the configuration, registered here (MOVEC serializes and refetches, S
// changes with a flush; the pins' changes are asynchronous): F0 and F1
// start from local flip-flops, not from the back end's
logic        sm_q, cdis_q, mdis_q;
logic [31:0] cacr_q, tc_q, itt0_q, itt1_q;
always_ff @(posedge clk) begin
	sm_q   <= smode;
	cdis_q <= cdis;
	mdis_q <= mdis;
	cacr_q <= cacr;
	tc_q   <= tc;
	itt0_q <= itt0;
	itt1_q <= itt1;
end
wire tc_e  = tc_q[15] && !mdis_q;
wire tc_p  = tc_q[14];
wire ic_en = cacr_q[15] && !cdis_q;

function automatic logic ttr_hit(input logic [31:0] t, input logic [31:0] a,
                                 input logic s);
	ttr_hit = t[15] && (((a[31:24] ^ t[31:24]) & ~t[23:16]) == 8'd0) &&
	          (t[14] || (t[13] == s));
endfunction

// the back end's stop and the DMU's I-ATC flushes, a clock late: they act
// on F0 and the ATC from registers (the BIU holds the fetch's requests
// while the processor is stopped, halted or resetting)
logic        stop_q;
(* preserve *) logic stop_f;    // (its copy for F0's go, kept apart: placed by F0)
logic        fl_all_q, fl_pg_q, fl_ng_q;
// the ATC's lookup address and space: fpc's copy, or a page flush's (in
// the clock fl_pg_q acts, F0 issues nothing) -- registers, no
// multiplexer in front of the lookup
logic [31:0] atc_la;
logic        atc_fc2;

//--------------------------------------------------------------------------
// queue
//--------------------------------------------------------------------------
logic [15:0]   qw [QN];
// the words' predecode: D1 reads only the head's, so it is a ring (no
// shift) with the head in a register (qp0) -- loaded from the ring at the
// head after D1's take, or from F6 when the queue empties into it.  Two
// LUT RAM banks by slot parity (F6's two words land in consecutive slots,
// one per bank, so each bank has one write a cycle), each in two copies:
// the reads at the head after the two takes D1 may make
(* ramstyle = "MLAB, no_rw_check" *) logic [$bits(pd_t)-1:0] qpr_ea [QN/2];
(* ramstyle = "MLAB, no_rw_check" *) logic [$bits(pd_t)-1:0] qpr_oa [QN/2];
(* ramstyle = "MLAB, no_rw_check" *) logic [$bits(pd_t)-1:0] qpr_eb [QN/2];
(* ramstyle = "MLAB, no_rw_check" *) logic [$bits(pd_t)-1:0] qpr_ob [QN/2];
logic  [3:0]   qh;             // the ring's head
pd_t           qp0;
// synthesis translate_off
pd_t           qps [QN];       // (simulation: the shifting model, checked)
// synthesis translate_on
logic [QN-1:0] qf, qd, qa;     // faulted, demand fetch, ATC fault
logic [QN-1:0] qb;             // ends a branch the BTB took
logic [30:0]   tq [TQN];       // their targets: a ring from tq_h (a take moves
logic  [2:0]   tq_h;           // the head only, so the entries are written
logic  [3:0]   tq_n;           // at the tail whatever D1 takes)
logic  [3:0]   qcnt_q;         // words in the window (0..8), registered
logic  [4:0]   cnt;
// the head is a one-pass instruction, it fits in the queue, and no queued
// word is faulted: registered with the queue's next state, so that D1's
// common take starts from flip-flops
logic          qsimp_q;
logic [31:0]   qpc_r;
logic          odd;

assign qcnt  = qcnt_q;
assign qpc   = qpc_r;
assign q_odd = odd;
assign pd0   = qp0;
assign q_simple = qsimp_q;
always_comb begin
	for (int i = 0; i < 8; i++) begin
		win[i]      = qw[i];
		win_flt[i]  = qf[i];
		win_fdem[i] = qd[i];
		win_fatc[i] = qa[i];
		win_bt[i]   = qb[i];
	end
end
wire [30:0] tq0 = tq[tq_h];
assign bt_tgt = {tq0, 1'b0};

//--------------------------------------------------------------------------
// the pipeline after F2: the words and their flags, predecoded alongside
//--------------------------------------------------------------------------
logic  [1:0]  o2_n;            // F2's words (into the predecoder)
logic [15:0]  o2_w [2];
logic         o2_f, o2_d, o2_a;
logic  [1:0]  o2_b;            // the word ends a BTB-taken branch
logic [30:0]  o2_t;
// (F3-F6: the words and their flags alongside the predecoder's stages;
// kept in flip-flops, not shift-register memory)
(* altera_attribute = "-name AUTO_SHIFT_REGISTER_RECOGNITION OFF" *) logic  [1:0]  f3_n, f4_n, f5_n, f6_n;
(* altera_attribute = "-name AUTO_SHIFT_REGISTER_RECOGNITION OFF" *) logic [15:0]  f3_w [2], f4_w [2], f5_w [2], f6_w [2];
(* altera_attribute = "-name AUTO_SHIFT_REGISTER_RECOGNITION OFF" *) logic         f3_f, f3_d, f3_a, f4_f, f4_d, f4_a, f5_f, f5_d, f5_a, f6_f, f6_d, f6_a;
(* altera_attribute = "-name AUTO_SHIFT_REGISTER_RECOGNITION OFF" *) logic  [1:0]  f3_b, f4_b, f5_b, f6_b;
(* altera_attribute = "-name AUTO_SHIFT_REGISTER_RECOGNITION OFF" *) logic [30:0]  f3_t, f4_t, f5_t, f6_t;
pd_t          f6_p [2];
genvar gp;
generate
	for (gp = 0; gp < 2; gp++) begin : g_pd
		ap68060_predec pd (.clk(clk), .op(f3_w[gp]), .pd(f6_p[gp]));
	end
endgenerate

//--------------------------------------------------------------------------
// I-ATC, BTB
//--------------------------------------------------------------------------
logic [31:0] fpc;              // F0 fetch address (A0 clear)

// BTB: one array per word of the long word (a branch ending at word 0,
// one ending at word 1), each valid, tag, kind, target[31:1]: two taken
// branches in one long word (BSR then RTS, a short branch then RTS) no
// longer evict each other
localparam int BTB_DW = 1 + BTB_TW + 2 + 31;
logic [31:0] f1_pc;              // F1's address
// (block RAM: F0 presents fpc, F1 compares with f1_pc; an entry D1 writes
// at the edge F0's read registers it is not trusted -- no prediction)
logic [BTB_DW-1:0] btb_q0, btb_q1;
logic        btb_cf;
// The branch cache (MC68060UM 5.11) is used only with CACR.EBC.  CACR.CABC
// or CUBC (no supervisor/user tag is kept: CUBC clears the supervisor
// entries too, which only costs predictions), every CINV or CPUSH of the
// instruction cache, and reset clear it at once: each entry carries the
// epoch it was written in, a clear starts a new epoch, and only entries of
// the current one are valid.  (After 16 clears an old entry could match
// again: a prediction D1 checks like any other, at worst a redirect.)
logic                inv_busy, inv_hold;
logic                c_we;
wire                 bc_go = bc_clr || (ic_inv && !inv_busy && !inv_hold && st != S_FILL && !c_we);
logic  [3:0]         bep;          // the current epoch
logic                bs_q;         // F1's read was made in a clear's clock
logic  [3:0]         bep0, bep1;   // the epochs read with the entries
ap68060_sdpr #(.AW(BTB_AW), .DW(BTB_DW + 4)) btb0 (
	.clk(clk), .we(btb_we && !btb_wslot), .waddr(btb_wi),
	.wdata({bep, btb_wv, btb_wtag, btb_wkind, btb_wtgt}),
	.raddr(fpc[BTB_AW+1:2]), .q({bep0, btb_q0})
);
ap68060_sdpr #(.AW(BTB_AW), .DW(BTB_DW + 4)) btb1 (
	.clk(clk), .we(btb_we && btb_wslot), .waddr(btb_wi),
	.wdata({bep, btb_wv, btb_wtag, btb_wkind, btb_wtgt}),
	.raddr(fpc[BTB_AW+1:2]), .q({bep1, btb_q1})
);
always_ff @(posedge clk) btb_cf <= btb_we && (btb_wi == fpc[BTB_AW+1:2]);
always_ff @(posedge clk) begin
	if (!nreset || bc_go) bep <= bep + 4'd1;
	bs_q <= bc_go || !nreset;
end
initial bep = 4'd0;
wire bvq0 = btb_q0[BTB_DW-1] && bep0 == bep && !bs_q;
wire bvq1 = btb_q1[BTB_DW-1] && bep1 == bep && !bs_q;
// the first branch of the fetch: word 0's when the fetch starts there
wire btb_on   = cacr_q[23];
// (the tag's low BTB_CW bits are compared: a prediction, which D1 checks
// word by word -- an alias costs a redirect and the entry's rewrite, and
// the compare stays two LUTs deep ahead of F0's address select)
localparam int BTB_CW = 12;
wire btb_hit0 = btb_on && !btb_cf && bvq0 && !f1_pc[1] &&
                btb_q0[BTB_DW-1-BTB_TW +: BTB_CW] == f1_pc[BTB_AW+2 +: BTB_CW];
wire btb_hit1 = btb_on && !btb_cf && bvq1 &&
                btb_q1[BTB_DW-1-BTB_TW +: BTB_CW] == f1_pc[BTB_AW+2 +: BTB_CW];

// the fetch's return stack: pushed by calls the BTB takes, popped by the
// returns it takes (their target), resynchronized from D1's on redirects
// (a LUT RAM: one write a cycle -- a call's push, or the top entry on a
// redirect -- and the top read; the entries below the top are the fetch's
// own: only predictions, EX checks every return)
(* ramstyle = "MLAB, no_rw_check" *) logic [30:0] fras [8];
// (the top's pointer a flip-flop of its own: not absorbed into the stack's
// read-address register, with the BTB's take ahead of it)
(* preserve *) logic [2:0] fras_tp;
logic  [3:0] fras_n;
logic        rs_p1, rs_p2;      // a redirect's resynchronization, two clocks on
logic        frw_q;             // the return stack's write, registered
logic  [2:0] frwa_q;
logic [30:0] frwd_q;
logic  [3:0] atc_hv;            // the lookup: each way's hit and entry
atce_t       atc_went [4];
// (a flip-flop of its own: not absorbed into the ATC's read-address
// register, with the BTB target select ahead of it)
(* preserve *) logic [3:0] atc_set;   // atc_la's set (TC.P picks the bits), registered
logic        atc_hit1;          // (simulation: the first port's lookup, the
atce_t       atc_e1;            // reference for the pipelined one)
// the set of an address
function automatic logic [3:0] atc_sx(input logic [31:0] a, input logic p8);
	atc_sx = p8 ? a[16:13] : a[15:12];
endfunction
ap68060_atc #(.SAME_SEARCH(0)) iatc (
	.clk(clk), .nreset(nreset), .p8k(tc_p), .half(tc_q[10]),
	.set(4'd0), .set_v(1'b0), .la(atc_la),
	.fc2(atc_fc2),
	.hit(atc_hit1), .ent(atc_e1), .hv(), .wpa(),   // (the first port: page flushes)
	.la_d(atc_la), .fc2_d(atc_fc2), .set_d(atc_set), .hv_d(atc_hv), .wpa_d(), .ent_d(atc_went),
	.wr(iatc_wr), .wla(iatc_wla), .wfc2(iatc_wfc2), .went(iatc_went),
	.wsame(1'b0), .wsame_hv(4'd0),
	.flush_all(fl_all_q), .flush_page(fl_pg_q),
	.flush_nonglobal(fl_ng_q)
);

//--------------------------------------------------------------------------
// cache arrays: tags and data in block RAM (read address registered in
// F0), valid bits in LUT RAM
//--------------------------------------------------------------------------
// line valid, per set and way: a word of the four ways per set in LUT
// RAMs written at the edge and read without a clock -- one copy at F1's
// set, one at the set every write goes to, so it forms the word written
// (the ways not written keep their bits).  Clearing every line (CINVA, a
// snooped clear, reset) runs the invalidation over the sets
(* ramstyle = "MLAB, no_rw_check" *) logic [3:0] ivf [128];
(* ramstyle = "MLAB, no_rw_check" *) logic [3:0] ivo [128];
wire   [3:0]   ivf_q, ivo_q;
logic  [6:0]   ivo_ra;
logic          inv_rst;         // the invalidation is reset's clear
// synthesis translate_off
logic  [3:0]   iv [128];         // (simulation: the flip-flop model, checked)
initial for (int i = 0; i < 128; i++) iv[i] = 4'd0;
// synthesis translate_on
logic  [1:0]   rr;
logic  [6:0]   c_raddr;
logic  [6:0]   c_waddr;
logic  [1:0]   c_wway;
logic [20:0]   c_wtag;
logic [127:0]  cd [4];
logic [23:0]   ct [4];
// the valid RAMs' write, a clock after its decision: the set, the ways,
// the value; a fill's (iwq_fill: the replacement counter steps when the
// set was full), an invalidation's (iwq_sn: the snooper's, which reports
// whether it removed a valid line)
logic          iwq_v, iwq_val, iwq_fill, iwq_sn, iwq_lbh;
logic  [1:0]   inv_dn;          // an invalidation's end, signalled with its last write
logic  [6:0]   iwq_set;
logic  [3:0]   iwq_m;

logic [127:0] m_line;            // the line being filled (and the line buffer)
logic  [3:0] m_got;              // ... the long words of it that arrived
genvar gw;
generate
	for (gw = 0; gw < 4; gw++) begin : g_way
		ap68060_sdp_be #(.AW(7), .NB(16), .OREG(0)) dram (
			.clk(clk), .we(c_we && c_wway == 2'(gw)), .waddr(c_waddr),
			.wbe(16'hFFFF), .wdata(m_line), .raddr(c_raddr), .oce(1'b1), .q(cd[gw])
		);
		ap68060_sdp_be #(.AW(7), .NB(3), .OREG(0)) tram (
			.clk(clk), .we(c_we && c_wway == 2'(gw)), .waddr(c_waddr),
			.wbe(3'b111), .wdata({3'b000, c_wtag}), .raddr(c_raddr), .oce(1'b1), .q(ct[gw])
		);
	end
endgenerate

//--------------------------------------------------------------------------
// state
//--------------------------------------------------------------------------
typedef enum logic [2:0] {
	S_RUN,     // pipelined fetch
	S_WALK,    // waiting for the DMU's table walk
	S_FILL,    // line fill burst
	S_CI,      // cache-inhibited long word stream
	S_HALT     // a fault was queued: wait for a redirect
} st_t;
st_t st;

logic        fnew;             // the next F0 starts a redirected stream
logic        fspec;            // ... a predicted one (D1's, the BTB's)

// F1
logic        f1_v;
logic        f1_btok;           // F0 had room for a taken branch's target
logic        f1_s, f1_dem, f1_spec;
// (the I-ATC is looked up through its pipelined port: F0 reads the set,
// at a registered set index, and compares each way's tag; F1 has every
// way's hit and entry in flip-flops and picks the hit way)
logic  [3:0] f1_hv;             // the ATC's ways that hit
atce_t       f1_went [4];       // ... their entries
logic        f1_wmok, f1_wmf;   // the page's last walk matches: translated / failed
logic        f1_hit;
atce_t       f1_ea, f1_ew;      // the ATC's entry, the page's last walk's
logic        f1_ah;             // ... the ATC hit
atce_t       f1_e;
logic        f1_wff;            // the page's last walk failed
always_comb begin
	f1_ah = |f1_hv;
	f1_ea = '0;
	for (int w = 0; w < 4; w++) if (f1_hv[w]) f1_ea = f1_ea | f1_went[w];
	f1_hit = f1_ah || f1_wmok;
	f1_wff = !f1_ah && f1_wmf;
end

// F2: F1's outcome, registered
logic        f2_v;
logic [31:0] f2_pc;
logic        f2_s, f2_dem, f2_spec;
logic  [3:0] f2_mj;             // the way that hit (one-hot)
logic        f2_lb;             // ... or the line buffer
logic [15:0] f2_cw [4][2];      // each way's words from the fetch address on
logic [15:0] f2_lw [2];         // ... the line buffer's
logic        f2_cach, f2_flt, f2_walk;
logic [31:0] f2_pa;
logic  [1:0] f2_cm, f2_upa;
logic  [3:0] f2_vv;             // the set's valid ways (replacement)
logic        f2_btk;            // the BTB took a branch in the chunk
logic        f2_bslot;          // ... ending at word 1
logic [30:0] f2_bt;             // ... its target
logic  [1:0] f2_rop;            // ... and moved the return stack: 1 push, 2 pop

// the miss engine's words (cache-inhibited stream, TCI line, faults)
logic        md_v;
logic  [1:0] md_n;
logic [15:0] md_w [2];
logic        md_f, md_d, md_a;

// the last walk's result for its page (cleared by a redirect or a flush)
logic        wf_v, wf_ok, wf_fc2;
logic [31:0] wf_la;
atce_t       wf_e;

// line buffer: the last line filled (it also serves TCI lines)
logic        m_tci;            // the fill in progress was answered with TCI
logic        lb_v;             // m_line holds the line at lb_pa (the last fill):
logic [27:0] lb_pa;            // it is also the cache's write data

// miss engine
logic        b_req_r;          // request pending (not granted yet)
logic        busy;             // a bus transaction is latched
logic        stale;            // ... for a stream that was redirected away
logic [31:0] m_la;             // the fetch address
logic [31:0] m_pa;
logic  [1:0] m_upa;
logic        m_s, m_dem, m_ci, m_alloc;
logic        f2_tt, m_tt;      // the fetch is mapped by ITT0/ITT1 (its bus error's FSLW.TTR)
logic  [1:0] m_way;

// invalidation
logic        inv_ph;
logic  [7:0] inv_i;
logic        inv_sn;           // the running invalidation is the snooper's
logic        inv_lbh;          // ... and the line buffer held its line
logic  [1:0] inv_scope;
logic [31:0] inv_pa;

wire        redir_any = redir_v || d_redir_v;
wire [31:0] redir_npc = redir_v ? redir_pc : d_redir_pc;

//--------------------------------------------------------------------------
// F1 evaluation
//--------------------------------------------------------------------------
logic        x_ok, x_flt, x_walk, x_cach, x_lb;
logic [31:0] x_pa;
logic  [1:0] x_cm, x_upa;
logic  [3:0] x_mj;
// the ITT matches are formed in F0 (the TTRs change only by MOVEC, which
// serializes and refetches), so F1 starts from them
logic        f1_tt0, f1_tt1;
// ... and whether translation is on (TCR.E, MDIS), taken with them: F1's
// translation and its tag compare select both start from flip-flops
logic        f1_tce;
assign f1_e = f1_ah ? f1_ea : f1_ew;
// the tag the ATC's translation gives
wire [20:0] f1_tg = {f1_e.pa[19:1], tc_p ? f1_pc[12] : f1_e.pa[0], f1_pc[11]};
always_comb begin
	logic tt0, tt1, use_t;
	logic [20:0] tg_a [4];
	logic [20:0] tg_w;
	x_ok = 1'b0; x_flt = 1'b0; x_walk = 1'b0;
	x_pa = f1_pc; x_cm = 2'b00; x_upa = 2'b00;
	tt0 = f1_tt0;
	tt1 = f1_tt1;
	if (tt0 || tt1) begin
		x_ok  = 1'b1;
		x_cm  = tt0 ? itt0_q[6:5] : itt1_q[6:5];
		x_upa = tt0 ? itt0_q[9:8] : itt1_q[9:8];
	end
	else if (!f1_tce) begin
		// the default translation: the TCR's DCI and DUI (MC68060UM 4.1.2)
		x_ok  = 1'b1;
		x_cm  = tc_q[4:3];
		x_upa = tc_q[2:1];
	end
	else if (f1_hit) begin
		x_pa  = {f1_tg[20:1], f1_pc[11:0]};
		x_cm  = f1_e.cm;
		x_upa = f1_e.upa;
		x_ok  = 1'b1;
	end
	else if (f1_wff) x_flt = 1'b1;   // the page's search failed
	else x_walk = 1'b1;
	x_cach = x_ok && ic_en && !x_cm[1];
	// the tags compare with both candidate addresses, the logical one (a
	// TTR or no translation) and the ATC's, and the translation selects
	// the outcome (the ATC's address matters only
	// with a hit, x_cach is clear without one)
	// (three compares apart: the logical address, the ATC entry's, the
	// page's last walk's -- the entry select is not ahead of them either)
	use_t  = !(tt0 || tt1) && f1_tce;
	// (each ATC way's tag apart: the hit way picks the outcome)
	tg_w   = {f1_ew.pa[19:1], tc_p ? f1_pc[12] : f1_ew.pa[0], f1_pc[11]};
	for (int w = 0; w < 4; w++)
		tg_a[w] = {f1_went[w].pa[19:1], tc_p ? f1_pc[12] : f1_went[w].pa[0], f1_pc[11]};
	for (int i = 0; i < 4; i++) begin
		logic [3:0] eqa;
		for (int w = 0; w < 4; w++) eqa[w] = (ct[i][20:0] == tg_a[w]);
		x_mj[i] = ivf_q[i] && (use_t ? (f1_ah ? |(f1_hv & eqa) : (ct[i][20:0] == tg_w))
		                             : (ct[i][20:0] == f1_pc[31:11]));
	end
	begin
		logic [3:0] lba;
		for (int w = 0; w < 4; w++) lba[w] = (lb_pa == {tg_a[w], f1_pc[10:4]});
		x_lb = x_cach && lb_v && (use_t ? (f1_ah ? |(f1_hv & lba) : (lb_pa == {tg_w, f1_pc[10:4]}))
		                                : (lb_pa == f1_pc[31:4]));
	end
end

// the BTB's taken branch: F0 goes to the target (a return's from the
// stack); the sequential fetch F0 makes in this clock is dropped.  Each
// slot's take, kind and target are formed apart and the hits pick last (a
// return with the stack empty takes nothing)
wire  [30:0] fras_top = fras[fras_tp];
wire   [1:0] bk0  = btb_q0[32:31];
wire   [1:0] bk1  = btb_q1[32:31];
wire         tk0  = btb_hit0 && !(bk0 == 2'd2 && fras_n == 4'd0);
wire         tk1  = !btb_hit0 && btb_hit1 && !(bk1 == 2'd2 && fras_n == 4'd0);
wire         f1_bt    = f1_btok && (tk0 || tk1);
wire         f1_bslot = !tk0;
wire         f1_bcall = (tk0 && bk0 == 2'd1) || (tk1 && bk1 == 2'd1);
wire         f1_bret  = (tk0 && bk0 == 2'd2) || (tk1 && bk1 == 2'd2);
wire  [30:0] f1_t     = tk0 ? ((bk0 == 2'd2) ? fras_top : btb_q0[30:0])
                            : ((bk1 == 2'd2) ? fras_top : btb_q1[30:0]);

//--------------------------------------------------------------------------
// F2 evaluation: the hit's words, or the miss
//--------------------------------------------------------------------------
// (the back end's and D1's redirects are far-away registers: they only
// override, last; a miss that starts in a redirect's clock is cancelled
// by it, below)
wire f2_hit  = (|f2_mj && f2_cach) || f2_lb;
wire f2_go   = f2_v;
wire f2_miss = f2_go && !f2_hit;
always_comb begin
	logic [15:0] w0, w1;
	// (one AND-OR over the sources: the hit way, the line buffer, the miss
	// engine -- never two at once -- straight into the predecoder; a fault
	// selects none, zero words)
	w0 = '0; w1 = '0;
	for (int i = 0; i < 4; i++) if (f2_mj[i]) begin w0 = w0 | f2_cw[i][0]; w1 = w1 | f2_cw[i][1]; end
	if (f2_lb) begin w0 = w0 | f2_lw[0]; w1 = w1 | f2_lw[1]; end
	if (md_v)  begin w0 = w0 | md_w[0];  w1 = w1 | md_w[1];  end
	o2_n = '0; o2_w[0] = w0; o2_w[1] = w1;
	o2_f = 1'b0; o2_d = 1'b0; o2_a = 1'b0;
	o2_b = '0; o2_t = f2_bt;
	if (f2_go && f2_hit) begin
		o2_n = f2_pc[1] ? 2'd1 : 2'd2;
		if (f2_btk) begin
			// a predicted taken branch ends in this chunk: the words up to
			// its last, that one flagged
			o2_n = (f2_pc[1] || !f2_bslot) ? 2'd1 : 2'd2;
			o2_b = (f2_pc[1] || !f2_bslot) ? 2'b01 : 2'b10;
		end
	end
	else if (f2_go && (f2_flt || (f2_walk && f2_spec))) begin
		// ATC fault (the page's search failed), or a predicted stream's
		// ATC miss (speculative): the long word's words carry it
		o2_n = f2_pc[1] ? 2'd1 : 2'd2;
		o2_f = 1'b1; o2_d = f2_dem; o2_a = 1'b1;
	end
	else if (md_v) begin
		o2_n = md_n;
		o2_f = md_f; o2_d = md_d; o2_a = md_a;
	end
end

//--------------------------------------------------------------------------
// F0 issue: room for this chunk and those in flight
//--------------------------------------------------------------------------
// (two words for each stage that may hold some, and the miss engine's)
wire [4:0] inflight = {f1_v, 1'b0} + {f2_v || md_v, 1'b0} + {3'd0, f3_n} +
                      {3'd0, f4_n} + {3'd0, f5_n} + {3'd0, f6_n};
// (registered: room_q, formed a clock ahead for each take and for F1
// receiving a fetch or not -- F0 issuing, the BTB's redirect not dropping
// it -- from what the stages will hold: F2 F1's fetch unless F2 misses
// (two words; the miss engine's), F3 F2's words, F4-F6 F3-F5's)
logic      room_q;
logic      m_setpc;            // the miss engine moves fpc on, a clock late
// (at most: F1's fetch and F2's words as if F2 hits -- F2's outcome, a
// tag compare, stays off the room; a miss only holds F0 a clock longer,
// and F0 rolls back then anyway)
wire [1:0] o2_ub   = f2_go ? (f2_pc[1] ? 2'd1 : 2'd2) : md_v ? md_n : 2'd0;
wire [5:0] infl_b  = {f1_v || (st != S_RUN), 2'b00} + {4'd0, o2_ub} +
                     {4'd0, f3_n} + {4'd0, f4_n} + {4'd0, f5_n};
logic [1:0] room_n [3];
// the target FIFO cannot overflow: count the flagged words on their way
wire       bt_room  = ({1'b0, tq_n} + {4'd0, f1_v} + {4'd0, f2_btk} + {4'd0, |f3_b} +
                       {4'd0, |f4_b} + {4'd0, |f5_b} + {4'd0, |f6_b}) < 5'(TQN);
// a snooped invalidation goes first (the snooper has priority, 4.5)
// (a redirect in the same clock overrides what F0 loads: it is not here)
wire       f0_go    = (st == S_RUN) && !odd && !stop_f && !m_setpc &&
                      !iatc_flush_page && !fl_pg_q && !inv_busy && !sn_inv && room_q;

// F0's next address: from the stages' registers (a redirect, F2's miss,
// F0's own advance, the miss engine's), or a BTB slot's target -- each
// slot's take, from its RAM's output, picks last, one LUT before fpc and
// atc_la (kept nets: synthesis would otherwise select the target first)
(* keep *) wire [31:0] fpc_b;
(* keep *) wire [31:0] atc_b;
(* keep *) wire [30:0] f1_t0;
(* keep *) wire [30:0] f1_t1;
(* keep *) wire        f1_tg0;
(* keep *) wire        f1_tg1;
// (F0's advance, a decision over many flops, picks the sequential address
// last: the others are formed apart)
(* keep *) wire [31:0] fpc_o;
(* keep *) wire        fpc_sq;
assign fpc_o  = redir_any ? {redir_npc[31:1], 1'b0} : f2_miss ? f2_pc : m_setpc ? m_la : fpc;
assign fpc_sq = f0_go && !redir_any && !f2_miss;
assign fpc_b  = fpc_sq ? {fpc[31:2], 2'b00} + 32'd4 : fpc_o;
assign atc_b  = iatc_flush_page ? iatc_flush_la : fpc_b;
assign f1_t0  = (bk0 == 2'd2) ? fras_top : btb_q0[30:0];
assign f1_t1  = (bk1 == 2'd2) ? fras_top : btb_q1[30:0];
assign f1_tg0 = f1_v && f1_btok && !f2_miss && !redir_any && tk0;
assign f1_tg1 = f1_v && f1_btok && !f2_miss && !redir_any && tk1;
(* keep *) wire        f1_ta0;     // (the ATC's: a page flush's address goes first)
(* keep *) wire        f1_ta1;
assign f1_ta0 = f1_tg0 && !iatc_flush_page;
assign f1_ta1 = f1_tg1 && !iatc_flush_page;
// the next-address registers' own selects: the qualifiers (far from the
// BTB's RAMs) kept apart, so each select is one function of them and the
// hits -- the hits' compare feeds one level, not the chain above
(* keep *) wire f1_qg = f1_v && f1_btok && !f2_miss && !redir_any;
(* keep *) wire f1_qa = f1_v && f1_btok && !f2_miss && !redir_any && !iatc_flush_page;
(* keep *) wire fras_e = (fras_n == 4'd0);
wire f1_tg0_l = f1_qg && btb_hit0 && !(bk0 == 2'd2 && fras_e);
wire f1_tg1_l = f1_qg && !btb_hit0 && btb_hit1 && !(bk1 == 2'd2 && fras_e);
wire f1_ta0_l = f1_qa && btb_hit0 && !(bk0 == 2'd2 && fras_e);
wire f1_ta1_l = f1_qa && !btb_hit0 && btb_hit1 && !(bk1 == 2'd2 && fras_e);
// F0's next address and the ATC's as one AND-OR each: every source's
// select is one function of the stage registers, F2's hit and the BTB's
// takes, so the chain fpc_o -> fpc_b -> atc_b -> the register is not walked
(* keep *) wire        f1_tkq = f1_v && f1_btok && (tk0 || tk1);
(* keep *) wire [31:0] fpc_p4 = {fpc[31:2], 2'b00} + 32'd4;
wire n_rd = redir_any;
wire n_ms = !redir_any && f2_miss;
wire n_ml = !redir_any && !f2_miss && !f1_tkq && m_setpc;
wire n_sq = !redir_any && !f2_miss && !f1_tkq && f0_go;
wire n_pc = !redir_any && !f2_miss && !f1_tkq && !f0_go && !m_setpc;
wire [31:0] fpc_nv = ({32{f1_tg0_l}} & {f1_t0, 1'b0}) | ({32{f1_tg1_l}} & {f1_t1, 1'b0}) |
                     ({32{n_rd}} & {redir_npc[31:1], 1'b0}) | ({32{n_ms}} & f2_pc) |
                     ({32{n_ml}} & m_la) | ({32{n_sq}} & fpc_p4) | ({32{n_pc}} & fpc);
wire [31:0] atc_nv = ({32{f1_ta0_l}} & {f1_t0, 1'b0}) | ({32{f1_ta1_l}} & {f1_t1, 1'b0}) |
                     ({32{iatc_flush_page}} & iatc_flush_la) |
                     ({32{!iatc_flush_page && n_rd}} & {redir_npc[31:1], 1'b0}) |
                     ({32{!iatc_flush_page && n_ms}} & f2_pc) |
                     ({32{!iatc_flush_page && n_ml}} & m_la) |
                     ({32{!iatc_flush_page && n_sq}} & fpc_p4) |
                     ({32{!iatc_flush_page && n_pc}} & fpc);
// synthesis translate_off
always_ff @(posedge clk)
	if (nreset && (fpc_nv !== (f1_tg0 ? {f1_t0, 1'b0} : f1_tg1 ? {f1_t1, 1'b0} : fpc_b) ||
	               atc_nv !== (f1_ta0 ? {f1_t0, 1'b0} : f1_ta1 ? {f1_t1, 1'b0} : atc_b))) begin
		$display("FAIL: fetch: the next addresses %h %h differ from the chain's %h %h", fpc_nv, atc_nv,
		         (f1_tg0 ? {f1_t0, 1'b0} : f1_tg1 ? {f1_t1, 1'b0} : fpc_b),
		         (f1_ta0 ? {f1_t0, 1'b0} : f1_ta1 ? {f1_t1, 1'b0} : atc_b));
		$finish;
	end
// synthesis translate_on
// synthesis translate_off
always_ff @(posedge clk)
	if (f1_tg0_l !== f1_tg0 || f1_tg1_l !== f1_tg1 || f1_ta0_l !== f1_ta0 || f1_ta1_l !== f1_ta1) begin
		$display("FAIL: fetch: the next-address selects %b%b%b%b differ from the take flags %b%b%b%b",
		         f1_tg0_l, f1_tg1_l, f1_ta0_l, f1_ta1_l, f1_tg0, f1_tg1, f1_ta0, f1_ta1);
		$finish;
	end
// synthesis translate_on

assign c_raddr = inv_busy ? inv_i[6:0] : fpc[10:4];

//--------------------------------------------------------------------------
// BIU request
//--------------------------------------------------------------------------
assign b_req = b_req_r;
always_comb begin
	b_breq = '0;
	b_breq.addr = {m_pa[31:2], 2'b00};
	b_breq.rd   = 1'b1;
	b_breq.tt   = TT_NORMAL;
	b_breq.tm   = m_s ? TM_SCODE : TM_UCODE;
	b_breq.upa  = m_upa;
	if (st == S_FILL) begin
		b_breq.siz = SIZ_LINE;
		b_breq.tln = m_way;
	end
	else begin
		b_breq.siz = SIZ_L;
		b_breq.ci  = m_ci;
	end
end

assign iw_va  = m_la;
assign iw_fc2 = m_s;

// a transaction still outstanding after this cycle
wire out_next = (b_req_r && !b_gnt) || b_gnt || (busy && !(b_done || b_err));

//--------------------------------------------------------------------------
// the queue's next state for the three takes, from registers: none, the
// head instruction (qp0.tot), the decided length (cr_use)
//--------------------------------------------------------------------------
typedef struct packed {
	logic [4:0]    cnt;
	logic [3:0]    qh;
	logic [31:0]   qpc;
	logic [3:0]    tq_n;
	logic          tq_sh;       // the targets shift (a flagged word was taken)
	logic [QN-1:0] qf, qd, qa, qb;
} qst_t;
logic [15:0] qw_n [3][QN];
qst_t        qs_n [3];
pd_t         qp0_n [3];
logic  [2:0] fits_n;
// a faulted word in the queue or arriving: (conservative: any queued word,
// not only the eight D1 sees -- the faulted word is the last one fetch
// delivers, so this only sends an instruction just before it down D1's
// general path; the bits past the count are clear)
wire         qfany = (|qf) || (f6_n != 2'd0 && f6_f);
// the count as a thermometer (bit j: j < cnt; a register, kept with the
// count), and the count with F6's words (j < cnt + f6_n), to 31
logic [QN-1:0] cth_q;
logic [QN+1:0] cthf_q;         // ... with F6's words: registered (F5's count)
logic   [31:0] cth, cthf;
assign cth  = {{(32 - QN){1'b0}}, cth_q};
assign cthf = {{(30 - QN){1'b0}}, cthf_q};
logic  [2:0] tk [3];
assign tk[0] = 3'd0;
assign tk[1] = qp0.tot[2:0];
assign tk[2] = cr_use;
// the ring's reads at the head after each take
pd_t         qpr_rda, qpr_rdb;
always_comb begin
	logic [3:0] ha, hb;
	ha = 4'(qh + 4'(tk[1]));
	hb = 4'(qh + 4'(tk[2]));
	qpr_rda = pd_t'(ha[0] ? qpr_oa[ha[3:1]] : qpr_ea[ha[3:1]]);
	qpr_rdb = pd_t'(hb[0] ? qpr_ob[hb[3:1]] : qpr_eb[hb[3:1]]);
end
wire [6:0] room_base = {2'b00, cnt} + {5'd0, f6_n} + {1'b0, infl_b};   // (the room compares' shared sum)
always_comb begin
	for (int c = 0; c < 3; c++) begin
		logic [4:0] keep;
		logic       cfl;
		keep = cnt - {2'b00, tk[c]};
		cfl  = 1'b0;
		for (int i = 0; i < 7; i++) if (3'(i) < tk[c] && qb[i]) cfl = 1'b1;
		qs_n[c] = '0;
		for (int i = 0; i < QN; i++) begin
			logic [5:0] src;
			logic       kept, k0;
			src  = 6'(i) + {3'b000, tk[c]};
			// i < keep and i < keep + F6's count, read off the count's
			// thermometers at i + the take (no subtraction in the words'
			// select); the F6 word's parity is i - keep's
			kept = cth[src[4:0]];
			k0   = i[0] ^ keep[0];
			// (a word past the count is never read: the words and their
			// flags but the fault flag take F6's there whether it brings
			// them or not -- F6's count stays off the words' select; the
			// fault flags past the count are clear: qfany)
			if (kept) begin
				qw_n[c][i]    = qw[src[3:0]];
				qs_n[c].qf[i] = qf[src[3:0]];
				qs_n[c].qd[i] = qd[src[3:0]];
				qs_n[c].qa[i] = qa[src[3:0]];
				qs_n[c].qb[i] = qb[src[3:0]];
			end
			else begin
				qw_n[c][i]    = f6_w[k0];
				qs_n[c].qf[i] = cthf[src[4:0]] && f6_f;
				qs_n[c].qd[i] = f6_d;
				qs_n[c].qa[i] = f6_a;
				qs_n[c].qb[i] = f6_b[k0];
			end
		end
		qs_n[c].cnt = keep + {3'b000, f6_n};
		qs_n[c].qh  = 4'(qh + 4'(tk[c]));
		// D1 took the end of a BTB-taken branch: the stream continues at
		// its target (the words after it in the queue came from there).
		// (A decode that ran past the flag is refetched at WB: the PC of
		// what follows it does not matter.)
		qs_n[c].qpc   = cfl ? {tq0, 1'b0} : qpc_r + {28'd0, tk[c], 1'b0};
		qs_n[c].tq_sh = cfl && tq_n != 4'd0;
		qs_n[c].tq_n  = tq_n - {3'd0, qs_n[c].tq_sh} +
		                {3'd0, f6_n != 2'd0 && f6_b != 2'b00};
	end
	// the head's predecode after each take
	qp0_n[0] = (cnt == 5'd0) ? f6_p[0] : qp0;
	qp0_n[1] = (cnt == {2'b00, tk[1]}) ? f6_p[0] : qpr_rda;
	qp0_n[2] = (cnt == {2'b00, tk[2]}) ? f6_p[0] : qpr_rdb;
	for (int c = 0; c < 3; c++) begin
		fits_n[c]  = (qs_n[c].cnt >= 5'd8) || ({1'b0, qp0_n[c].tot} <= qs_n[c].cnt);
		// (cnt - tk + f6_n + infl_b <= QN-2, as one shared sum of the
		// registers against a threshold raised by the take: two adders in
		// series, not three; the take never exceeds the count)
		room_n[c][0] = room_base <= 7'(QN - 2) + {4'd0, tk[c]};
		room_n[c][1] = room_base <= 7'(QN - 4) + {4'd0, tk[c]};
		// synthesis translate_off
		// (a candidate taking more than the queue holds is never the one
		// taken: its count wrapped, its room is not compared)
		if ({2'b00, tk[c]} <= cnt &&
		    (({1'b0, qs_n[c].cnt} + infl_b <= 6'(QN - 2)) != room_n[c][0] ||
		     ({1'b0, qs_n[c].cnt} + infl_b + 6'd2 <= 6'(QN - 2)) != room_n[c][1])) begin
			$display("FAIL: fetch: the queue's room %b%b differs from its sum (cnt %0d take %0d f6_n %0d infl %0d)",
			         room_n[c][0], room_n[c][1], cnt, tk[c], f6_n, infl_b);
			$finish;
		end
		// synthesis translate_on
	end
end
// The take (the head instruction, the decided length, or none) picks the
// queue's next state last, one LUT before each flop: it is D1's decision,
// one LUT from registers but fanned out over the whole queue.  The three
// candidates are kept nets -- synthesis would otherwise share their three
// shifters as one shifter by the take's length, with the take first.
typedef struct packed {
	logic [QN-1:0][15:0] w;
	qst_t                s;
	pd_t                 p0;
	logic                simple;
	logic          [1:0] room;
	logic          [3:0] qcnt;
	logic       [QN-1:0] cth;
	logic       [QN+1:0] cthf;
} qc_t;
qc_t qc [3];
always_comb
	for (int c = 0; c < 3; c++) begin
		for (int i = 0; i < QN; i++) qc[c].w[i] = qw_n[c][i];
		qc[c].s      = qs_n[c];
		qc[c].p0     = qp0_n[c];
		qc[c].simple = qp0_n[c].simple && fits_n[c] && !qfany;
		qc[c].room   = room_n[c];
		qc[c].qcnt   = (qs_n[c].cnt > 5'd8) ? 4'd8 : qs_n[c].cnt[3:0];
		for (int j = 0; j < QN; j++) qc[c].cth[j] = (5'(j) < qs_n[c].cnt);
		for (int j = 0; j < QN + 2; j++) qc[c].cthf[j] = (6'(j) < {1'b0, qs_n[c].cnt} + {4'd0, f5_n});
	end
(* keep *) wire [$bits(qc_t)-1:0] qk0;
(* keep *) wire [$bits(qc_t)-1:0] qk1;
(* keep *) wire [$bits(qc_t)-1:0] qk2;
assign qk0 = qc[0];
assign qk1 = qc[1];
assign qk2 = qc[2];
qc_t qn;                       // the take's
assign qn = qc_t'(cs_fire ? qk1 : cr_fire ? qk2 : qk0);

//--------------------------------------------------------------------------
// the queue's data and the words on their way to it: no reset (the count
// and the stages' word counts qualify them), no enable
//--------------------------------------------------------------------------
always_ff @(posedge clk) begin
	for (int i = 0; i < QN; i++) qw[i] <= qn.w[i];
	qf    <= qn.s.qf;
	qd    <= qn.s.qd;
	qa    <= qn.s.qa;
	qb    <= qn.s.qb;
	qp0   <= qn.p0;
	qpc_r <= redir_any ? redir_npc : qn.s.qpc;
	// the predecode ring: appended at head + count (wherever D1's
	// take puts the head)
	begin
		// (one write statement per bank, the word by the slot's parity:
		// two statements read as two write ports and keep it in logic)
		logic [3:0] w0, w1;
		logic       we_e, we_o;
		w0 = 4'(qh + cnt[3:0]);
		w1 = 4'(qh + cnt[3:0] + 4'd1);
		we_e = (f6_n != 2'd0 && !w0[0]) || (f6_n == 2'd2 && !w1[0]);
		we_o = (f6_n != 2'd0 &&  w0[0]) || (f6_n == 2'd2 &&  w1[0]);
		if (we_e) begin
			qpr_ea[w0[0] ? w1[3:1] : w0[3:1]] <= w0[0] ? f6_p[1] : f6_p[0];
			qpr_eb[w0[0] ? w1[3:1] : w0[3:1]] <= w0[0] ? f6_p[1] : f6_p[0];
		end
		if (we_o) begin
			qpr_oa[w0[0] ? w0[3:1] : w1[3:1]] <= w0[0] ? f6_p[0] : f6_p[1];
			qpr_ob[w0[0] ? w0[3:1] : w1[3:1]] <= w0[0] ? f6_p[0] : f6_p[1];
		end
	end
	// the targets of flagged words: F6's joins at the tail (the first
	// leaves when D1 takes its word: the head moves, below)
	if (f6_n != 2'd0 && f6_b != 2'b00) tq[3'(tq_h + tq_n[2:0])] <= f6_t;
	f6_w  <= f5_w; f6_f <= f5_f; f6_d <= f5_d; f6_a <= f5_a; f6_t <= f5_t;
	f5_w  <= f4_w; f5_f <= f4_f; f5_d <= f4_d; f5_a <= f4_a; f5_t <= f4_t;
	f4_w  <= f3_w; f4_f <= f3_f; f4_d <= f3_d; f4_a <= f3_a; f4_t <= f3_t;
	f3_w  <= o2_w; f3_f <= o2_f; f3_d <= o2_d; f3_a <= o2_a; f3_t <= o2_t;
end

//--------------------------------------------------------------------------
// sequential
//--------------------------------------------------------------------------
// (the reset, registered here: the core's is a gate on its reset counters
// and reaches every module; the fetch leaves and enters reset a clock
// later, long before the reset exception's redirect)
logic nrst;
always_ff @(posedge clk) nrst <= nreset;
always_ff @(posedge clk) begin
	logic        frw;              // the return stack's write (below)
	logic  [2:0] frwa;
	logic [30:0] frwd;
	logic  [6:0] iw_set;           // the valid RAMs' write decision (below)
	logic  [3:0] iw_m;
	logic        iw_v, iw_fill;
	frw = 1'b0; frwa = '0; frwd = '0;
	iw_set = '0; iw_m = '0; iw_v = 1'b0; iw_fill = 1'b0;
	if (!nrst) begin
		cnt   <= '0;
		odd   <= 1'b0;
		fpc   <= '0;
		atc_la <= '0;
		atc_set <= '0;
		atc_fc2 <= 1'b0;
		fnew  <= 1'b1;
		fspec <= 1'b0;
		st    <= S_RUN;
		stop_q <= 1'b1;
		stop_f <= 1'b1;
		fl_all_q <= 1'b0; fl_pg_q <= 1'b0; fl_ng_q <= 1'b0;
		f1_v  <= 1'b0; f1_pc <= '0; f1_s <= 1'b0; f1_dem <= 1'b0;
		f1_hv <= '0; f1_ew <= '0; f1_wmok <= 1'b0; f1_wmf <= 1'b0;
		f1_spec <= 1'b0;
		f2_v  <= 1'b0; f2_btk <= 1'b0; f2_rop <= '0;
		md_v  <= 1'b0;
		wf_v  <= 1'b0; wf_ok <= 1'b0; wf_fc2 <= 1'b0; wf_la <= '0; wf_e <= '0;
		if_cause <= '0;
		f1_tt0 <= 1'b0; f1_tt1 <= 1'b0; f1_tce <= 1'b0;
		f1_btok <= 1'b0;
		fras_tp <= '0; fras_n <= '0;
		rs_p1 <= 1'b0; rs_p2 <= 1'b0;
		tq_n <= '0;
		tq_h <= '0;
		f3_n  <= '0; f4_n <= '0; f5_n <= '0; f6_n <= '0;
		f3_b  <= '0; f4_b <= '0; f5_b <= '0; f6_b <= '0;
		lb_v  <= 1'b0; lb_pa <= '0; m_tci <= 1'b0;
		busy  <= 1'b0; stale <= 1'b0; b_req_r <= 1'b0;
		m_la  <= '0; m_pa <= '0; m_upa <= '0; m_s <= 1'b0; m_dem <= 1'b0; m_ci <= 1'b0; m_tt <= 1'b0;
		m_alloc <= 1'b0; m_way <= '0; m_line <= '0;
		iw_req <= 1'b0;
		c_we  <= 1'b0; c_waddr <= '0; c_wway <= '0; c_wtag <= '0;
		rr    <= '0;
		iwq_v <= 1'b0; iwq_val <= 1'b0; iwq_fill <= 1'b0; iwq_sn <= 1'b0; iwq_set <= '0; iwq_m <= '0;
		iwq_lbh <= 1'b0; inv_dn <= '0;
		// (reset leaves the cache alone, 5.3: its valid bits have their
		// power-up values from their initial blocks)
		inv_busy <= 1'b0; inv_hold <= 1'b0; inv_ph <= 1'b0; inv_i <= '0;
		inv_sn <= 1'b0; inv_scope <= 2'd3; inv_pa <= '0; inv_lbh <= 1'b0;
		inv_rst <= 1'b0;
		sn_ihit <= 1'b0;
		ic_inv_done <= 1'b0;
		sn_inv_done <= 1'b0;
		qh  <= '0;
		qsimp_q <= 1'b0;
		qcnt_q  <= '0;
		cth_q   <= '0;
		cthf_q  <= '0;
		room_q  <= 1'b1;
		m_setpc <= 1'b0;
	end
	else begin
		c_we        <= 1'b0;
		ic_inv_done <= 1'b0;
		sn_inv_done <= 1'b0;
		sn_ihit     <= 1'b0;
		inv_hold    <= 1'b0;
		stop_q      <= stop;
		stop_f      <= stop;
		// F0's address, and the ATC's (fpc's copy, or a page flush's)
		fpc         <= fpc_nv;
		atc_la      <= atc_nv;
		atc_set     <= atc_sx(atc_nv, tc_p);
		atc_fc2     <= smode;
		fl_all_q    <= iatc_flush_all;
		fl_pg_q     <= iatc_flush_page;
		fl_ng_q     <= iatc_flush_ng;

		//------------------------------------------------------------------
		// queue: D1's take, F6 appended; F2 -> F3 -> ... -> F6
		//------------------------------------------------------------------
		// synthesis translate_off
		begin
			logic [4:0] keep;
			logic [1:0] tsel;
			tsel = cs_fire ? 2'd1 : cr_fire ? 2'd2 : 2'd0;
			keep = cnt - {2'b00, tk[tsel]};
			for (int i = 0; i < QN; i++) begin
				logic [5:0] src;
				logic [4:0] k;
				src = 6'(i) + {3'b000, tk[tsel]};
				k   = 5'(i) - keep;
				if (5'(i) < keep) qps[i] <= qps[src[3:0]];
				else if (k < {3'b000, f6_n}) qps[i] <= f6_p[k[0]];
			end
		end
		// synthesis translate_on
		cnt     <= qn.s.cnt;
		qh      <= qn.s.qh;
		qsimp_q <= qn.simple;
		qcnt_q  <= qn.qcnt;
		cth_q   <= qn.cth;
		cthf_q  <= qn.cthf;
		// (F1 receives a fetch: F0 issues and no BTB take drops it -- the
		// kept takes; a redirect overrides the room below)
		room_q  <= qn.room[f0_go && !f1_tg0 && !f1_tg1 && !f2_miss];
		tq_n    <= qn.s.tq_n;
		tq_h    <= tq_h + {2'b00, qn.s.tq_sh};
		f6_n  <= f5_n;
		f6_b  <= f5_b;
		f5_n  <= f4_n;
		f5_b  <= f4_b;
		f4_n  <= f3_n;
		f4_b  <= f3_b;
		f3_n  <= o2_n;
		f3_b  <= o2_b;
		md_v  <= 1'b0;

		// the redirect's return stack, two clocks on: D1's registers then
		// hold the state a back-end redirect restored (and D1 took nothing
		// since).  (A BTB call or return F1 takes in this clock -- the
		// first fetch after the redirect -- is not kept: a prediction.)
		rs_p1 <= 1'b0;
		rs_p2 <= rs_p1;

		// the miss engine's next long word: fpc follows m_la a clock late
		// (F0 waits for it: m_setpc)
		m_setpc <= 1'b0;

		//------------------------------------------------------------------
		// F0 -> F1
		//------------------------------------------------------------------
		f1_v    <= 1'b0;
		f1_btok <= 1'b0;
		if (f0_go) begin
			f1_v   <= 1'b1;
			f1_btok <= bt_room;
			f1_pc  <= fpc;
			f1_s   <= sm_q;
			f1_tt0 <= ttr_hit(itt0_q, fpc, sm_q);
			f1_tt1 <= ttr_hit(itt1_q, fpc, sm_q);
			f1_tce <= tc_e;
			f1_dem <= fnew && !fspec;
			f1_spec <= fspec;
			begin
				// the ATC, else the page's last walk (formed here, as the
				// ATC's lookup is registered, so F1 starts from flip-flops)
				logic  wm;
				wm = wf_v && wf_fc2 == sm_q && wf_la[31:13] == fpc[31:13] &&
				     (tc_p || wf_la[12] == fpc[12]);
				f1_hv   <= atc_hv;
				f1_went <= atc_went;
				f1_ew   <= wf_e;
				f1_wmok <= wm && wf_ok;
				f1_wmf  <= wm && !wf_ok;
			end
			fnew   <= 1'b0;
		end

		//------------------------------------------------------------------
		// F1 -> F2: the outcome registered; a BTB-taken branch redirects
		// F0 now (unless F2 misses: F2's chunk is older)
		//------------------------------------------------------------------
		f2_v    <= f1_v && !f2_miss;
		f2_pc   <= f1_pc;
		f2_s    <= f1_s;
		f2_dem  <= f1_dem;
		f2_spec <= f1_spec;
		// (the selects of F2's AND-OR: clear without a fetch in F1, the line
		// buffer before the ways)
		f2_mj   <= (f1_v && !x_lb) ? (x_mj & {4{x_cach}}) : 4'd0;
		f2_lb   <= f1_v && x_lb;
		for (int i = 0; i < 4; i++) begin
			logic [31:0] lw;
			lw = cd[i][127 - 32 * f1_pc[3:2] -: 32];
			f2_cw[i][0] <= f1_pc[1] ? lw[15:0] : lw[31:16];
			f2_cw[i][1] <= lw[15:0];
		end
		begin
			logic [31:0] lw;
			lw = m_line[127 - 32 * f1_pc[3:2] -: 32];
			f2_lw[0] <= f1_pc[1] ? lw[15:0] : lw[31:16];
			f2_lw[1] <= lw[15:0];
		end
		f2_cach <= x_cach;
		f2_flt  <= x_flt;
		f2_walk <= x_walk;
		f2_pa   <= x_pa;
		f2_cm   <= x_cm;
		f2_tt   <= f1_tt0 || f1_tt1;
		f2_upa  <= x_upa;
		f2_vv   <= ivf_q;
		f2_btk  <= 1'b0;
		f2_rop  <= 2'd0;
		f2_bslot <= f1_bslot;
		f2_bt   <= f1_t;
		if (f1_v && !f2_miss && f1_bt) begin
			// a predicted taken branch ends in this chunk: F0 goes to the
			// target and its sequential fetch of this cycle is dropped; a
			// call pushes its return address
			f2_btk <= 1'b1;
			f1_v    <= 1'b0;
			f1_btok <= 1'b0;
			fnew  <= 1'b0;
			fspec <= 1'b1;
			if (f1_bcall) begin
				frw  = 1'b1;
				frwa = fras_tp + 3'd1;
				frwd = f1_bslot ? ({f1_pc[31:2], 1'b1} + 31'd1) : ({f1_pc[31:2], 1'b0} + 31'd1);
				fras_tp <= fras_tp + 3'd1;
				if (fras_n != 4'd8) fras_n <= fras_n + 4'd1;
				f2_rop  <= 2'd1;
			end
			else if (f1_bret) begin
				fras_tp <= fras_tp - 3'd1;
				fras_n  <= fras_n - 4'd1;
				f2_rop  <= 2'd2;
			end
		end

		//------------------------------------------------------------------
		// F2: a miss goes to the miss engine, F0 rolls back to the missed
		// address (F1's younger fetch is dropped), and the return stack
		// moves back if the chunk's BTB branch moved it
		//------------------------------------------------------------------
		if (f2_miss) begin
			f1_v   <= 1'b0;
			f1_btok <= 1'b0;
			fnew   <= f2_dem;
			m_la   <= f2_pc;
			m_pa   <= f2_pa;
			m_upa  <= f2_upa;
			m_s    <= f2_s;
			m_dem  <= f2_dem;
			m_ci   <= f2_cm[1];
			m_tt   <= f2_tt;
			if (f2_rop == 2'd1) begin
				fras_tp <= fras_tp - 3'd1;
				fras_n  <= fras_n - 4'd1;
			end
			else if (f2_rop == 2'd2) begin
				fras_tp <= fras_tp + 3'd1;
				fras_n  <= fras_n + 4'd1;
			end
			if (f2_walk && f2_spec) begin
				// a predicted stream does not search: a speculative ATC
				// fault (o2), refetched on demand by the back end
				st <= S_HALT;
			end
			else if (f2_walk) begin
				st     <= S_WALK;
				iw_req <= 1'b1;
			end
			else if (f2_flt) begin
				// ATC fault: the long word's words carry it (o2)
				st <= S_HALT;
			end
			else if (f2_cach) begin
				logic [3:0] vv;
				vv = f2_vv;
				st      <= S_FILL;
				m_got   <= 4'd0;
				lb_v    <= 1'b0;          // its beats go into m_line
				m_alloc <= !cacr_q[14];   // NAI: no allocation
				m_tci   <= 1'b0;
				m_way   <= !vv[0] ? 2'd0 : !vv[1] ? 2'd1 :
				           cacr_q[13] ? {1'b0, rr[0]} :
				           !vv[2] ? 2'd2 : !vv[3] ? 2'd3 : rr;
				b_req_r <= 1'b1;
			end
			else begin
				st      <= S_CI;
				b_req_r <= 1'b1;
			end
		end

		//------------------------------------------------------------------
		// miss engine
		//------------------------------------------------------------------
		if (b_gnt) begin
			b_req_r <= 1'b0;
			busy    <= 1'b1;
		end
		case (st)
		S_WALK:
			if (iw_done) begin
				iw_req <= 1'b0;
				st     <= S_RUN;
				wf_v   <= 1'b1;
				wf_ok  <= iw_ent.r;
				wf_e   <= iw_ent;
				wf_la  <= m_la;
				wf_fc2 <= m_s;
				if (!iw_ent.r) if_cause <= iw_cause;
			end
		S_FILL: begin
			logic [127:0] ln;
			ln = m_line;
			if (b_rvalid) begin
				ln[127 - 32 * b_rbeat -: 32] = b_rdata;
				m_line <= ln;
				m_got[b_rbeat] <= 1'b1;
				if (b_rtci) begin m_alloc <= 1'b0; m_tci <= 1'b1; end
			end
			if (busy && b_done) begin
				busy  <= 1'b0;
				stale <= 1'b0;
				if (m_alloc && !(b_rvalid && b_rtci)) begin
					c_we    <= 1'b1;
					c_waddr <= m_pa[10:4];
					c_wway  <= m_way;
					c_wtag  <= m_pa[31:11];
					iw_set = m_pa[10:4]; iw_m[m_way] = 1'b1; iw_v = 1'b1; iw_fill = 1'b1;
				end
				lb_pa <= m_pa[31:4];
				st    <= S_RUN;
				if (m_tci || (b_rvalid && b_rtci)) begin
					// a TCI line is not kept (5.7.1: sequential access only
					// until the burst's last long word arrives): the long word
					// being fetched goes straight to the decoder, the next one
					// is fetched anew (unless a redirect takes the stream)
					lb_v <= 1'b0;
					if (!stale) begin
						logic [31:0] lw, nla;
						lw  = ln[127 - 32 * m_pa[3:2] -: 32];
						nla = {m_la[31:2], 2'b00} + 32'd4;
						md_v <= 1'b1;
						if (m_la[1]) begin
							md_n    <= 2'd1;
							md_w[0] <= lw[15:0];
						end
						else begin
							md_n    <= 2'd2;
							md_w[0] <= lw[31:16];
							md_w[1] <= lw[15:0];
						end
						md_f  <= 1'b0; md_d <= 1'b0; md_a <= 1'b0;
						m_la  <= nla;
						m_pa  <= {m_pa[31:2], 2'b00} + 32'd4;
						m_dem <= 1'b0;
						m_setpc <= 1'b1;
						fnew  <= 1'b0;
					end
				end
				else lb_v <= 1'b1;
			end
			if (busy && b_err) begin
				busy  <= 1'b0;
				stale <= 1'b0;
				if (stale) st <= S_RUN;
				else if (m_got[m_pa[3:2]]) begin
					// a later beat failed, the long word being fetched had
					// arrived (the burst starts with it): it is used, the line
					// is not allocated, and no exception (MC68060UM 7.9.1,
					// 5.7.1).  The next long word is fetched anew.
					logic [31:0] lw, nla;
					lw  = ln[127 - 32 * m_pa[3:2] -: 32];
					nla = {m_la[31:2], 2'b00} + 32'd4;
					md_v <= 1'b1;
					if (m_la[1]) begin
						md_n    <= 2'd1;
						md_w[0] <= lw[15:0];
					end
					else begin
						md_n    <= 2'd2;
						md_w[0] <= lw[31:16];
						md_w[1] <= lw[15:0];
					end
					md_f  <= 1'b0; md_d <= 1'b0; md_a <= 1'b0;
					m_la  <= nla;
					m_pa  <= {m_pa[31:2], 2'b00} + 32'd4;
					m_dem <= 1'b0;
					m_setpc <= 1'b1;
					fnew  <= 1'b0;
					st    <= S_RUN;
				end
				else begin
					md_v <= 1'b1;
					md_n <= m_la[1] ? 2'd1 : 2'd2;
					for (int i = 0; i < 2; i++) md_w[i] <= 16'd0;
					md_f <= 1'b1; md_d <= m_dem; md_a <= 1'b0;
					// (a bus error's cause: the fetch mapped by a TTR, 8.4.4)
					if_cause <= m_tt ? FSLW_TTR : 32'd0;
					st   <= S_HALT;
				end
			end
		end
		S_CI: begin
			if (busy && b_rvalid && !stale) begin
				md_v <= 1'b1;
				if (m_la[1]) begin
					md_n    <= 2'd1;
					md_w[0] <= b_rdata[15:0];
				end
				else begin
					md_n    <= 2'd2;
					md_w[0] <= b_rdata[31:16];
					md_w[1] <= b_rdata[15:0];
				end
				md_f <= 1'b0; md_d <= 1'b0; md_a <= 1'b0;
			end
			if (busy && b_done) begin
				logic [31:0] nla;
				nla   = {m_la[31:2], 2'b00} + 32'd4;
				busy  <= 1'b0;
				stale <= 1'b0;
				if (stale) st <= S_RUN;
				else begin
					m_la  <= nla;
					m_pa  <= {m_pa[31:2], 2'b00} + 32'd4;
					m_dem <= 1'b0;
					m_setpc <= 1'b1;
					fnew  <= 1'b0;
					// the page ends (or the cache mode changed): translate again
					if (nla[11:0] == 12'd0 || (!m_ci && ic_en)) st <= S_RUN;
				end
			end
			else if (busy && b_err) begin
				busy  <= 1'b0;
				stale <= 1'b0;
				if (!stale) begin
					md_v    <= 1'b1;
					md_n    <= m_la[1] ? 2'd1 : 2'd2;
					md_w[0] <= 16'd0;
					md_w[1] <= 16'd0;
					md_f <= 1'b1; md_d <= m_dem; md_a <= 1'b0;
					if_cause <= m_tt ? FSLW_TTR : 32'd0;
					st   <= S_HALT;
				end
				else st <= S_RUN;
			end
			else if (!busy && !b_req_r && !b_gnt && !redir_any && !stop_q &&
			         {1'b0, cnt} + {1'b0, inflight} <= 6'(QN - 2))
				b_req_r <= 1'b1;        // the next long word of the stream
		end
		default: ;
		endcase

		// (the resynchronization over F1's and F2's moves of this clock)
		if (rs_p2) begin
			frw  = 1'b1;
			frwa = d_ras_tp;
			frwd = d_ras;
			fras_tp <= d_ras_tp;
			fras_n  <= d_ras_n;
		end

		//------------------------------------------------------------------
		// redirect: empty the queue and the pipeline.  A walk runs to its
		// end (the entry is installed); a line fill completes and is
		// installed; a cache-inhibited read completes and is discarded.
		//------------------------------------------------------------------
		if (redir_any || iatc_flush_all || iatc_flush_page) wf_v <= 1'b0;
		if (redir_any) begin
			cnt   <= '0;
			qsimp_q <= 1'b0;
			qcnt_q  <= '0;
			cth_q   <= '0;
		cthf_q  <= '0;
			room_q  <= 1'b1;
			m_setpc <= 1'b0;
			f3_n  <= '0;
			f4_n  <= '0;
			f5_n  <= '0;
			f6_n  <= '0;
			f3_b  <= '0;
			f4_b  <= '0;
			f5_b  <= '0;
			f6_b  <= '0;
			md_v  <= 1'b0;
			tq_n  <= '0;
			f1_v  <= 1'b0;
			f1_btok <= 1'b0;
			f2_v  <= 1'b0;
			rs_p1  <= 1'b1;
			rs_p2  <= 1'b0;
			fspec <= !redir_v;
			odd   <= redir_npc[0];
			fnew  <= 1'b1;
			if (st == S_FILL || st == S_CI) begin
				if (out_next) stale <= 1'b1;
				else st <= S_RUN;
			end
			if (st == S_HALT) st <= S_RUN;
			if (st == S_RUN) begin
				// (a miss F2 started in this clock: none)
				st      <= S_RUN;
				iw_req  <= 1'b0;
				b_req_r <= 1'b0;
			end
		end

		// a page flush looks its page up a clock later (F0 issues nothing then)
		if (iatc_flush_page) atc_fc2 <= iatc_flush_fc2;

		//------------------------------------------------------------------
		// CINV/CPUSH of the instruction cache
		//------------------------------------------------------------------
		if ((ic_inv || sn_inv) && !inv_busy && !inv_hold && st != S_FILL && !c_we) begin
			lb_v <= 1'b0;
			inv_sn    <= !ic_inv;
			inv_scope <= ic_inv ? ic_inv_scope : 2'd1;
			inv_pa    <= ic_inv ? ic_inv_pa : sn_inv_pa;
			inv_lbh   <= !ic_inv && lb_v && lb_pa == sn_inv_pa[31:4];
			if (ic_inv ? (ic_inv_scope == 2'd3) : sn_inv_all) begin
				// every line: the sweep below (scope 3 matches every way)
				inv_scope <= 2'd3;
				sn_ihit   <= !ic_inv;
				inv_busy  <= 1'b1;
				inv_ph    <= 1'b0;
				inv_i     <= 8'd0;
			end
			else begin
				inv_busy <= 1'b1;
				inv_ph   <= 1'b0;
				inv_i    <= (!ic_inv || ic_inv_scope == 2'd1) ?
				            {1'b0, (ic_inv ? ic_inv_pa[10:4] : sn_inv_pa[10:4])} : 8'd0;
			end
		end
		if (inv_busy) begin
			// this cycle reads set inv_i; the previous set's tags compare
			// (the valid bits are written next clock)
			if (inv_ph) begin
				logic [6:0] s;
				s = (inv_scope == 2'd1) ? inv_pa[10:4] : 7'(inv_i - 8'd1);
				// (the set from registers, the ways from the compare)
				iw_set = s; iw_v = 1'b0;
				for (int w = 0; w < 4; w++) begin
					logic m;
					if (inv_scope == 2'd3)
						m = 1'b1;
					else if (inv_scope == 2'd1)
						m = ct[w][20:0] == inv_pa[31:11];
					else if (tc_p)
						m = ct[w][20:2] == inv_pa[31:13];
					else
						m = ct[w][20:1] == inv_pa[31:12];
					if (m) iw_m[w] = 1'b1;
				end
			end
			inv_ph <= 1'b1;
			if (inv_scope == 2'd1) begin
				if (inv_ph) begin
					inv_busy    <= 1'b0;
					inv_hold    <= 1'b1;
				end
			end
			else begin
				inv_i <= inv_i + 8'd1;
				if (inv_i == 8'd128) begin
					inv_busy    <= 1'b0;
					inv_hold    <= 1'b1;
					inv_rst     <= 1'b0;
				end
			end
		end
		// the end is signalled with the last valid write (next clock), and
		// the request is not taken again until it has seen it
		inv_dn <= 2'd0;
		if (inv_busy && inv_ph && (inv_scope == 2'd1 || inv_i == 8'd128))
			inv_dn <= {inv_sn, !inv_sn && !(inv_scope != 2'd1 && inv_rst)};
		if (inv_dn != 2'd0) begin
			ic_inv_done <= inv_dn[0];
			sn_inv_done <= inv_dn[1];
			inv_hold    <= 1'b1;
		end

		// the valid RAMs' write decision, performed next clock (below);
		// a snooped invalidation reports a valid line it removed then
		iwq_v    <= iw_m != 4'd0;
		iwq_set  <= iw_set;
		iwq_m    <= iw_m;
		iwq_val  <= iw_v;
		iwq_fill <= iw_fill;
		iwq_sn   <= inv_sn && !iw_fill;
		iwq_lbh  <= inv_busy && inv_ph && inv_sn && inv_lbh;
		if ((iwq_v && iwq_sn && (ivo_q & iwq_m) != 4'd0) || iwq_lbh) sn_ihit <= 1'b1;   // (ivo_ra: iwq_set)
		if (iwq_v && iwq_fill && ivo_q == 4'hF) rr <= rr + 2'd1;
	end
	// the return stack's one write port, a clock after its decision (from
	// registers: a call's return address waits on the BTB's compare; a
	// return predicted in the clock right after is read from the entry as
	// it was -- a prediction D1 checks).  (frw_q is clear in reset.)
	frw_q  <= frw && nrst;
	frwa_q <= frwa;
	frwd_q <= frwd;
	if (frw_q) fras[frwa_q] <= frwd_q;
	// the valid RAMs' one write (both copies; iwq_v is a register: a write
	// decided before a reset completes in its first clock)
	begin
		if (iwq_v) begin
			ivf[iwq_set] <= (ivo_q & ~iwq_m) | (iwq_m & {4{iwq_val}});
			ivo[iwq_set] <= (ivo_q & ~iwq_m) | (iwq_m & {4{iwq_val}});
		end
		// synthesis translate_off
		for (int j = 0; j < 4; j++) if (iwq_v && iwq_m[j]) iv[iwq_set][j] <= iwq_val;
		if (iwq_v && iwq_set != ivo_ra) begin
			$display("FAIL: fetch: a valid write at set %0d, the second copy read at %0d", iwq_set, ivo_ra);
			$finish;
		end
		// synthesis translate_on
	end
end

// the valid copies' reads: F1's set; the pending write's
always_comb ivo_ra = iwq_set;
assign ivf_q = ivf[f1_pc[10:4]];
assign ivo_q = ivo[ivo_ra];
initial for (int i = 0; i < 128; i++) begin ivf[i] = 4'd0; ivo[i] = 4'd0; end
// synthesis translate_off
always_ff @(posedge clk)
	if (nreset && !inv_rst && (ivf_q != iv[f1_pc[10:4]] || ivo_q != iv[ivo_ra])) begin
		$display("FAIL: fetch: valid RAM %b/%b, model %b/%b", ivf_q, ivo_q, iv[f1_pc[10:4]], iv[ivo_ra]);
		$finish;
	end
// synthesis translate_on

// synthesis translate_off
always_ff @(posedge clk)
	if (nreset && cnt != 5'd0 && qp0 != qps[0]) begin
		$display("FAIL: fetch: the predecode ring's head %h, the queue's %h", qp0, qps[0]);
		$finish;
	end
always_ff @(posedge clk)
	if (nreset && !$onehot0({f2_mj, f2_lb, md_v})) begin
		$display("FAIL: fetch: F2's words from more than one source (%b %b %b)", f2_mj, f2_lb, md_v);
		$finish;
	end
always_ff @(posedge clk)
	if (nreset && q_simple && !(qp0.legal && !qp0.slow && !qp0.x0 && !qp0.x1 && qp0.tot <= 4'd7 &&
	                            {1'b0, qp0.tot} <= cnt && qf[7:0] == 8'd0)) begin
		$display("FAIL: fetch: the head's one-pass flag");
		$finish;
	end
always_ff @(posedge clk)
	if (nrst) for (int j = 0; j < QN + 2; j++) if (cthf_q[j] != (6'(j) < {1'b0, cnt} + {4'd0, f6_n})) begin
		$display("FAIL: fetch: the count-with-F6 thermometer %b, the count %0d + %0d", cthf_q, cnt, f6_n);
		$finish;
	end
always_ff @(posedge clk)
	if (nrst) for (int j = 0; j < QN; j++) if (cth_q[j] != (5'(j) < cnt)) begin
		$display("FAIL: fetch: the count's thermometer %b, the count %0d", cth_q, cnt);
		$finish;
	end
// the pipelined lookup against the one-port lookup of the same address
logic  ref_v, ref_hit;
atce_t ref_e;
always_ff @(posedge clk) begin
	ref_v   <= f0_go && (atc_set == atc_sx(atc_la, tc_p));   // (a TC.P change aside)
	ref_hit <= atc_hit1;
	ref_e   <= atc_e1;
	if (nrst && ref_v && f1_v && (f1_ah != ref_hit || (ref_hit && f1_ea != ref_e))) begin
		$display("FAIL: fetch: the I-ATC's pipelined lookup %b %h, the one-port lookup's %b %h",
		         f1_ah, f1_ea, ref_hit, ref_e);
		$finish;
	end
end
always_ff @(posedge clk)
	if (nreset && md_v && f2_v) begin
		$display("FAIL: fetch: the miss engine's words with F2's");
		$finish;
	end
always_ff @(posedge clk)
	if (nreset && ((cs_fire && cr_fire) || (cs_fire && {2'b00, qp0.tot} > cnt) ||
	               (cr_fire && {2'b00, cr_use} > cnt))) begin
		$display("FAIL: fetch: a take of more words than the queue holds");
		$finish;
	end
// synthesis translate_on

endmodule
