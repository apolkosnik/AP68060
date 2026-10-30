//--------------------------------------------------------------------------//
// AP68060 - pipelined MC68040                                            //
//                                                                          //
// ap68060_decode.sv - D1: instruction parse                                 //
//                                                                          //
//   operation word | fixed extension words | immediate | EA0 ext | EA1 ext //
//                                                                          //
// Two stages.  D1a takes the instruction at the head of the queue: its    //
// length from the queue's predecode (assuming brief index extensions; D1a //
// checks bit 8 of the words at the index extension positions), and its   //
// words and the terms D1b needs (the immediate, the extension words of   //
// each EA, PC-relative bases, the branch target, the exception vector)   //
// into the instruction register IR.  The common case -- a whole one-pass //
// instruction in the queue, no fault in it -- is taken in the clock D1a  //
// sees it, from registers only; anything else (a full-format extension,  //
// an FPU immediate, an illegal word, a fault, the multi-part slow path:  //
// one phase per part, operation word with fixed words and immediate,     //
// then EA0's extension, then EA1's) is decided in one clock and taken in //
// the next.  D1a also forms the EAs (but a one-pass instruction's        //
// displacements, which D1b forms from the two words D1a selects) and the //
// predictions' static terms.  D1b builds the record from IR: the PC-     //
// relative displacements, the routine, the predictions (Bcc by the static//
// rule and a history table read at IR's address, JSR/JMP absolute and   //
// PC-relative, RTS by a return stack), which redirect fetch unless the  //
// fetch BTB already took the                                              //
// same branch (D1b checks every word the BTB flagged), the return stack  //
// and the loop exit predictor.  A redirect drops the instruction D1a     //
// took meanwhile.  Records go to D2 through a four-entry FIFO; D1a takes //
// an instruction only when the FIFO will have room for it, so neither   //
// side depends on the other combinationally.  Illegal, line A/F,        //
// privilege, TRAP #n, odd PC and fetch faults become records carrying    //
// their exception vector.                                                 //
//--------------------------------------------------------------------------//

module ap68060_decode
	import ap68060_pkg::*, ap68060_upkg::*;
#(
	parameter bit LOOP_PRED = 1          // the loop exit predictor
)
(
	input  logic        clk,
	input  logic        nreset,
	input  logic        flush,          // back end redirect: drop everything
	input  logic        ras_rv,         // ... with the return stack to restore
	input  logic  [2:0] ras_rtp,
	input  logic  [3:0] ras_rn,

	input  logic [15:0] win [8],
	input  logic  [7:0] win_flt,
	input  logic  [7:0] win_fdem,     // the faulted fetch was a demand fetch
	input  logic  [7:0] win_fatc,     // ... an ATC (MMU) fault
	input  logic  [7:0] win_bt,       // the word ends a branch the fetch BTB took
	input  logic [31:0] bt_tgt,       // its target (the first flagged word's)
	input  pd_t         pd0,          // predecode of win[0]
	input  logic        q_simple,     // ... a one-pass instruction, all in the queue,
	                                  // no word D1 sees faulted (registers)
	input  logic  [3:0] qcnt,
	input  logic [31:0] qpc,
	input  logic        q_odd,
	input  logic        smode,

	// what D1a takes from the queue: the head instruction (pd0.tot words),
	// or cr_use words (decided the clock before)
	output logic        cs_fire,
	output logic        cr_fire,
	output logic  [2:0] cr_use,
	output logic        d_redir_v,
	output logic [31:0] d_redir_pc,

	// record FIFO towards D2: rq0 is the head
	output logic  [2:0] rq_n,
	output dinst_t      rq0,
	output dinst_t      rq1,
	input  logic        rq_pop,

	// Bcc history training (from EX)
	input  logic        bht_we,
	input  logic  [7:0] bht_wa,
	input  logic        bht_dis,        // ... the branch went against the static rule
	// loop exit training (from EX): a backward Bcc/DBcc, taken or not
	input  logic        lp_we,
	input  logic [11:0] lp_wa,
	input  logic        lp_tk,
	input  logic        lp_mp,          // ... and it was mispredicted

	// BTB maintenance (the fetch unit's table)
	output logic        btb_we,
	output logic [BTB_AW-1:0] btb_wi,
	output logic        btb_wv,
	output logic [BTB_TW-1:0] btb_wtag,
	output logic        btb_wslot,
	output logic  [1:0] btb_wkind,
	output logic [30:0] btb_wtgt,
	output logic [30:0] ras_o,        // the return stack's top, for the fetch's
	output logic  [2:0] ras_tp_o,
	output logic  [3:0] ras_n_o
);

// (the reset, registered here: the core's is a gate on its reset counters
// and reaches every module; the front end leaves and enters reset a clock
// later -- the back end's reset exception reaches D2 the clock after its
// own reset ends, a clock later still)
logic nrst;
always_ff @(posedge clk) nrst <= nreset;

`include "gen/ap68060_dec_pla.svh"

// S, registered here (it changes with a flush: D1 sees no instruction in
// the clock after), and the back end's flush a clock late (it drops what
// D1a took in the flush's clock)
logic sm_q, flush_q;
always_ff @(posedge clk) begin
	sm_q    <= smode;
	flush_q <= flush;
end


//--------------------------------------------------------------------------
// helpers
//--------------------------------------------------------------------------
function automatic logic [2:0] ea_len(input logic [3:0] idx, input logic [15:0] w,
                                      input logic [2:0] immlen);
	logic [2:0] n;
	case (idx)
		EM_AD16, EM_ABSW, EM_PC16: n = 3'd1;
		EM_ABSL: n = 3'd2;
		EM_IMM:  n = immlen;
		EM_AX, EM_PCX: begin
			n = 3'd1;
			if (w[8]) begin
				case (w[5:4])
					2'd2: n = n + 3'd1;
					2'd3: n = n + 3'd2;
					default: ;
				endcase
				case (w[1:0])
					2'd2: n = n + 3'd1;
					2'd3: n = n + 3'd2;
					default: ;
				endcase
			end
		end
		default: n = 3'd0;
	endcase
	ea_len = n;
endfunction

function automatic logic [31:0] sx16(input logic [15:0] v);
	sx16 = {{16{v[15]}}, v};
endfunction

// the base displacement of an index extension word w0 (brief or full)
function automatic logic [31:0] ea_bdx(input logic [15:0] w0, input logic [15:0] w1,
                                       input logic [15:0] w2);
	case (w0[5:4])
		2'd2:    ea_bdx = sx16(w1);
		2'd3:    ea_bdx = {w1, w2};
		default: ea_bdx = 32'd0;
	endcase
	if (!w0[8]) ea_bdx = {{24{w0[7]}}, w0[7:0]};
endfunction
// the displacement a PC-relative EA adds to its extension word's address
function automatic logic [31:0] ea_pcd(input logic [3:0] idx, input logic [15:0] w0,
                                       input logic [15:0] w1, input logic [15:0] w2);
	ea_pcd = (idx == EM_PC16) ? sx16(w0) : ea_bdx(w0, w1, w2);
endfunction
// a one-pass instruction's EA displacement from its two words (mk_ea's,
// brief extensions; the PC-relative modes' is the PC sum, added apart)
function automatic logic [31:0] ea_fbd(input logic [3:0] idx, input logic [2:0] immlen,
                                       input logic [15:0] w0, input logic [15:0] w1);
	case (idx)
		EM_AD16, EM_ABSW: ea_fbd = sx16(w0);
		EM_ABSL:          ea_fbd = {w0, w1};
		EM_IMM:           ea_fbd = (immlen == 3'd1) ? {16'd0, w0} : {w0, w1};
		EM_AX:            ea_fbd = {{24{w0[7]}}, w0[7:0]};
		default:          ea_fbd = 32'd0;
	endcase
endfunction

// EA record from its extension words w0.. at instruction word offset p;
// pcs: the PC-relative modes' address, the extension word's address plus
// the displacement (D1a adds it, ea_pcd), already in bd for those modes
function automatic ea_t mk_ea(input logic [3:0] idx, input logic [2:0] r,
                              input logic [3:0] p, input logic [2:0] immlen,
                              input logic [31:0] pcs,
                              input logic [15:0] w0, input logic [15:0] w1,
                              input logic [15:0] w2, input logic [15:0] w3,
                              input logic [15:0] w4);
	ea_t e;
	logic [31:0] bdx;
	bdx = ea_bdx(w0, w1, w2);
	e = '0;
	e.m    = idx;
	e.r    = r;
	e.xoff = p[2:0];
	case (idx)
		EM_AD16, EM_ABSW: e.bd = sx16(w0);
		EM_PC16: e.bd = pcs;
		EM_ABSL: e.bd = {w0, w1};
		EM_IMM:  e.bd = (immlen == 3'd1) ? {16'd0, w0} : {w0, w1};
		EM_AX, EM_PCX: begin
			e.xa = w0[15];
			e.xr = w0[14:12];
			e.xl = w0[11];
			e.sc = w0[10:9];
			if (w0[8]) begin
				logic [15:0] o0, o1;
				e.bs = w0[7];
				e.is = w0[6];
				case (w0[5:4])
					2'd2: begin o0 = w2; o1 = w3; end
					2'd3: begin o0 = w3; o1 = w4; end
					default: begin o0 = w1; o1 = w2; end
				endcase
				if (w0[1:0] != 2'd0)
					e.mi = (w0[2] && !w0[6]) ? 2'd2 : 2'd1;
				case (w0[1:0])
					2'd2: e.od = sx16(o0);
					2'd3: e.od = {o0, o1};
					default: e.od = 32'd0;
				endcase
			end
			// PC-relative with the base not suppressed: the PC is added here
			e.bd = (idx == EM_PCX && !e.bs) ? pcs : bdx;
		end
		default: ;
	endcase
	mk_ea = e;
endfunction

// the EAs' kinds D2 selects on: {EA1, EA0 displacement, EA1, EA0 memory
// indirect, EA1, EA0 in memory}
function automatic logic [1:0] ea_dk(input ea_t e);
	case (e.m)
		EM_APD:                               ea_dk = 2'd3;
		EM_AX, EM_PCX:                        ea_dk = (e.mi != 2'd0) ? 2'd2 : 2'd1;
		EM_AD16, EM_ABSW, EM_ABSL, EM_PC16:   ea_dk = 2'd1;
		default:                              ea_dk = 2'd0;
	endcase
endfunction
function automatic logic [7:0] ea_kind(input ea_t e1, input ea_t e0);
	ea_kind[0]   = !(e0.m == EM_DN || e0.m == EM_AN || e0.m == EM_IMM);
	ea_kind[1]   = !(e1.m == EM_DN || e1.m == EM_AN || e1.m == EM_IMM);
	ea_kind[2]   = (e0.m == EM_AX || e0.m == EM_PCX) && e0.mi != 2'd0;
	ea_kind[3]   = (e1.m == EM_AX || e1.m == EM_PCX) && e1.mi != 2'd0;
	ea_kind[5:4] = ea_dk(e0);
	ea_kind[7:6] = ea_dk(e1);
endfunction

// the first n words of the window (n <= 7)
function automatic logic [7:0] bmask(input logic [2:0] n);
	bmask = 8'((9'd1 << n) - 9'd1);
endfunction

// effective first micro-instruction given the routine's leading jump
function automatic logic jcond(input logic [4:0] jc, input logic [3:0] e0,
                               input logic [3:0] e1, input logic [15:0] ext1,
                               input logic [1:0] s, input logic [2:0] ry);
	logic m0, m1;
	m0 = (e0 >= EM_AI) && (e0 != EM_IMM) && (e0 != EM_NONE);
	m1 = (e1 >= EM_AI) && (e1 != EM_IMM) && (e1 != EM_NONE);
	case (jc)
		JC_ALWAYS:   jcond = 1'b1;
		JC_EA0_MEM:  jcond = m0;
		JC_EA1_MEM:  jcond = m1;
		JC_EA0_REG:  jcond = (e0 == EM_DN) || (e0 == EM_AN);
		JC_EA0_DN:   jcond = (e0 == EM_DN);
		JC_EA0_AN:   jcond = (e0 == EM_AN);
		JC_EA0_IMM:  jcond = (e0 == EM_IMM);
		JC_NOT_EA0_MEM: jcond = !m0;
		JC_EXT11:    jcond = ext1[11];
		JC_EXT10:    jcond = ext1[10];
		JC_SZ_L:     jcond = (s == SZ_L);
		JC_SZ_B:     jcond = (s == SZ_B);
		JC_AY7:      jcond = (ry == 3'd7);
		JC_BOTH_MEM: jcond = m0 && m1;
		JC_MASK0:    jcond = (ext1 == 16'd0);
		JC_X1A:      jcond = ext1[15];
		JC_CREG_RF:  jcond = (ext1[11:0] == 12'h800);    // USP
		JC_FP_OC0:   jcond = (ext1[15:13] == 3'b000);
		default:     jcond = 1'b0;
	endcase
endfunction

// FPU immediate length from the command word's source format
function automatic logic [2:0] fp_immlen(input logic [15:0] cmd);
	logic [2:0] crl;
	crl = (cmd[12:10] == 3'd0) ? 3'b001 : cmd[12:10];
	if (cmd[15:13] == 3'b100)
		// FMOVE(M) #imm to control registers: a long word each
		fp_immlen = {2'(crl[0]) + 2'(crl[1]) + 2'(crl[2]), 1'b0};
	else if (cmd[15:13] != 3'b010)
		fp_immlen = 3'd0;
	else case (cmd[12:10])
		3'd0, 3'd1: fp_immlen = 3'd2;   // L, S
		3'd2, 3'd3: fp_immlen = 3'd6;   // X, P
		3'd4, 3'd6: fp_immlen = 3'd1;   // W, B
		3'd5:       fp_immlen = 3'd4;   // D
		default:    fp_immlen = 3'd0;
	endcase
endfunction

// the 68060's control registers: SFC DFC CACR TC ITT0 ITT1 DTT0 DTT1 BUSCR
// ($000-$008), USP VBR ($800 $801), URP SRP PCR ($806-$808)
function automatic logic movec_ok(input logic [11:0] r);
	movec_ok = (r <= 12'h008) || (r == 12'h800) || (r == 12'h801) ||
	           (r == 12'h806) || (r == 12'h807) || (r == 12'h808);
endfunction

function automatic logic has_ext(input logic [3:0] i);
	has_ext = (i != EM_NONE) && (i >= EM_AD16);
endfunction

function automatic logic [31:0] sx8(input logic [7:0] v);
	sx8 = {{24{v[7]}}, v};
endfunction

// the FPU general instruction's straight-line routine for its common forms
// (FPU_GEN's paths without their jump words); v clear: the record's own
function automatic logic [10:0] fpu_k(input logic [15:0] x, input logic [3:0] m);
	logic [9:0] k;
	logic       v;
	v = 1'b1;
	k = '0;
	if (x[15:13] == 3'b010 && x[12:10] != 3'd7 && m != EM_AN) begin
		if (m == EM_DN) k = UA_FPU_K_IND;
		else case (x[12:10])
			3'd0, 3'd1: k = UA_FPU_K_IN4;
			3'd6:       k = UA_FPU_K_IN1;
			3'd4:       k = UA_FPU_K_IN2;
			3'd5:       k = UA_FPU_K_IN8;
			3'd2, 3'd3: k = UA_FPU_K_IN12;
			default:    v = 1'b0;
		endcase
	end
	else if (x[15:13] == 3'b011 && m != EM_AN) begin
		if (m == EM_DN) k = UA_FPU_K_OUTD;
		else case (x[12:10])
			3'd0, 3'd1: k = UA_FPU_K_OUT4;
			3'd6:       k = UA_FPU_K_OUT1;
			3'd4:       k = UA_FPU_K_OUT2;
			3'd5:       k = UA_FPU_K_OUT8;
			3'd2:       k = UA_FPU_K_OUT12;
			default:    v = 1'b0;
		endcase
	end
	else v = 1'b0;
	fpu_k = {v, k};
endfunction

// the kind of a routine's first word D1 put in the record by its address
// (the routines D1 substitutes; any other keeps the attribute's kind)
function automatic logic [18:0] kind_fix(input logic [9:0] rt, input logic [18:0] k);
	case (rt)
		UA_DEC_EXC:     kind_fix = UK_DEC_EXC;
		UA_JSR_K:       kind_fix = UK_JSR_K;
		UA_FPU_K_IND:   kind_fix = UK_FPU_K_IND;
		UA_FPU_K_IN1:   kind_fix = UK_FPU_K_IN1;
		UA_FPU_K_IN2:   kind_fix = UK_FPU_K_IN2;
		UA_FPU_K_IN4:   kind_fix = UK_FPU_K_IN4;
		UA_FPU_K_IN8:   kind_fix = UK_FPU_K_IN8;
		UA_FPU_K_IN12:  kind_fix = UK_FPU_K_IN12;
		UA_FPU_K_OUTD:  kind_fix = UK_FPU_K_OUTD;
		UA_FPU_K_OUT1:  kind_fix = UK_FPU_K_OUT1;
		UA_FPU_K_OUT2:  kind_fix = UK_FPU_K_OUT2;
		UA_FPU_K_OUT4:  kind_fix = UK_FPU_K_OUT4;
		UA_FPU_K_OUT8:  kind_fix = UK_FPU_K_OUT8;
		UA_FPU_K_OUT12: kind_fix = UK_FPU_K_OUT12;
		default:        kind_fix = k;
	endcase
endfunction

// the consumed words of a decode n words long, and the last of them
function automatic logic [15:0] lmask(input logic [3:0] n);
	case (n)
		4'd0:    lmask = {8'h00, 8'h00};
		4'd1:    lmask = {8'h01, 8'h01};
		4'd2:    lmask = {8'h03, 8'h02};
		4'd3:    lmask = {8'h07, 8'h04};
		4'd4:    lmask = {8'h0F, 8'h08};
		4'd5:    lmask = {8'h1F, 8'h10};
		4'd6:    lmask = {8'h3F, 8'h20};
		4'd7:    lmask = {8'h7F, 8'h40};
		default: lmask = {8'hFF, 8'h80};
	endcase
endfunction

typedef enum logic [1:0] { PH_IDLE, PH_EA0, PH_EA1 } ph_t;

//==========================================================================
// D1a
//==========================================================================
wire [15:0] opw = win[0];
wire  [2:0] fb  = pd0.b;
wire  [2:0] fp1 = pd0.p1;

ph_t         ph;            // the part D1a parses next
pd_t         part_pd;       // a multi-part instruction's first part
logic  [3:0] part_len;      // ... words consumed so far
logic  [2:0] part_imml;     // ... EA0 immediate length
logic [31:0] part_pc;       // ... its address
logic  [5:0] part_rr;       // ... its EA register fields (opw[11:9], opw[2:0])
logic        part_fg;       // ... an FPU general instruction (opw F200-F23F)
// a decision for the head made the clock before (anything but the common
// case): take r_use words; a record (r_go) or a part (r_part), then r_nph
logic        r_v, r_go, r_part;
logic        rx_q;          // ... the head has a decode-time exception
logic  [2:0] r_use;
ph_t         r_nph;

// the record FIFO's room for what D1a takes now (D1b pushes it next
// clock): the records held and the one in IR fewer than four.  A register,
// formed a clock ahead from the count's next value and IR's next take
// (D2's pop, late, picks last): the take fans out over the whole queue.
logic        ir_v, ir_go;
logic        hold_redir;    // a predicted branch is redirecting fetch
logic        room_q;
logic        ph_idle;       // ph == PH_IDLE (a register alongside)
// (not the back end's flush, a far register: D1a may take an instruction
// in the flush's clock -- the queue is emptied by the redirect with it, or
// a redirect follows (an exception's) -- and IR drops it a clock later)
wire         stall = hold_redir || d_redir_v || !room_q;

// A fetch fault is raised for the lowest faulted word of the window (the
// words before it are good, so it is the first faulted word an instruction
// uses).  Codes: EXC_IFB a bus error, EXC_IFA an ATC fault, on a demand
// fetch; EXC_IFS/EXC_IFSA the same on a speculative prefetch, which the
// back end refetches on demand first.  The record's target carries the
// faulting word's address (the format $7 fault address).
// A faulted word in the queue is the last one fetch delivers until a
// redirect (it waits in S_HALT), so an instruction that needs more words
// than the queue holds is decoded with what is there: its words reach the
// faulted one, and it takes the fault.  Waiting for the rest would wait
// forever -- an opword fault at an address with A1 set queues ONE word,
// shorter than the two-word instruction its zero word decodes as.
// (the fault flags past the queue's count are clear: no mask by the count)
wire         q_flt  = |win_flt;
logic  [7:0] f_code;
logic [31:0] f_addr;
always_comb begin
	logic [2:0] fi;
	fi = 3'd0;
	for (int i = 7; i >= 0; i--) if (win_flt[i]) fi = 3'(i);
	f_addr = qpc + {28'd0, fi, 1'b0};
	f_code = win_fdem[fi] ? (win_fatc[fi] ? EXC_IFA : EXC_IFB)
	                      : (win_fatc[fi] ? EXC_IFSA : EXC_IFS);
end

// the common case, from registers: a whole one-pass instruction (no index
// extension), no fault in the words D1 sees (the fetch's q_simple).  An
// index EA's instruction is taken by the general case.
wire full0  = pd0.x0 && win[fb][8];
wire full1  = pd0.x1 && win[fp1][8];
wire s_ok   = ph_idle && q_simple;
assign cs_fire = s_ok && !r_v && !stall;      // (registers: one LUT)
assign cr_fire = r_v && !stall;
assign cr_use  = r_use;
wire   fire    = cs_fire || cr_fire;

// the general case (decided a clock ahead of its fire): the length of the
// part at the head, whether it completes a record, the next part
logic        g_go, g_part, g_exc;
logic  [2:0] g_use;
ph_t         g_nph;
// Each case's terms apart, as kept nets, and the case picks last (one-pass
// or not -- full-format words at the EA positions -- is the deep one):
//   EA extension at the head (a part): its length l, fits/faulted, the part's
//   whole one-pass instruction: its length tot
//   first part (operation word, fixed words, immediate): its length b
(* keep *) wire       k_fast;
(* keep *) wire       k_gop, k_flp, k_gof, k_gos, k_fls;
(* keep *) wire [2:0] k_up, k_uf, k_us;
assign k_fast = pd0.legal && !pd0.slow && !full0 && !full1 && (pd0.tot <= 4'd7);
assign k_gof  = (pd0.tot <= qcnt) || q_flt;
assign k_uf   = (pd0.tot <= qcnt) ? pd0.tot[2:0] : qcnt[2:0];      // (q_flt: what is queued)
assign k_gos  = ({1'b0, pd0.b} <= qcnt) || q_flt;
assign k_us   = ({1'b0, pd0.b} <= qcnt) ? pd0.b : qcnt[2:0];
assign k_fls  = |(win_flt & bmask(pd0.b));
wire   [2:0] p_l = ea_len((ph == PH_EA0) ? part_pd.i0 : part_pd.i1, win[0],
                          (ph == PH_EA0) ? part_imml : 3'd0);
assign k_gop  = ({1'b0, p_l} <= qcnt) || q_flt;
assign k_up   = ({1'b0, p_l} <= qcnt) ? p_l : qcnt[2:0];
assign k_flp  = |(win_flt & bmask(p_l));
always_comb begin
	g_go = 1'b0; g_part = 1'b0; g_use = 3'd0; g_nph = ph;
	if (ph != PH_IDLE) begin
		// slow path: the EA extension at the head
		if (k_gop) begin
			g_use = k_up;
			if (ph == PH_EA0 && has_ext(part_pd.i1) && !k_flp) begin
				g_part = 1'b1;
				g_nph  = PH_EA1;
			end
			else begin
				g_go  = 1'b1;
				g_nph = PH_IDLE;
			end
		end
	end
	else if (q_odd && qcnt == 4'd0) begin
		// the stream starts at an odd address: address error
		g_go = 1'b1;
	end
	else if (qcnt != 4'd0) begin
		if (!pd0.legal) begin
			g_use = 3'd1;
			g_go  = 1'b1;
		end
		else if (k_fast) begin
			if (k_gof) begin
				g_go  = 1'b1;
				g_use = k_uf;
			end
		end
		else if (k_gos) begin
			// slow path, first part: operation word, fixed words, immediate
			g_use = k_us;
			// (stop at the faulted part; an exception needs no more parsing
			// either: rx_q, registered alongside, turns the part into the
			// record at the take)
			if (k_fls) g_go = 1'b1;
			else begin
				g_part = 1'b1;
				g_nph  = has_ext(pd0.i0) ? PH_EA0 : PH_EA1;
			end
		end
	end
end

// the instruction's exception (the first part's, or a fault in the words
// taken): IR gets the vector, D1b only tests it
logic [31:0] a_imm, a_ntgt;  // the head's immediate and branch target (below)
logic  [7:0] x_exc;         // the head's decode-time exception, faults aside
always_comb begin
	x_exc = 8'd0;
	if (!pd0.legal)
		x_exc = (opw[15:12] == 4'hA) ? 8'd10 : (opw[15:12] == 4'hF) ? 8'd11 : 8'd4;
	// (68060, WinUAE's order: a control register number the 68060 does
	// not have is an illegal instruction in either mode, and LPSTOP checks
	// its extension word before the privilege)
	else if (pd0.cls == CL_MOVEC && !movec_ok(win[1][11:0]))
		x_exc = 8'd4;
	else if (pd0.cls == CL_LPSTOP && win[1] != 16'h01C0)
		x_exc = 8'd11;
	else if (pd0.priv && !sm_q)
		x_exc = 8'd8;
	// STOP and LPSTOP whose new SR leaves supervisor mode: privilege
	// violation at once (the 68060)
	else if ((pd0.cls == CL_STOP || pd0.cls == CL_LPSTOP) && !a_imm[13])
		x_exc = 8'd8;
	else if (pd0.cls == CL_TRAP)
		x_exc = 8'd32 + {4'd0, opw[3:0]};
	else if (pd0.cls == CL_ILLEGAL)
		x_exc = 8'd4;
	// unimplemented integer instructions: MOVEP, CHK2/CMP2, CAS2, and
	// MULx.L / DIVx.L with a 64-bit product or dividend (ext bit 10)
	else if (pd0.cls == CL_UNIMPL || ((opw[15:7] == 9'b0100_1100_0) && win[1][10]))
		x_exc = 8'd61;
end
// (whether there is one: the cases' OR, no priority -- every vector above
// is nonzero)
wire x_any = !pd0.legal || (pd0.cls == CL_MOVEC && !movec_ok(win[1][11:0])) ||
             (pd0.cls == CL_LPSTOP && win[1] != 16'h01C0) || (pd0.priv && !sm_q) ||
             ((pd0.cls == CL_STOP || pd0.cls == CL_LPSTOP) && !a_imm[13]) ||
             pd0.cls == CL_TRAP || pd0.cls == CL_ILLEGAL || pd0.cls == CL_UNIMPL ||
             ((opw[15:7] == 9'b0100_1100_0) && win[1][10]);
assign g_exc = x_any;

// what the fire takes: the common case's, or the decision's
wire  [2:0] u_use  = r_v ? r_use : pd0.tot[2:0];
wire        u_go   = r_v ? (r_go || (r_part && rx_q)) : 1'b1;
wire        u_part = r_v && r_part && !rx_q;
wire [15:0] umask  = lmask({1'b0, u_use});

//--------------------------------------------------------------------------
// IR: the instruction (or part) D1a took, for D1b
//--------------------------------------------------------------------------
ph_t         ir_ph;         // the part it is
logic        ir_part;       // a part (a record follows later)
logic        ir_odd;        // the odd-address record
logic        ir_flt;        // a fault in the words taken
logic [15:0] ir_w [5];      // the words at the head
ea_t         ir_ea0, ir_ea1; // the EAs, formed in D1a (a part's: its own)
logic        ir_epc;        // ... EA0's (a part's) displacement is the PC sum
logic        ir_fimm;       // ... EA0 is an FPU instruction's immediate: its
                            // displacement is its address (ir_pcb0)
logic        ir_e0f, ir_e1f; // ... a one-pass instruction's displacement is
logic [15:0] ir_e0 [2];     // formed in D1b from its EA's two words
logic [15:0] ir_e1 [2];
pd_t         ir_pd;         // the instruction's predecode (its first part's)
logic [31:0] ir_pc;         // the part's address
logic  [3:0] ir_len;        // ... and with it
logic [31:0] ir_npc;        // the address after it
logic [31:0] ir_pcb0;       // EA0's extension word's address (or the part's)
logic [31:0] ir_pcs0;       // ... plus its PC-relative displacement (EA1 has none)
logic        ir_jj;         // JSR/JMP to a known target (absolute, PC-relative)
logic  [1:0] ir_tsel;       // the record's target: 0 none, 1 ir_ntgt, 2 the return stack's
logic [10:0] ir_fpk;        // FPU straight-line routine (valid, address)
logic [18:0] ir_fpkk;       // ... its first word's kind
logic [31:0] ir_imm;
logic  [7:0] ir_exc;        // exception vector (a fault's included)
logic        ir_xany;       // ... it is one (ir_exc != 0)
logic        ir_nr0;        // the prediction: the class's rule and the history
logic        ir_nrb;        // ... Bcc: the history counter's (D1b reads it)
logic        ir_lpb;        // ... a loop exit may overrule it
logic        ir_rts;        // ... RTS (the return stack's count decides)
logic [31:0] ir_faddr;      // a fault's address
logic        ir_jtake;      // the routine's first jump is taken
logic        ir_jsrk;       // JSR to a known target
logic [31:0] ir_ntgt;       // the branch target (Bcc, DBcc, FBcc, FDBcc, JSR/JMP)
logic        ir_dsign;      // the displacement's sign
logic  [7:0] ir_lph;        // loop table entries tagged with its address
logic [31:0] ir_bt_tgt;     // the BTB's target for the first flagged word
logic        ir_bt_end;     // the BTB flagged its last word only
logic        ir_bt_restart; // ... a word inside it
logic [31:0] ir_bt_fa;      // ... the first such word's address
logic [31:0] ir_lastpc;     // its last word's address

// Bcc history: read with the head's address
logic  [1:0] bht_q;
logic        bw_we;
logic  [7:0] bw_wa;
logic  [1:0] bw_wd;
ap68060_lutram #(.AW(8), .DW(2)) bht (
	.clk(clk), .we(bw_we), .waddr(bw_wa), .wdata(bw_wd), .raddr(ir_pc[8:1]), .q(bht_q)
);

// loop exit predictor entries (D1b's), their tags compared here
localparam int LPN = 8;
logic [11:0] lp_tag  [LPN];
logic        lp_v    [LPN];
logic [LPN-1:0] lp_hit;
always_comb
	for (int i = 0; i < LPN; i++)
		lp_hit[i] = lp_v[i] && (lp_tag[i] == qpc[12:1]);

// D1a's terms for IR
logic        a_dsign, a_jtake;
always_comb begin
	logic [2:0]  pi, nimm;
	logic [31:0] disp;
	// the immediate: after the operation word and the fixed words
	pi   = 3'd1 + {1'b0, pd0.nfix};
	nimm = pd0.b - pi;
	case (nimm)
		3'd1:    a_imm = pd0.bzx ? {24'd0, win[pi][7:0]} : sx16(win[pi]);
		3'd2:    a_imm = {win[pi], win[3'(pi + 1)]};
		default: a_imm = 32'd0;
	endcase
	// the branch displacement and target (JSR/JMP: the absolute address,
	// or the PC-relative one): the sums for each displacement form side by
	// side, from the registers, and the form picks one
	begin
		logic [31:0] t8, t16, t32, tfd;
		t8  = qpc + 32'd2 + sx8(opw[7:0]);
		t16 = qpc + 32'd2 + sx16(win[1]);
		t32 = qpc + 32'd2 + {win[1], win[2]};
		tfd = qpc + 32'd4 + sx16(win[2]);
		case (pd0.cls)
			CL_BCC, CL_BSR: begin
				disp   = (opw[7:0] == 8'h00) ? sx16(win[1]) :
				         (opw[7:0] == 8'hFF) ? {win[1], win[2]} : sx8(opw[7:0]);
				a_ntgt = (opw[7:0] == 8'h00) ? t16 : (opw[7:0] == 8'hFF) ? t32 : t8;
			end
			CL_FBCC: begin
				disp   = opw[6] ? {win[1], win[2]} : sx16(win[1]);
				a_ntgt = opw[6] ? t32 : t16;
			end
			CL_FDBCC: begin
				disp   = sx16(win[2]);
				a_ntgt = tfd;
			end
			default: begin                  // DBcc, JSR/JMP (d16,PC)
				disp   = sx16(win[1]);
				a_ntgt = t16;
			end
		endcase
	end
	a_dsign = disp[31];
	if ((pd0.cls == CL_JSR || pd0.cls == CL_JMP) && pd0.i0 == EM_ABSW) a_ntgt = sx16(win[1]);
	if ((pd0.cls == CL_JSR || pd0.cls == CL_JMP) && pd0.i0 == EM_ABSL) a_ntgt = {win[1], win[2]};
	a_jtake = jcond(pd0.jc0, pd0.i0, pd0.i1, win[1], pd0.sz, opw[2:0]);
end

// the BTB's flags on the words taken (tables, no arithmetic)
logic        a_bt_end, a_bt_any;
logic  [2:0] a_bt_j;
always_comb begin
	a_bt_any = |(win_bt & umask[15:8]);
	a_bt_end = u_go && !u_part && (ph == PH_IDLE) && ((win_bt & umask[15:8]) == umask[7:0]);
	a_bt_j = 3'd0;
	for (int i = 7; i >= 0; i--) if (win_bt[i] && umask[8 + i]) a_bt_j = 3'(i);
end

always_ff @(posedge clk) begin
	if (!nrst) begin
		ph        <= PH_IDLE;
		part_pd   <= '0;
		part_len  <= '0;
		part_imml <= '0;
		part_pc   <= '0;
		part_rr   <= '0;
		part_fg   <= 1'b0;
		r_v       <= 1'b0;
		r_go      <= 1'b0;
		r_part    <= 1'b0;
		r_use     <= '0;
		r_nph     <= PH_IDLE;
		rx_q      <= 1'b0;
		ir_v      <= 1'b0;
		ph_idle   <= 1'b1;
	end
	else begin
		ir_v <= fire;
		if (fire) begin
			// the parse
			ph      <= (r_v && !(r_part && rx_q)) ? r_nph : PH_IDLE;
			ph_idle <= !(r_v && !(r_part && rx_q)) || (r_nph == PH_IDLE);
			if (u_part) begin
				if (ph == PH_IDLE) begin
					part_pd   <= pd0;
					part_len  <= {1'b0, u_use};
					part_imml <= (pd0.cls == CL_FPU_GEN) ? fp_immlen(win[1]) :
					             (pd0.sz == SZ_L) ? 3'd2 : 3'd1;
					part_pc   <= qpc;
					part_rr   <= {win[0][11:9], win[0][2:0]};
					part_fg   <= (win[0][15:6] == 10'b1111_0010_00);
				end
				else
					part_len <= part_len + {1'b0, u_use};
			end
		end
		// the general case: decided now, taken next clock (the head stays:
		// nothing is taken this clock).  The decision's terms load while
		// none is held; it is held, stalled or not, until taken.
		if (!r_v) begin
			r_go   <= g_go;
			r_part <= g_part;
			r_use  <= g_use;
			r_nph  <= g_nph;
			rx_q   <= (ph == PH_IDLE) && g_exc;
		end
		r_v <= r_v ? !fire : (!fire && !s_ok && (g_go || g_part));
		// a redirect drops the head and what D1a took meanwhile
		if (flush || d_redir_v) begin
			ph      <= PH_IDLE;
			ph_idle <= 1'b1;
			r_v     <= 1'b0;
		end
	end
end

// IR: loaded every clock, without reset (D1b takes it the clock after,
// never later; ir_v says whether it holds a take): no enable from the late
// take, no reset in its data path
always_ff @(posedge clk) begin
	begin
		ir_go    <= u_go;
		ir_part  <= u_part;
		ir_ph    <= ph;
		ir_odd   <= (ph == PH_IDLE) && q_odd && qcnt == 4'd0;
		ir_flt   <= (win_flt & umask[15:8]) != 8'd0;
		for (int i = 0; i < 5; i++) ir_w[i] <= win[i];
		ir_pd    <= (ph == PH_IDLE) ? pd0 : part_pd;
		begin
			// the EAs, from the words at the head: a one-pass instruction's
			// both (its extension words at b and p1), a first part's those
			// without extension words, a part's own with its extension at
			// 0; the fixed operand forms.  (A PC-relative displacement is
			// the PC sum D1b adds in: ir_epc.)
			ea_t        e0, e1, ep;
			logic       fast, pc0, f0, f1;
			logic [3:0] ip;
			fast = pd0.legal && !pd0.slow && !full0 && !full1 && (pd0.tot <= 4'd7);
			ip   = (ph == PH_EA0) ? part_pd.i0 : part_pd.i1;
			ep   = mk_ea(ip, (ph == PH_EA0) ? part_rr[2:0] : part_rr[5:3], part_len,
			             (ph == PH_EA0) ? part_imml : 3'd0, 32'd0,
			             win[0], win[1], win[2], win[3], win[4]);
			e0   = '0;
			e1   = '0;
			pc0  = 1'b0;
			f0   = 1'b0;
			f1   = 1'b0;
			if (!(q_odd && qcnt == 4'd0)) begin
				e0.m = EM_NONE;
				e1.m = EM_NONE;
				if (pd0.legal) begin
					if (fast) begin
						// (the displacements: D1b's, from the words, f0/f1)
						e0  = mk_ea(pd0.i0, win[0][2:0], {1'b0, pd0.b},
						            (pd0.sz == SZ_L) ? 3'd2 : 3'd1, 32'd0,
						            win[fb], win[3'(fb + 3'd1)], 16'd0, 16'd0, 16'd0);
						e1  = mk_ea(pd0.i1, win[0][11:9], {1'b0, pd0.p1}, 3'd0, 32'd0,
						            win[fp1], win[3'(fp1 + 3'd1)], 16'd0, 16'd0, 16'd0);
						e0.bd = 32'd0;
						e1.bd = 32'd0;
						f0  = 1'b1;
						f1  = 1'b1;
						pc0 = (pd0.i0 == EM_PC16) || (pd0.i0 == EM_PCX && !e0.bs);
					end
					else begin
						e0 = mk_ea(pd0.i0, win[0][2:0], 4'd0, 3'd0, 32'd0,
						           16'd0, 16'd0, 16'd0, 16'd0, 16'd0);
						e1 = mk_ea(pd0.i1, win[0][11:9], 4'd0, 3'd0, 32'd0,
						           16'd0, 16'd0, 16'd0, 16'd0, 16'd0);
					end
					// fixed operand forms: (Ay)+,(Ax)+ and -(Ay),-(Ax); MOVE16
					if (pd0.fea == 2'd3) begin
						pc0 = 1'b0;
						f0  = 1'b0;
						f1  = 1'b0;
						if (win[0][5]) begin
							e0.m = EM_AIP; e0.r = win[0][2:0];
							e1.m = EM_AIP; e1.r = win[1][14:12];
						end
						else begin
							ea_t ra, aa;
							ra = '0; aa = '0;
							ra.m = win[0][4] ? EM_AI : EM_AIP; ra.r = win[0][2:0];
							aa.m = EM_ABSL; aa.bd = {win[1], win[2]};
							e0 = win[0][3] ? aa : ra;
							e1 = win[0][3] ? ra : aa;
						end
					end
					else if (pd0.fea != 2'd0) begin
						e0.m = (pd0.fea == 2'd1) ? EM_AIP : EM_APD;
						e1.m = (pd0.fea == 2'd1) ? EM_AIP : EM_APD;
						e0.r = win[0][2:0];
						e1.r = win[0][11:9];
					end
				end
			end
			ir_ea0 <= (ph == PH_IDLE) ? e0 : ep;
			ir_ea1 <= (ph == PH_IDLE) ? e1 : ep;
			ir_epc <= (ph == PH_IDLE) ? pc0 : ((ip == EM_PC16) || (ip == EM_PCX && !ep.bs));
			ir_e0f <= (ph == PH_IDLE) && f0 && !pc0;
			ir_fimm <= (ph == PH_IDLE) ? (pd0.legal && !(q_odd && qcnt == 4'd0) &&
			                              pd0.cls == CL_FPU_GEN && e0.m == EM_IMM)
			                           : (ph == PH_EA0 && ep.m == EM_IMM && part_fg);
			ir_e1f <= (ph == PH_IDLE) && f1;
			ir_e0[0] <= win[fb];
			ir_e0[1] <= win[3'(fb + 3'd1)];
			ir_e1[0] <= win[fp1];
			ir_e1[1] <= win[3'(fp1 + 3'd1)];
		end
		ir_pc    <= qpc;
		ir_len   <= ((ph == PH_IDLE) ? 4'd0 : part_len) + {1'b0, u_use};
		ir_npc   <= ((ph == PH_IDLE) ? qpc : part_pc) +
		            {27'd0, ((ph == PH_IDLE) ? 4'd0 : part_len) + {1'b0, u_use}, 1'b0};
		ir_pcb0  <= ((ph == PH_IDLE) ? qpc : part_pc) +
		            {27'd0, (ph == PH_IDLE) ? {1'b0, fb} : part_len, 1'b0};
		begin
			// (the EA at the head: a one-pass instruction's EA0 at b, or
			// the part's EA with its full-format words at 0)
			logic [31:0] base;
			base = (ph == PH_IDLE) ? qpc + {28'd0, fb, 1'b0} :
			                         part_pc + {27'd0, part_len, 1'b0};
			ir_pcs0 <= base + ((ph == PH_IDLE) ?
			                   ea_pcd(pd0.i0, win[fb], 16'd0, 16'd0) :
			                   ea_pcd((ph == PH_EA0) ? part_pd.i0 : part_pd.i1,
			                          win[0], win[1], win[2]));
		end
		ir_jj    <= (pd0.cls == CL_JSR || pd0.cls == CL_JMP) && u_go && !u_part &&
		            (pd0.i0 == EM_ABSW || pd0.i0 == EM_ABSL || pd0.i0 == EM_PC16);
		ir_tsel  <= (!pd0.legal || (q_odd && qcnt == 4'd0)) ? 2'd0 :
		            (pd0.cls == CL_BCC || pd0.cls == CL_BSR || pd0.cls == CL_DBCC ||
		             pd0.cls == CL_FBCC || pd0.cls == CL_FDBCC ||
		             ((pd0.cls == CL_JSR || pd0.cls == CL_JMP) && u_go && !u_part &&
		              (pd0.i0 == EM_ABSW || pd0.i0 == EM_ABSL || pd0.i0 == EM_PC16))) ? 2'd1 :
		            (pd0.cls == CL_RTS) ? 2'd2 : 2'd0;
		ir_imm   <= a_imm;
		// the exception: a fault in the words taken goes first
		if ((win_flt & umask[15:8]) != 8'd0) begin
			ir_exc  <= f_code;
			ir_xany <= 1'b1;
		end
		else if (ph == PH_IDLE && q_odd && qcnt == 4'd0) begin
			ir_exc  <= 8'd3;
			ir_xany <= 1'b1;
		end
		else if (ph == PH_IDLE) begin
			ir_exc  <= x_exc;
			ir_xany <= x_any;
		end
		else begin
			ir_exc  <= 8'd0;
			ir_xany <= 1'b0;
		end
		ir_faddr <= f_addr;
		ir_jtake <= a_jtake;
		begin
			// the prediction's terms (a legal one-part record's): the rule by
			// class; D1b's: Bcc's history counter (read at IR's address), the
			// loop exit's overrule, RTS by the return stack's count
			logic ok, jj;
			ok = pd0.legal && !(q_odd && qcnt == 4'd0);
			jj = (pd0.cls == CL_JSR || pd0.cls == CL_JMP) && u_go && !u_part &&
			     (pd0.i0 == EM_ABSW || pd0.i0 == EM_ABSL || pd0.i0 == EM_PC16);
			ir_nr0 <= ok && ((pd0.cls == CL_BSR) || (pd0.cls == CL_BCC && opw[11:8] == 4'h0) || jj ||
			                 (pd0.cls == CL_DBCC && a_dsign) ||
			                 (pd0.cls == CL_FBCC && (opw[5:0] == 6'h0F || a_dsign)) ||
			                 (pd0.cls == CL_FDBCC && a_dsign));
			ir_nrb <= ok && (pd0.cls == CL_BCC) && opw[11:8] != 4'h0;
			ir_lpb <= ok && a_dsign && ((pd0.cls == CL_BCC && opw[11:9] != 3'd0) || pd0.cls == CL_DBCC);
			ir_rts <= ok && (pd0.cls == CL_RTS);
		end
		ir_jsrk  <= (pd0.cls == CL_JSR) && u_go && !u_part &&
		            (pd0.i0 == EM_ABSW || pd0.i0 == EM_ABSL || pd0.i0 == EM_PC16);
		ir_fpk   <= fpu_k(win[1], pd0.i0);
		ir_fpkk  <= kind_fix(10'(fpu_k(win[1], pd0.i0)), 19'd0);
		ir_ntgt  <= a_ntgt;
		ir_dsign <= a_dsign;
		ir_lph   <= lp_hit;
		ir_bt_tgt <= bt_tgt;
		ir_bt_end <= a_bt_end;
		ir_bt_restart <= (u_go || u_part) && a_bt_any && !a_bt_end;
		ir_bt_fa  <= qpc + {28'd0, a_bt_j, 1'b0};
		ir_lastpc <= qpc + {28'd0, u_use - 3'd1, 1'b0};
	end
end

//==========================================================================
// D1b
//==========================================================================
// a redirect (D1b's, the clock before) drops what D1a took meanwhile
wire ir_ok = ir_v && !flush && !flush_q && !d_redir_v;

// the entry's attributes: a ROM (block RAM) whose address register is
// loaded with the head's entry every clock, so its answer is the entry of
// the instruction D1a took (IR) -- not a 276-entry table in logic
pla_t a0;
(* romstyle = "M10K" *) logic [$bits(pla_t)-1:0] attr_rom [0:511];
initial begin
`include "gen/ap68060_attr_rom.svh"
end
logic [$bits(pla_t)-1:0] attr_q;
always_ff @(posedge clk) attr_q <= attr_rom[pd0.ent];
always_comb a0 = ir_pd.legal ? pla_t'(attr_q) : '0;
pla_t attr_q_p;                  // (ungated: the routine of an illegal word is overridden)
assign attr_q_p = pla_t'(attr_q);
// synthesis translate_off
always_ff @(posedge clk)
	if (nrst && ir_v && ir_ph == PH_IDLE && a0 != ent_attr(ir_pd.ent, ir_pd.legal)) begin
		$display("FAIL: decode: the attribute ROM's %h, the table's %h (entry %0d)",
		         a0, ent_attr(ir_pd.ent, ir_pd.legal), ir_pd.ent);
		$finish;
	end
// synthesis translate_on

dinst_t      part;          // record under construction (slow path)
logic        part_call, part_ret, part_fpg, part_bt;
logic [10:0] part_fpk;
logic [18:0] part_fpkk;

// the return stack (a LUT RAM: one push; never cleared -- an entry is read
// only below the count of entries pushed), in two copies: the prediction
// reads at D1b's pointer, the fetch's resynchronization at the pointer a
// back-end redirect restores (the restore is a far register: kept off the
// prediction)
(* ramstyle = "MLAB, no_rw_check" *) logic [31:0] ras [8];
(* ramstyle = "MLAB, no_rw_check" *) logic [31:0] ras_f [8];
// the top entry, kept in a register: the pushed address, the entry under
// it after a pop (copy ras read at ras_tp - 1), the restored pointer's after
// a back-end redirect (copy ras_f)
logic [31:0] ras_top;
wire  [31:0] ras_m1 = ras[ras_tp - 3'd1];
logic        rr_v;              // a back-end restore: the top is read a clock on
logic  [2:0] ras_tp;           // the top entry
logic  [3:0] ras_n;            // entries held (0..8)

// Branch prediction (the back end verifies every prediction in EX):
//  * Bcc: the static rule (backward taken), overruled by a 256-entry
//    history table of 2-bit counters indexed by PC[8:1].  A counter counts
//    how often the branch disagreed with the static rule, so the power-up
//    zeros mean "static"; at 2 or more the prediction is inverted.
//  * JSR/JMP to an absolute or PC-relative address: the target.
//  * RTS: an 8-entry return stack, pushed by BSR/JSR records, popped by
//    RTS records (not repaired after a redirect: a wrong entry only costs
//    the redirect EX would have made anyway).
// training counts from the table's current value (a copy read at the
// training address), not from the value D1 read when it decoded the
// branch: two passes of a short loop are often decoded before the first
// one is trained.  The copy is a block RAM: the training request is
// registered with its read address, and the clock after it the counter is
// read, updated and written to both copies.  A write in the clock the RAM
// registered the read (the request just before, to the same counter) is
// not in what it returns: that write's value is taken instead.
logic  [1:0] bht_t, bht_c, bht_nd;
logic        tr_v, tr_dis;           // the training request, a clock on
logic  [7:0] tr_a;
logic        wq_v;                   // the write of the clock before
logic  [7:0] wq_a;
logic  [1:0] wq_d;
ap68060_sdpr #(.AW(8), .DW(2)) bht_tr (
	.clk(clk), .we(bw_we), .waddr(bw_wa), .wdata(bw_wd), .raddr(bht_wa), .q(bht_t)
);
assign bht_c  = (wq_v && wq_a == tr_a) ? wq_d : bht_t;
assign bht_nd = tr_dis ? ((bht_c == 2'd3) ? 2'd3 : bht_c + 2'd1)
                       : ((bht_c == 2'd0) ? 2'd0 : bht_c - 2'd1);
assign bw_we  = tr_v;
assign bw_wa  = tr_a;
assign bw_wd  = bht_nd;
always_ff @(posedge clk) begin
	tr_v   <= nrst && bht_we;
	tr_a   <= bht_wa;
	tr_dis <= bht_dis;
	wq_v   <= bw_we;
	wq_a   <= bw_wa;
	wq_d   <= bw_wd;
end

// Loop exits: a backward Bcc/DBcc that has been taken the same number of
// times (the trip) before falling through, twice running, is predicted to
// fall through when its count comes round again.  EX trains the count of
// each entry (lp_cnt: taken since the last exit); D1 counts the same
// branch as it predicts it (lp_sp), ahead of EX by the instances in
// flight, and goes back to EX's count after every flush (lp_sync: until
// the first record after it).  lp_x is registered: confident and lp_sp
// at the trip.  8 entries, tagged with pc[12:1]; a mispredicted exit of a
// branch without an entry takes one (round robin).  (D1a compares the
// tags with the instruction's address: ir_lph.)
logic        lp_cf   [LPN];
logic  [7:0] lp_trip [LPN];
logic  [7:0] lp_cnt  [LPN];
logic  [7:0] lp_sp   [LPN];
logic        lp_x    [LPN];
logic [$clog2(LPN)-1:0] lp_rr;
logic        lp_sync;
logic        lp_exit, lp_br, lp_ovr;
always_comb begin
	lp_exit = 1'b0;
	for (int i = 0; i < LPN; i++)
		if (LOOP_PRED && ir_lph[i] && lp_x[i]) lp_exit = 1'b1;
end

//--------------------------------------------------------------------------
// the record
//--------------------------------------------------------------------------
dinst_t      nrec;
logic [31:0] ntarget;
logic        teq;           // ntarget is the BTB's target
logic        nr_ref, lpb_ref; // (simulation: the prediction by the class's case,
logic        dbg_pred;       // and whether D1a's terms agree)
// (the two targets' compares apart, kept: the target's select by the
// class, D1a's ir_tsel, picks the outcome rather than the operand)
(* keep *) wire teq_n;
(* keep *) wire teq_r;
assign teq_n = (ir_ntgt == ir_bt_tgt);
assign teq_r = (ras_top == ir_bt_tgt);
logic        nredir;
wire         ir_lg = ir_pd.legal && !ir_odd;      // (an illegal word has no class)
wire         rcall = (ir_ph == PH_IDLE) ? ir_lg && (ir_pd.cls == CL_BSR || ir_pd.cls == CL_JSR) : part_call;
wire         rret  = (ir_ph == PH_IDLE) ? ir_lg && (ir_pd.cls == CL_RTS) : part_ret;
wire         fpg   = (ir_ph == PH_IDLE) ? ir_lg && (ir_pd.cls == CL_FPU_GEN) : part_fpg;
wire  [10:0] fpk   = (ir_ph == PH_IDLE) ? ir_fpk : part_fpk;
wire  [18:0] fpkk  = (ir_ph == PH_IDLE) ? ir_fpkk : part_fpkk;
always_comb begin
	logic flt;
	nrec    = '0;
	ntarget = '0;
	teq     = (ir_tsel == 2'd2) ? teq_r : teq_n;   // (used with a prediction only)
	nredir  = 1'b0;
	lp_br   = 1'b0;
	lp_ovr  = 1'b0;
	nr_ref  = 1'b0;
	lpb_ref = 1'b0;
	dbg_pred = 1'b1;
	flt     = ir_flt;

	if (ir_ph != PH_IDLE) begin
		// slow path: the EA extension
		ea_t e;
		nrec = part;
		e = (ir_ph == PH_EA0) ? ir_ea0 : ir_ea1;
		if (ir_epc) e.bd = ir_pcs0;
		// an FPU immediate is read from the instruction stream by the
		// microcode: its address follows the extension word (the
		// operation word says FPU: the record's routine is past the
		// first word D1 resolves) -- D1a's flag
		if (ir_fimm) e.bd = ir_pcb0;
		if (ir_ph == PH_EA0) nrec.ea0 = e; else nrec.ea1 = e;
		nrec.ilen = ir_len;
		if (flt) begin
			nrec.exc    = ir_exc;
			nrec.target = ir_faddr;
			nrec.rt     = UA_DEC_EXC;
		end
	end
	else begin
		nrec.pc    = ir_pc;
		nrec.ilen  = ir_len;
		nrec.exc   = ir_exc;
		// (the EAs: D1a's; a one-pass instruction's displacements here)
		nrec.ea0 = ir_ea0;
		if (ir_epc) nrec.ea0.bd = ir_pcs0;
		if (ir_e0f) nrec.ea0.bd = ea_fbd(ir_pd.i0, (ir_pd.sz == SZ_L) ? 3'd2 : 3'd1, ir_e0[0], ir_e0[1]);
		nrec.ea1 = ir_ea1;
		if (ir_e1f) nrec.ea1.bd = ea_fbd(ir_pd.i1, 3'd0, ir_e1[0], ir_e1[1]);
		if (!ir_odd) begin
			nrec.opw   = ir_w[0];
			nrec.ext1  = ir_w[1];
			nrec.ext2  = ir_w[2];
			nrec.sz    = ir_pd.sz;
			nrec.eop   = a0.eop;
			nrec.econd = a0.econd;
			nrec.ccr   = a0.ccr;
			nrec.imm   = ir_imm;
			if (ir_pd.legal) begin
				if (ir_jtake) begin
					nrec.rt = a0.jt0;
					nrec.k0 = a0.kj;
				end
				else begin
					nrec.rt = a0.rtn;
					nrec.k0 = (a0.jc0 != JC_NEVER) ? a0.kn : a0.k0;
				end
			end
		end
		if (ir_xany) nrec.rt = UA_DEC_EXC;

		// PC-relative branches: target and prediction
		if (ir_pd.legal && !ir_odd) case (ir_pd.cls)
			CL_BCC, CL_BSR: begin
				ntarget  = ir_ntgt;
				nrec.bst = ir_dsign;
				nrec.bhc = bht_q;
				nr_ref   = (ir_pd.cls == CL_BSR) || (ir_w[0][11:8] == 4'h0) || (ir_dsign ^ bht_q[1]);
				lpb_ref  = (ir_pd.cls == CL_BCC) && (ir_w[0][11:9] != 3'd0) && ir_dsign;
				if (lpb_ref && lp_exit) nr_ref = 1'b0;
			end
			CL_JSR, CL_JMP:
				if (ir_jj) begin
					ntarget = ir_ntgt;
					nr_ref  = 1'b1;
				end
			CL_RTS:
				if (ras_n != 4'd0) begin
					ntarget = ras_top;
					nr_ref  = 1'b1;
				end
			CL_DBCC: begin
				ntarget  = ir_ntgt;
				nrec.bst = ir_dsign;
				lpb_ref  = ir_dsign;
				nr_ref   = ir_dsign && !lp_exit;
			end
			CL_FBCC: begin
				ntarget = ir_ntgt;
				nr_ref  = (ir_w[0][5:0] == 6'h0F) || ir_dsign;
			end
			CL_FDBCC: begin
				ntarget = ir_ntgt;
				nr_ref  = ir_dsign;
			end
			default: ;
		endcase
		// the prediction from D1a's terms (the class's static rule, the
		// history counter; whether a loop exit can overrule it) and the
		// two D1b keeps: the loop exit, the return stack's count.  (The
		// case above: the reference, for the simulation's check.)
		nredir = ir_rts ? (ras_n != 4'd0)
		                : ((ir_nr0 || (ir_nrb && (ir_dsign ^ bht_q[1]))) && !(ir_lpb && lp_exit));
		lp_br  = ir_lpb;
		lp_ovr = ir_lpb && lp_exit;
		// synthesis translate_off
		dbg_pred = (nredir == nr_ref) && (lp_br == lpb_ref);
		// synthesis translate_on
		// (FPU_GEN: the operation word and the command word, b = 2; D1a's
		// flag)
		if (ir_fimm) nrec.ea0.bd = ir_pcb0;
		// (the target as the prediction's case picks it, by a select formed
		// in D1a: off the class decode)
		nrec.target = flt ? ir_faddr :
		              (ir_tsel == 2'd1) ? ir_ntgt :
		              (ir_tsel == 2'd2 && ras_n != 4'd0) ? ras_top : 32'd0;
		nrec.pred   = nredir && !ir_xany && ir_pd.legal && !ir_odd;
	end
end

// synthesis translate_off
// the EAs D1a formed against the one-clock form from IR's words
logic        ir_fast;
logic  [3:0] ir_plen;
logic  [2:0] ir_il;
always_ff @(posedge clk) begin
	ir_fast  <= pd0.legal && !pd0.slow && !full0 && !full1 && (pd0.tot <= 4'd7);
	ir_plen  <= (ph == PH_IDLE) ? 4'd0 : part_len;
	ir_il    <= (ph == PH_EA0) ? part_imml : 3'd0;
end
always_ff @(posedge clk)
	if (nrst && ir_v) begin
		ea_t r0, r1;
		r0 = '0; r1 = '0;
		if (ir_ph != PH_IDLE) begin
			ea_t e;
			e = mk_ea((ir_ph == PH_EA0) ? ir_pd.i0 : ir_pd.i1,
			          (ir_ph == PH_EA0) ? part.opw[2:0] : part.opw[11:9], ir_plen, ir_il,
			          ir_pcs0, ir_w[0], ir_w[1], ir_w[2], ir_w[3], ir_w[4]);
			r0 = (ir_ph == PH_EA0) ? e : nrec.ea0;
			r1 = (ir_ph == PH_EA1) ? e : nrec.ea1;
			if (ir_ph == PH_EA0 && r0.m == EM_IMM && part.opw[15:6] == 10'b1111_0010_00)
				r0.bd = ir_pcb0;
		end
		else if (!ir_odd) begin
			r0 = '0; r0.m = EM_NONE;
			r1 = '0; r1.m = EM_NONE;
			if (ir_pd.legal) begin
				if (ir_fast) begin
					r0 = mk_ea(ir_pd.i0, ir_w[0][2:0], {1'b0, ir_pd.b},
					           (ir_pd.sz == SZ_L) ? 3'd2 : 3'd1, ir_pcs0,
					           ir_e0[0], ir_e0[1], 16'd0, 16'd0, 16'd0);
					r1 = mk_ea(ir_pd.i1, ir_w[0][11:9], {1'b0, ir_pd.p1}, 3'd0, 32'd0,
					           ir_e1[0], ir_e1[1], 16'd0, 16'd0, 16'd0);
				end
				else begin
					r0 = mk_ea(ir_pd.i0, ir_w[0][2:0], 4'd0, 3'd0, 32'd0,
					           16'd0, 16'd0, 16'd0, 16'd0, 16'd0);
					r1 = mk_ea(ir_pd.i1, ir_w[0][11:9], 4'd0, 3'd0, 32'd0,
					           16'd0, 16'd0, 16'd0, 16'd0, 16'd0);
				end
				if (a0.fea == 2'd3) begin
					if (ir_w[0][5]) begin
						r0.m = EM_AIP; r0.r = ir_w[0][2:0];
						r1.m = EM_AIP; r1.r = ir_w[1][14:12];
					end
					else begin
						ea_t ra, aa;
						ra = '0; aa = '0;
						ra.m = ir_w[0][4] ? EM_AI : EM_AIP; ra.r = ir_w[0][2:0];
						aa.m = EM_ABSL; aa.bd = {ir_w[1], ir_w[2]};
						r0 = ir_w[0][3] ? aa : ra;
						r1 = ir_w[0][3] ? ra : aa;
					end
				end
				else if (a0.fea != 2'd0) begin
					r0.m = (a0.fea == 2'd1) ? EM_AIP : EM_APD;
					r1.m = (a0.fea == 2'd1) ? EM_AIP : EM_APD;
					r0.r = ir_w[0][2:0];
					r1.r = ir_w[0][11:9];
				end
			end
			if (fpg && r0.m == EM_IMM) r0.bd = ir_pcb0;
		end
		if (r0 != nrec.ea0 || r1 != nrec.ea1) begin
			$display("FAIL: decode: the EAs D1a formed (ph %0d): %h %h, the one-clock form's %h %h",
			         ir_ph, nrec.ea0, nrec.ea1, r0, r1);
			$finish;
		end
	end
// synthesis translate_on

// the record as it enters the FIFO: refetched at WB if a BTB flag was
// inside the instruction (now, or in an earlier part).  Its routine and
// that routine's first word's kind are chosen by flags from IR, the
// attribute ROM's answer last
dinst_t nrec_q;
always_comb begin
	logic        snr, exc_any;
	logic  [9:0] rt_b;
	logic [18:0] k0_b;
	snr     = ir_go && (ir_bt_restart || part_bt);
	exc_any = (ir_ph == PH_IDLE) ? ir_xany : ir_flt;
	if (ir_ph != PH_IDLE) begin
		rt_b = part.rt;
		k0_b = part.k0;
	end
	else if (ir_jtake) begin
		rt_b = attr_q_p.jt0;
		k0_b = attr_q_p.kj;
	end
	else begin
		rt_b = attr_q_p.rtn;
		k0_b = (ir_pd.jc0 != JC_NEVER) ? attr_q_p.kn : attr_q_p.k0;
	end
	nrec_q = nrec;
	if (snr || exc_any) begin
		nrec_q.rt = UA_DEC_EXC;
		nrec_q.k0 = UK_DEC_EXC;
	end
	else if (ir_jsrk && ir_ph == PH_IDLE) begin
		nrec_q.rt = UA_JSR_K;
		nrec_q.k0 = UK_JSR_K;
	end
	else if (fpg && fpk[10]) begin
		nrec_q.rt = fpk[9:0];
		nrec_q.k0 = fpkk;
	end
	else begin
		nrec_q.rt = rt_b;
		nrec_q.k0 = k0_b;
	end
	nrec_q.ras.v  = 1'b1;
	nrec_q.ras.tp = ras_tp;
	nrec_q.ras.n  = ras_n;
	nrec_q.ras.k  = (nrec.exc != 8'd0) ? 2'd0 :
	                rcall ? 2'd1 :
	                (rret && ras_n != 4'd0) ? 2'd2 : 2'd0;
	// a branch the fetch already took, as D1 decodes it (the branch cache's
	// prediction, confirmed)
	nrec_q.bcp = (ir_ph == PH_IDLE) && ir_bt_end && nrec.pred && teq;
	nrec_q.eak = ea_kind(nrec.ea1, nrec.ea0);
	if (ir_go && (ir_bt_restart || part_bt)) begin
		nrec_q.bcp  = 1'b0;
		nrec_q.exc  = EXC_SNR;
		nrec_q.rt   = UA_DEC_EXC;
		nrec_q.pred = 1'b0;
		nrec_q.ras.k = 2'd0;
	end
end
// synthesis translate_off
always_ff @(posedge clk)
	if (nrst && push && nrec_q.k0 != kind_fix(nrec_q.rt, (nrec_q.rt == nrec.rt) ? nrec.k0 : 19'h7FFFF)) begin
		$display("FAIL: decode: the record's routine %0d and its first word's kind %h", nrec_q.rt, nrec_q.k0);
		$finish;
	end
// synthesis translate_on
// synthesis translate_off
always @(posedge clk)
	if (nrst && ir_ok && ir_ph == PH_IDLE && nrec.target != (ir_flt ? ir_faddr : ntarget)) begin
		$display("FAIL: decode: the record's target %h, the prediction's %h", nrec.target, ntarget);
		$finish;
	end
always @(posedge clk)
	if (nrst && ir_ok && ir_ph == PH_IDLE && ir_pd.legal && !ir_odd && !ir_part &&
	    ir_go && ir_exc == 8'd0 && !ir_pd.slow &&
	    (ir_pd.cls == CL_JSR || ir_pd.cls == CL_JMP) &&
	    (ir_pd.i0 == EM_ABSW || ir_pd.i0 == EM_ABSL || ir_pd.i0 == EM_PC16) &&
	    nrec.ea0.bd != ir_ntgt) begin
		$display("FAIL: decode: the JSR/JMP target %h, its EA %h", ir_ntgt, nrec.ea0.bd);
		$finish;
	end
// synthesis translate_on

wire  push = ir_ok && ir_go;
// (D2 pops only a record it is sequencing: rq0's, rq_n > 0)
wire  pop  = rq_pop;
// (the FIFO's data are written by IR's own flags; a write a kill drops
// lands past the count: the kills, far or late, only touch the count)
wire  wr   = ir_v && ir_go;
wire  btb_ok = (ntarget[0] == 1'b0) && (nrec.exc == 8'd0);
// the prediction for either value of Bcc's history bit (nredir's terms:
// D1a's but the loop exit and the return stack's count), and the redirect
// and BTB write decided from each: {bq_wv, bq_we, d_redir_v}
wire  pred_ok = !ir_xany && ir_pd.legal && !ir_odd;
wire  pred_h0 = pred_ok && (ir_rts ? (ras_n != 4'd0)
                                   : ((ir_nr0 || (ir_nrb &&  ir_dsign)) && !(ir_lpb && lp_exit)));
wire  pred_h1 = pred_ok && (ir_rts ? (ras_n != 4'd0)
                                   : ((ir_nr0 || (ir_nrb && !ir_dsign)) && !(ir_lpb && lp_exit)));
// (a record refetched at WB predicts nothing: nrec_q.pred)
wire  snr_q   = ir_go && (ir_bt_restart || part_bt);
function automatic logic [2:0] rdec(input logic pr, input logic pu, input logic idle, input logic be,
                                    input logic te, input logic sn, input logic lo, input logic bo);
	rdec = 3'b000;
	if (pu && idle && be) begin
		// the fetch already took this branch: fine if D1 agrees
		// (a loop exit keeps the entry: the next pass takes it)
		if (!(pr && te)) rdec = {pr && bo, !lo, 1'b1};
	end
	else if (pu && pr && !sn && idle)
		rdec = {1'b1, bo, 1'b1};          // next time the fetch takes it
endfunction
(* keep *) wire [2:0] dr_h0;
(* keep *) wire [2:0] dr_h1;
assign dr_h0 = rdec(pred_h0, push, ir_ph == PH_IDLE, ir_bt_end, teq, snr_q, lp_ovr, btb_ok);
assign dr_h1 = rdec(pred_h1, push, ir_ph == PH_IDLE, ir_bt_end, teq, snr_q, lp_ovr, btb_ok);
// the entry's kind: a call pushes the fetch's return stack, a return pops it
wire  [1:0] btb_kind = rcall ? 2'd1 : rret ? 2'd2 : 2'd0;
always_comb begin
	// a back-end redirect resynchronizes the fetch's stack from the
	// restored pointer (its pointer and the top entry)
	// (D1b's own registers: the fetch samples them two clocks after any
	// redirect, when a back-end restore has settled -- D1b takes nothing
	// in those clocks)
	ras_tp_o = ras_tp;
	ras_n_o  = ras_n;
	ras_o    = ras_top[31:1];
end

// loop exit table updates.  EX's training a clock late: registered here
// with each entry's tag compare (and the entry the training in front of it
// is taking, so two trainings of a new branch do not both take one)
logic        lpt_we, lpt_tk, lpt_mp;
logic [11:0] lpt_wa;
logic [LPN-1:0] lpt_hit;         // the entries holding the trained branch
wire         lpt_whit = |lpt_hit;
wire         lpt_alloc = lpt_we && !lpt_tk && lpt_mp && !lpt_whit;   // it takes lp_rr
always_ff @(posedge clk) begin
	lpt_we <= nrst && lp_we;
	lpt_wa <= lp_wa;
	lpt_tk <= lp_tk;
	lpt_mp <= lp_mp;
	for (int i = 0; i < LPN; i++)
		lpt_hit[i] <= (lp_v[i] && lp_tag[i] == lp_wa) ||
		              (lpt_alloc && lp_rr == $clog2(LPN)'(i) && lpt_wa == lp_wa);
end
wire lp_push = push && (ir_ph == PH_IDLE) && lp_br && (nrec.exc == 8'd0);
always_ff @(posedge clk) begin
	if (!nrst) begin
		lp_rr   <= '0;
		lp_sync <= 1'b0;
		for (int i = 0; i < LPN; i++) begin
			lp_tag[i]  <= '0;
			lp_v[i]    <= 1'b0;
			lp_cf[i]   <= 1'b0;
			lp_trip[i] <= '0;
			lp_cnt[i]  <= '0;
			lp_sp[i]   <= '0;
			lp_x[i]    <= 1'b0;
		end
	end
	else begin
		if (flush) lp_sync <= 1'b1;
		else if (push) lp_sync <= 1'b0;
		if (lpt_alloc) lp_rr <= lp_rr + 1'b1;
		for (int i = 0; i < LPN; i++) begin
			logic        v_n, cf_n;
			logic  [7:0] trip_n, cnt_n, base, inc;
			logic [11:0] tag_n;
			v_n    = lp_v[i];
			cf_n   = lp_cf[i];
			trip_n = lp_trip[i];
			cnt_n  = lp_cnt[i];
			tag_n  = lp_tag[i];
			if (lpt_we && lpt_hit[i]) begin
				if (lpt_tk)
					cnt_n = (lp_cnt[i] == 8'hFF) ? 8'hFF : lp_cnt[i] + 8'd1;
				else begin
					cf_n   = (lp_cnt[i] == lp_trip[i]) && (lp_cnt[i] != 8'hFF);
					trip_n = lp_cnt[i];
					cnt_n  = 8'd0;
				end
			end
			else if (lpt_alloc && lp_rr == $clog2(LPN)'(i)) begin
				v_n    = 1'b1;
				tag_n  = lpt_wa;
				cf_n   = 1'b0;
				trip_n = 8'hFF;
				cnt_n  = 8'd0;
			end
			// D1's count: one more for a pushed instance predicted taken,
			// none for one predicted to exit; back to EX's count from a flush
			// to the first record after it (a clock behind EX: nothing is
			// in flight to train it then).  lp_x, confident and the count at
			// the trip, is formed for each case and the push picks last.
			base = lp_sp[i];
			inc  = (base == 8'hFF) ? 8'hFF : base + 8'd1;
			lp_v[i]    <= v_n;
			lp_tag[i]  <= tag_n;
			lp_cf[i]   <= cf_n;
			lp_trip[i] <= trip_n;
			lp_cnt[i]  <= cnt_n;
			if (lp_push && ir_lph[i]) begin
				lp_sp[i] <= nrec.pred ? inc : 8'd0;
				lp_x[i]  <= v_n && cf_n && (nrec.pred ? (inc == trip_n) : (trip_n == 8'd0));
			end
			else if (flush || lp_sync) begin
				lp_sp[i] <= cnt_n;
				lp_x[i]  <= v_n && cf_n && (cnt_n == trip_n);
			end
			else begin
				lp_sp[i] <= base;
				lp_x[i]  <= v_n && cf_n && (base == trip_n);
			end
		end
	end
end

// the records' data and the part under construction: no reset (the count
// and IR's flags qualify them)
always_ff @(posedge clk) begin
	case ({wr, pop})
		2'b01: begin
			rq0 <= rq1; rq1 <= rq2; rq2 <= rq3;
		end
		2'b10: begin
			case (rq_n)
				3'd0:    rq0 <= nrec_q;
				3'd1:    rq1 <= nrec_q;
				3'd2:    rq2 <= nrec_q;
				default: rq3 <= nrec_q;
			endcase
		end
		2'b11: begin
			rq0 <= (rq_n == 3'd1) ? nrec_q : rq1;
			rq1 <= (rq_n == 3'd2) ? nrec_q : rq2;
			rq2 <= (rq_n == 3'd3) ? nrec_q : rq3;
			rq3 <= nrec_q;
		end
		default: ;
	endcase
	if (ir_v && ir_part) part <= nrec;
end

//--------------------------------------------------------------------------
// D1b registers: the redirect, the BTB's maintenance, the return stack,
// the parts of a multi-part instruction, the record FIFO
//--------------------------------------------------------------------------
dinst_t rq2, rq3;
// the BTB write D1b decides, written out a clock later (its decision is
// the redirect's: a deep path, kept off the write's fields)
logic        bq_we, bq_wv, bq_wslot;
logic [BTB_AW-1:0] bq_wi;
logic [BTB_TW-1:0] bq_wtag;
logic  [1:0] bq_wkind;
logic [30:0] bq_wtgt;
always_ff @(posedge clk) begin
	btb_we    <= nrst && bq_we;
	btb_wv    <= bq_wv;
	btb_wi    <= bq_wi;
	btb_wtag  <= bq_wtag;
	btb_wslot <= bq_wslot;
	btb_wkind <= bq_wkind;
	btb_wtgt  <= bq_wtgt;
end
always_ff @(posedge clk) begin
	if (!nrst) begin
		rq_n       <= 3'd0;
		room_q     <= 1'b1;
		d_redir_v  <= 1'b0;
		d_redir_pc <= '0;
		hold_redir <= 1'b0;
		bq_we      <= 1'b0;
		bq_wi      <= '0;
		bq_wv      <= 1'b0;
		bq_wtag    <= '0;
		bq_wslot   <= 1'b0;
		bq_wkind   <= '0;
		bq_wtgt    <= '0;
		part_bt    <= 1'b0;
		part_call  <= 1'b0;
		part_ret   <= 1'b0;
		part_fpg   <= 1'b0;
		part_fpk   <= '0;
		part_fpkk  <= '0;
		ras_tp     <= '0;
		ras_n      <= '0;
		ras_top    <= '0;
		rr_v       <= 1'b0;
	end
	else begin
		d_redir_v  <= 1'b0;
		hold_redir <= d_redir_v;
		bq_we      <= 1'b0;
		rr_v       <= flush && ras_rv;
		if (rr_v) ras_top <= ras_f[ras_tp];

		if (ir_ok) begin
			// (a flagged word inside an instruction drops its entry -- the
			// record is refetched at WB: bq_we below)
			// the flag of an earlier part stays with the instruction
			part_bt <= ir_part && (part_bt || ir_bt_restart);
			// a call writes the slot above the top, push or not: only the
			// push moves the top.  A call that never pushes (faulted) can
			// cost the entry it overwrote -- the oldest, or one a recovery
			// would restore -- which only costs a prediction: EX checks
			// every return
			if (rcall) begin
				ras[ras_tp + 3'd1]   <= ir_npc;
				ras_f[ras_tp + 3'd1] <= ir_npc;
			end
			if (ir_part) begin
				if (ir_ph == PH_IDLE) begin
					part_call <= rcall;
					part_ret  <= rret;
					part_fpg  <= fpg;
					part_fpk  <= ir_fpk;
					part_fpkk <= ir_fpkk;
				end
			end
		end
		if (flush) part_bt <= 1'b0;

		if (push && nrec_q.exc == 8'd0) begin
			if (rcall) begin
				ras_tp  <= ras_tp + 3'd1;
				ras_top <= ir_npc;
				if (ras_n != 4'd8) ras_n <= ras_n + 4'd1;
			end
			else if (rret && ras_n != 4'd0) begin
				ras_tp  <= ras_tp - 3'd1;
				ras_top <= ras_m1;
				ras_n   <= ras_n - 4'd1;
			end
		end

		// the redirect's and the BTB write's fields, every clock (they are
		// used only with d_redir_v / bq_we): the decision, which waits for
		// the target compare, only sets those
		// (the decision is formed for either value of the history counter's
		// bit, which D1b reads from its table this clock and picks last)
		d_redir_pc <= (bht_q[1] ? pred_h1 : pred_h0) ? ntarget : ir_npc;
		bq_wi      <= ir_bt_restart ? ir_bt_fa[BTB_AW+1:2]  : ir_lastpc[BTB_AW+1:2];
		bq_wtag    <= ir_bt_restart ? ir_bt_fa[31:BTB_AW+2] : ir_lastpc[31:BTB_AW+2];
		bq_wslot   <= ir_bt_restart ? ir_bt_fa[1] : ir_lastpc[1];
		bq_wkind   <= btb_kind;
		bq_wtgt    <= ntarget[31:1];
		d_redir_v  <= bht_q[1] ? dr_h1[0] : dr_h0[0];
		// (a flagged word inside an instruction drops the entry: the
		// decision never fires with it)
		bq_we      <= (ir_ok && ir_bt_restart) || (bht_q[1] ? dr_h1[1] : dr_h0[1]);
		bq_wv      <= bht_q[1] ? dr_h1[2] : dr_h0[2];

		// four records: D1a took this one only with room for it (the data:
		// below, without reset)
		rq_n <= rq_n + {2'd0, push} - {2'd0, pop};
		// the room D1a sees next clock: the count after this clock's push
		// and pop, and the record IR then holds (this clock's take)
		begin
			logic [3:0] rn;
			rn = {1'b0, rq_n} + {3'd0, push} +
			     {3'd0, cs_fire || (cr_fire && (r_go || (r_part && rx_q)))};
			room_q <= pop ? (rn <= 4'd4) : (rn < 4'd4);
		end

		if (flush) begin
			rq_n       <= 3'd0;
			room_q     <= 1'b1;
			if (ras_rv) begin
				ras_tp  <= ras_rtp;
				ras_n   <= ras_rn;
			end
			d_redir_v  <= 1'b0;
			hold_redir <= 1'b0;
		end
	end
end
// synthesis translate_off
always_ff @(posedge clk)
	if (nrst && !rr_v && ras_n != 4'd0 && ras_top != ras[ras_tp]) begin
		$display("FAIL: decode: the return stack's top register %h, the stack's %h", ras_top, ras[ras_tp]);
		$finish;
	end
always_ff @(posedge clk)
	if (nrst && room_q != (({1'b0, rq_n} + {3'd0, ir_v && ir_go}) < 4'd4)) begin
		$display("FAIL: decode: the registered room %b, the count %0d", room_q, rq_n);
		$finish;
	end
always_ff @(posedge clk)
	if (nrst && !flush && !flush_q && !hold_redir && !d_redir_v &&
	    q_flt != |(win_flt & 8'((9'd1 << qcnt) - 9'd1))) begin
		$display("FAIL: decode: a fault flag past the queue's count");
		$finish;
	end
always_ff @(posedge clk)
	if (nrst && ir_v && ir_ph == PH_IDLE && !dbg_pred) begin
		$display("FAIL: decode: the prediction from D1a's terms, the class's case's");
		$finish;
	end
always_ff @(posedge clk)
	if (nrst && ir_v && ir_ph == PH_IDLE && (bht_q[1] ? pred_h1 : pred_h0) != nrec.pred) begin
		$display("FAIL: decode: the prediction by the history bit's two cases");
		$finish;
	end
always_ff @(posedge clk)
	if (nrst && rq_pop && rq_n == 3'd0) begin
		$display("FAIL: decode: a pop of the empty record FIFO");
		$finish;
	end
always_ff @(posedge clk)
	if (nrst && ir_v && ir_xany != (ir_exc != 8'd0)) begin
		$display("FAIL: decode: ir_xany");
		$finish;
	end
always_ff @(posedge clk)
	if (nrst && ph_idle != (ph == PH_IDLE)) begin
		$display("FAIL: decode: ph_idle");
		$finish;
	end
always_ff @(posedge clk)
	if (nrst && push && !pop && rq_n == 3'd4) begin
		$display("FAIL: decode: a record into a full FIFO");
		$finish;
	end
// synthesis translate_on

endmodule
