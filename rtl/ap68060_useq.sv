//--------------------------------------------------------------------------//
// AP68060 - pipelined MC68040                                            //
//                                                                          //
// ap68060_useq.sv - D2: micro-sequencer                                     //
//                                                                          //
// Turns a decoded instruction into uops by walking its microcode routine   //
// (rtl/gen/ap68060_ucode_rom.svh), one uop per cycle:                       //
//   * symbolic operands (EA0, DX, IMM, ...) become registers, immediates   //
//     or the uop's own memory access; A7 maps to USP or ISP by S (the      //
//     68060 has one supervisor stack; M is only a bit of SR)               //
//   * an EA with memory indirection gets a pointer load (into T13 for EA0, //
//     T12 for EA1) before the uop that uses it                             //
//   * a source (An)+/-(An) update marked noupd is deferred into the uop    //
//     flagged upd2 (merged when both EAs use the same register), so an     //
//     instruction that faults on its second access restarts cleanly        //
//   * the back end's exception entry starts an exception routine (the     //
//     clock after it, from its kind registered here)                       //
//                                                                          //
// Three stages:                                                            //
//   D2a  the microcode address (the ROM's address register).  The next    //
//        address comes from registers only: every word carries the kind   //
//        (last, jump, target) of the word after it and of its jump target,//
//        so the kind of the word being read is a register (a_k) and the   //
//        ROM's output only feeds registers.  A jump word is resolved here //
//        (an unconditional one at once, a conditional one a clock later). //
//        What depends on the word and its record alone and sits deep in  //
//        the uop (the memory EA, its access, the explicit AG modes'       //
//        displacement, the operand selectors' registers, the pointer-load //
//        candidates, the MOVEM terms) is formed here; the record carries  //
//        its EAs' kinds from D1 (no mode decode here).                     //
//   D2b  the uop from the word and its record in registers, without the   //
//        instruction's running state: a memory-indirect EA's uop through //
//        its pointer, and the pointer's address apart.                     //
//   D2c  the uop the state makes of it: the pointer load first, MOVEM's    //
//        transfers and fix-ups, a deferred update merged; into the uop    //
//        FIFO (two words, a ring: the back end's pop moves its read      //
//        pointer only).  A word that repeats (pointer load, MOVEM) holds //
//        D2b and D2a; whether the word will repeat is itself a register   //
//        (c_stay_q), formed a clock ahead, as are MOVEM's per-transfer    //
//        terms (a transfer ahead).                                         //
//--------------------------------------------------------------------------//

module ap68060_useq
	import ap68060_pkg::*, ap68060_upkg::*;
(
	input  logic        clk,
	input  logic        nreset,
	input  logic        flush,

	// record FIFO from D1
	input  logic  [2:0] rq_n,
	input  dinst_t      rq0,
	input  dinst_t      rq1,
	output logic        rq_pop,

	input  logic        smode,

	input  logic        exc_go,
	input  logic  [3:0] exc_kind,
	input  logic  [4:0] exc_ssp,

	// uop FIFO towards the back end: uq0 is the head
	output logic  [1:0] uq_n,
	output uop_t        uq0,
	input  logic        uq_pop      // the back end takes uq0 this cycle
);

// (the reset, registered here: the core's is a gate on its reset counters
// and reaches every module; the front end leaves and enters reset a clock
// later -- the back end's reset exception reaches D2 the clock after its
// own reset ends, a clock later still)
logic nrst;
always_ff @(posedge clk) nrst <= nreset;

// an operand selector's register terms (osel, below)
typedef struct packed {
	logic       v;
	logic [1:0] src;
	logic [4:0] r;
} osel_t;

// S, registered here (it changes with a flush: no word is in D2 the clock
// after); the register naming (A7) and JC_SUPER read it
logic smode_q;
always_ff @(posedge clk) smode_q <= smode;

//--------------------------------------------------------------------------
// D2a state: the word being read
//--------------------------------------------------------------------------
logic  [9:0] upc;               // its address (the ROM's address register's copy)
logic  [9:0] upc1;              // ... plus one
logic        a_v;               // it belongs to the current instruction
ukind_t      a_k;               // its kind
logic        a_exc;             // it is an exception routine's
logic        a_pl;              // it is an instruction's last word, no jump
                                // (a_v, a_k, a_exc: the record pop's term)
logic        a_cjv, a_cjt;      // a conditional jump's condition, evaluated
uword_t      ur;                // the word (the ROM's output)

//--------------------------------------------------------------------------
// D2b state: the word the base uop is built from
//--------------------------------------------------------------------------
logic        b_v;
uword_t      ub;                // the word
dinst_t      rb;                // its record ('0 for an exception routine)
logic        b_exr;             // an exception routine's
// per word, formed in D2a: the pointer-load candidates (the EA the uop
// accesses is EA0 / EA1 and memory indirect), the MOVEM terms, the uop's
// memory EA (the first memory operand, or its AG mode's), whether it
// computes an address, its access; an explicit AG mode's displacement;
// the mask's lowest bit for MOVEM's first transfer; the next PC
logic        b_pc0, b_pc1;
logic        b_base_in, b_fb_in, b_fx_in, b_e1pc1;
logic        b_mev, b_me1, b_agin;
logic [12:0] b_ia, b_ib;        // the A and B operands' immediate source (one-hot)
logic        b_e1pc2, b_e1pc3;  // the MOVEM mask has at most two / three bits
logic  [1:0] b_mem;
osel_t       b_oa, b_ob, b_od;  // the operand selectors' register terms
logic [31:0] b_dsv;             // an explicit AG mode's displacement ...
logic        b_dov;             // ... which replaces the memory EA's
logic  [4:0] b_amt0, b_amt1;    // EA0's, EA1's (An)+/-(An) step
logic  [3:0] b_bi1;
logic [15:0] b_low1;
logic [31:0] b_npc;

//--------------------------------------------------------------------------
// D2c state: the base uop and its terms, and the instruction's state
//--------------------------------------------------------------------------
typedef struct packed {
	logic        base_v;
	logic  [4:0] base;
	logic        idx_v;
	logic  [4:0] idx;
	logic        idx_l;
	logic  [1:0] scale;
	logic [31:0] disp;
	logic        mprog;
} agf_t;                         // a uop's address fields
logic        c_v;
uop_t        cu;                // the base uop
agf_t        c_v1;              // the pointer's address (a memory-indirect EA)
logic        c_pc0, c_pc1;      // pointer-load candidates
logic        c_loop, c_base_in, c_fb_in, c_fx_in, c_e1pc1;
logic        c_upd2, c_m16;     // the deferred update lands here; MOVE16
logic  [1:0] c_memi;            // the memory EA's indirection
logic        c_mem_aip;         // ... it is (An)+
logic        c_aipd;            // ... (An)+ or -(An)
logic        c_mupd_v;          // the EA's own update (B's update read)
logic  [4:0] c_mupd_reg;
logic        c_bupd_x;          // (simulation: B's update read as one clock forms it)
logic        c_e1pc2, c_e1pc3;  // the record's mask has at most two / three bits
logic        mv_pc2;            // the mask has at most two bits
logic        c_bok, c_beq;      // the word's AG base is valid; it is the deferred
logic  [7:0] c_mamt;            // register; its own step plus the deferred one
logic        c_def_set;         // the uop defers its source update
logic  [4:0] c_def_reg;
logic  [7:0] c_def_amt;
// MOVEM: -(An), (An)+, the transfer size, An and the index register, the
// register numbers of An and the index in mask order, the first
// transfer's lowest bit, the continuation's cond/target (indexed modes),
// a load list
logic        c_pd, c_pi, c_mvcm, c_mvld;
logic  [2:0] c_sb;
logic  [4:0] c_an, c_xreg;
logic        c_stay_q;          // the word repeats after this clock's uop
logic        first;
logic        ind0, ind1;        // pointer of EA0 / EA1 already loaded
logic        def_v;             // a deferred source update is pending
logic  [4:0] def_reg;
logic  [7:0] def_amt;
logic  [4:0] ssp;
// MOVEM loop
logic        mv_act;            // the loop has started for this instruction
logic        mv_is0, mv_pc1;    // no register / at most one still to transfer
logic        mv_t11;            // the address is in T11 (base register in the list)
logic  [1:0] mv_fix;            // fix-up uops emitted after the last transfer
// ... the transfer's terms, formed a transfer ahead (the first's from D2b's
// as the word enters D2c): the registers still to transfer (this one's
// included), this one's bit, its register, whether that is An (T7 for a
// loaded base) or the index (T6), the destination a load writes, whether
// it is the last transfer, its address offset, the bytes after it
logic [15:0] t_mask, t_low;
logic  [4:0] t_r, t_dreg;
logic        t_isan, t_dv, t_last;
logic [31:0] t_off;
logic  [7:0] t_amt;
logic [15:0] t_rest;            // ... the registers after this transfer
// (whether a transfer is An's or the index's: the mask bits still below
// theirs, counted down per transfer, and whether theirs is in the mask)
logic  [4:0] c_ban, c_bx;
logic        c_anin, c_xin;

// the D2b build's view: the word and record (D2b's)
dinst_t      src;
assign src = rb;
uword_t      uw;
assign uw = ub;

// uop FIFO: two words, a ring; uq0 is the one at the read pointer (one
// LUT from flip-flops).  The back end's pop (late) moves the pointer only:
// the words are written at the write pointer by D2c's emit, from registers
(* ramstyle = "logic" *) uop_t ue [2];
logic        uq_rp, uq_wp;
// ... but the register numbers the back end reads its register file with
// at once (A, B, the AG's base and index) are the head's own flip-flops,
// the pop the last select (few of them: the late pop fans out little)
logic  [4:0] h_areg, h_breg, h_base, h_idx;
always_comb begin
	uq0        = uq_rp ? ue[1] : ue[0];
	uq0.a_reg  = h_areg;
	uq0.b_reg  = h_breg;
	uq0.base   = h_base;
	uq0.idx    = h_idx;
end
wire         room_out = (uq_n != 2'd2);

// at most one bit set
function automatic logic pc1(input logic [15:0] m);
	pc1 = ((m & (m - 16'd1)) == 16'd0);
endfunction
// at most k bits set (k <= 3)
function automatic logic pcle(input logic [15:0] m, input logic [1:0] k);
	logic [4:0] n;
	n = '0;
	for (int i = 0; i < 16; i++) n = n + {4'd0, m[i]};
	pcle = (n <= {3'd0, k});
endfunction
// set bits
function automatic logic [4:0] popc16(input logic [15:0] m);
	popc16 = '0;
	for (int i = 0; i < 16; i++) popc16 = popc16 + {4'd0, m[i]};
endfunction
// the lowest set bit's number
function automatic logic [3:0] enc16(input logic [15:0] m);
	enc16 = 4'd0;
	for (int i = 15; i >= 0; i--) if (m[i]) enc16 = 4'(i);
endfunction

//--------------------------------------------------------------------------
// register naming
//--------------------------------------------------------------------------
wire [4:0] sp_reg = !smode_q ? R_USP : R_ISP;

function automatic logic [4:0] areg(input logic [2:0] n, input logic [4:0] spr);
	areg = (n == 3'd7) ? spr : (R_A0 + {2'b00, n});
endfunction

function automatic logic [4:0] dreg(input logic [2:0] n);
	dreg = {2'b00, n};
endfunction
// MOVEM: a transfer's register from its mask bit number (-(An): the mask
// is reversed, bit 0 is A7), and what a load writes: T7 for a loaded base,
// T6 for a loaded index (both committed after the last transfer), nothing
// for (An)+'s own An
function automatic logic [4:0] mvreg(input logic [3:0] bi, input logic pd, input logic [4:0] spr);
	logic [3:0] rn;
	rn    = pd ? ~bi : bi;
	mvreg = rn[3] ? areg(rn[2:0], spr) : dreg(rn[2:0]);
endfunction
function automatic logic [5:0] mvdst(input logic [4:0] r, input logic isan, input logic isx,
                                     input logic fb, input logic fx, input logic pd, input logic pi);
	if (fb && isan)            mvdst = {5'(R_T0 + 5'd7), 1'b1};
	else if (fx && !pd && isx) mvdst = {5'(R_T0 + 5'd6), 1'b1};
	else                       mvdst = {r, !(pi && isan)};
endfunction
// an operand selector's register terms (valid, source kind, register):
// D2a forms them from the word and the record, D2b takes them as they are
// (its immediates come by class: imm_cls)
// an EA as an operand: its register, the immediate, the uop's memory
// access, or (a register-only selector) none
function automatic osel_t ea_os(input ea_t e, input logic [4:0] spr, input logic ronly);
	osel_t o;
	o     = '0;
	o.v   = 1'b1;
	o.src = OS_REG;
	if (e.m == EM_DN) o.r = dreg(e.r);
	else if (e.m == EM_AN) o.r = areg(e.r, spr);
	else if (e.m == EM_IMM) o.src = OS_IMM;
	else if (ronly) o.v = 1'b0;
	else o.src = OS_MEM;
	ea_os = o;
endfunction
function automatic osel_t osel(input logic [5:0] s, input dinst_t rc, input logic [4:0] spr,
                               input logic [4:0] sspr);
	osel_t o;
	o     = '0;
	o.v   = (s != S_NONE);
	o.src = OS_REG;
	if (s >= 6'd40) o.r = 5'(s - 6'd25);
	else case (s)
		// (each EA's terms from the record alone: the selector picks last)
		S_EA0:   o = ea_os(rc.ea0, spr, 1'b0);
		S_EA0R:  o = ea_os(rc.ea0, spr, 1'b1);
		S_EA1:   o = ea_os(rc.ea1, spr, 1'b0);
		S_EA1R:  o = ea_os(rc.ea1, spr, 1'b1);
		S_DX:    o.r = dreg(rc.opw[11:9]);
		S_DY:    o.r = dreg(rc.opw[2:0]);
		S_AX:    o.r = areg(rc.opw[11:9], spr);
		S_AY:    o.r = areg(rc.opw[2:0], spr);
		S_SP:    o.r = spr;
		S_SSP:   o.r = sspr;
		S_X1R:   o.r = rc.ext1[15] ? areg(rc.ext1[14:12], spr) : dreg(rc.ext1[14:12]);
		S_X1DL:  o.r = dreg(rc.ext1[14:12]);
		S_X1DH:  o.r = dreg(rc.ext1[2:0]);
		S_X1DU:  o.r = dreg(rc.ext1[8:6]);
		S_X2DC:  o.r = dreg(rc.ext2[2:0]);
		S_X2DU:  o.r = dreg(rc.ext2[8:6]);
		S_X2R:   o.r = rc.ext2[15] ? areg(rc.ext2[14:12], spr) : dreg(rc.ext2[14:12]);
		S_IMM, S_QUICK, S_MOVEQ, S_NPC, S_PC, S_CONST, S_CREG, S_OPW, S_EXT1: o.src = OS_IMM;
		S_SHCNT: if (rc.opw[5]) o.r = dreg(rc.opw[11:9]); else o.src = OS_IMM;
		S_ZERO:  o.src = OS_ZERO;
		S_LD:    o.src = OS_MEM;
		S_BFO:   if (rc.ext1[11]) o.r = dreg(rc.ext1[8:6]); else o.src = OS_IMM;
		S_BFW:   if (rc.ext1[5]) o.r = dreg(rc.ext1[2:0]); else o.src = OS_IMM;
		S_FPDL:  o.r = dreg(rc.ext1[6:4]);
		S_CREGR: o.r = R_USP;
		default: o.v = 1'b0;
	endcase
	osel = o;
endfunction

function automatic logic is_mem(input logic [3:0] m);
	is_mem = (m >= EM_AI) && (m != EM_IMM) && (m != EM_NONE);
endfunction

function automatic logic is_reg(input logic [3:0] m);
	is_reg = (m == EM_DN) || (m == EM_AN);
endfunction

// minus a small amount: the low bits negated, the rest the sign -- no
// 32-bit carry chain after the sequencer's selects
function automatic logic [31:0] neg5(input logic [4:0] x);
	neg5 = {{27{x != 5'd0}}, 5'd0 - x};
endfunction
function automatic logic [31:0] neg7(input logic [6:0] x);
	neg7 = {{25{x != 7'd0}}, 7'd0 - x};
endfunction

function automatic logic [4:0] size_bytes(input logic [1:0] s, input logic is_sp);
	case (s)
		SZ_B:    size_bytes = is_sp ? 5'd2 : 5'd1;
		SZ_W:    size_bytes = 5'd2;
		SZ_L:    size_bytes = 5'd4;
		default: size_bytes = 5'd16;      // line (MOVE16)
	endcase
endfunction

typedef struct packed {
	logic        v;          // operand present
	logic  [1:0] src;
	logic  [4:0] r;
	logic        imm_v;
	logic [31:0] imm;
	logic        mem;        // the operand is this uop's memory EA
	logic        ea1;        // ... and it is EA1
} opsel_t;

// FPU operand bytes for AG modes FEA / FUPD: opclass 010/011 by format (a
// byte through A7 moves it by two), FMOVE(M) of control registers four per
// register; the other FPU instructions move their address register by a
// run-time amount (ADDT0)
function automatic logic [6:0] fpsz(input logic [15:0] ext1, input logic [2:0] r);
	logic [2:0] crl;
	crl = (ext1[12:10] == 3'd0) ? 3'b001 : ext1[12:10];
	case (ext1[15:13])
		3'b010, 3'b011: case (ext1[12:10])
			3'd0, 3'd1: fpsz = 7'd4;
			3'd4:       fpsz = 7'd2;
			3'd6:       fpsz = (r == 3'd7) ? 7'd2 : 7'd1;
			3'd5:       fpsz = 7'd8;
			default:    fpsz = 7'd12;
		endcase
		3'b100, 3'b101: fpsz = {3'd0, 2'(crl[0]) + 2'(crl[1]) + 2'(crl[2]), 2'b00};
		default: fpsz = 7'd0;
	endcase
endfunction
wire [6:0] fp_size = fpsz(src.ext1, src.ea0.r);

// an operand's immediate, by its source (one-hot, formed in D2a from the
// selector and the record): EA0's, EA1's, the record's, QUICK (and a quick
// shift count), MOVEQ, NPC, PC, the word's constant, MOVEC's register,
// the bit field's offset and width, the operation word, the extension
function automatic logic [12:0] imm_cls(input logic [5:0] s, input dinst_t r);
	imm_cls = '0;
	case (s)
		S_EA0, S_EA0R: imm_cls[0]  = (r.ea0.m == EM_IMM);
		S_EA1, S_EA1R: imm_cls[1]  = (r.ea1.m == EM_IMM);
		S_IMM:         imm_cls[2]  = 1'b1;
		S_QUICK:       imm_cls[3]  = 1'b1;
		S_SHCNT:       imm_cls[3]  = !r.opw[5];
		S_MOVEQ:       imm_cls[4]  = 1'b1;
		S_NPC:         imm_cls[5]  = 1'b1;
		S_PC:          imm_cls[6]  = 1'b1;
		S_CONST:       imm_cls[7]  = 1'b1;
		S_CREG:        imm_cls[8]  = 1'b1;
		S_BFO:         imm_cls[9]  = !r.ext1[11];
		S_BFW:         imm_cls[10] = !r.ext1[5];
		S_OPW:         imm_cls[11] = 1'b1;
		S_EXT1:        imm_cls[12] = 1'b1;
		default: ;
	endcase
endfunction
function automatic logic [31:0] imm_val(input logic [12:0] c, input dinst_t r,
                                        input logic [15:0] cval, input logic [31:0] npc);
	logic [31:0] q;
	q = (r.opw[11:9] == 3'd0) ? 32'd8 : {29'd0, r.opw[11:9]};
	imm_val = ({32{c[0]}}  & r.ea0.bd) |
	          ({32{c[1]}}  & r.ea1.bd) |
	          ({32{c[2]}}  & r.imm) |
	          ({32{c[3]}}  & q) |
	          ({32{c[4]}}  & {{24{r.opw[7]}}, r.opw[7:0]}) |
	          ({32{c[5]}}  & npc) |
	          ({32{c[6]}}  & r.pc) |
	          ({32{c[7]}}  & {{16{cval[15]}}, cval}) |
	          ({32{c[8]}}  & {27'd0, r.ext1[11], r.ext1[3:0]}) |
	          ({32{c[9]}}  & {27'd0, r.ext1[10:6]}) |
	          ({32{c[10]}} & {27'd0, r.ext1[4:0]}) |
	          ({32{c[11]}} & {16'd0, r.opw}) |
	          ({32{c[12]}} & {16'd0, r.ext1});
endfunction

// D2b's memory EA (D2a picked which)
ea_t         bme;
assign bme = b_me1 ? rb.ea1 : rb.ea0;

//--------------------------------------------------------------------------
// D2b: the base uop from (uw, src), no per-instruction state.  A memory-
// indirect EA's uop goes through its pointer (T13 for EA0, T12 for EA1);
// the pointer's address (D2c's pointer load) is formed apart: nb_v1.
//--------------------------------------------------------------------------
uop_t        nb;
agf_t        nb_v1;
logic        nb_mupd_v;      // the EA's own update (before the explicit AG modes)
logic  [4:0] nb_mupd_reg;
logic        dbg_med;        // (simulation: the memory EA's displacement and step
                             // differ from the mode's,
logic        dbg_bupd;       // B's update read, as one clock forms it,
logic        dbg_imm;        // and the operands' immediates by their selectors)
logic        nb_def_set;     // the uop defers its source update
logic  [4:0] nb_def_reg;
logic  [7:0] nb_def_amt;
always_comb begin
	opsel_t oa, ob, od;
	uop_t   nu;
	ea_t    me;              // the uop's memory EA
	logic   me_v, me_ea1;
	logic [1:0] osz, msz;
	logic [2:0] mszf;
	logic [4:0] amt;
	logic       me_sp;
	logic [31:0] q;
	logic [6:0] op;
	logic [3:0] cnd;

	nu         = '0;
	dbg_med    = 1'b0;
	nb_v1      = '0;
	nb_mupd_v  = 1'b0;
	nb_mupd_reg = '0;
	dbg_bupd   = 1'b0;
	nb_def_set = 1'b0;
	nb_def_reg = '0;
	nb_def_amt = '0;

	osz = (uw.sz  == 2'd0) ? src.sz : (uw.sz  - 2'd1);
	mszf = (uw.msz == 3'd0) ? {1'b0, src.sz} : (uw.msz - 3'd1);
	msz  = mszf[1:0];
	q   = (src.opw[11:9] == 3'd0) ? 32'd8 : {29'd0, src.opw[11:9]};

	//----------------------------------------------------------------------
	// operand selectors
	//----------------------------------------------------------------------
	for (int k = 0; k < 3; k++) begin
		logic [5:0] s;
		opsel_t o;
		s = (k == 0) ? uw.a : (k == 1) ? uw.b : uw.d;
		o = '0;
		o.v = (s != S_NONE);
		o.src = OS_REG;
		if (s >= 6'd40) o.r = 5'(s - 6'd25);
		else case (s)
			S_EA0, S_EA0R, S_EA1, S_EA1R: begin
				ea_t e;
				e = (s == S_EA0 || s == S_EA0R) ? src.ea0 : src.ea1;
				if (e.m == EM_DN) o.r = dreg(e.r);
				else if (e.m == EM_AN) o.r = areg(e.r, sp_reg);
				else if (e.m == EM_IMM) begin
					o.src = OS_IMM; o.imm_v = 1'b1; o.imm = e.bd;
				end
				else if (s == S_EA0R || s == S_EA1R) o.v = 1'b0;
				else begin
					o.src = OS_MEM; o.mem = 1'b1; o.ea1 = (s == S_EA1);
				end
			end
			S_DX:    o.r = dreg(src.opw[11:9]);
			S_DY:    o.r = dreg(src.opw[2:0]);
			S_AX:    o.r = areg(src.opw[11:9], sp_reg);
			S_AY:    o.r = areg(src.opw[2:0], sp_reg);
			S_SP:    o.r = sp_reg;
			S_SSP:   o.r = ssp;
			S_X1R:   o.r = src.ext1[15] ? areg(src.ext1[14:12], sp_reg) : dreg(src.ext1[14:12]);
			S_X1DL:  o.r = dreg(src.ext1[14:12]);
			S_X1DH:  o.r = dreg(src.ext1[2:0]);
			S_X1DU:  o.r = dreg(src.ext1[8:6]);
			S_X2DC:  o.r = dreg(src.ext2[2:0]);
			S_X2DU:  o.r = dreg(src.ext2[8:6]);
			S_X2R:   o.r = src.ext2[15] ? areg(src.ext2[14:12], sp_reg) : dreg(src.ext2[14:12]);
			S_IMM:   begin o.src = OS_IMM; o.imm_v = 1'b1; o.imm = src.imm; end
			S_QUICK: begin o.src = OS_IMM; o.imm_v = 1'b1; o.imm = q; end
			S_MOVEQ: begin o.src = OS_IMM; o.imm_v = 1'b1; o.imm = {{24{src.opw[7]}}, src.opw[7:0]}; end
			S_SHCNT: begin
				if (src.opw[5]) o.r = dreg(src.opw[11:9]);
				else begin o.src = OS_IMM; o.imm_v = 1'b1; o.imm = q; end
			end
			S_NPC:   begin o.src = OS_IMM; o.imm_v = 1'b1; o.imm = b_npc; end
			S_PC:    begin o.src = OS_IMM; o.imm_v = 1'b1; o.imm = src.pc; end
			S_CONST: begin o.src = OS_IMM; o.imm_v = 1'b1; o.imm = {{16{uw.cval[15]}}, uw.cval}; end
			S_ZERO:  o.src = OS_ZERO;
			S_LD:    o.src = OS_MEM;
			// MOVEC register number, as the special register selector:
			// $000-$008 -> $00-$08, $800-$808 -> $10-$18
			S_CREG:  begin o.src = OS_IMM; o.imm_v = 1'b1; o.imm = {27'd0, src.ext1[11], src.ext1[3:0]}; end
			S_BFO: begin
				if (src.ext1[11]) o.r = dreg(src.ext1[8:6]);
				else begin o.src = OS_IMM; o.imm_v = 1'b1; o.imm = {27'd0, src.ext1[10:6]}; end
			end
			S_BFW: begin
				if (src.ext1[5]) o.r = dreg(src.ext1[2:0]);
				else begin o.src = OS_IMM; o.imm_v = 1'b1; o.imm = {27'd0, src.ext1[4:0]}; end
			end
			S_OPW:   begin o.src = OS_IMM; o.imm_v = 1'b1; o.imm = {16'd0, src.opw}; end
			S_EXT1:  begin o.src = OS_IMM; o.imm_v = 1'b1; o.imm = {16'd0, src.ext1}; end
			S_FPDL:  o.r = dreg(src.ext1[6:4]);      // FMOVEM dynamic list register
			S_CREGR: o.r = R_USP;
			default: o.v = 1'b0;
		endcase
		if (k == 0) oa = o; else if (k == 1) ob = o; else od = o;
	end


	//----------------------------------------------------------------------
	// operation fields
	//----------------------------------------------------------------------
	op  = uw.op_inst ? src.eop : uw.op;
	case (uw.cond_inst)
		3'd1:    cnd = src.opw[11:8];
		3'd2:    cnd = src.econd;
		3'd3:    cnd = {2'b00, src.ext1[10], src.ext1[11]};
		3'd4:    cnd = {1'b0, src.opw[10:8]};     // bit field, register
		3'd5:    cnd = {1'b1, src.opw[10:8]};     // bit field, memory
		3'd6:    cnd = {2'b00, !src.ext1[15], src.ext1[11]};  // CHK2/CMP2
		default: cnd = uw.cond;
	endcase
	nu.op     = op;
	nu.sz     = osz;
	nu.msz    = msz;
	nu.cond   = cnd;
	nu.ccr_we = uw.ccr_inst ? src.ccr : uw.ccr;
	nu.a_src  = b_oa.v ? b_oa.src : OS_ZERO;
	nu.a_reg  = b_oa.r;
	nu.a_sxw  = (uw.sxw == 2'd1) || (uw.sxw == 2'd2 && src.sz == SZ_W);
	nu.b_src  = b_ob.v ? b_ob.src : OS_ZERO;
	nu.b_reg  = b_ob.r;
	// (the immediates by their sources' one-hot classes from D2a: an
	// AND-OR, not the selector's decode; A takes the word's constant when
	// it has no immediate of its own)
	nu.imm    = imm_val(b_ia, src, uw.cval, b_npc);
	nu.imm_b  = imm_val(b_ib, src, uw.cval, b_npc);
	// synthesis translate_off
	dbg_imm = (nu.imm == (oa.imm_v ? oa.imm : {{16{uw.cval[15]}}, uw.cval})) && (nu.imm_b == ob.imm) &&
	          {oa.v, oa.src, oa.r} == b_oa && {ob.v, ob.src, ob.r} == b_ob && {od.v, od.src, od.r} == b_od;
	// synthesis translate_on
	nu.mfc    = (uw.mfc == MFC_EAP) ? MFC_NORM : uw.mfc;
	nu.dyn    = uw.dyn;
	nu.dynk   = uw.dynk;
	nu.mlock  = uw.lock;
	nu.mlocke = uw.locke;
	nu.br     = uw.br;
	nu.target = src.target;
	nu.pred   = src.pred && (uw.br != BR_NONE);
	nu.bst    = src.bst;
	nu.bhc    = src.bhc;
	nu.ser    = uw.ser;
	// FMOVEM / FMOVE of control registers to memory are T0 trace points
	nu.bcp    = src.bcp && (uw.br != BR_NONE);
	nu.pc     = src.pc;
	nu.ilen   = src.ilen;
	nu.ras    = src.ras;
	nu.exr    = b_exr;
	nu.first  = 1'b0;               // (D2c's)
	nu.last   = uw.last;
	nu.exc    = (src.exc != 8'd0) ? src.exc : 8'd0;

	// destination
	if (b_od.v && b_od.src == OS_REG) begin
		nu.d_v   = 1'b1;
		nu.d_reg = b_od.r;
	end

	//----------------------------------------------------------------------
	// memory access and address generation
	//----------------------------------------------------------------------
	// (the uop's memory EA, its access and whether it computes an
	// address: formed in D2a from the word and the record, b_me*)
	me_v   = b_mev;
	me_ea1 = b_me1;
	me     = bme;
	me_sp  = (me.r == 3'd7);
	amt    = me_ea1 ? b_amt1 : b_amt0;       // (used by (An)+ and -(An) only)
	nu.mem = b_mem;

	if (b_agin) begin
		logic [4:0] bre;
		nu.ag = 1'b1;
		bre   = areg(me.r, sp_reg);
		case (me.m)
			EM_AI: begin nu.base_v = 1'b1; nu.base = bre; end
			EM_AIP: begin
				nu.base_v = 1'b1; nu.base = bre;
				nu.pinc = 1'b1; nu.upd_v = 1'b1; nu.upd_reg = bre;
				nu.upd_amt = {3'd0, amt};
			end
			EM_APD: begin
				nu.base_v = 1'b1; nu.base = bre;
				nu.upd_v = 1'b1; nu.upd_reg = bre;
			end
			EM_AD16: begin nu.base_v = 1'b1; nu.base = bre; end
			EM_PC16: nu.mprog = 1'b1;          // (D1 added the PC)
			EM_AX, EM_PCX: begin
				logic [4:0] xr;
				xr = me.xa ? areg(me.xr, sp_reg) : dreg(me.xr);
				// the address (or the pointer's address) from base/index/bd
				if (me.m == EM_AX) begin nb_v1.base_v = !me.bs; nb_v1.base = bre; nb_v1.disp = me.bd; end
				else begin nb_v1.disp = me.bd; nb_v1.mprog = !me.bs; end   // D1 added the PC
				nb_v1.idx_v = !me.is && (me.mi != 2'd2);
				nb_v1.idx   = xr;
				nb_v1.idx_l = me.xl;
				nb_v1.scale = me.sc;
				if (me.mi == 2'd0) begin
					nu.base_v = nb_v1.base_v; nu.base = nb_v1.base;
					nu.idx_v = nb_v1.idx_v; nu.idx = nb_v1.idx; nu.idx_l = nb_v1.idx_l;
					nu.scale = nb_v1.scale; nu.mprog = nb_v1.mprog;
				end
				else begin
					// through the pointer: T13 (EA0) / T12 (EA1) + index + od
					nu.base_v = 1'b1;
					nu.base   = me_ea1 ? (R_T0 + 5'd12) : (R_T0 + 5'd13);
					nu.idx_v  = !me.is && (me.mi == 2'd2);
					nu.idx    = xr;
					nu.idx_l  = me.xl;
					nu.scale  = me.sc;
					nu.mprog  = 1'b0;
				end
			end
			default: ;
		endcase
		// the displacement, by the memory EA's kind (D1's, in the record):
		// bd, od (through the pointer), -(An)'s step
		case (me_ea1 ? src.eak[7:6] : src.eak[5:4])
			2'd1:    nu.disp = me.bd;
			2'd2:    nu.disp = me.od;
			2'd3:    nu.disp = neg5(amt);
			default: nu.disp = 32'd0;
		endcase
		// synthesis translate_off
		dbg_med = ((me.m == EM_APD) ? neg5(size_bytes(msz, me_sp)) :
		           ((me.m == EM_AX || me.m == EM_PCX) && me.mi != 2'd0) ? me.od :
		           (me.m == EM_AD16 || me.m == EM_ABSW || me.m == EM_ABSL || me.m == EM_PC16 ||
		            me.m == EM_AX || me.m == EM_PCX) ? me.bd : 32'd0) != nu.disp ||
		          ((me.m == EM_AIP || me.m == EM_APD) && amt != size_bytes(msz, me_sp));
		// synthesis translate_on

		// deferral of the source update into a later uop
		if (uw.noupd && !me_ea1 && (me.m == EM_AIP || me.m == EM_APD)) begin
			logic xuse;
			// an EA1 index naming the same register cannot see a deferred
			// value: update in place then
			xuse = (src.ea1.m == EM_AX || src.ea1.m == EM_PCX) && !src.ea1.is &&
			       src.ea1.xa && (areg(src.ea1.xr, sp_reg) == bre);
			if (!xuse) begin
				nu.upd_v   = 1'b0;
				nu.pinc    = 1'b0;
				nb_def_set = 1'b1;
				nb_def_reg = bre;
				nb_def_amt = (me.m == EM_AIP) ? {3'd0, amt} : -{3'd0, amt};
			end
		end
	end
	// ADDA.L (A0)+,A0 / CMPA.L -(A0),A0: the destination register is read
	// after the source EA's update (the 68000 family computes the EA first):
	// D2c compares B with the EA's own update (before the explicit AG modes)
	nb_mupd_v   = nu.upd_v;
	nb_mupd_reg = nu.upd_reg;
	// synthesis translate_off
	dbg_bupd = nu.upd_v && !me_ea1 && (me.m == EM_AIP || me.m == EM_APD) &&
	           nu.b_src == OS_REG && nu.b_reg == nu.upd_reg;
	// synthesis translate_on
	nu.b_upd = 1'b0;
	// (the 68060 MOVES Rn,(An)+/-(An) stores the original Rn, as MOVE does --
	// WinUAE gencpu; the 68040 computed the EA first)
	nu.a_upd = 1'b0;

	//----------------------------------------------------------------------
	// explicit AG modes (a word with one never takes a deferred update)
	//----------------------------------------------------------------------
	begin
		logic [4:0]  rb, rw;
		logic        rb_v;
		rb_v = (uw.agb != S_NONE) && (uw.agb != S_ZERO);
		case (uw.agb)
			S_X1R: rb = src.ext1[15] ? areg(src.ext1[14:12], sp_reg) : dreg(src.ext1[14:12]);
			S_X2R: rb = src.ext2[15] ? areg(src.ext2[14:12], sp_reg) : dreg(src.ext2[14:12]);
			S_AX:  rb = areg(src.opw[11:9], sp_reg);
			S_AY:  rb = areg(src.opw[2:0], sp_reg);
			S_SP:  rb = sp_reg;
			S_SSP: rb = ssp;
			default: rb = 5'(uw.agb - 6'd25);   // USP ISP MSP T0-T13 (>= 40)
		endcase
		case (uw.agw)
			S_AX:  rw = areg(src.opw[11:9], sp_reg);
			S_AY:  rw = areg(src.opw[2:0], sp_reg);
			S_SP:  rw = sp_reg;
			S_SSP: rw = ssp;
			default: rw = 5'(uw.agw - 6'd25);
		endcase
		case (uw.ag)
			AGM_PUSH: begin
				nu.ag = 1'b1; nu.base_v = 1'b1; nu.base = sp_reg;
				nu.upd_v = 1'b1; nu.upd_reg = sp_reg;
			end
			AGM_POP: begin
				nu.ag = 1'b1; nu.base_v = 1'b1; nu.base = sp_reg;
				nu.pinc = 1'b1; nu.upd_v = 1'b1; nu.upd_reg = sp_reg;
				nu.upd_amt = {3'd0, size_bytes(msz, 1'b1)};
			end
			AGM_POPR: begin
				nu.ag = 1'b1; nu.base_v = 1'b1; nu.base = rb;
				nu.pinc = 1'b1; nu.upd_v = 1'b1; nu.upd_reg = rw;
				nu.upd_amt = {3'd0, size_bytes(msz, 1'b0)};
			end
			AGM_BASED, AGM_BASEDU: begin
				nu.ag = 1'b1; nu.base_v = rb_v; nu.base = rb;
				if (uw.ag == AGM_BASEDU) begin
					nu.upd_v = 1'b1;
					nu.upd_reg = (uw.agw != S_NONE) ? rw : rb;
				end
				else if (uw.agw != S_NONE) begin
					nu.upd_v = 1'b1; nu.upd_reg = rw;
				end
			end
			AGM_ADDC: begin
				nu.ag = 1'b1; nu.base_v = rb_v; nu.base = rb;
				nu.upd_v = 1'b1; nu.upd_reg = rw;
			end
			AGM_VAL0, AGM_ADDV: begin
				nu.ag = 1'b1;
				nu.base_v = (uw.ag == AGM_ADDV); nu.base = rb;
				if (src.ea0.m != EM_IMM) begin
					nu.idx_v = 1'b1;
					nu.idx   = (src.ea0.m == EM_AN) ? areg(src.ea0.r, sp_reg) : dreg(src.ea0.r);
					nu.idx_l = (src.sz == SZ_L);
				end
				nu.upd_v = 1'b1; nu.upd_reg = rw;
			end
			AGM_ADDT0: begin
				nu.ag = 1'b1; nu.base_v = 1'b1; nu.base = rb;
				nu.idx_v = 1'b1; nu.idx = R_T0; nu.idx_l = 1'b1;
				nu.upd_v = 1'b1; nu.upd_reg = rw;
			end
			AGM_LEA0: begin
				if (uw.agw != S_NONE) begin nu.upd_v = 1'b1; nu.upd_reg = rw; end
				nu.mem = M_NONE;
			end
			AGM_FEA: begin
				// the FPU operand's address into agw: (An)+ An, -(An) An -
				// size (the register moves with FUPD), #imm its place in the
				// instruction stream (D1: bd = PC + 4; a byte is the word's
				// low byte), else the EA
				nu.mem = M_NONE;
				if (src.ea0.m == EM_AIP || src.ea0.m == EM_APD) begin
					nu.ag = 1'b1; nu.base_v = 1'b1; nu.base = areg(src.ea0.r, sp_reg);
					nu.idx_v = 1'b0; nu.pinc = 1'b0;
				end
				else if (src.ea0.m == EM_IMM) begin
					nu.ag = 1'b1; nu.base_v = 1'b0; nu.idx_v = 1'b0; nu.pinc = 1'b0;
					nu.mprog = 1'b1;
				end
				nu.upd_v = 1'b1; nu.upd_reg = rw;
			end
			AGM_FUPD: begin
				if (src.ea0.m == EM_AIP || src.ea0.m == EM_APD) begin
					nu.ag = 1'b1; nu.base_v = 1'b1; nu.base = areg(src.ea0.r, sp_reg);
					nu.idx_v = 1'b0; nu.pinc = 1'b0;
					nu.upd_v = 1'b1; nu.upd_reg = areg(src.ea0.r, sp_reg);
				end
			end
			default: ;
		endcase
		// the mode's displacement (PUSH, BASED, ADDC, VAL0/ADDV of an
		// immediate, FEA and FUPD of (An)+/-(An)/#imm): D2a's, whole
		if (b_dov) nu.disp = b_dsv;
	end

	// FMOVEM of control registers: a slot whose register is not in the
	// (static) list is no operation
	if (uw.dyn == 3'd4) begin
		logic [2:0] crl;
		crl = (src.ext1[12:10] == 3'd0) ? 3'b001 : src.ext1[12:10];
		if (!crl[2 - uw.dynk[1:0]]) begin
			nu.mem = M_NONE; nu.ag = 1'b0; nu.upd_v = 1'b0; nu.upd2_v = 1'b0;
			nu.d_v = 1'b0;
		end
		nu.dyn = 3'd0;
	end

	// FPU operand transfers read the EA's space: program space for
	// PC-relative and immediate sources
	if (uw.mfc == MFC_EAP)
		nu.mprog = (src.ea0.m == EM_PC16 || src.ea0.m == EM_PCX || src.ea0.m == EM_IMM);


	nb = nu;
end

//--------------------------------------------------------------------------
// D2c: the uop the instruction's state makes of the base uop
//--------------------------------------------------------------------------
uop_t        nu;
logic        n_ptr;          // nu is a pointer load (memory indirect)
logic        n_mv_pre;       // nu is the MOVEM address copy into T11
logic        n_mv_tx;        // ... a MOVEM transfer
logic [15:0] n_mv_rest;      // mask after this transfer
logic        dbg_dm;             // (simulation: the merge and the
logic [31:0] dbg_dpre;           // displacement it was formed for;
logic        dbg_mvd;            // a MOVEM offset for AG, and the offset)
logic [31:0] dbg_mvo;
always_comb begin
	logic        dmerge;
	nu        = cu;
	nu.first  = first;
	nu.b_upd  = c_mupd_v && !c_me1 && c_aipd && cu.b_src == OS_REG && cu.b_reg == c_mupd_reg;
	dbg_mvo   = '0;
	n_ptr     = (c_pc0 && !ind0) || (c_pc1 && !ind1);
	n_mv_pre  = 1'b0;
	n_mv_tx   = 1'b0;
	n_mv_rest = '0;

	//----------------------------------------------------------------------
	// MOVEM: one transfer per set mask bit.  -(An): mask bit 0 is A7 and the
	// addresses descend from An; other modes ascend from the EA.  An is
	// updated by the last transfer only.
	//----------------------------------------------------------------------
	if (c_loop) begin
		// the transfer's terms are registers (t_*), formed a transfer
		// ahead: the first's as the word enters D2c, each next one's as a
		// transfer is emitted
		n_mv_rest = t_mask & ~t_low;
		// A load list that contains a register of the EA (the base, or the
		// index of an indexed mode) works from a copy of the address in
		// T11, and the loaded base/index commits only after the last
		// transfer (through T7/T6 and fix-up uops): a fault on a later
		// transfer restarts the instruction with the original EA.  (An)+
		// discards a loaded An: the final An is the incremented address.
		nu.last = t_last && !c_fb_in && !c_fx_in;
		if (c_mvld) begin
			nu.d_v = t_dv; nu.d_reg = t_dreg;
		end
		else begin
			nu.a_src = OS_REG; nu.a_reg = t_r;
			// -(An) storing An itself writes An - size (68020 and later)
			if (c_pd && t_isan) begin
				nu.op = OP_SUB; nu.sz = SZ_L;
				nu.a_src = OS_IMM; nu.imm = {29'd0, c_sb};
				nu.b_src = OS_REG; nu.b_reg = c_an;
			end
		end
		if (mv_act && mv_is0) begin
			// after the last transfer: commit the deferred base, then index
			uop_t p;
			logic fxb;
			fxb = c_fb_in && mv_fix == 2'd0;
			p = '0;
			p.pc = cu.pc; p.ilen = cu.ilen; p.exr = cu.exr; p.first = 1'b0;
			p.op = OP_MOV; p.sz = SZ_L; p.ccr_we = 5'd0;
			p.a_src = OS_REG; p.a_reg = fxb ? (R_T0 + 5'd7) : (R_T0 + 5'd6);
			p.b_src = OS_ZERO;
			p.d_v = 1'b1; p.d_reg = fxb ? c_an : c_xreg;
			p.last = !(fxb && c_fx_in);
			p.exc = cu.exc;
			nu = p;
			n_mv_rest = 16'd0;
		end
		else if (c_base_in && !mv_t11) begin
			// first: T11 = the address, then the transfers from T11
			uop_t p;
			p = '0;
			p.pc = cu.pc; p.ilen = cu.ilen; p.exr = cu.exr; p.first = first;
			p.ag = 1'b1;
			p.base_v = cu.base_v; p.base = cu.base;
			p.idx_v = cu.idx_v; p.idx = cu.idx; p.idx_l = cu.idx_l; p.scale = cu.scale;
			p.disp = cu.disp; p.mprog = cu.mprog;
			p.upd_v = 1'b1; p.upd_reg = R_T0 + 5'd11;
			p.exc = cu.exc;
			nu = p;
			n_mv_pre = 1'b1;
		end
		else begin
			n_mv_tx = 1'b1;
			if (mv_t11) begin
				nu.base_v = 1'b1; nu.base = R_T0 + 5'd11;
				nu.idx_v = 1'b0; nu.disp = 32'd0;
			end
			// MOVEM continuation (SSW CM): for the indexed and PC-relative
			// modes a transfer carries its offset from the calculated EA;
			// a fault stacks that EA, and RTE resumes from it
			if (c_mvcm) begin
				nu.cond   = 4'hC;
				nu.target = t_off;
			end
			// address of this transfer: the offset (within +-64) is added
			// by AG as the uop enters it, as a deferred update's is (off
			// the sequencer's EA selects)
			if (c_pd) nu.disp = t_off;        // undo the generic -(An) offset
			else begin
				nu.dmv = 1'b1;
				nu.dma = t_off[7:0];
				dbg_mvo = t_off;
			end
			nu.pinc = 1'b0;
			nu.upd_v = 1'b0;
			if (t_last && c_pd) begin
				nu.upd_v = 1'b1; nu.upd_reg = c_an;          // An = lowest address
			end
			if (t_last && c_pi) begin
				nu.pinc = 1'b1; nu.upd_v = 1'b1; nu.upd_reg = c_an;
				nu.upd_amt = t_amt;
			end
		end
	end

	// the deferred update lands on the upd2 uop, memory or not (FRESTORE
	// commits (An)+ only after its frame check).  (No upd2 word has an
	// explicit AG mode or a MOVEM loop: the base's address fields are its
	// EA's.)  The displacement merge is AG's, as the uop enters it.
	dmerge = 1'b0;
	if (c_upd2 && def_v) begin
		logic same;
		// (the base's compare with the deferred register and the merged
		// step: registers, formed as the word entered D2c)
		same = c_bok && c_beq && (c_memi == 2'd0 || !(c_me1 ? ind1 : ind0));
		// MOVE16 (Ax)+,(Ay)+ with Ax = Ay: both transfers use the original
		// address and the register advances once (WinUAE, the cputest
		// oracle); CMPM-style pairs see the first update
		if (same && c_m16) begin
			// the destination's own (An)+ is the only update
		end
		else if (same) begin
			dmerge = 1'b1;
			if (c_mem_aip) nu.upd_amt = c_mamt;
		end
		if (!(same && (c_aipd || c_m16))) begin
			nu.upd2_v   = 1'b1;
			nu.upd2_reg = def_reg;
			nu.upd2_amt = def_amt;
		end
	end
	dbg_dm   = dmerge;
	dbg_dpre = cu.disp;
	dbg_mvd  = nu.dmv;              // (MOVEM's offset, before a merge)
	if (dmerge) begin
		nu.dmv  = 1'b1;
		nu.dma  = def_amt;
	end

	// a pointer load replaces the uop this cycle (the condition from the
	// candidates D2a formed: the EA selection is not on its path).  (A
	// MOVEM transfer's offset is zero before its pointer is loaded.)
	if (n_ptr) begin
		uop_t p;
		p = '0;
		p.pc = cu.pc; p.ilen = cu.ilen; p.exr = cu.exr; p.first = first; p.last = 1'b0;
		p.op = OP_MOV; p.sz = SZ_L; p.msz = SZ_L; p.ccr_we = 5'd0;
		p.a_src = OS_MEM; p.b_src = OS_ZERO;
		p.ag = 1'b1;
		p.base_v = c_v1.base_v; p.base = c_v1.base;
		p.idx_v = c_v1.idx_v; p.idx = c_v1.idx; p.idx_l = c_v1.idx_l; p.scale = c_v1.scale;
		p.disp = c_v1.disp;
		// MOVE.L -(A7),([A7],...): the pointer's address sees the source's
		// deferred update, merged as for a base (the final uop, through
		// T12, carries upd2)
		if (c_upd2 && def_v && c_v1.base_v && c_v1.base == def_reg) begin
			p.dmv = 1'b1;
			p.dma = def_amt;
		end
		p.mem = M_LD; p.mprog = c_v1.mprog;
		p.d_v = 1'b1;
		p.d_reg = c_pc1 ? (R_T0 + 5'd12) : (R_T0 + 5'd13);
		p.exc = cu.exc;
		nu = p;
	end
end

// Whether a word repeats, from its per-word terms and the state its next
// uop sees: a pointer load first (its EA's pointer not loaded yet); MOVEM:
// the T11 copy, then a transfer per register, then the fix-ups
function automatic logic stay_fn(input logic loop, input logic c0, input logic c1,
                                 input logic i0, input logic i1, input logic base_in,
                                 input logic fb_in, input logic fx_in, input logic act,
                                 input logic is0, input logic mpc1, input logic t11,
                                 input logic [1:0] fix, input logic e1pc1);
	logic fixp, pre, last;
	fixp = act && is0;
	pre  = !fixp && base_in && !t11;
	last = fixp ? !(fb_in && fix == 2'd0 && fx_in) : ((act ? mpc1 : e1pc1) && !fb_in && !fx_in);
	stay_fn = (c0 && !i0) || (c1 && !i1) || (loop && (pre || !last));
endfunction

//--------------------------------------------------------------------------
// D2a: the per-word terms D2b keeps, from the ROM's output and the record
//--------------------------------------------------------------------------
dinst_t as;                      // D2a's record
assign as = a_exc ? '0 : rq0;

function automatic logic omem(input logic [5:0] s, input dinst_t r, output logic e1);
	e1   = (s == S_EA1);
	omem = (s == S_EA0 && !(r.ea0.m == EM_DN || r.ea0.m == EM_AN || r.ea0.m == EM_IMM)) ||
	       (s == S_EA1 && !(r.ea1.m == EM_DN || r.ea1.m == EM_AN || r.ea1.m == EM_IMM));
endfunction
// (the same from the record's EA kinds, D1's: no mode decode here)
function automatic logic omemk(input logic [5:0] s, input logic [7:0] k, output logic e1);
	e1    = (s == S_EA1);
	omemk = (s == S_EA0 && k[0]) || (s == S_EA1 && k[1]);
endfunction

logic a_pc0, a_pc1, a_base_in, a_fb_in, a_fx_in, a_e1pc1;
logic        a_mev, a_me1, a_agin;
logic [12:0] a_ia, a_ib;
logic  [1:0] a_mem;
logic [31:0] a_dsv;
logic        a_dov;
osel_t       a_oa, a_ob, a_od;
logic  [4:0] a_amt0, a_amt1;
logic [15:0] a_low1;
logic  [3:0] a_bi1;
always_comb begin
	logic ma, mb, md, ea, eb, ed, mv, e1, agin, c;
	logic [31:0] q, dsv;
	logic [2:0] mszf;
	logic [6:0] fps;
	logic [4:0] spr, an, xreg;
	logic pdm, pim;
	// the uop's memory EA (as the build below selects it)
	ma = omemk(ur.a, as.eak, ea);
	mb = omemk(ur.b, as.eak, eb);
	md = omemk(ur.d, as.eak, ed);
	mv = ma || mb || md;
	e1 = ma ? ea : mb ? eb : ed;
	if (ur.ag == AGM_EA0 || ur.ag == AGM_LEA0 || ur.ag == AGM_FEA) e1 = 1'b0;
	if (ur.ag == AGM_EA1) e1 = 1'b1;
	agin = mv || ur.ag == AGM_EA0 || ur.ag == AGM_EA1 || ur.ag == AGM_LEA0 ||
	       (ur.ag == AGM_FEA && as.ea0.m != EM_IMM);
	a_mev  = mv;
	a_me1  = e1;
	a_agin = agin;
	a_ia   = imm_cls(ur.a, as);
	if (a_ia == 13'd0) a_ia = 13'h0080;       // (A without one: the word's constant)
	a_ib   = imm_cls(ur.b, as);
	a_oa   = osel(ur.a, as, sp_reg, ssp);
	a_ob   = osel(ur.b, as, sp_reg, ssp);
	a_od   = osel(ur.d, as, sp_reg, ssp);
	if (ur.mem != 2'd0)
		a_mem = ur.mem;
	else if (ur.a == S_LD || ur.b == S_LD)
		a_mem = M_LD;
	else if (mv) begin
		if ((mb || ma) && md) a_mem = M_RMW;
		else if (md)          a_mem = M_ST;
		else                  a_mem = M_LD;
	end
	else a_mem = M_NONE;
	// an explicit AG mode's displacement, whole (a_dov: the mode sets
	// one): D2b takes it or the memory EA's, no arithmetic or mode decode
	// there
	q = (as.opw[11:9] == 3'd0) ? 32'd8 : {29'd0, as.opw[11:9]};
	case (ur.dsel)
		DS_IMM:    dsv = as.imm;
		DS_NIMM:   dsv = -((as.sz == SZ_W) ? {{16{as.ea0.bd[15]}}, as.ea0.bd[15:0]} : as.ea0.bd);
		DS_QUICK:  dsv = q;
		DS_NQUICK: dsv = neg5(q[4:0]);      // q: 1..8
		DS_IMMC:   dsv = as.imm + {{16{ur.cval[15]}}, ur.cval};
		DS_SZB:    dsv = {27'd0, size_bytes(as.sz, 1'b0)};
		default:   dsv = {{16{ur.cval[15]}}, ur.cval};
	endcase
	mszf = (ur.msz == 3'd0) ? {1'b0, as.sz} : (ur.msz - 3'd1);
	fps  = fpsz(as.ext1, as.ea0.r);
	// (An)+/-(An)'s step for either EA (a byte through A7 moves it by two)
	a_amt0 = size_bytes(mszf[1:0], as.ea0.r == 3'd7);
	a_amt1 = size_bytes(mszf[1:0], as.ea1.r == 3'd7);
	a_dov = 1'b0;
	a_dsv = dsv;
	case (ur.ag)
		AGM_PUSH: begin a_dov = 1'b1; a_dsv = neg5(size_bytes(mszf[1:0], 1'b1)); end
		AGM_BASED, AGM_BASEDU, AGM_ADDC: a_dov = 1'b1;
		AGM_VAL0, AGM_ADDV:
			if (as.ea0.m == EM_IMM) begin
				a_dov = 1'b1;
				a_dsv = (as.sz == SZ_W) ? {{16{as.ea0.bd[15]}}, as.ea0.bd[15:0]} : as.ea0.bd;
			end
		AGM_FEA:
			if (as.ea0.m == EM_AIP || as.ea0.m == EM_APD) begin
				a_dov = 1'b1;
				a_dsv = (as.ea0.m == EM_APD) ? neg7(fps) : 32'd0;
			end
			else if (as.ea0.m == EM_IMM) begin
				a_dov = 1'b1;
				a_dsv = as.ea0.bd + ((as.ext1[15:13] == 3'b010 && as.ext1[12:10] == 3'd6) ? 32'd1 : 32'd0);
			end
		AGM_FUPD:
			if (as.ea0.m == EM_AIP || as.ea0.m == EM_APD) begin
				a_dov = 1'b1;
				a_dsv = (as.ea0.m == EM_APD) ? neg7(fps) : {25'd0, fps};
			end
		default: ;
	endcase
	// MOVEM's first transfer: the mask's lowest bit
	a_low1 = as.ext1 & (~as.ext1 + 16'd1);
	a_bi1  = 4'd0;
	for (int i = 15; i >= 0; i--) if (as.ext1[i]) a_bi1 = 4'(i);
	c  = agin && (e1 ? as.eak[3] : as.eak[2]);
	a_pc0 = c && !e1;
	a_pc1 = c && e1;
	// MOVEM: a base or index register in a load list (see the loop below)
	spr = !smode_q ? R_USP : R_ISP;
	pdm = (as.ea0.m == EM_APD);
	pim = (as.ea0.m == EM_AIP);
	an  = areg(as.ea0.r, spr);
	xreg = as.ea0.xa ? areg(as.ea0.xr, spr) : dreg(as.ea0.xr);
	a_fb_in = (ur.d == S_MVR) && !pdm && !pim &&
	          (as.ea0.m == EM_AI || as.ea0.m == EM_AD16 || (as.ea0.m == EM_AX && !as.ea0.bs)) &&
	          as.ext1[{1'b1, as.ea0.r}];
	a_fx_in = (ur.d == S_MVR) && (as.ea0.m == EM_AX || as.ea0.m == EM_PCX) &&
	          !as.ea0.is && as.ext1[{as.ea0.xa, as.ea0.xr}] && !(a_fb_in && xreg == an);
	a_base_in = a_fb_in || a_fx_in || ((ur.d == S_MVR) && pim && as.ext1[{1'b1, as.ea0.r}]);
	a_e1pc1 = pc1(as.ext1);
end

// the next instruction's address, for S_NPC (D2a: an adder off the uop)
wire [31:0] a_npc = as.pc + {27'd0, as.ilen, 1'b0};

//--------------------------------------------------------------------------
// D2c and D2b flow.  Every word in D2c makes a uop when the uop FIFO has
// room; a word that repeats (pointer load, MOVEM) stays.  D2b's word moves
// on when D2c is free; D2b takes D2a's when it is empty or moving on.
//--------------------------------------------------------------------------
// (the back end's flush and exception entry are far registers: they only
// override, last -- the FIFOs empty, the stages and the state clear)
wire c_step = c_v && room_out;
wire emit   = c_step;
// (the combinational form, checked against c_stay_q below)
wire stay   = n_ptr || n_mv_pre || (c_loop && !nu.last);
logic c_last;                                              // the word's last bit
wire done   = emit && !c_stay_q && c_last;                 // instruction ends
wire c_free = !c_v || (c_step && !c_stay_q);
wire b_go   = b_v && c_free;                               // D2b's word to D2c
wire b_free = !b_v || c_free;                              // (registers only)
// D2c's uop sets the deferred update (the source EA's) this clock
wire def_ld = emit && !n_ptr && !n_mv_pre && !(c_loop && c_stay_q) && c_def_set;
logic        c_me1;             // the memory EA is EA1

//--------------------------------------------------------------------------
// D2a sequencing.  The word at upc is passed to D2b when D2b is free; a
// jump word is resolved here instead (no uop).  Its record is popped as
// its last word passes.
//--------------------------------------------------------------------------
wire a_jmp  = a_k.jmp;
wire pass   = a_v && !a_jmp && b_free;
// (one LUT from registers: D1's FIFO shifts on it)
assign rq_pop   = a_pl && b_free;

// a conditional jump's condition, on the record (a clock: a_cjv)
function automatic logic jc_eval(input logic [5:0] jc, input dinst_t s, input logic sm);
	case (jc)
		JC_ALWAYS:   jc_eval = 1'b1;
		JC_EXT10:    jc_eval = s.ext1[10];
		JC_EXT11:    jc_eval = s.ext1[11];
		JC_EA0_MEM:  jc_eval = is_mem(s.ea0.m);
		JC_EA0_DN:   jc_eval = (s.ea0.m == EM_DN);
		JC_EA0_IMM:  jc_eval = (s.ea0.m == EM_IMM);
		JC_SZ_L:     jc_eval = (s.sz == SZ_L);
		JC_AY7:      jc_eval = (s.opw[2:0] == 3'd7);
		JC_SUPER:    jc_eval = sm;
		JC_MASK0:    jc_eval = (s.ext1 == 16'd0);
		JC_X1A:      jc_eval = s.ext1[15];
		JC_SZ_B:     jc_eval = (s.sz == SZ_B);
		JC_EA0_AN:   jc_eval = (s.ea0.m == EM_AN);
		JC_EA0_APD:  jc_eval = (s.ea0.m == EM_APD);
		JC_EA0_AIP:  jc_eval = (s.ea0.m == EM_AIP);
		JC_EXT13:    jc_eval = s.ext1[13];
		JC_FP_OC0:   jc_eval = (s.ext1[15:13] == 3'b000);
		JC_FP_OC2:   jc_eval = (s.ext1[15:13] == 3'b010);
		JC_FP_OC3:   jc_eval = (s.ext1[15:13] == 3'b011);
		JC_FP_OC45:  jc_eval = (s.ext1[15:14] == 2'b10);
		JC_FP_OC67:  jc_eval = (s.ext1[15:14] == 2'b11);
		JC_FMT_1:    jc_eval = (s.ext1[12:10] == 3'd6);
		JC_FMT_2:    jc_eval = (s.ext1[12:10] == 3'd4);
		JC_FMT_8:    jc_eval = (s.ext1[12:10] == 3'd5);
		JC_FMT_12:   jc_eval = (s.ext1[12:10] == 3'd2 || s.ext1[12:10] == 3'd3);
		JC_FMT_7:    jc_eval = (s.ext1[12:10] == 3'd7);
		JC_CREG_RF:  jc_eval = (s.ext1[11:0] == 12'h800);    // USP
		default:     jc_eval = 1'b0;
	endcase
endfunction

logic  [9:0] na;          // next microcode address
logic        nv;
ukind_t      nk;          // ... its kind
logic        x_go;        // the back end's exception entry, a clock on
logic  [3:0] x_kind;
always_ff @(posedge clk) begin
	x_go   <= nrst && exc_go;
	x_kind <= exc_kind;
end
always_comb begin
	na = upc;
	nv = a_v;
	nk = a_k;
	if (a_v && a_jmp) begin
		// an unconditional jump at once, a conditional one when its
		// condition is in (a_cjt)
		if (a_k.jalw || a_cjv) begin
			if (a_k.jalw || a_cjt) begin na = a_k.jt; nk = ur.tk; end
			else begin na = upc1; nk = ur.nk; end
		end
	end
	else if (pass) begin
		if (!a_k.last) begin na = upc1; nk = ur.nk; end
		else if (!a_exc && rq_n >= 3'd2) begin na = rq1.rt; nk = rq1.k0; end   // next instruction
		else nv = 1'b0;
	end
	else if (!a_v && !a_exc && rq_n != 3'd0) begin
		na = rq0.rt;                                         // first word
		nk = rq0.k0;
		nv = 1'b1;
	end
	if (flush) nv = 1'b0;
	// the exception routine starts the clock after the back end's entry,
	// from its kind registered here (the entry, a far register, only
	// empties D2a in its own clock: it is not on the ROM's address)
	if (x_go && !flush) begin
		case (x_kind)
			4'd1:    begin na = UA_EXC_FMT2;  nk = UK_EXC_FMT2;  end
			4'd2:    begin na = UA_EXC_FMT4;  nk = UK_EXC_FMT4;  end
			4'd3:    begin na = UA_EXC_IRQ;   nk = UK_EXC_IRQ;   end
			4'd4:    begin na = UA_EXC_RESET; nk = UK_EXC_RESET; end
			default: begin na = UA_EXC_FMT0;  nk = UK_EXC_FMT0;  end
		endcase
		nv = 1'b1;
	end
	if (exc_go) nv = 1'b0;
end

// the microcode ROM in block RAM: its address register is the sequencing
// register (ur is the word at upc, read a clock after its address)
(* romstyle = "M10K" *) logic [$bits(uword_t)-1:0] urom [0:1023];
initial begin
`include "gen/ap68060_ucode_init.svh"
end
logic [$bits(uword_t)-1:0] uw_raw;
always_ff @(posedge clk) uw_raw <= urom[na];
assign ur = uword_t'(uw_raw);

always_ff @(posedge clk) begin
	if (!nrst) begin
		upc      <= '0;
		upc1     <= 10'd1;
		a_v      <= 1'b0;
		a_k      <= '0;
		a_exc    <= 1'b0;
		a_pl     <= 1'b0;
		a_cjv    <= 1'b0;
		a_cjt    <= 1'b0;
	end
	else begin
		logic xn;
		upc  <= na;
		upc1 <= na + 10'd1;
		a_v  <= nv;
		a_k  <= nk;
		a_cjv <= 1'b0;
		if (a_v && a_jmp && !a_k.jalw && !a_cjv) begin
			a_cjv <= 1'b1;
			a_cjt <= jc_eval(a_k.jc, as, smode_q);
		end
		// an exception routine ends with its last word
		xn = a_exc && !(pass && a_k.last);
		if (flush && !exc_go) xn = 1'b0;
		if (exc_go) xn = 1'b1;
		a_exc <= xn;
		a_pl  <= nv && nk.last && !nk.jmp && !xn;
	end
end

//--------------------------------------------------------------------------
// the stages' data and the uop FIFO's: no reset (the valid bits and the
// count qualify them)
//--------------------------------------------------------------------------
always_ff @(posedge clk) begin
	if (pass) begin
		ub       <= ur;
		rb       <= as;
		b_exr    <= a_exc;
		b_pc0    <= a_pc0;
		b_pc1    <= a_pc1;
		b_base_in <= a_base_in;
		b_fb_in  <= a_fb_in;
		b_fx_in  <= a_fx_in;
		b_e1pc1  <= a_e1pc1;
		b_mev    <= a_mev;
		b_me1    <= a_me1;
		b_agin   <= a_agin;
		b_ia     <= a_ia;
		b_ib     <= a_ib;
		b_e1pc2  <= pcle(as.ext1, 2'd2);
		b_e1pc3  <= pcle(as.ext1, 2'd3);
		b_mem    <= a_mem;
		b_dsv    <= a_dsv;
		b_dov    <= a_dov;
		b_oa     <= a_oa;
		b_ob     <= a_ob;
		b_od     <= a_od;
		b_amt0   <= a_amt0;
		b_amt1   <= a_amt1;
		b_low1   <= a_low1;
		b_bi1    <= a_bi1;
		b_npc    <= a_npc;
	end
	if (b_go) begin
		cu       <= nb;
		c_v1     <= nb_v1;
		c_pc0    <= b_pc0;
		c_pc1    <= b_pc1;
		c_loop   <= ub.loop;
		c_base_in <= b_base_in;
		c_fb_in  <= b_fb_in;
		c_fx_in  <= b_fx_in;
		c_e1pc1  <= b_e1pc1;
		c_upd2   <= ub.upd2;
		c_m16    <= (rb.opw[15:3] == 13'h1EC4);
		c_memi   <= bme.mi;
		c_me1    <= b_me1;
		c_mem_aip <= (bme.m == EM_AIP);
		c_aipd   <= (bme.m == EM_AIP || bme.m == EM_APD);
		c_mupd_v <= nb_mupd_v;
		c_mupd_reg <= nb_mupd_reg;
		c_bupd_x <= dbg_bupd;
		c_e1pc2  <= b_e1pc2;
		c_e1pc3  <= b_e1pc3;
		c_def_set <= nb_def_set;
		c_def_reg <= nb_def_reg;
		c_def_amt <= nb_def_amt;
		c_pd     <= (rb.ea0.m == EM_APD);
		c_pi     <= (rb.ea0.m == EM_AIP);
		c_mvcm   <= (rb.ea0.m == EM_AX || rb.ea0.m == EM_PC16 || rb.ea0.m == EM_PCX);
		c_mvld   <= (ub.d == S_MVR);
		c_sb     <= (rb.sz == SZ_L) ? 3'd4 : 3'd2;
		c_an     <= areg(rb.ea0.r, sp_reg);
		c_xreg   <= rb.ea0.xa ? areg(rb.ea0.xr, sp_reg) : dreg(rb.ea0.xr);
		c_last   <= ub.last;
		// (against the deferred update as it will be: one D2c is emitting
		// now lands with this clock)
		c_bok    <= nb.ag && nb.base_v;
		c_beq    <= nb.base == (def_ld ? c_def_reg : def_reg);
		c_mamt   <= nb.upd_amt + (def_ld ? c_def_amt : def_amt);
	end
	// MOVEM's transfer terms: the first transfer's as the word enters D2c,
	// the next one's as a transfer is emitted (the word stays: D2c takes
	// no word then).  The register numbers of An and the index in mask
	// order are one-to-one with the registers, so the transfer's register
	// is An exactly when its mask bit is An's.
	if (b_go) begin
		logic       pd, pi, isan, isx;
		logic [3:0] kan, kx;
		logic [4:0] r;
		logic [2:0] sb;
		pd   = (rb.ea0.m == EM_APD);
		pi   = (rb.ea0.m == EM_AIP);
		sb   = (rb.sz == SZ_L) ? 3'd4 : 3'd2;
		kan  = pd ? ~{1'b1, rb.ea0.r} : {1'b1, rb.ea0.r};
		kx   = {rb.ea0.xa, rb.ea0.xr};
		isan = (b_bi1 == kan);
		isx  = (b_bi1 == kx);
		r    = mvreg(b_bi1, pd, sp_reg);
		c_ban    <= popc16(rb.ext1 & ((16'd1 << kan) - 16'd1));
		c_bx     <= popc16(rb.ext1 & ((16'd1 << kx) - 16'd1));
		c_anin   <= rb.ext1[kan];
		c_xin    <= rb.ext1[kx];
		t_mask   <= rb.ext1;
		t_low    <= b_low1;
		t_rest   <= rb.ext1 & ~b_low1;
		t_r      <= r;
		t_isan   <= isan;
		{t_dreg, t_dv} <= mvdst(r, isan, isx, b_fb_in, b_fx_in, pd, pi);
		t_last   <= b_e1pc1;
		t_off    <= pd ? -{29'd0, sb} : 32'd0;
		t_amt    <= {5'd0, sb};
	end
	else if (emit && !n_ptr && !n_mv_pre && c_loop && c_stay_q) begin
		logic [15:0] lown;
		logic  [3:0] bin;
		logic        isan, isx;
		logic  [4:0] r;
		// (from t_rest, a register: no mask arithmetic ahead of the
		// encoder; An's and the index's by their counts)
		lown = t_rest & (~t_rest + 16'd1);
		bin  = enc16(t_rest);
		isan = (c_ban == 5'd1) && c_anin;
		isx  = (c_bx == 5'd1) && c_xin;
		r    = mvreg(bin, c_pd, sp_reg);
		c_ban  <= (c_ban != 5'd0) ? c_ban - 5'd1 : 5'd0;
		c_bx   <= (c_bx != 5'd0) ? c_bx - 5'd1 : 5'd0;
		t_mask <= t_rest;
		t_low  <= lown;
		t_rest <= t_rest & ~lown;
		t_r    <= r;
		t_isan <= isan;
		{t_dreg, t_dv} <= mvdst(r, isan, isx, c_fb_in, c_fx_in, c_pd, c_pi);
		t_last <= mv_act ? mv_pc2 : c_e1pc2;
		t_off  <= t_off + (c_pd ? -{29'd0, c_sb} : {29'd0, c_sb});
		t_amt  <= t_amt + {5'd0, c_sb};
	end
	// uop FIFO: D2c's uop at the write pointer; the head's register numbers
	// (the next head: the other word, or D2c's uop if it becomes the head)
	if (emit) ue[uq_wp] <= nu;
	begin
		uop_t nh;
		nh = (uq_n == 2'd2) ? (uq_rp ? ue[0] : ue[1]) : nu;
		if (uq_pop && uq_n != 2'd0) begin
			h_areg <= nh.a_reg; h_breg <= nh.b_reg; h_base <= nh.base; h_idx <= nh.idx;
		end
		else if (uq_n == 2'd0) begin
			h_areg <= nu.a_reg; h_breg <= nu.b_reg; h_base <= nu.base; h_idx <= nu.idx;
		end
	end
end

//--------------------------------------------------------------------------
// D2b and D2c registers, the instruction's state, the uop FIFO
//--------------------------------------------------------------------------
always_ff @(posedge clk) begin
	if (!nrst) begin
		b_v      <= 1'b0;
		c_v      <= 1'b0;
		c_stay_q <= 1'b0;

		first    <= 1'b1;
		ind0     <= 1'b0;
		ind1     <= 1'b0;
		def_v    <= 1'b0;
		def_reg  <= '0;
		def_amt  <= '0;
		ssp      <= R_ISP;
		mv_act   <= 1'b0;
		mv_is0   <= 1'b1;
		mv_pc1   <= 1'b1;
		mv_t11   <= 1'b0;
		mv_fix   <= 2'd0;
		uq_n     <= 2'd0;
		uq_rp    <= 1'b0;
		uq_wp    <= 1'b0;
	end
	else begin
		logic        i0n, i1n, actn, is0n, pc1n, t11n;
		logic  [1:0] fixn;
		// the per-instruction state after this clock's uop (the word stays)
		i0n  = ind0 || (emit && n_ptr && !c_pc1);
		i1n  = ind1 || (emit && n_ptr && c_pc1);
		actn = mv_act; is0n = mv_is0; pc1n = mv_pc1; t11n = mv_t11; fixn = mv_fix;
		if (emit && !n_ptr) begin
			if (n_mv_pre) t11n = 1'b1;
			else if (c_loop && c_stay_q) begin
				if (mv_act && mv_is0) fixn = mv_fix + 2'd1;
				actn = 1'b1;
				// (the mask less its lowest bit: none left when it had at
				// most one, at most one when it had at most two)
				is0n = mv_act ? mv_pc1 : c_e1pc1;
				pc1n = mv_act ? mv_pc2 : c_e1pc2;
			end
		end

		// D2b's word: a new one from D2a, or it stays (D2c busy), or none
		if (pass) b_v <= 1'b1;
		else if (b_go) b_v <= 1'b0;

		// D2c's word: D2b's, the same one, or none
		if (b_go) begin
			c_v      <= 1'b1;
			// (a MOVEM word starts its instruction's loop: the loop state is
			// clear; the pointers are the instruction's unless it ends now)
			c_stay_q <= stay_fn(ub.loop, b_pc0, b_pc1, ind0 && !done, ind1 && !done,
			                    b_base_in, b_fb_in, b_fx_in, 1'b0, 1'b0, 1'b0, 1'b0, 2'd0,
			                    b_e1pc1);
		end
		else if (c_v && !(c_step && !c_stay_q)) begin
			if (c_step)
				c_stay_q <= stay_fn(c_loop, c_pc0, c_pc1, i0n, i1n, c_base_in, c_fb_in,
				                    c_fx_in, actn, is0n, pc1n, t11n, fixn, c_e1pc1);
		end
		else begin
			c_v      <= 1'b0;
			c_stay_q <= 1'b0;
		end

		// uop FIFO: both outcomes formed without the pop (the back end's
		// stall, a late signal), which selects between them last
		if (uq_pop && uq_n != 2'd0) uq_n <= emit ? uq_n : uq_n - 2'd1;
		else                        uq_n <= emit ? uq_n + 2'd1 : uq_n;
		if (emit) uq_wp <= !uq_wp;
		if (uq_pop && uq_n != 2'd0) uq_rp <= !uq_rp;

		if (emit) begin
			first <= 1'b0;
			if (n_ptr) begin
				if (c_pc1) ind1 <= 1'b1; else ind0 <= 1'b1;
			end
			else if (n_mv_pre) begin
				mv_t11 <= 1'b1;
			end
			else if (c_loop && c_stay_q) begin
				if (mv_act && mv_is0) mv_fix <= mv_fix + 2'd1;
				mv_act  <= 1'b1;
				mv_is0  <= mv_act ? mv_pc1 : c_e1pc1;
				mv_pc1  <= mv_act ? mv_pc2 : c_e1pc2;
				mv_pc2  <= mv_act ? pcle(t_mask, 2'd3) : c_e1pc3;
			end
			else if (c_def_set) begin
				def_v   <= 1'b1;
				def_reg <= c_def_reg;
				def_amt <= c_def_amt;
			end
		end
		if (done) begin
			// the next instruction starts with clean per-instruction state
			first  <= 1'b1;
			ind0   <= 1'b0;
			ind1   <= 1'b0;
			def_v  <= 1'b0;
			mv_act <= 1'b0;
			mv_t11 <= 1'b0;
			mv_fix <= 2'd0;
		end

		// the back end's flush and exception entry: last
		if (flush || exc_go) begin
			b_v      <= 1'b0;
			c_v      <= 1'b0;
			c_stay_q <= 1'b0;
		end
		if (flush) begin
			first <= 1'b1;
			ind0  <= 1'b0;
			ind1  <= 1'b0;
			def_v <= 1'b0;
			mv_act <= 1'b0;
			mv_t11 <= 1'b0;
			mv_fix <= 2'd0;
			uq_n  <= 2'd0;
			uq_rp <= 1'b0;
			uq_wp <= 1'b0;
		end
		if (exc_go) begin
			first    <= 1'b1;
			ssp      <= exc_ssp;
			uq_n     <= 2'd0;
			uq_rp    <= 1'b0;
			uq_wp    <= 1'b0;
		end
	end
end

// synthesis translate_off
// (the record's mask, as the word entered D2c)
logic [15:0] dbg_ext1;
always_ff @(posedge clk) if (b_go) dbg_ext1 <= rb.ext1;
always_ff @(posedge clk)
	if (nrst && emit && (dbg_mvd && (dbg_dm || dbg_mvo[31:7] != {25{dbg_mvo[7]}}))) begin
		$display("FAIL: useq: a MOVEM offset %h for AG (merge %b)", dbg_mvo, dbg_dm);
		$finish;
	end
always_ff @(posedge clk)
	if (nrst && emit && dbg_dm && nu.disp != dbg_dpre) begin
		$display("FAIL: useq: a merged uop's displacement %h changed from %h", nu.disp, dbg_dpre);
		$finish;
	end
// the head's register numbers against the ring's word
always_ff @(posedge clk) begin
	uop_t h;
	h = uq_rp ? ue[1] : ue[0];
	if (nrst && uq_n != 2'd0 && {h_areg, h_breg, h_base, h_idx} != {h.a_reg, h.b_reg, h.base, h.idx}) begin
		$display("FAIL: useq: the uop FIFO head's register numbers");
		$finish;
	end
end
// the registered forms against the combinational ones
always_ff @(posedge clk)
	if (nrst && c_v && c_upd2 && def_v &&
	    (c_bok != (cu.ag && cu.base_v) || c_beq != (cu.base == def_reg) ||
	     c_mamt != cu.upd_amt + def_amt)) begin
		$display("FAIL: useq: the deferred update's merge terms");
		$finish;
	end
always_ff @(posedge clk)
	if (nrst && a_v && !a_exc && as.eak[3:0] != {(as.ea1.m == EM_AX || as.ea1.m == EM_PCX) && as.ea1.mi != 2'd0,
	                                       (as.ea0.m == EM_AX || as.ea0.m == EM_PCX) && as.ea0.mi != 2'd0,
	                                       !(as.ea1.m == EM_DN || as.ea1.m == EM_AN || as.ea1.m == EM_IMM),
	                                       !(as.ea0.m == EM_DN || as.ea0.m == EM_AN || as.ea0.m == EM_IMM)}) begin
		$display("FAIL: useq: the record's EA kinds %b", as.eak);
		$finish;
	end
// the explicit AG mode's displacement D2a formed, against D2b's word and
// record (BASED/ADDC: the selector's value itself is D2a's)
always_ff @(posedge clk)
	if (nrst && b_v) begin
		logic        ov, ck;
		logic [31:0] ovv;
		logic  [2:0] mf;
		mf  = (ub.msz == 3'd0) ? {1'b0, rb.sz} : (ub.msz - 3'd1);
		ov  = 1'b0;
		ck  = 1'b0;
		ovv = '0;
		case (ub.ag)
			AGM_PUSH: begin ov = 1'b1; ck = 1'b1; ovv = neg5(size_bytes(mf[1:0], 1'b1)); end
			AGM_BASED, AGM_BASEDU, AGM_ADDC: ov = 1'b1;
			AGM_VAL0, AGM_ADDV:
				if (rb.ea0.m == EM_IMM) begin
					ov = 1'b1; ck = 1'b1;
					ovv = (rb.sz == SZ_W) ? {{16{rb.ea0.bd[15]}}, rb.ea0.bd[15:0]} : rb.ea0.bd;
				end
			AGM_FEA:
				if (rb.ea0.m == EM_AIP || rb.ea0.m == EM_APD) begin
					ov = 1'b1; ck = 1'b1;
					ovv = (rb.ea0.m == EM_APD) ? neg7(fp_size) : 32'd0;
				end
				else if (rb.ea0.m == EM_IMM) begin
					ov = 1'b1; ck = 1'b1;
					ovv = rb.ea0.bd + ((rb.ext1[15:13] == 3'b010 && rb.ext1[12:10] == 3'd6) ? 32'd1 : 32'd0);
				end
			AGM_FUPD:
				if (rb.ea0.m == EM_AIP || rb.ea0.m == EM_APD) begin
					ov = 1'b1; ck = 1'b1;
					ovv = (rb.ea0.m == EM_APD) ? neg7(fp_size) : {25'd0, fp_size};
				end
			default: ;
		endcase
		if (ov != b_dov || (ck && ovv != b_dsv)) begin
			$display("FAIL: useq: an AG mode's displacement %b %h, D2b's %b %h", b_dov, b_dsv, ov, ovv);
			$finish;
		end
	end
always_ff @(posedge clk)
	if (nrst && a_pl != (a_v && a_k.last && !a_k.jmp && !a_exc)) begin
		$display("FAIL: useq: the pop term a_pl");
		$finish;
	end
always_ff @(posedge clk)
	if (nrst && c_v) begin
		if (c_stay_q != stay) begin
			$display("FAIL: useq: stay %b/%b ptr %b pre %b loop %b last %b", c_stay_q, stay,
			         n_ptr, n_mv_pre, c_loop, nu.last);
			$finish;
		end
		if (c_loop && ((mv_act && (mv_is0 != (t_mask == 16'd0) || mv_pc1 != pc1(t_mask) ||
		                           mv_pc2 != pcle(t_mask, 2'd2))) ||
		               (!mv_act && t_mask != dbg_ext1))) begin
			$display("FAIL: useq: the MOVEM mask terms");
			$finish;
		end
		// a transfer's terms against the one-clock forms from the mask
		if (c_loop && !(mv_act && mv_is0)) begin
			logic [4:0] r, nd;
			logic [5:0] dst;
			logic [7:0] sb;
			r   = mvreg(enc16(t_mask), c_pd, sp_reg);
			dst = mvdst(r, r == c_an, r == c_xreg, c_fb_in, c_fx_in, c_pd, c_pi);
			nd  = 5'd0;
			for (int i = 0; i < 16; i++) nd = nd + {4'd0, dbg_ext1[i] && !t_mask[i]};
			sb  = {5'd0, c_sb};
			if (t_low != (t_mask & (~t_mask + 16'd1)) || t_rest != (t_mask & ~t_low) ||
			    t_r != r || t_isan != (r == c_an) ||
			    {t_dreg, t_dv} != dst || t_last != (mv_act ? mv_pc1 : pc1(t_mask)) ||
			    (t_mask & ~dbg_ext1) != 16'd0 || t_amt != 8'(({3'd0, nd} + 8'd1) * sb) ||
			    t_off != (c_pd ? -32'(({3'd0, nd} + 8'd1) * sb) : 32'(nd * sb))) begin
				$display("FAIL: useq: the MOVEM transfer terms (mask %h r %0d/%0d dst %h/%h off %h amt %h)",
				         t_mask, t_r, r, {t_dreg, t_dv}, dst, t_off, t_amt);
				$finish;
			end
		end
		if (c_upd2 && (c_loop || cu.dmv)) begin
			$display("FAIL: useq: a deferred update's word with a MOVEM loop or offset");
			$finish;
		end
	end
// the immediates by class against the selectors', B's update read, the
// MOVEM count terms
always_ff @(posedge clk)
	if (nrst && b_v && dbg_med) begin
		$display("FAIL: useq: the memory EA's displacement by the record's kind at %0d", upc);
		$finish;
	end
always_ff @(posedge clk)
	if (nrst && b_v && !dbg_imm) begin
		$display("FAIL: useq: an operand's terms (its immediate by class, D2a's register) at %0d", upc);
		$finish;
	end
always_ff @(posedge clk)
	if (nrst && c_v && !n_ptr && !(c_loop && ((mv_act && mv_is0) || (c_base_in && !mv_t11))) &&
	    nu.b_upd != c_bupd_x) begin
		$display("FAIL: useq: B's update read %b, the one-clock form %b", nu.b_upd, c_bupd_x);
		$finish;
	end
// D2a's terms against the word and record they were formed from
always_ff @(posedge clk)
	if (nrst && b_v) begin
		logic ma, mb, md, ea, eb, ed, mv, e1;
		logic [1:0] mm;
		ma = omem(ub.a, rb, ea);
		mb = omem(ub.b, rb, eb);
		md = omem(ub.d, rb, ed);
		mv = ma || mb || md;
		e1 = ma ? ea : mb ? eb : ed;
		if (ub.ag == AGM_EA0 || ub.ag == AGM_LEA0 || ub.ag == AGM_FEA) e1 = 1'b0;
		if (ub.ag == AGM_EA1) e1 = 1'b1;
		mm = (ub.mem != 2'd0) ? ub.mem : (ub.a == S_LD || ub.b == S_LD) ? M_LD :
		     !mv ? M_NONE : ((mb || ma) && md) ? M_RMW : md ? M_ST : M_LD;
		if (b_mev != mv || b_me1 != e1 || b_mem != mm ||
		    b_agin != (mv || ub.ag == AGM_EA0 || ub.ag == AGM_EA1 || ub.ag == AGM_LEA0 ||
		               (ub.ag == AGM_FEA && rb.ea0.m != EM_IMM)) ||
		    b_low1 != (rb.ext1 & (~rb.ext1 + 16'd1)) || b_bi1 != enc16(rb.ext1) ||
		    b_npc != rb.pc + {27'd0, rb.ilen, 1'b0} ||
		    (ub.upd2 && !(ub.ag == AGM_NONE && !ub.loop))) begin
			$display("FAIL: useq: D2a's terms for the word at %0d", upc);
			$finish;
		end
	end
// the kind kept for the word being read is the word's
always_ff @(posedge clk)
	if (nrst && a_v && (a_k.last != ur.last || a_k.jmp != (ur.jc != JC_NEVER) ||
	                      a_k.jalw != (ur.jc == JC_ALWAYS) || a_k.jc != ur.jc ||
	                      (a_k.jmp && a_k.jt != ur.jt))) begin
		$display("FAIL: useq: the kind %h of the word at %0d", a_k, upc);
		$finish;
	end
always_ff @(posedge clk)
	if (nrst && a_v && !a_exc && rq_n == 3'd0) begin
		$display("FAIL: useq: a word without its record at %0d", upc);
		$finish;
	end
always_ff @(posedge clk)
	if (nrst && b_go && ub.loop && !done && (mv_act || mv_t11 || mv_fix != 2'd0)) begin
		$display("FAIL: useq: a MOVEM word with the loop state set");
		$finish;
	end
// synthesis translate_on

endmodule
