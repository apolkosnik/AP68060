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
//   * the back end's exception entry starts an exception routine           //
//                                                                          //
// Two stages:                                                              //
//   D2a  the microcode address (the ROM's address register).  The next    //
//        address comes from registers only: every word carries the kind   //
//        (last, jump, target) of the word after it and of its jump target,//
//        so the kind of the word being read is a register (a_k) and the   //
//        ROM's output only feeds registers.  A jump word is resolved here //
//        (an unconditional one at once, a conditional one a clock later). //
//   D2b  the uop, from the word and its record in registers.  A word that //
//        repeats (pointer load, MOVEM) holds D2a; whether the word will   //
//        repeat is itself a register (b_stay_q), formed a clock ahead.    //
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

//--------------------------------------------------------------------------
// D2a state: the word being read
//--------------------------------------------------------------------------
logic  [9:0] upc;               // its address (the ROM's address register's copy)
logic  [9:0] upc1;              // ... plus one
logic        a_v;               // it belongs to the current instruction
ukind_t      a_k;               // its kind
logic        a_exc;             // it is an exception routine's
logic        a_cjv, a_cjt;      // a conditional jump's condition, evaluated
uword_t      ur;                // the word (the ROM's output)

//--------------------------------------------------------------------------
// D2b state: the word a uop is built from
//--------------------------------------------------------------------------
logic        b_v;
uword_t      ub;                // the word
dinst_t      rb;                // its record ('0 for an exception routine)
logic        b_exr;             // an exception routine's
logic        b_stay_q;          // the word repeats after this clock's uop
// per word, formed in D2a: the pointer-load candidates (the EA the uop
// accesses is EA0 / EA1 and memory indirect), and the MOVEM terms
logic        b_pc0, b_pc1;
logic        b_base_in, b_fb_in, b_fx_in, b_e1pc1;
// ... the uop's memory EA (the first memory operand, or its AG mode's),
// whether it computes an address, its access; an explicit AG mode's
// displacement; the mask's lowest bit for MOVEM's first transfer
logic        b_mev, b_me1, b_agin;
ea_t         b_me;
logic  [1:0] b_mem;
logic [31:0] b_dsv;
logic [15:0] b_low1;
logic  [3:0] b_bi1;
logic        first;
logic        ind0, ind1;        // pointer of EA0 / EA1 already loaded
logic        def_v;             // a deferred source update is pending
logic  [4:0] def_reg;
logic  [7:0] def_amt;
logic  [4:0] ssp;
// MOVEM loop
logic        mv_act;            // the loop has started for this instruction
logic [15:0] mv_mask;           // registers still to transfer
logic        mv_is0, mv_pc1;    // ... none / at most one (formed with it)
logic [15:0] mv_low;            // ... its lowest bit, one-hot
logic  [3:0] mv_bi;             // ... and its number
logic [31:0] mv_off;            // address offset of the next transfer
logic  [7:0] mv_cnt;            // bytes transferred so far
logic        mv_t11;            // the address is in T11 (base register in the list)
logic  [1:0] mv_fix;            // fix-up uops emitted after the last transfer

// the instruction a uop is built for
dinst_t      src;
assign src = rb;
logic        src_first;
assign src_first = first;

uword_t      uw;
assign uw = ub;

// uop FIFO
uop_t        uq1;
wire         room_out = (uq_n != 2'd2);

// at most one bit set
function automatic logic pc1(input logic [15:0] m);
	pc1 = ((m & (m - 16'd1)) == 16'd0);
endfunction
// the lowest set bit's number
function automatic logic [3:0] enc16(input logic [15:0] m);
	enc16 = 4'd0;
	for (int i = 15; i >= 0; i--) if (m[i]) enc16 = 4'(i);
endfunction

logic        n_ptr_p;          // the pointer load (from the D2a candidates)
//--------------------------------------------------------------------------
// register naming
//--------------------------------------------------------------------------
wire [4:0] sp_reg = !smode ? R_USP : R_ISP;

function automatic logic [4:0] areg(input logic [2:0] n, input logic [4:0] spr);
	areg = (n == 3'd7) ? spr : (R_A0 + {2'b00, n});
endfunction

function automatic logic [4:0] dreg(input logic [2:0] n);
	dreg = {2'b00, n};
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

//--------------------------------------------------------------------------
// one uop from (uw, src)
//--------------------------------------------------------------------------
typedef struct packed {
	logic        v;          // operand present
	logic  [1:0] src;
	logic  [4:0] r;
	logic        imm_v;
	logic [31:0] imm;
	logic        mem;        // the operand is this uop's memory EA
	logic        ea1;        // ... and it is EA1
} opsel_t;

uop_t        nu;
logic        n_ptr;          // nu is a pointer load (memory indirect)
logic        n_ptr_ea1;
logic        n_jump;         // pure jump micro-instruction
logic  [9:0] n_jt;
logic        n_mv_pre;       // nu is the MOVEM address copy into T11
logic [15:0] n_mv_rest;      // mask after this transfer
logic        n_def_set;      // nu defers its source update
logic  [4:0] n_def_reg;
logic  [7:0] n_def_amt;

// FPU operand bytes for AG modes FEA / FUPD: opclass 010/011 by format (a
// byte through A7 moves it by two), FMOVE(M) of control registers four per
// register; the other FPU instructions move their address register by a
// run-time amount (ADDT0)
logic [6:0] fp_size;
always_comb begin
	logic [2:0] crl;
	crl = (src.ext1[12:10] == 3'd0) ? 3'b001 : src.ext1[12:10];
	case (src.ext1[15:13])
		3'b010, 3'b011: case (src.ext1[12:10])
			3'd0, 3'd1: fp_size = 7'd4;
			3'd4:       fp_size = 7'd2;
			3'd6:       fp_size = (src.ea0.r == 3'd7) ? 7'd2 : 7'd1;
			3'd5:       fp_size = 7'd8;
			default:    fp_size = 7'd12;
		endcase
		3'b100, 3'b101: fp_size = {3'd0, 2'(crl[0]) + 2'(crl[1]) + 2'(crl[2]), 2'b00};
		default: fp_size = 7'd0;
	endcase
end

logic        dbg_dm;             // (simulation: the merge and the
logic [31:0] dbg_dpre;           // displacement it was formed for;
logic        dbg_mvd;            // a MOVEM offset for AG, and the offset)
logic [31:0] dbg_mvo;
always_comb begin
	opsel_t oa, ob, od;
	ea_t    me;              // the uop's memory EA
	logic   me_v, me_ea1;
	logic [31:0] disp_pre;       // the uop's displacement before a deferred-update merge
	logic        dmerge;         // ... which applies (see the upd2 block)
	logic [1:0] osz, msz;
	logic [2:0] mszf;
	logic [4:0] amt;
	logic       me_sp;
	logic [31:0] q, shc;
	logic [6:0] op;
	logic [3:0] cnd;
	logic       jc_true;
	logic [31:0] dval;
	logic [31:0] pcx;

	nu        = '0;
	dbg_mvo   = '0;
	n_ptr     = 1'b0;
	n_ptr_ea1 = 1'b0;
	n_jump    = 1'b0;
	n_jt      = uw.jt;
	n_def_set = 1'b0;
	n_def_reg = '0;
	n_def_amt = '0;
	n_mv_pre  = 1'b0;
	n_mv_rest = '0;

	osz = (uw.sz  == 2'd0) ? src.sz : (uw.sz  - 2'd1);
	mszf = (uw.msz == 3'd0) ? {1'b0, src.sz} : (uw.msz - 3'd1);
	msz  = mszf[1:0];
	q   = (src.opw[11:9] == 3'd0) ? 32'd8 : {29'd0, src.opw[11:9]};

	// pure jump words (a routine's first jump is resolved in D1)
	case (uw.jc)
		JC_NEVER:    jc_true = 1'b0;
		JC_ALWAYS:   jc_true = 1'b1;
		JC_EXT10:    jc_true = src.ext1[10];
		JC_EXT11:    jc_true = src.ext1[11];
		JC_EA0_MEM:  jc_true = is_mem(src.ea0.m);
		JC_EA0_DN:   jc_true = (src.ea0.m == EM_DN);
		JC_EA0_IMM:  jc_true = (src.ea0.m == EM_IMM);
		JC_SZ_L:     jc_true = (src.sz == SZ_L);
		JC_AY7:      jc_true = (src.opw[2:0] == 3'd7);
		JC_SUPER:    jc_true = smode;
		JC_MASK0:    jc_true = (src.ext1 == 16'd0);
		JC_FP_OC0:   jc_true = (src.ext1[15:13] == 3'b000);
		JC_FP_OC2:   jc_true = (src.ext1[15:13] == 3'b010);
		JC_FP_OC3:   jc_true = (src.ext1[15:13] == 3'b011);
		JC_FP_OC45:  jc_true = (src.ext1[15:14] == 2'b10);
		JC_FP_OC67:  jc_true = (src.ext1[15:14] == 2'b11);
		JC_FMT_1:    jc_true = (src.ext1[12:10] == 3'd6);
		JC_FMT_2:    jc_true = (src.ext1[12:10] == 3'd4);
		JC_FMT_8:    jc_true = (src.ext1[12:10] == 3'd5);
		JC_FMT_12:   jc_true = (src.ext1[12:10] == 3'd2 || src.ext1[12:10] == 3'd3);
		JC_FMT_7:    jc_true = (src.ext1[12:10] == 3'd7);
		JC_EA0_APD:  jc_true = (src.ea0.m == EM_APD);
		JC_EA0_AIP:  jc_true = (src.ea0.m == EM_AIP);
		JC_EXT13:    jc_true = src.ext1[13];
		default:     jc_true = 1'b0;
	endcase
	// a micro-instruction with a jump condition is a pure jump
	n_jump = (uw.jc != JC_NEVER);

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
			S_NPC:   begin o.src = OS_IMM; o.imm_v = 1'b1; o.imm = src.pc + {27'd0, src.ilen, 1'b0}; end
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
	nu.a_src  = oa.v ? oa.src : OS_ZERO;
	nu.a_reg  = oa.r;
	nu.a_sxw  = (uw.sxw == 2'd1) || (uw.sxw == 2'd2 && src.sz == SZ_W);
	nu.b_src  = ob.v ? ob.src : OS_ZERO;
	nu.b_reg  = ob.r;
	nu.imm    = oa.imm_v ? oa.imm : {{16{uw.cval[15]}}, uw.cval};
	nu.imm_b  = ob.imm;
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
	nu.first  = src_first;
	nu.last   = uw.last;
	nu.exc    = (src.exc != 8'd0) ? src.exc : 8'd0;

	// destination
	if (od.v && od.src == OS_REG) begin
		nu.d_v   = 1'b1;
		nu.d_reg = od.r;
	end

	//----------------------------------------------------------------------
	// memory access and address generation
	//----------------------------------------------------------------------
	// (the uop's memory EA, its access and whether it computes an
	// address: formed in D2a from the word and the record, b_me*)
	me_v   = b_mev;
	me_ea1 = b_me1;
	me     = b_me;
	me_sp  = (me.r == 3'd7);
	amt    = size_bytes(msz, me_sp && (me.m == EM_AIP || me.m == EM_APD));
	nu.mem = b_mem;

	if (b_agin) begin
		logic [4:0] bre;
		logic       mi_stage1;
		nu.ag = 1'b1;
		bre   = areg(me.r, sp_reg);
		mi_stage1 = (me.mi != 2'd0) && !(me_ea1 ? ind1 : ind0);
		case (me.m)
			EM_AI: begin nu.base_v = 1'b1; nu.base = bre; end
			EM_AIP: begin
				nu.base_v = 1'b1; nu.base = bre;
				nu.pinc = 1'b1; nu.upd_v = 1'b1; nu.upd_reg = bre;
				nu.upd_amt = {3'd0, amt};
			end
			EM_APD: begin
				nu.base_v = 1'b1; nu.base = bre;
				nu.disp = neg5(amt);
				nu.upd_v = 1'b1; nu.upd_reg = bre;
			end
			EM_AD16: begin nu.base_v = 1'b1; nu.base = bre; nu.disp = me.bd; end
			EM_ABSW, EM_ABSL: nu.disp = me.bd;
			EM_PC16: begin nu.disp = me.bd; nu.mprog = 1'b1; end   // D1 added the PC
			EM_AX, EM_PCX: begin
				logic [4:0] xr;
				xr = me.xa ? areg(me.xr, sp_reg) : dreg(me.xr);
				if (me.mi == 2'd0 || mi_stage1) begin
					// address (or the pointer address) from base/index/bd
					if (me.m == EM_AX) begin nu.base_v = !me.bs; nu.base = bre; nu.disp = me.bd; end
					else begin nu.disp = me.bd; nu.mprog = !me.bs; end   // D1 added the PC
					nu.idx_v = !me.is && (me.mi != 2'd2);
					nu.idx   = xr;
					nu.idx_l = me.xl;
					nu.scale = me.sc;
				end
				else begin
					// through the pointer: T13 (EA0) / T12 (EA1) + index + od
					nu.base_v = 1'b1;
					nu.base   = me_ea1 ? (R_T0 + 5'd12) : (R_T0 + 5'd13);
					nu.disp   = me.od;
					nu.idx_v  = !me.is && (me.mi == 2'd2);
					nu.idx    = xr;
					nu.idx_l  = me.xl;
					nu.scale  = me.sc;
					nu.mprog  = 1'b0;
				end
				if (mi_stage1) begin
					// this cycle: the pointer load only
					n_ptr     = 1'b1;
					n_ptr_ea1 = me_ea1;
				end
			end
			default: ;
		endcase

		// deferral of the source update into a later uop
		if (uw.noupd && !me_ea1 && (me.m == EM_AIP || me.m == EM_APD)) begin
			logic xuse;
			// an EA1 index naming the same register cannot see a deferred
			// value: update in place then
			xuse = (src.ea1.m == EM_AX || src.ea1.m == EM_PCX) && !src.ea1.is &&
			       src.ea1.xa && (areg(src.ea1.xr, sp_reg) == bre);
			if (!xuse) begin
				nu.upd_v  = 1'b0;
				nu.pinc   = 1'b0;
				n_def_set = 1'b1;
				n_def_reg = bre;
				n_def_amt = (me.m == EM_AIP) ? {3'd0, amt} : -{3'd0, amt};
			end
		end
	end
	// ADDA.L (A0)+,A0 / CMPA.L -(A0),A0: the destination register is read
	// after the source EA's update (the 68000 family computes the EA first)
	nu.b_upd = nu.upd_v && !me_ea1 && (me.m == EM_AIP || me.m == EM_APD) &&
	           nu.b_src == OS_REG && nu.b_reg == nu.upd_reg;
	// (the 68060 MOVES Rn,(An)+/-(An) stores the original Rn, as MOVE does --
	// WinUAE gencpu; the 68040 computed the EA first)
	nu.a_upd = 1'b0;
	// the deferred update lands on the upd2 uop, memory or not (FRESTORE
	// commits (An)+ only after its frame check).  Its displacement merge is
	// applied after the MOVEM block (no MOVEM uop has upd2), so the two
	// additions are parallel rather than chained.
	disp_pre = nu.disp;
	dmerge   = 1'b0;
	if (uw.upd2 && def_v) begin
		logic same, m16;
		same = nu.ag && nu.base_v && nu.base == def_reg &&
		       (me.mi == 2'd0 || !(me_ea1 ? ind1 : ind0));
		// MOVE16 (Ax)+,(Ay)+ with Ax = Ay: both transfers use the original
		// address and the register advances once (WinUAE, the cputest
		// oracle); CMPM-style pairs see the first update
		m16 = (src.opw[15:3] == 13'h1EC4);
		if (same && m16) begin
			// the destination's own (An)+ is the only update
		end
		else if (same) begin
			dmerge = 1'b1;
			if (me.m == EM_AIP) nu.upd_amt = nu.upd_amt + def_amt;
		end
		if (!(same && (me.m == EM_AIP || me.m == EM_APD || m16))) begin
			nu.upd2_v   = 1'b1;
			nu.upd2_reg = def_reg;
			nu.upd2_amt = def_amt;
		end
	end

	//----------------------------------------------------------------------
	// explicit AG modes
	//----------------------------------------------------------------------
	begin
		logic [4:0]  rb, rw;
		logic        rb_v;
		logic [31:0] dsv;
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
		dsv = b_dsv;                    // (D2a's: its sums and negations)
		case (uw.ag)
			AGM_PUSH: begin
				nu.ag = 1'b1; nu.base_v = 1'b1; nu.base = sp_reg;
				nu.disp = neg5(size_bytes(msz, 1'b1));
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
				nu.ag = 1'b1; nu.base_v = rb_v; nu.base = rb; nu.disp = dsv;
				if (uw.ag == AGM_BASEDU) begin
					nu.upd_v = 1'b1;
					nu.upd_reg = (uw.agw != S_NONE) ? rw : rb;
				end
				else if (uw.agw != S_NONE) begin
					nu.upd_v = 1'b1; nu.upd_reg = rw;
				end
			end
			AGM_ADDC: begin
				nu.ag = 1'b1; nu.base_v = rb_v; nu.base = rb; nu.disp = dsv;
				nu.upd_v = 1'b1; nu.upd_reg = rw;
			end
			AGM_VAL0, AGM_ADDV: begin
				nu.ag = 1'b1;
				nu.base_v = (uw.ag == AGM_ADDV); nu.base = rb;
				if (src.ea0.m == EM_IMM)
					nu.disp = (src.sz == SZ_W) ? {{16{src.ea0.bd[15]}}, src.ea0.bd[15:0]} : src.ea0.bd;
				else begin
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
				if (uw.agw != S_NONE) begin nu.upd_v = !n_ptr; nu.upd_reg = rw; end
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
					nu.disp = (src.ea0.m == EM_APD) ? neg7(fp_size) : 32'd0;
				end
				else if (src.ea0.m == EM_IMM) begin
					nu.ag = 1'b1; nu.base_v = 1'b0; nu.idx_v = 1'b0; nu.pinc = 1'b0;
					nu.disp = src.ea0.bd + ((src.ext1[15:13] == 3'b010 &&
					                         src.ext1[12:10] == 3'd6) ? 32'd1 : 32'd0);
					nu.mprog = 1'b1;
				end
				nu.upd_v = !n_ptr; nu.upd_reg = rw;
			end
			AGM_FUPD: begin
				if (src.ea0.m == EM_AIP || src.ea0.m == EM_APD) begin
					nu.ag = 1'b1; nu.base_v = 1'b1; nu.base = areg(src.ea0.r, sp_reg);
					nu.idx_v = 1'b0; nu.pinc = 1'b0;
					nu.disp = (src.ea0.m == EM_APD) ? neg7(fp_size) : {25'd0, fp_size};
					nu.upd_v = 1'b1; nu.upd_reg = areg(src.ea0.r, sp_reg);
				end
			end
			default: ;
		endcase
	end

	//----------------------------------------------------------------------
	// MOVEM: one transfer per set mask bit.  -(An): mask bit 0 is A7 and the
	// addresses descend from An; other modes ascend from the EA.  An is
	// updated by the last transfer only.
	//----------------------------------------------------------------------
	if (uw.loop) begin
		logic [15:0] m;
		logic  [3:0] bi, rn;
		logic  [4:0] r;
		logic  [2:0] sb;
		logic        pd, pi, lastx, base_in, fb_in, fx_in;
		logic  [4:0] an, xreg;
		logic [31:0] off;
		logic [15:0] low;
		logic  [3:0] k_an, k_x;
		logic        is_an, is_x;
		m  = mv_act ? mv_mask : src.ext1;
		pd = (src.ea0.m == EM_APD);
		pi = (src.ea0.m == EM_AIP);
		// the mask's lowest set bit, one-hot and its number, kept with the
		// mask (the first transfer's formed in D2a): no encoder here
		bi = mv_act ? mv_bi : b_bi1;
		rn = pd ? (4'd15 - bi) : bi;
		r  = rn[3] ? areg(rn[2:0], sp_reg) : dreg(rn[2:0]);
		low   = mv_act ? mv_low : b_low1;
		n_mv_rest = m & ~low;
		lastx = mv_act ? mv_pc1 : b_e1pc1;
		sb = (src.sz == SZ_L) ? 3'd4 : 3'd2;
		an = areg(src.ea0.r, sp_reg);
		// whether this transfer's register is An or the index register,
		// from the mask's lowest set bit directly (not through the encoder
		// and the register number: a timing path); the register numbers
		// are one-to-one, so r == an exactly when rn is {1, An}
		k_an  = pd ? (4'd15 - {1'b1, src.ea0.r}) : {1'b1, src.ea0.r};
		k_x   = {src.ea0.xa, src.ea0.xr};
		is_an = low[k_an];
		is_x  = low[k_x];
		// A load list that contains a register of the EA (the base, or the
		// index of an indexed mode) works from a copy of the address in
		// T11, and the loaded base/index commits only after the last
		// transfer (through T7/T6 and fix-up uops): a fault on a later
		// transfer restarts the instruction with the original EA.  (An)+
		// discards a loaded An: the final An is the incremented address.
		fb_in   = (uw.d == S_MVR) && !pd && !pi &&
		          (src.ea0.m == EM_AI || src.ea0.m == EM_AD16 ||
		           (src.ea0.m == EM_AX && !src.ea0.bs)) &&
		          src.ext1[{1'b1, src.ea0.r}];
		xreg    = src.ea0.xa ? areg(src.ea0.xr, sp_reg) : dreg(src.ea0.xr);
		fx_in   = (uw.d == S_MVR) && (src.ea0.m == EM_AX || src.ea0.m == EM_PCX) &&
		          !src.ea0.is && src.ext1[{src.ea0.xa, src.ea0.xr}] &&
		          !(fb_in && xreg == an);
		base_in = fb_in || fx_in ||
		          ((uw.d == S_MVR) && pi && src.ext1[{1'b1, src.ea0.r}]);
		off = mv_act ? mv_off : (pd ? -{29'd0, sb} : 32'd0);
		nu.last = lastx && !fb_in && !fx_in;
		if (uw.d == S_MVR) begin
			nu.d_v = 1'b1; nu.d_reg = r;
			if (fb_in && is_an)           nu.d_reg = R_T0 + 5'd7;
			else if (fx_in && !pd && is_x) nu.d_reg = R_T0 + 5'd6;
			else if (pi && is_an)         nu.d_v   = 1'b0;
		end
		else begin
			nu.a_src = OS_REG; nu.a_reg = r;
			// -(An) storing An itself writes An - size (68020 and later)
			if (pd && is_an) begin
				nu.op = OP_SUB; nu.sz = SZ_L;
				nu.a_src = OS_IMM; nu.imm = {29'd0, sb};
				nu.b_src = OS_REG; nu.b_reg = an;
			end
		end
		if (mv_act && mv_mask == 16'd0) begin
			// after the last transfer: commit the deferred base, then index
			uop_t p;
			logic fxb;
			fxb = fb_in && mv_fix == 2'd0;
			p = '0;
			p.pc = nu.pc; p.ilen = nu.ilen; p.exr = nu.exr; p.first = 1'b0;
			p.op = OP_MOV; p.sz = SZ_L; p.ccr_we = 5'd0;
			p.a_src = OS_REG; p.a_reg = fxb ? (R_T0 + 5'd7) : (R_T0 + 5'd6);
			p.b_src = OS_ZERO;
			p.d_v = 1'b1; p.d_reg = fxb ? an : xreg;
			p.last = !(fxb && fx_in);
			p.exc = nu.exc;
			nu = p;
			n_mv_rest = 16'd0;
		end
		else if (base_in && !mv_t11) begin
			// first: T11 = the address, then the transfers from T11
			uop_t p;
			p = '0;
			p.pc = nu.pc; p.ilen = nu.ilen; p.exr = nu.exr; p.first = nu.first;
			p.ag = 1'b1;
			p.base_v = nu.base_v; p.base = nu.base;
			p.idx_v = nu.idx_v; p.idx = nu.idx; p.idx_l = nu.idx_l; p.scale = nu.scale;
			p.disp = nu.disp; p.mprog = nu.mprog;
			p.upd_v = 1'b1; p.upd_reg = R_T0 + 5'd11;
			p.exc = nu.exc;
			nu = p;
			n_mv_pre = 1'b1;
		end
		else begin
			if (mv_t11) begin
				nu.base_v = 1'b1; nu.base = R_T0 + 5'd11;
				nu.idx_v = 1'b0; nu.disp = 32'd0;
			end
			// MOVEM continuation (SSW CM): for the indexed and PC-relative
			// modes a transfer carries its offset from the calculated EA;
			// a fault stacks that EA, and RTE resumes from it
			if (src.ea0.m == EM_AX || src.ea0.m == EM_PC16 || src.ea0.m == EM_PCX) begin
				nu.cond   = 4'hC;
				nu.target = off;
			end
			// address of this transfer: the offset (within +-64) is added
			// by AG as the uop enters it, as a deferred update's is (off
			// the sequencer's EA selects)
			if (pd) nu.disp = off;          // undo the generic -(An) offset
			else begin
				nu.dmv = 1'b1;
				nu.dma = off[7:0];
				dbg_mvo = off;
			end
			nu.pinc = 1'b0;
			nu.upd_v = 1'b0;
			if (lastx && pd) begin
				nu.upd_v = 1'b1; nu.upd_reg = an;          // An = lowest address
			end
			if (lastx && pi) begin
				nu.pinc = 1'b1; nu.upd_v = 1'b1; nu.upd_reg = an;
				nu.upd_amt = (mv_act ? mv_cnt : 8'd0) + {5'd0, sb};
			end
		end
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

	// (the addition itself is AG's, as the uop enters it: off the
	// microcode ROM's path to the uop queue.  No upd2 uop has an explicit
	// AG mode, so its displacement is still disp_pre -- checked below)
	dbg_dm   = dmerge;
	dbg_dpre = disp_pre;
	dbg_mvd  = nu.dmv;              // (MOVEM's offset, before a merge)
	if (dmerge) begin
		nu.dmv  = 1'b1;
		nu.dma  = def_amt;
	end

	// a pointer load replaces the uop this cycle (the condition from the
	// candidates D2a formed: the EA selection is not on its path)
	if (n_ptr_p) begin
		uop_t p;
		p = '0;
		p.pc = nu.pc; p.ilen = nu.ilen; p.exr = nu.exr; p.first = nu.first; p.last = 1'b0;
		p.op = OP_MOV; p.sz = SZ_L; p.msz = SZ_L; p.ccr_we = 5'd0;
		p.a_src = OS_MEM; p.b_src = OS_ZERO;
		p.ag = 1'b1;
		p.base_v = nu.base_v; p.base = nu.base;
		p.idx_v = nu.idx_v; p.idx = nu.idx; p.idx_l = nu.idx_l; p.scale = nu.scale;
		p.disp = nu.disp; p.dmv = nu.dmv; p.dma = nu.dma;
		p.mem = M_LD; p.mprog = nu.mprog;
		p.d_v = 1'b1;
		p.d_reg = n_ptr_ea1 ? (R_T0 + 5'd12) : (R_T0 + 5'd13);
		p.exc = nu.exc;
		nu = p;
		n_def_set = 1'b0;
	end
end
//--------------------------------------------------------------------------
// D2b sequencing.  Every word in D2b makes a uop when the uop FIFO has
// room; a word that repeats (pointer load, MOVEM) stays.
//--------------------------------------------------------------------------
wire b_step = b_v && room_out && !flush && !exc_go;
wire emit   = b_step;
assign n_ptr_p = (b_pc0 && !ind0) || (b_pc1 && !ind1);
// (the combinational form, checked against b_stay_q below)
wire stay   = n_ptr || n_mv_pre || (uw.loop && !nu.last);
wire done   = emit && !b_stay_q && uw.last;                 // instruction ends
// D2b takes a word from D2a this clock (registers only)
wire b_free = !b_v || (b_step && !b_stay_q);

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

logic a_pc0, a_pc1, a_base_in, a_fb_in, a_fx_in, a_e1pc1;
logic        a_mev, a_me1, a_agin;
ea_t         a_me;
logic  [1:0] a_mem;
logic [31:0] a_dsv;
logic [15:0] a_low1;
logic  [3:0] a_bi1;
always_comb begin
	logic ma, mb, md, ea, eb, ed, mv, e1, agin, c;
	logic [31:0] q;
	logic [3:0] m;
	logic [1:0] mi;
	logic [4:0] spr, an, xreg;
	logic pdm, pim;
	// the uop's memory EA (as the build below selects it)
	ma = omem(ur.a, as, ea);
	mb = omem(ur.b, as, eb);
	md = omem(ur.d, as, ed);
	mv = ma || mb || md;
	e1 = ma ? ea : mb ? eb : ed;
	if (ur.ag == AGM_EA0 || ur.ag == AGM_LEA0 || ur.ag == AGM_FEA) e1 = 1'b0;
	if (ur.ag == AGM_EA1) e1 = 1'b1;
	agin = mv || ur.ag == AGM_EA0 || ur.ag == AGM_EA1 || ur.ag == AGM_LEA0 ||
	       (ur.ag == AGM_FEA && as.ea0.m != EM_IMM);
	a_mev  = mv;
	a_me1  = e1;
	a_agin = agin;
	a_me   = e1 ? as.ea1 : as.ea0;
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
	// an explicit AG mode's displacement
	q = (as.opw[11:9] == 3'd0) ? 32'd8 : {29'd0, as.opw[11:9]};
	case (ur.dsel)
		DS_IMM:    a_dsv = as.imm;
		DS_NIMM:   a_dsv = -((as.sz == SZ_W) ? {{16{as.ea0.bd[15]}}, as.ea0.bd[15:0]} : as.ea0.bd);
		DS_QUICK:  a_dsv = q;
		DS_NQUICK: a_dsv = neg5(q[4:0]);      // q: 1..8
		DS_IMMC:   a_dsv = as.imm + {{16{ur.cval[15]}}, ur.cval};
		DS_SZB:    a_dsv = {27'd0, size_bytes(as.sz, 1'b0)};
		default:   a_dsv = {{16{ur.cval[15]}}, ur.cval};
	endcase
	// MOVEM's first transfer: the mask's lowest bit
	a_low1 = as.ext1 & (~as.ext1 + 16'd1);
	a_bi1  = 4'd0;
	for (int i = 15; i >= 0; i--) if (as.ext1[i]) a_bi1 = 4'(i);
	m  = e1 ? as.ea1.m : as.ea0.m;
	mi = e1 ? as.ea1.mi : as.ea0.mi;
	c  = agin && (m == EM_AX || m == EM_PCX) && mi != 2'd0;
	a_pc0 = c && !e1;
	a_pc1 = c && e1;
	// MOVEM: a base or index register in a load list (see the loop below)
	spr = !smode ? R_USP : R_ISP;
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

//--------------------------------------------------------------------------
// D2a sequencing.  The word at upc is passed to D2b when D2b is free; a
// jump word is resolved here instead (no uop).  Its record is popped as
// its last word passes.
//--------------------------------------------------------------------------
wire a_jmp  = a_k.jmp;
wire pass   = a_v && !a_jmp && b_free && !flush && !exc_go;
assign rq_pop   = pass && a_k.last && !a_exc;

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
always_comb begin
	na = upc;
	nv = a_v;
	nk = a_k;
	if (exc_go) begin
		case (exc_kind)
			4'd1:    begin na = UA_EXC_FMT2;  nk = UK_EXC_FMT2;  end
			4'd2:    begin na = UA_EXC_FMT4;  nk = UK_EXC_FMT4;  end
			4'd3:    begin na = UA_EXC_IRQ;   nk = UK_EXC_IRQ;   end
			4'd4:    begin na = UA_EXC_RESET; nk = UK_EXC_RESET; end
			default: begin na = UA_EXC_FMT0;  nk = UK_EXC_FMT0;  end
		endcase
		nv = 1'b1;
	end
	else if (flush) begin
		nv = 1'b0;
	end
	else if (a_v && a_jmp) begin
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
	if (!nreset) begin
		upc      <= '0;
		upc1     <= 10'd1;
		a_v      <= 1'b0;
		a_k      <= '0;
		a_exc    <= 1'b0;
		a_cjv    <= 1'b0;
		a_cjt    <= 1'b0;
	end
	else begin
		upc  <= na;
		upc1 <= na + 10'd1;
		a_v  <= nv;
		a_k  <= nk;
		a_cjv <= 1'b0;
		if (!flush && !exc_go && a_v && a_jmp && !a_k.jalw && !a_cjv) begin
			a_cjv <= 1'b1;
			a_cjt <= jc_eval(a_k.jc, as, smode);
		end
		// an exception routine ends with its last word
		if (pass && a_k.last && a_exc) a_exc <= 1'b0;
		if (flush && !exc_go) a_exc <= 1'b0;
		if (exc_go) a_exc <= 1'b1;
	end
end

//--------------------------------------------------------------------------
// D2b registers
//--------------------------------------------------------------------------
always_ff @(posedge clk) begin
	if (!nreset) begin
		b_v      <= 1'b0;
		ub       <= '0;
		rb       <= '0;
		b_exr    <= 1'b0;
		b_stay_q <= 1'b0;
		b_pc0    <= 1'b0;
		b_pc1    <= 1'b0;
		b_base_in <= 1'b0;
		b_fb_in  <= 1'b0;
		b_fx_in  <= 1'b0;
		b_e1pc1  <= 1'b0;
		b_mev    <= 1'b0;
		b_me1    <= 1'b0;
		b_agin   <= 1'b0;
		b_me     <= '0;
		b_mem    <= '0;
		b_dsv    <= '0;
		b_low1   <= '0;
		b_bi1    <= '0;
		first    <= 1'b1;
		ind0     <= 1'b0;
		ind1     <= 1'b0;
		def_v    <= 1'b0;
		def_reg  <= '0;
		def_amt  <= '0;
		ssp      <= R_ISP;
		mv_act   <= 1'b0;
		mv_mask  <= '0;
		mv_is0   <= 1'b1;
		mv_pc1   <= 1'b1;
		mv_low   <= '0;
		mv_bi    <= '0;
		mv_off   <= '0;
		mv_cnt   <= '0;
		mv_t11   <= 1'b0;
		mv_fix   <= 2'd0;
		uq_n     <= 2'd0;
		uq0      <= '0;
		uq1      <= '0;
	end
	else begin
		logic        i0n, i1n, actn, is0n, pc1n, t11n;
		logic  [1:0] fixn;
		// the per-instruction state after this clock's uop (the word stays)
		i0n  = ind0 || (emit && n_ptr_p && !b_pc1);
		i1n  = ind1 || (emit && n_ptr_p && b_pc1);
		actn = mv_act; is0n = mv_is0; pc1n = mv_pc1; t11n = mv_t11; fixn = mv_fix;
		if (emit && !n_ptr_p) begin
			if (n_mv_pre) t11n = 1'b1;
			else if (uw.loop && b_stay_q) begin
				if (mv_act && mv_mask == 16'd0) fixn = mv_fix + 2'd1;
				actn = 1'b1;
				is0n = (n_mv_rest == 16'd0);
				pc1n = pc1(n_mv_rest);
			end
		end

		// D2b's word: a new one from D2a, the same one, or none
		if (flush || exc_go) begin
			b_v      <= 1'b0;
			b_stay_q <= 1'b0;
		end
		else if (pass) begin
			b_v      <= 1'b1;
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
			b_me     <= a_me;
			b_mem    <= a_mem;
			b_dsv    <= a_dsv;
			b_low1   <= a_low1;
			b_bi1    <= a_bi1;
			// (a MOVEM word starts its instruction's loop: the loop state is
			// clear; the pointers are the instruction's unless it ends now)
			b_stay_q <= stay_fn(ur.loop, a_pc0, a_pc1, ind0 && !done, ind1 && !done,
			                    a_base_in, a_fb_in, a_fx_in, 1'b0, 1'b0, 1'b0, 1'b0, 2'd0,
			                    a_e1pc1);
		end
		else if (b_v && !(b_step && !b_stay_q)) begin
			if (b_step)
				b_stay_q <= stay_fn(uw.loop, b_pc0, b_pc1, i0n, i1n, b_base_in, b_fb_in,
				                    b_fx_in, actn, is0n, pc1n, t11n, fixn, b_e1pc1);
		end
		else begin
			b_v      <= 1'b0;
			b_stay_q <= 1'b0;
		end

		// uop FIFO: both outcomes formed without the pop (the back end's
		// stall, a late signal), which selects between them last
		if (uq_pop && uq_n != 2'd0) begin
			uq0  <= (emit && uq_n == 2'd1) ? nu : uq1;
			uq1  <= (emit && uq_n == 2'd2) ? nu : uq1;
			uq_n <= emit ? uq_n : uq_n - 2'd1;
		end
		else begin
			uq0  <= (emit && uq_n == 2'd0) ? nu : uq0;
			uq1  <= (emit && uq_n == 2'd1) ? nu : uq1;
			uq_n <= emit ? uq_n + 2'd1 : uq_n;
		end

		if (emit) begin
			first <= 1'b0;
			if (n_ptr_p) begin
				if (b_pc1) ind1 <= 1'b1; else ind0 <= 1'b1;
			end
			else if (n_mv_pre) begin
				mv_t11 <= 1'b1;
			end
			else if (uw.loop && b_stay_q) begin
				if (mv_act && mv_mask == 16'd0) mv_fix <= mv_fix + 2'd1;
				mv_act  <= 1'b1;
				mv_mask <= n_mv_rest;
				mv_is0  <= (n_mv_rest == 16'd0);
				mv_pc1  <= pc1(n_mv_rest);
				mv_low  <= n_mv_rest & (~n_mv_rest + 16'd1);
				mv_bi   <= enc16(n_mv_rest);
				mv_off  <= (mv_act ? mv_off : ((src.ea0.m == EM_APD) ?
				            -{29'd0, (src.sz == SZ_L) ? 3'd4 : 3'd2} : 32'd0)) +
				           ((src.ea0.m == EM_APD) ? -{29'd0, (src.sz == SZ_L) ? 3'd4 : 3'd2}
				                                  :  {29'd0, (src.sz == SZ_L) ? 3'd4 : 3'd2});
				mv_cnt  <= (mv_act ? mv_cnt : 8'd0) + ((src.sz == SZ_L) ? 8'd4 : 8'd2);
			end
			else if (n_def_set) begin
				def_v   <= 1'b1;
				def_reg <= n_def_reg;
				def_amt <= n_def_amt;
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

		if (flush) begin
			first <= 1'b1;
			ind0  <= 1'b0;
			ind1  <= 1'b0;
			def_v <= 1'b0;
			mv_act <= 1'b0;
			mv_t11 <= 1'b0;
			mv_fix <= 2'd0;
			uq_n  <= 2'd0;
		end
		if (exc_go) begin
			first    <= 1'b1;
			ssp      <= exc_ssp;
			uq_n     <= 2'd0;
		end
	end
end

// synthesis translate_off
always_ff @(posedge clk)
	if (nreset && emit && (dbg_mvd && (dbg_dm || dbg_mvo[31:7] != {25{dbg_mvo[7]}}))) begin
		$display("FAIL: useq: a MOVEM offset %h for AG (merge %b)", dbg_mvo, dbg_dm);
		$finish;
	end
always_ff @(posedge clk)
	if (nreset && emit && dbg_dm && nu.disp != dbg_dpre) begin
		$display("FAIL: useq: a merged uop's displacement %h changed from %h", nu.disp, dbg_dpre);
		$finish;
	end
// the registered forms against the combinational ones
always_ff @(posedge clk)
	if (nreset && b_v) begin
		if (b_stay_q != stay || n_ptr_p != n_ptr || n_jump) begin
			$display("FAIL: useq: stay %b/%b ptr %b/%b jump %b at upc %0d", b_stay_q, stay,
			         n_ptr_p, n_ptr, n_jump, upc);
			$finish;
		end
		if (uw.loop && (mv_act ? (mv_is0 != (mv_mask == 16'd0) || mv_pc1 != pc1(mv_mask)) : 1'b0)) begin
			$display("FAIL: useq: the MOVEM mask terms");
			$finish;
		end
	end
// D2a's terms against the word and record they were formed from
always_ff @(posedge clk)
	if (nreset && b_v) begin
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
		if (b_mev != mv || b_me1 != e1 || b_me != (e1 ? rb.ea1 : rb.ea0) || b_mem != mm ||
		    b_agin != (mv || ub.ag == AGM_EA0 || ub.ag == AGM_EA1 || ub.ag == AGM_LEA0 ||
		               (ub.ag == AGM_FEA && rb.ea0.m != EM_IMM)) ||
		    b_low1 != (rb.ext1 & (~rb.ext1 + 16'd1)) || b_bi1 != enc16(rb.ext1) ||
		    (uw.loop && mv_act && (mv_low != (mv_mask & (~mv_mask + 16'd1)) || mv_bi != enc16(mv_mask)))) begin
			$display("FAIL: useq: D2a's terms for the word at %0d", upc);
			$finish;
		end
	end
// the kind kept for the word being read is the word's
always_ff @(posedge clk)
	if (nreset && a_v && (a_k.last != ur.last || a_k.jmp != (ur.jc != JC_NEVER) ||
	                      a_k.jalw != (ur.jc == JC_ALWAYS) || a_k.jc != ur.jc ||
	                      (a_k.jmp && a_k.jt != ur.jt))) begin
		$display("FAIL: useq: the kind %h of the word at %0d", a_k, upc);
		$finish;
	end
always_ff @(posedge clk)
	if (nreset && a_v && !a_exc && rq_n == 3'd0) begin
		$display("FAIL: useq: a word without its record at %0d", upc);
		$finish;
	end
always_ff @(posedge clk)
	if (nreset && pass && ur.loop && !done && (mv_act || mv_t11 || mv_fix != 2'd0)) begin
		$display("FAIL: useq: a MOVEM word with the loop state set");
		$finish;
	end
// synthesis translate_on

endmodule
