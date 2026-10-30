//--------------------------------------------------------------------------//
// AP68060 - pipelined MC68040                                            //
//                                                                          //
// ap68060_alu.sv - EX-stage single-cycle integer unit: arithmetic, logic,  //
// bit operations, condition codes (shifts and BCD: ap68060_alu_slow)       //
//                                                                          //
// Conventions (same as the 68k "OP src,dst"):                              //
//   a  source operand, b destination operand, res = b OP a                 //
//   res is the full 32-bit register image: for byte and word sizes the     //
//   bits above the operand size are b's (so a data register write merges)  //
//   flags are {X,N,Z,V,C}; flags_out is the complete new CCR, the uop's    //
//   ccr_we mask decides which bits are actually written                    //
//                                                                          //
// Undefined-flag behaviour is the 68040's, as WinUAE's newcpu_common.cpp   //
// models it (verified against the cputest 68040 corpus on the previous     //
// core): BCD leaves N and V unchanged; CHK writes only N (68060: N is     //
// clear unless it traps, then N = Dn < 0; WinUAE setchkundefinedflags);   //
// CHK2/CMP2 write only Z and C (68060: unimplemented, vector 61).          //
//--------------------------------------------------------------------------//

module ap68060_alu
	import ap68060_pkg::*;
(
	input  logic  [6:0] op,
	input  logic  [1:0] sz,
	input  logic  [3:0] cond,
	input  logic [31:0] a,
	input  logic [31:0] b,
	input  logic [31:0] ea,
	input  logic  [4:0] flags_in,   // X N Z V C
	input  logic [15:0] rk,         // the result's case, one-hot, decoded from op a stage
	                                // early (alu_rk): MOV AND OR EOR NOT CLR EXT EXTB SWAP
	                                // TAS BCHG BCLR BSET SCC EA DBCC, none: b
	output logic [31:0] res_f,      // res for those ops, as one AND-OR of them
	input  logic  [3:0] ac,         // the adder's controls, decoded from op a stage
	                                // early (alu_ctl): {CMP, X in, subtract, NEG(X)}
	output logic [31:0] res,
	output logic [31:0] res_sum,    // the adder's result merged at the size (ADD SUB
	                                // ADDX SUBX NEGX NEG)
	output logic [31:0] sum_o,      // the adder's sum (its size's bits; the user merges)
	output logic  [4:0] sum_fl,     // the flags of ADD SUB ADDX SUBX NEG NEGX CMP
	output logic  [4:0] flags_out,
	output logic        cc_true,    // cond evaluated on flags_in
	output logic        trap        // CHK / CHK2 / TRAPcc exception
);

wire f_x = flags_in[4];
wire f_n = flags_in[3];
wire f_z = flags_in[2];
wire f_v = flags_in[1];
wire f_c = flags_in[0];

//--------------------------------------------------------------------------
// condition codes (MC68040UM table 3-19 / PRM 3.6)
//--------------------------------------------------------------------------
always_comb begin
	case (cond)
		4'h0: cc_true = 1'b1;                       // T
		4'h1: cc_true = 1'b0;                       // F
		4'h2: cc_true = !f_c && !f_z;               // HI
		4'h3: cc_true = f_c || f_z;                 // LS
		4'h4: cc_true = !f_c;                       // CC
		4'h5: cc_true = f_c;                        // CS
		4'h6: cc_true = !f_z;                       // NE
		4'h7: cc_true = f_z;                        // EQ
		4'h8: cc_true = !f_v;                       // VC
		4'h9: cc_true = f_v;                        // VS
		4'hA: cc_true = !f_n;                       // PL
		4'hB: cc_true = f_n;                        // MI
		4'hC: cc_true = f_n == f_v;                 // GE
		4'hD: cc_true = f_n != f_v;                 // LT
		4'hE: cc_true = !f_z && (f_n == f_v);       // GT
		default: cc_true = f_z || (f_n != f_v);     // LE
	endcase
end

//--------------------------------------------------------------------------
// sized views
//--------------------------------------------------------------------------
logic [31:0] szmask;
logic  [5:0] nbits;
always_comb begin
	case (sz)
		SZ_B:    begin szmask = 32'h0000_00FF; nbits = 6'd8;  end
		SZ_W:    begin szmask = 32'h0000_FFFF; nbits = 6'd16; end
		default: begin szmask = 32'hFFFF_FFFF; nbits = 6'd32; end
	endcase
end

wire [31:0] am = a & szmask;
wire [31:0] bm = b & szmask;

function automatic logic msb_of(input logic [31:0] v, input logic [1:0] s);
	case (s)
		SZ_B:    msb_of = v[7];
		SZ_W:    msb_of = v[15];
		default: msb_of = v[31];
	endcase
endfunction

function automatic logic carry_of(input logic [32:0] v, input logic [1:0] s);
	case (s)
		SZ_B:    carry_of = v[8];
		SZ_W:    carry_of = v[16];
		default: carry_of = v[32];
	endcase
endfunction

wire a_msb = msb_of(a, sz);
wire b_msb = msb_of(b, sz);

// merge a sized result into b's upper bits
function automatic logic [31:0] merge(input logic [31:0] r, input logic [31:0] bb,
                                      input logic [31:0] m);
	merge = (r & m) | (bb & ~m);
endfunction

//--------------------------------------------------------------------------
// adder: one shared add/subtract with optional X-in, l + (r ^ sub) + cin,
// its controls registered (ac).  The operands enter unmasked, so each
// adder bit's inputs are four signals and fit the adder's own LUTs (no
// level in front of the carry chain): the bits above the size are
// garbage, and the carry out at the size is recovered from the sum's next
// bit and that bit's inputs
//--------------------------------------------------------------------------
wire        a_cmp  = ac[3];
wire        use_x  = ac[2];
wire        is_sub = ac[1];
wire        b_zero = ac[0];

// NEG is 0 - b: the subtrahend is b, the minuend zero
wire [31:0] add_l  = b_zero ? 32'd0 : b;
wire [31:0] add_r  = (b_zero ? b : a) ^ {32{is_sub}};
wire        cin    = (use_x & f_x) ^ is_sub;
wire [32:0] sum    = {1'b0, add_l} + {1'b0, add_r} + {32'd0, cin};
// the carry into bit 8 or 16, or out of bit 31 (subtracting: no borrow)
wire        c_out  = (sz == SZ_B) ? (sum[8]  ^ add_l[8]  ^ add_r[8])  :
                     (sz == SZ_W) ? (sum[16] ^ add_l[16] ^ add_r[16]) : sum[32];
wire        sum_c  = c_out ^ is_sub;
wire        sum_n  = msb_of(sum[31:0], sz);
wire        sum_z  = (sum[31:0] & szmask) == 32'd0;
wire        l_msb  = b_zero ? 1'b0  : b_msb;
wire        r_msb  = b_zero ? b_msb : a_msb;
wire        sum_v  = is_sub ? ((l_msb != r_msb) && (sum_n != l_msb))
                            : ((l_msb == r_msb) && (sum_n != l_msb));
assign      res_sum = merge(sum[31:0], b, szmask);
assign      sum_o   = sum[31:0];
// (CMP keeps X; ADDX SUBX NEGX clear Z only)
assign      sum_fl  = {a_cmp ? f_x : sum_c, sum_n, use_x ? (f_z & sum_z) : sum_z, sum_v, sum_c};

//--------------------------------------------------------------------------
// bit operations: a holds the bit number, already reduced mod 32 (register)
// or mod 8 (memory byte) by the size
//--------------------------------------------------------------------------
wire  [4:0] bitn     = (sz == SZ_B) ? {2'b00, a[2:0]} : a[4:0];
wire [31:0] bit_mask = 32'd1 << bitn;
wire        bit_set  = |(b & bit_mask);

//--------------------------------------------------------------------------
// CHK (b = Dn, a = upper bound), sized W or L, signed
//--------------------------------------------------------------------------
wire        chk_dn_neg = b_msb;
wire        chk_ub_neg = a_msb;
wire [32:0] chk_diff   = {1'b0, am} - {1'b0, bm};   // bound - Dn
wire        chk_gt     = (chk_ub_neg != chk_dn_neg) ? chk_ub_neg
                                                    : msb_of(chk_diff[31:0], sz);
wire        chk_trap   = chk_dn_neg || chk_gt;

//--------------------------------------------------------------------------
// CHK2/CMP2: b is the value, a the bound; signedness from cond[1]
// (signed when the instruction's register is an address register or the
// bounds compare signed: the 68040 compares as signed when the bounds are
// signed, i.e. lower > upper as unsigned is the wrap case).  The two steps
// keep the lower-bound result in chk2_lo for the second.
//--------------------------------------------------------------------------
// handled in the back end (needs state across two uops)

//--------------------------------------------------------------------------
// result and flags
//--------------------------------------------------------------------------
// the result's cases as one AND-OR, their selects registered (rk)
always_comb begin
	logic [31:0] c [16];
	c[0]  = merge(am, b, szmask);
	c[1]  = merge(bm & am, b, szmask);
	c[2]  = merge(bm | am, b, szmask);
	c[3]  = merge(bm ^ am, b, szmask);
	c[4]  = merge(~bm & szmask, b, szmask);
	c[5]  = merge(32'd0, b, szmask);
	c[6]  = (sz == SZ_W) ? {b[31:16], {8{b[7]}}, b[7:0]} : {{16{b[15]}}, b[15:0]};
	c[7]  = {{24{b[7]}}, b[7:0]};
	c[8]  = {b[15:0], b[31:16]};
	c[9]  = {b[31:8], 1'b1, b[6:0]};
	c[10] = b ^ bit_mask;
	c[11] = b & ~bit_mask;
	c[12] = b | bit_mask;
	c[13] = {b[31:8], {8{cc_true}}};
	c[14] = ea;
	c[15] = {b[31:16], b[15:0] - 16'd1};
	res_f = (rk == 16'd0) ? b : 32'd0;
	for (int k = 0; k < 16; k++) res_f = res_f | ({32{rk[k]}} & c[k]);
end

always_comb begin
	logic [31:0] r;
	r         = b;
	flags_out = flags_in;
	trap      = 1'b0;
	res       = b;

	case (op)
		OP_MOV: begin
			r = merge(am, b, szmask);
			flags_out = {f_x, msb_of(am, sz), am == 32'd0, 1'b0, 1'b0};
		end

		OP_ADD, OP_SUB: begin
			r = merge(sum[31:0], b, szmask);
			flags_out = {sum_c, sum_n, sum_z, sum_v, sum_c};
		end

		OP_ADDX, OP_SUBX, OP_NEGX: begin
			r = merge(sum[31:0], b, szmask);
			flags_out = {sum_c, sum_n, f_z & sum_z, sum_v, sum_c};
		end

		OP_NEG: begin
			r = merge(sum[31:0], b, szmask);
			flags_out = {sum_c, sum_n, sum_z, sum_v, sum_c};
		end

		OP_CMP: begin
			r = b;
			flags_out = {f_x, sum_n, sum_z, sum_v, sum_c};
		end

		OP_AND, OP_OR, OP_EOR, OP_NOT: begin
			logic [31:0] l;
			case (op)
				OP_AND:  l = bm & am;
				OP_OR:   l = bm | am;
				OP_EOR:  l = bm ^ am;
				default: l = ~bm & szmask;
			endcase
			r = merge(l, b, szmask);
			flags_out = {f_x, msb_of(l, sz), l == 32'd0, 1'b0, 1'b0};
		end

		OP_CLR: begin
			r = merge(32'd0, b, szmask);
			flags_out = {f_x, 1'b0, 1'b1, 1'b0, 1'b0};
		end

		OP_EXT: begin
			if (sz == SZ_W) begin
				r = {b[31:16], {8{b[7]}}, b[7:0]};
				flags_out = {f_x, b[7], b[7:0] == 8'd0, 1'b0, 1'b0};
			end
			else begin
				r = {{16{b[15]}}, b[15:0]};
				flags_out = {f_x, b[15], b[15:0] == 16'd0, 1'b0, 1'b0};
			end
		end

		OP_EXTB: begin
			r = {{24{b[7]}}, b[7:0]};
			flags_out = {f_x, b[7], b[7:0] == 8'd0, 1'b0, 1'b0};
		end

		OP_SWAP: begin
			r = {b[15:0], b[31:16]};
			flags_out = {f_x, b[15], b == 32'd0, 1'b0, 1'b0};
		end

		OP_TAS: begin
			r = {b[31:8], 1'b1, b[6:0]};
			flags_out = {f_x, b[7], b[7:0] == 8'd0, 1'b0, 1'b0};
		end

		OP_BTST: begin
			r = b;
			flags_out = {f_x, f_n, !bit_set, f_v, f_c};
		end
		OP_BCHG: begin
			r = b ^ bit_mask;
			flags_out = {f_x, f_n, !bit_set, f_v, f_c};
		end
		OP_BCLR: begin
			r = b & ~bit_mask;
			flags_out = {f_x, f_n, !bit_set, f_v, f_c};
		end
		OP_BSET: begin
			r = b | bit_mask;
			flags_out = {f_x, f_n, !bit_set, f_v, f_c};
		end

		OP_SCC: r = {b[31:8], {8{cc_true}}};

		OP_EA: r = ea;

		OP_DBCC: r = {b[31:16], b[15:0] - 16'd1};

		OP_TRAPCC: trap = cc_true;

		OP_CHK: begin
			trap = chk_trap;
			// 68060: N clear, or Dn < 0 on the trap; Z, V, C unchanged
			flags_out = {f_x, chk_trap && chk_dn_neg, f_z, f_v, f_c};
		end

		OP_CCRLOG: begin
			case (cond[1:0])
				2'd0:    flags_out = flags_in & a[4:0];
				2'd1:    flags_out = flags_in | a[4:0];
				2'd2:    flags_out = flags_in ^ a[4:0];
				default: flags_out = a[4:0];
			endcase
		end

		default: r = b;
	endcase
	res = r;
end

endmodule
