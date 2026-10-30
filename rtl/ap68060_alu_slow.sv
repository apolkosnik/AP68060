//--------------------------------------------------------------------------//
// AP68060 - pipelined MC68040                                            //
//                                                                          //
// ap68060_alu_slow.sv - EX-stage multi-cycle integer unit: shifts and      //
// rotates, BCD, PACK/UNPK.  A registered first stage (the funnel's input   //
// and count, the BCD and PACK results) is loaded from EX's operands every  //
// clock; the outputs are formed from it, and the back end registers them   //
// a clock later (the op holds EX: its operands stay put meanwhile).        //
//                                                                          //
// Same conventions and 68040 flag rules as ap68060_alu.                    //
//--------------------------------------------------------------------------//

module ap68060_alu_slow
	import ap68060_pkg::*;
(
	input  logic        clk,
	input  logic  [6:0] op,
	input  logic  [1:0] sz,
	input  logic [31:0] a,
	input  logic [31:0] b,
	input  logic  [5:0] cnt,        // shift count: a[5:0], without a's mux
	input  logic  [5:0] nx_q,       // ROXL/ROXR: the count mod (size + 1), registered
	output logic  [5:0] nx,         // ... formed here from cnt (for that register)
	input  logic  [4:0] flags_in,   // X N Z V C
	output logic [31:0] res,        // (from the first stage's registers)
	output logic  [4:0] flags_out
);

function automatic logic msb_of(input logic [31:0] v, input logic [1:0] s);
	case (s)
		SZ_B:    msb_of = v[7];
		SZ_W:    msb_of = v[15];
		default: msb_of = v[31];
	endcase
endfunction

function automatic logic [31:0] merge(input logic [31:0] r, input logic [31:0] bb,
                                      input logic [31:0] m);
	merge = (r & m) | (bb & ~m);
endfunction

//--------------------------------------------------------------------------
// first stage: from EX's operands into registers
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

wire [31:0] bm    = b & szmask;
wire        b_msb = msb_of(b, sz);
wire        f_x   = flags_in[4];

// BCD (byte).  Decimal corrections apply to the whole byte, so a +/-6
// low-nibble adjust ripples into the high nibble; the carry comes from the
// corrected value (68040 hardware, cputest 68040_default).
wire [4:0] abcd_lo  = {1'b0, b[3:0]} + {1'b0, a[3:0]} + {4'd0, f_x};
wire [9:0] abcd_sum = {2'd0, b[7:0]} + {2'd0, a[7:0]} + {9'd0, f_x}
                    + ((abcd_lo > 5'd9) ? 10'd6 : 10'd0);
wire       abcd_c   = ((abcd_sum & 10'h3F0) > 10'h090);
wire [9:0] abcd_res = abcd_sum + (abcd_c ? 10'h060 : 10'd0);

// SBCD b - a - X and NBCD 0 - b - X share one datapath
wire [7:0] sb_l     = (op == OP_NBCD) ? 8'd0 : b[7:0];
wire [7:0] sb_r     = (op == OP_NBCD) ? b[7:0] : a[7:0];
wire       sbcd_lb  = ({1'b0, sb_l[3:0]} < ({1'b0, sb_r[3:0]} + {4'd0, f_x}));
wire [9:0] sbcd_raw = {2'd0, sb_l} - {2'd0, sb_r} - {9'd0, f_x};
wire [9:0] sbcd_cor = sbcd_raw - (sbcd_lb ? 10'd6 : 10'd0);
wire [9:0] sbcd_res = sbcd_cor - (sbcd_raw[9] ? 10'h060 : 10'd0);
wire       sbcd_c   = sbcd_cor[9];

// PACK: b holds the two unpacked bytes of the word, a the adjustment
wire [15:0] pk_sum  = b[15:0] + a[15:0];
wire  [7:0] pack_r  = {pk_sum[11:8], pk_sum[3:0]};
// UNPK: b holds the packed byte
wire [15:0] unpk_r  = {4'd0, b[7:4], 4'd0, b[3:0]} + a[15:0];

//--------------------------------------------------------------------------
// shifts and rotates: closed forms of the 68040's per-step semantics.
// count = cnt (a[5:0]; register counts are mod 64, immediate counts 1..8).
// One right funnel (66 bits) does LSR, ASR, ROR, ROL, ROXR and ROXL; the
// same funnel does LSL and ASL.  The right funnel's input is the operand
// zero- or sign-extended (with a 0 below it: the bit just below the
// result is the last bit shifted out, the carry), or repeated with the
// rotate's period (the size, or the size + 1 with X for ROXR/ROXL), so a
// right shift by the rotate count leaves the rotated value at the bottom.
// A left rotate is a right rotate by the period less the count.  The
// funnel's input and count are registered (the first stage), the shift
// runs from them.
//--------------------------------------------------------------------------
logic  [5:0] sh_n, sh_nm, sh_nx;
logic [65:0] rv;
logic  [6:0] ra;
logic        vl;
always_comb begin
	logic [31:0] yv, tm;
	sh_n  = cnt;
	sh_nm = sh_n & (nbits - 6'd1);
	// n mod (size + 1) for ROXL/ROXR by subtracting multiples (no divider):
	// formed here (nx), registered by the back end in a cycle of its own,
	// and used from that register (nx_q)
	sh_nx = sh_n;
	case (nbits)
		6'd8: begin
			if (sh_nx >= 6'd36) sh_nx = sh_nx - 6'd36;
			if (sh_nx >= 6'd18) sh_nx = sh_nx - 6'd18;
			if (sh_nx >= 6'd9)  sh_nx = sh_nx - 6'd9;
		end
		6'd16: begin
			if (sh_nx >= 6'd34) sh_nx = sh_nx - 6'd34;
			if (sh_nx >= 6'd17) sh_nx = sh_nx - 6'd17;
		end
		default: if (sh_nx >= 6'd33) sh_nx = sh_nx - 6'd33;
	endcase
	nx    = sh_nx;
	sh_nx = nx_q;
	case (op)
		OP_ASR:  begin
			rv = {{34{b_msb}}, bm, 1'b0};
			case (sz)
				SZ_B:    rv[65:9]  = {57{b_msb}};
				SZ_W:    rv[65:17] = {49{b_msb}};
				default: ;
			endcase
			ra = {1'b0, sh_n};
		end
		OP_ROL, OP_ROR: begin
			case (sz)
				SZ_B:    rv = {50'd0, bm[7:0], bm[7:0]};
				SZ_W:    rv = {34'd0, bm[15:0], bm[15:0]};
				default: rv = {2'd0, bm, bm};
			endcase
			ra = (op == OP_ROR) ? {1'b0, sh_nm} : {1'b0, (nbits - sh_nm) & (nbits - 6'd1)};
		end
		OP_ROXL, OP_ROXR: begin
			case (sz)
				SZ_B:    rv = {48'd0, f_x, bm[7:0], f_x, bm[7:0]};
				SZ_W:    rv = {32'd0, f_x, bm[15:0], f_x, bm[15:0]};
				default: rv = {f_x, bm, f_x, bm};
			endcase
			ra = (op == OP_ROXR) ? {1'b0, sh_nx}
			                     : ((sh_nx == 6'd0) ? 7'd0 : ({1'b0, nbits} + 7'd1 - {1'b0, sh_nx}));
		end
		OP_ASL, OP_LSL: begin
			// a left shift through the same funnel: the operand above 31
			// zeros, shifted right by 31 - n (the count inverted, no
			// subtraction), is the operand shifted left by n; n >= 32
			// shifts everything out (C of a long shifted by 32: below)
			rv = {3'd0, bm, 31'd0};
			ra = sh_n[5] ? 7'd127 : {2'd0, ~sh_n[4:0]};
		end
		default: begin   // LSR (and the ASR sign fill above)
			rv = {33'd0, bm, 1'b0};
			ra = {1'b0, sh_n};
		end
	endcase
	// ASL: V when the bits shifted out and the new sign bit are not all
	// equal, i.e. the top count+1 bits of the operand (at the size) differ
	// somewhere: an adjacent pair among them differs.  yv[i] = x[i] ^ x[i-1],
	// tm marks the top count positions; no shifter on the path
	yv = bm ^ {bm[30:0], 1'b0};
	for (int i = 0; i < 32; i++)
		tm[i] = (6'(i) < nbits) && ((nbits - 6'd1 - 6'(i)) < sh_n);
	vl = (yv & tm) != 32'd0;
end

// the first stage's registers
logic  [6:0] q_op;
logic  [1:0] q_sz;
logic [31:0] q_b, q_bm, q_szm;
logic  [5:0] q_nbits, q_n;
logic  [4:0] q_f;
logic [65:0] q_rv;
logic  [6:0] q_ra;
logic        q_asr, q_vl, q_bnz;
logic  [7:0] q_abcd, q_sbcd, q_pack;
logic        q_abcd_c, q_sbcd_c;
logic [15:0] q_unpk;
always_ff @(posedge clk) begin
	q_op    <= op;
	q_sz    <= sz;
	q_b     <= b;
	q_bm    <= bm;
	q_szm   <= szmask;
	q_nbits <= nbits;
	q_n     <= (op == OP_ROXL || op == OP_ROXR) ? nx_q : cnt;
	q_f     <= flags_in;
	q_rv    <= rv;
	q_ra    <= ra;
	q_asr   <= (op == OP_ASR);
	q_vl    <= vl;
	q_bnz   <= (bm != 32'd0);
	q_abcd  <= abcd_res[7:0];
	q_abcd_c <= abcd_c;
	q_sbcd  <= sbcd_res[7:0];
	q_sbcd_c <= sbcd_c;
	q_pack  <= pack_r;
	q_unpk  <= unpk_r;
end

//--------------------------------------------------------------------------
// second stage: the shift, and the results, from the registers
//--------------------------------------------------------------------------
logic [65:0] ro;
logic [31:0] sh_r;
logic        sh_c, sh_x, sh_v;
always_comb begin
	logic [5:0] n;
	n  = q_n;
	ro = q_asr ? 66'($signed(q_rv) >>> q_ra) : (q_rv >> q_ra);
	sh_r = q_bm;
	sh_c = 1'b0;
	sh_x = q_f[4];
	sh_v = 1'b0;
	if (q_op == OP_ROXL || q_op == OP_ROXR) begin
		// (a count of 0 leaves the operand: C = X, X unchanged)
		if (n != 6'd0) begin
			sh_x = ro[q_nbits];
			sh_r = ro[31:0] & q_szm;
		end
		sh_c = sh_x;
	end
	else if (q_n != 6'd0) begin
		case (q_op)
			OP_ASL, OP_LSL: begin
				sh_r = ro[31:0] & q_szm;
				sh_c = (q_n <= q_nbits) && (q_n[5] ? q_bm[0] : ro[q_nbits]);
				sh_x = sh_c;
				if (q_op == OP_ASL)
					sh_v = (q_n >= q_nbits) ? q_bnz : q_vl;
			end
			OP_LSR, OP_ASR: begin
				sh_r = ro[32:1] & q_szm;
				sh_c = ro[0];
				sh_x = sh_c;
			end
			default: begin   // ROL, ROR
				sh_r = ro[31:0] & q_szm;
				sh_c = (q_op == OP_ROL) ? sh_r[0] : sh_r[q_nbits - 6'd1];
			end
		endcase
	end
end

// (shifts: N and Z are left to the back end, from the registered result)
always_comb begin
	res       = q_b;
	flags_out = q_f;
	case (q_op)
		OP_ABCD: begin
			res = {q_b[31:8], q_abcd};
			flags_out = {q_abcd_c, q_f[3], q_f[2] & (q_abcd == 8'd0), q_f[1], q_abcd_c};
		end
		OP_SBCD, OP_NBCD: begin
			res = {q_b[31:8], q_sbcd};
			flags_out = {q_sbcd_c, q_f[3], q_f[2] & (q_sbcd == 8'd0), q_f[1], q_sbcd_c};
		end
		OP_PACK: res = {q_b[31:8], q_pack};
		OP_UNPK: res = {q_b[31:16], q_unpk};
		OP_ASL, OP_ASR, OP_LSL, OP_LSR, OP_ROL, OP_ROR, OP_ROXL, OP_ROXR: begin
			res = merge(sh_r, q_b, q_szm);
			flags_out = {sh_x, 1'b0, 1'b0, sh_v, sh_c};
		end
		default: ;
	endcase
end

endmodule
