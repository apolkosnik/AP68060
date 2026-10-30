//--------------------------------------------------------------------------//
// AP68060 - pipelined MC68040                                            //
//                                                                          //
// ap68060_muldiv.sv - iterative multiply and divide (from AP040's          //
// ap040_muldiv.v, which the cputest corpus validated)                       //
//                                                                          //
// MUL: 32x32 -> 64, signed or unsigned: four registered 16x16 DSP       //
//      partial products, their sum, then the sign                         //
// DIV: 32/32 -> q32,r32 restoring, 1 bit per cycle (the 68060 has no     //
//      64-bit dividend: those forms trap); ovf for the one signed quotient //
//      that does not fit, -2^31 / -1.  Signed quotients truncate toward    //
//      zero, the remainder takes the dividend's sign.                       //
// kill abandons a running operation (pipeline flush).                       //
//--------------------------------------------------------------------------//

module ap68060_muldiv
(
	input             clk,
	input             nreset,
	input             kill,

	input             start,
	input             is_div,
	input             sign_op,
	input      [31:0] op_a,        // multiplier / divisor
	input      [31:0] op_lo,       // multiplicand / dividend

	output            busy,
	output reg        done,        // one cycle pulse
	output reg [31:0] res_hi,      // product high / remainder
	output reg [31:0] res_lo,      // product low / quotient
	output reg        ovf
);

reg        running;
reg        div_r;
reg        neg_q;                 // negate quotient / product
reg        neg_r;                 // negate remainder
reg  [6:0] count;
reg [31:0] den;                   // divisor / multiplier (absolute)
reg [31:0] mcand;                 // multiplicand (absolute)
reg [31:0] pp [4];                // the four 16x16 partial products
reg [63:0] prod;                  // their sum
reg [64:0] acc;                   // divide: remainder [64:32], dividend/quotient [31:0]
reg  [1:0] mph;                   // multiply: 2 partial products, 1 their sum, 0 the result

// absolute values for signed operations
wire [31:0] abs_a  = (sign_op && op_a[31]) ? (32'd0 - op_a) : op_a;
wire [31:0] abs_m  = (sign_op && op_lo[31]) ? (32'd0 - op_lo) : op_lo;   // also |dividend|

// divide: one restoring step per cycle (32 rounds: at 120 MHz the time
// the former two steps a cycle took at 60), each exactly one former
// one-bit iteration; the divisor is an explicit argument so the function
// stays pure (module-level variables read from inside a function are
// unreliable in continuous assignments under iverilog)
function [64:0] div_step;
	input [64:0] a;
	input [31:0] d;
	reg   [64:0] sh;
	reg   [33:0] t;
	begin
		sh = {a[63:0], 1'b0};
		t  = {1'b0, sh[64:32]} - {2'b00, d};
		if (!t[33]) div_step = {t[32:0], sh[31:1], 1'b1};
		else        div_step = sh;
	end
endfunction

wire [63:0] q_raw = div_r ? {32'd0, acc[31:0]} : prod;
wire [31:0] r_raw = acc[63:32];

always @(posedge clk) begin
	if (!nreset) begin
		running <= 0;
		done    <= 0;
		div_r   <= 0;
		neg_q   <= 0;
		neg_r   <= 0;
		count   <= 0;
		den     <= 0;
		mcand   <= 0;
		prod    <= 0;
		acc     <= 0;
		mph     <= 0;
		res_hi  <= 0;
		res_lo  <= 0;
		ovf     <= 0;
	end
	else begin
		done <= 0;

		if (start) begin
			div_r   <= is_div;
			running <= 1;
			ovf     <= 0;
			if (is_div) begin
				den   <= abs_a;
				acc   <= {33'd0, abs_m};
				count <= 7'd32;
				neg_q <= sign_op && (op_lo[31] ^ op_a[31]);
				neg_r <= sign_op && op_lo[31];
			end
			else begin
				den   <= abs_a;
				mcand <= abs_m;
				count <= 7'd0;
				mph   <= 2'd2;
				neg_q <= sign_op && (op_a[31] ^ op_lo[31]) && (op_a != 0) && (op_lo != 0);
				neg_r <= 0;
			end
		end
		else if (running) begin
			if (div_r && count != 0) begin
				count <= count - 7'd1;
				acc   <= div_step(acc, den);
			end
			else if (!div_r && mph == 2'd2) begin
				// the 16x16 partial products (four DSP multipliers)
				pp[0] <= mcand[15:0]  * den[15:0];
				pp[1] <= mcand[31:16] * den[15:0];
				pp[2] <= mcand[15:0]  * den[31:16];
				pp[3] <= mcand[31:16] * den[31:16];
				mph   <= 2'd1;
			end
			else if (!div_r && mph == 2'd1) begin
				// their sum: exactly the 32x32 product
				prod <= {pp[3], pp[0]} + ({31'd0, {1'b0, pp[1]} + {1'b0, pp[2]}} << 16);
				mph  <= 2'd0;
			end
			else begin
				running <= 1'b0;
				done    <= 1'b1;
				if (div_r) begin
					res_lo <= neg_q ? (32'd0 - q_raw[31:0]) : q_raw[31:0];
					res_hi <= neg_r ? (32'd0 - r_raw) : r_raw;
					ovf    <= sign_op & !neg_q & q_raw[31];
				end
				else begin
					{res_hi, res_lo} <= neg_q ? (64'd0 - q_raw) : q_raw;
					ovf    <= 0;
				end
			end
		end
		// a kill only stops the unit: the data registers go on as they
		// would (the late kill reaches two flip-flops, not every one)
		if (kill) begin
			running <= 1'b0;
			done    <= 1'b0;
		end
	end
end

// synthesis translate_off
always @(posedge clk)
	if (nreset && running && !div_r && mph == 2'd1 &&
	    {pp[3], pp[0]} + ({31'd0, {1'b0, pp[1]} + {1'b0, pp[2]}} << 16) != 64'(mcand) * 64'(den)) begin
		$display("FAIL: muldiv: partial products sum to %h, the product is %h",
		         {pp[3], pp[0]} + ({31'd0, {1'b0, pp[1]} + {1'b0, pp[2]}} << 16), 64'(mcand) * 64'(den));
		$finish;
	end
// synthesis translate_on

assign busy = running;

endmodule
