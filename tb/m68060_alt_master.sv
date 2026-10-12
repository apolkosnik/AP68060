//--------------------------------------------------------------------------//
// AP68060 test bench                                                     //
//                                                                          //
// m68060_alt_master.sv - an alternate bus master on the 68060 bus           //
//                         (MC68060UM 7.11 arbitration, 7.12 snooping)       //
//                                                                          //
// Also the bus arbiter: the 68060 has BG while this master is idle.  A     //
// command runs up to four transfers in one bus tenure:                     //
//   go         pulse: start (n transfers, 1..4)                            //
//   trig_en    first wait for trig (a bench event), then request the bus  //
//   delay      bus clocks to hold the bus before the first TS             //
//   x_addr[k]  address; x_ctl[k] bit 0 read, 2:1 SIZ (00 long, 01 byte,   //
//              10 word, 11 line), 4:3 nonzero: SNOOP asserted              //
//   x_wd[k]    write data: a line's four long words (A3:A2 = 0 first), or //
//              in [127:96] the long word as it goes on D31-D0             //
//   x_rd[k]    read data, the same layout (a single transfer in [127:96]) //
//   btt_mode   the MC68060 arbitration protocol alone (7.11.2): BB is   //
//              never driven (pulled up), and BG goes back to the 68060   //
//              as the last transfer starts -- the 68060 must wait for    //
//              this master's BTT before it starts a transfer             //
//   rep, step  then rep more times, in the same tenure and without a gap:  //
//              the transfers again, every address step bytes further, a   //
//              single write's data plus the repetition's number           //
//   status     0 idle or busy, 1 done, 2 ended in a bus error             //
//                                                                          //
// Arbitration: BG to the 68060 is negated (with BGR asserted when        //
// bgr_mode is set), and the bus is taken once the 68060 has released BB. //
// The tenure ends 68060-style: BTT asserted one bus clock with BB going   //
// negated, then both released.  Transfers follow the 68040 termination   //
// protocol: TS for one bus clock, then TA/TEA; TA with TEA is a retry     //
// (the transfer is run again from its start); TBI with the first TA of a  //
// line turns the rest into three long-word transfers.  Every bus output  //
// is a flip-flop that changes on a bus clock edge.                        //
//--------------------------------------------------------------------------//

module m68060_alt_master (
	input  logic        clk,
	input  logic        nreset,
	input  logic        bclk_en,

	input  logic        go,
	input  logic  [2:0] n,
	input  logic        trig_en,
	input  logic        trig,
	input  logic [15:0] delay,
	input  logic [15:0] rep,
	input  logic [31:0] step,
	input  logic [31:0] x_addr [4],
	input  logic  [4:0] x_ctl  [4],
	input  logic [127:0] x_wd  [4],
	output logic [127:0] x_rd  [4],
	output logic  [1:0] status,

	input  logic        bgr_mode,
	input  logic        btt_mode,
	output logic        cpu_bg_n,
	output logic        cpu_bgr_n,
	input  logic        cpu_bb_n,

	output logic        drive,
	output logic [31:0] a,
	output logic        rw_n,
	output logic  [1:0] siz,
	output logic  [1:0] tt,
	output logic  [2:0] tm,
	output logic        snoop_n,
	output logic        ts_n,
	output logic        bb_n,
	output logic        btt_n,
	output logic [31:0] d,
	output logic        d_oe,
	input  logic [31:0] d_bus,
	input  logic        ta_n,
	input  logic        tea_n,
	input  logic        tbi_n
);

typedef enum logic [3:0] {
	A_IDLE, A_TRIG, A_REQ, A_OWN, A_TS, A_DATA, A_REL, A_REL2
} st_t;

st_t         st;
logic  [2:0] nx;              // transfers in this command
logic  [1:0] k;               // the transfer running
logic  [1:0] beat;
logic        inh;             // a burst-inhibited line, as long words
logic [15:0] dly;
logic [15:0] rl, ri;          // repetitions left, this one's number
logic [31:0] aoff;            // its address offset

wire  [4:0] ctl    = x_ctl[k];
wire        c_rd   = ctl[0];
wire  [1:0] c_siz  = ctl[2:1];
wire        c_line = (c_siz == 2'b11);
wire [31:0] xa     = x_addr[k] + aoff;
wire  [1:0] a32    = xa[3:2] + beat;          // the long word of this beat

function automatic logic [31:0] lw(input logic [127:0] l, input logic [1:0] i);
	lw = l[127 - 32 * i -: 32];
endfunction

always_ff @(posedge clk) begin
	if (!nreset) begin
		st <= A_IDLE; nx <= '0; k <= '0; beat <= '0; inh <= 1'b0; dly <= '0;
		rl <= '0; ri <= '0; aoff <= '0;
		status <= 2'd0;
		cpu_bg_n <= 1'b0; cpu_bgr_n <= 1'b1;
		drive <= 1'b0; a <= '0; rw_n <= 1'b1; siz <= '0; tt <= '0; tm <= '0; snoop_n <= 1'b1;
		ts_n <= 1'b1; bb_n <= 1'b1; btt_n <= 1'b1; d <= '0; d_oe <= 1'b0;
		for (int i = 0; i < 4; i++) x_rd[i] <= '0;
	end
	else begin
		if (go && st == A_IDLE) begin
			nx     <= n;
			k      <= 2'd0;
			rl     <= rep;
			ri     <= '0;
			aoff   <= '0;
			status <= 2'd0;
			st     <= trig_en ? A_TRIG : A_REQ;
		end
		if (st == A_TRIG && trig) st <= A_REQ;

		if (bclk_en) begin
			case (st)
			A_REQ: begin
				// take BG away; the bus is ours once the 68060 lets BB go
				cpu_bg_n  <= 1'b1;
				cpu_bgr_n <= !bgr_mode;
				if (cpu_bg_n && cpu_bb_n) begin
					bb_n  <= btt_mode;
					drive <= 1'b1;
					dly   <= delay;
					st    <= A_OWN;
				end
			end
			A_OWN: begin
				if (dly != 16'd0) dly <= dly - 16'd1;
				else begin
					beat <= 2'd0;
					inh  <= 1'b0;
					st   <= A_TS;
				end
			end
			A_TS: begin
				ts_n <= 1'b0;
				// (BTT protocol: the bus is granted onwards with the last one)
				if (btt_mode && {1'b0, k} + 3'd1 >= nx && rl == 16'd0) begin
					cpu_bg_n  <= 1'b0;
					cpu_bgr_n <= 1'b1;
				end
				a    <= inh ? {xa[31:4], a32, 2'b00} : xa;
				rw_n <= c_rd;
				siz  <= inh ? 2'b00 : c_siz;
				tt   <= 2'd0;
				tm   <= 3'd1;
				snoop_n <= (ctl[4:3] == 2'b00);
				d    <= c_line ? lw(x_wd[k], a32) : x_wd[k][127:96] + {16'd0, ri};
				d_oe <= !c_rd;
				st   <= A_DATA;
			end
			A_DATA: begin
				ts_n <= 1'b1;
				if (!ts_n) begin
					// TS is out this bus clock: the slave samples it now
				end
				else if (!ta_n && !tea_n) begin
					if (beat == 2'd0 && !inh) st <= A_TS;      // retry
					else begin status <= 2'd2; st <= A_REL; end
				end
				else if (!tea_n) begin
					status <= 2'd2;
					st     <= A_REL;
				end
				else if (!ta_n) begin
					if (c_rd) begin
						if (c_line) x_rd[k][127 - 32 * a32 -: 32] <= d_bus;
						else x_rd[k][127:96] <= d_bus;
					end
					if (c_line && beat != 2'd3) begin
						beat <= beat + 2'd1;
						if (inh || (beat == 2'd0 && !tbi_n)) begin
							inh <= 1'b1;
							st  <= A_TS;
						end
						else d <= lw(x_wd[k], a32 + 2'd1);
					end
					else if ({1'b0, k} + 3'd1 < nx) begin
						k    <= k + 2'd1;
						beat <= 2'd0;
						inh  <= 1'b0;
						st   <= A_TS;
					end
					else if (rl != 16'd0) begin
						rl   <= rl - 16'd1;
						ri   <= ri + 16'd1;
						aoff <= aoff + step;
						k    <= 2'd0;
						beat <= 2'd0;
						inh  <= 1'b0;
						st   <= A_TS;
					end
					else begin
						status <= 2'd1;
						st     <= A_REL;
					end
				end
			end
			A_REL: begin
				d_oe  <= 1'b0;
				bb_n  <= 1'b1;           // negate BB, BTT for one bus clock,
				btt_n <= 1'b0;           // then let the lines go
				st    <= A_REL2;
			end
			A_REL2: begin
				btt_n     <= 1'b1;
				drive     <= 1'b0;
				snoop_n   <= 1'b1;
				cpu_bg_n  <= 1'b0;
				cpu_bgr_n <= 1'b1;
				st        <= A_IDLE;
			end
			default: ;
			endcase
		end
	end
end

endmodule
