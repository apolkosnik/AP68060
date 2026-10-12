//--------------------------------------------------------------------------//
// AP68060 test bench                                                     //
//                                                                          //
// m68060_bus_slave.sv - memory on the 68060 bus (MC68060UM section 7)       //
//                                                                          //
// A synchronous slave: TS is sampled on a bus clock edge and TA/TEA/TRA/   //
// TBI/TCI/CLA are driven from flip-flops updated on bus clock edges.  A    //
// burst continues one beat per clock.  The slave counts the beats of a     //
// line itself and wraps inside the line; A3:A2 must stay as driven with   //
// TS (cla_mode 0) or advance once per CLA the processor samples (cla_mode //
// 1: CLA comes with every TA of a burst but the last, and at random in    //
// its wait and ignored clocks and with a TBI, 7.7.7).  The long-word      //
// transfers that finish a burst-inhibited line must move the long words   //
// after the first one, in order, wrapping in the line (7.7.2, 7.7.4).     //
//                                                                          //
// The special modes the processor registered at reset (7.14) are inputs: //
//   m_ign/m_cnt  the slave counts the ignore states of every beat of a    //
//                transfer the processor runs and answers no earlier than  //
//                the first clock the processor samples in; with ign_junk  //
//                it drives TA, TEA and TRA asserted in every ignored      //
//                clock (they must not count)                               //
//   m_native     a retry is TRA alone (else TA with TEA)                   //
//   m_hold       after a write beat's TA the data must stay driven one    //
//                more clock; that clock is no sampling clock              //
// Write data is taken on the edge the processor samples TA on.            //
//                                                                          //
// Protocol checks on the processor's transfers (perr pulses, with a       //
// message): BS3-BS0 match SIZ and A1:A0 (table 2-6), SAS is asserted in   //
// exactly the first sampling clock of every beat (2.4.3, 7.14.1), A3:A2   //
// follow the CLA rule, and the extra write hold keeps D31-D0 driven and   //
// unchanged.                                                               //
//                                                                          //
// Stress controls (set by the bench, may change between runs):             //
//   wait_mode  0: no wait states, 1: 0..3 random wait states per beat      //
//   tbi_mode   0: bursts allowed, 1: TBI on every line, 2: random TBI      //
//   retry_pct  percent chance of a retry on a retryable beat               //
//   tea_req    asserted by the bench while it wants the transfer that is   //
//              being answered to end in TEA (sampled when the beat would   //
//              be acknowledged)                                            //
//   hold       no answer while high: wait states                           //
//                                                                          //
// Every completed beat is published on the ev_* outputs for one clk so a   //
// bench can implement MMIO and protocol checks (a read when it is         //
// answered, a write when its TA is sampled, a bus error when answered).   //
//                                                                          //
// EXT = 1: the bench owns the memory.  Reads return ext_rdata for the      //
// beat's address (xfer_addr), an address with ext_inmem low ends in TEA,   //
// and writes are left to the bench (it applies the ev_* write events).     //
//--------------------------------------------------------------------------//

module m68060_bus_slave #(
	parameter int  AW   = 24,          // memory size 2**AW bytes at address 0
	parameter int  SEED = 1,
	parameter bit  EXT  = 0            // memory supplied by the bench
)(
	input  logic        clk,
	input  logic        nreset,
	input  logic        bclk_en,

	input  logic [31:0] a,
	input  logic [31:0] d_cpu,
	input  logic        d_oe_cpu,      // the processor drives D31-D0
	input  logic        rw_n,
	input  logic  [1:0] siz,
	input  logic  [3:0] bs_n,          // BS3-BS0 (bs_n[0] = BS0, D31-D24)
	input  logic  [1:0] tt,
	input  logic  [2:0] tm,
	input  logic        ts_n,
	input  logic        sas_n,
	input  logic        cpu_xfer,      // the processor (not another master) drives TS
	output logic [31:0] d_mem,
	output logic        ta_n,
	output logic        tea_n,
	output logic        tra_n,
	output logic        tbi_n,
	output logic        tci_n,
	output logic        cla_n,
	output logic        avec_n,

	// the processor's modes (7.14)
	input  logic        m_ign,
	input  logic        m_native,
	input  logic        m_hold,
	input  logic [15:0] m_cnt,

	input  logic  [1:0] wait_mode,
	input  logic  [1:0] tbi_mode,
	input  int          retry_pct,
	input  logic        cla_mode,
	input  logic        ign_junk,
	input  logic        tea_req,
	input  logic        retry_req,     // retry this transfer's first beat
	input  logic        tci_req,
	input  logic        hold,          // insert wait states while high
	input  logic  [7:0] iack_vector,   // 0 = answer with AVEC
	input  logic [31:0] ext_rdata,     // EXT: the long word at xfer_addr
	input  logic        ext_inmem,     // EXT: xfer_addr is backed by memory

	// the transfer being answered (valid while xfer_v)
	output logic        xfer_v,
	output logic [31:0] xfer_addr,     // address of the current beat
	output logic        xfer_rd,
	output logic  [1:0] xfer_siz,
	output logic  [1:0] xfer_tt,
	output logic  [2:0] xfer_tm,
	output logic  [1:0] xfer_beat,

	// completed-beat event (one clk)
	output logic        ev,
	output logic        ev_rd,
	output logic [31:0] ev_addr,       // long-word aligned for lines
	output logic [31:0] ev_data,
	output logic  [3:0] ev_be,         // byte lanes D31-24 .. D7-0
	output logic  [1:0] ev_siz,
	output logic  [1:0] ev_tt,
	output logic  [2:0] ev_tm,
	output logic        ev_err,        // the beat ended in TEA

	output logic        perr           // (one clk) a protocol check failed
);

localparam int NW = 1 << (AW - 2);

logic [31:0] mem [0:NW-1];

logic        busy;
logic [31:0] base;          // address as driven with TS
logic        rd_q;
logic  [1:0] siz_q, tt_q;
logic  [2:0] tm_q;
logic        cpu_q;         // the processor's transfer
logic  [1:0] beat;
logic  [2:0] waits;
logic        burst;          // still a burst (no TBI given)
logic  [1:0] inh_left;       // long-word transfers left of a burst-inhibited line
logic  [4:0] ic;             // ignored clocks left in this beat, from the next one
logic        exp_sas;        // the clock now running is the beat's first sampling clock
logic        sampling;       // the processor samples terminations this beat
logic        junk;           // the terminations now driven are ignore-state junk
logic        ack;            // a real termination is driven this clock
logic        wr_pend;        // ... a write TA: take the data when it is sampled
logic [31:0] wr_addr;
logic  [3:0] wr_be;
logic        hold_chk;       // check the extra write hold this clock
logic [31:0] hold_d;
logic        cla_q;          // CLA asserted with this TA
logic  [1:0] cla_cnt;        // CLAs the processor sampled in this line
logic [31:0] ln_base;        // the burst-inhibited line's first address

// byte lanes selected by SIZ/A1/A0 (table 2-6), D31-24 in bit 3
function automatic logic [3:0] lanes(input logic [1:0] s, input logic [1:0] a10);
	case (s)
		2'b01: lanes = 4'b1000 >> a10;
		2'b10: lanes = a10[1] ? 4'b0011 : 4'b1100;
		default: lanes = 4'b1111;
	endcase
endfunction

// BS3-BS0 for those lanes (bs_n[0] = BS0 = D31-D24), asserted low
function automatic logic [3:0] bs_of(input logic [3:0] l);
	bs_of = ~{l[0], l[1], l[2], l[3]};
endfunction

function automatic logic [2:0] pick_waits();
	if (wait_mode == 2'd0) return 3'd0;
	return 3'($urandom % 4);
endfunction

// ignored clocks of a beat of the processor's transfer: the primary or
// secondary count (7.14.1), plus the clock of an extra write hold
function automatic logic [4:0] ign_clocks(input logic rd, input logic secondary, input logic held);
	logic [3:0] n;
	if (!m_ign) n = 4'd0;
	else case ({rd, secondary})
		2'b10:   n = m_cnt[15:12];
		2'b11:   n = m_cnt[11:8];
		2'b00:   n = m_cnt[7:4];
		default: n = m_cnt[3:0];
	endcase
	ign_clocks = {1'b0, n} + {4'd0, held};
endfunction

wire  [1:0] a32   = base[3:2] + beat;
wire [31:0] baddr = (siz_q == 2'b11) ? {base[31:4], a32, 2'b00} : base;
wire        inmem = EXT ? ext_inmem : ((baddr >> AW) == 0);
wire [31:0] rword = EXT ? ext_rdata : mem[baddr[AW-1:2]];

assign xfer_v    = busy;
assign xfer_addr = baddr;
assign xfer_rd   = rd_q;
assign xfer_siz  = siz_q;
assign xfer_tt   = tt_q;
assign xfer_tm   = tm_q;
assign xfer_beat = beat;

initial begin
	void'($urandom(SEED));
end

// CLA in a clock that is not the beat's answer: at random, in a burst
// (CLA mode)
function automatic logic early_cla();
	return cla_mode && cpu_q && burst && siz_q == 2'b11 && inh_left == 2'd0 &&
	       ($urandom % 2) == 0;
endfunction

// a beat starts on this edge: its ignored clocks, junk for the first one
task automatic beat_start(input logic [4:0] rem);
	ic       <= (rem != 0) ? rem - 5'd1 : 5'd0;
	sampling <= (rem == 0);
	exp_sas  <= (rem == 0);
	if (rem != 0 && ign_junk) begin
		ta_n <= 1'b0; tea_n <= 1'b0; tra_n <= 1'b0; junk <= 1'b1;
	end
endtask

task automatic take_ts();
	busy  <= 1'b1;
	base  <= a;
	rd_q  <= rw_n;
	siz_q <= siz;
	tt_q  <= tt;
	tm_q  <= tm;
	cpu_q <= cpu_xfer;
	beat  <= 2'd0;
	cla_cnt <= 2'd0;
	// the rest of a burst-inhibited line: the next long word of it
	if (cpu_xfer && inh_left != 2'd0) begin
		logic [1:0] k;
		k = 2'd0 - inh_left;     // (4 - inh_left: the first follow-up is 1)
		if (a != {ln_base[31:4], ln_base[3:2] + k, 2'b00} || siz != 2'b00) begin
			$display("PROTOCOL: burst-inhibited line at %08x: transfer %0d at %08x SIZ %0d, expected %08x long",
			         ln_base, k, a, siz, {ln_base[31:4], ln_base[3:2] + k, 2'b00});
			perr <= 1'b1;
		end
	end
	burst <= (siz == 2'b11);
	waits <= pick_waits();
	if (cpu_xfer) begin
		beat_start(ign_clocks(rw_n, 1'b0, 1'b0));
		if (bs_n != bs_of(lanes(siz, a[1:0]))) begin
			$display("PROTOCOL: BS3-BS0 %b for SIZ %0d A1:A0 %0d (a=%08x), expected %b",
			         bs_n, siz, a[1:0], a, bs_of(lanes(siz, a[1:0])));
			perr <= 1'b1;
		end
	end
	else begin
		ic <= '0; sampling <= 1'b1; exp_sas <= 1'b0;
	end
endtask

always_ff @(posedge clk) begin
	ev   <= 1'b0;
	perr <= 1'b0;
	if (!nreset) begin
		busy   <= 1'b0;
		ta_n   <= 1'b1;
		tea_n  <= 1'b1;
		tra_n  <= 1'b1;
		tbi_n  <= 1'b1;
		tci_n  <= 1'b1;
		cla_n  <= 1'b1;
		avec_n <= 1'b1;
		d_mem  <= '0;
		beat   <= 2'd0;
		waits  <= 3'd0;
		burst  <= 1'b0;
		inh_left <= 2'd0;
		ic     <= '0;
		exp_sas <= 1'b0;
		sampling <= 1'b0;
		junk   <= 1'b0;
		ack    <= 1'b0;
		wr_pend <= 1'b0;
		hold_chk <= 1'b0;
		cla_q  <= 1'b0;
		cla_cnt <= 2'd0;
		ln_base <= '0;
		cpu_q  <= 1'b0;
	end
	else if (bclk_en) begin
		ta_n   <= 1'b1;
		tea_n  <= 1'b1;
		tra_n  <= 1'b1;
		tbi_n  <= 1'b1;
		tci_n  <= 1'b1;
		cla_n  <= 1'b1;
		avec_n <= 1'b1;
		junk   <= 1'b0;
		ack    <= 1'b0;
		wr_pend <= 1'b0;
		hold_chk <= 1'b0;
		cla_q  <= 1'b0;
		exp_sas <= 1'b0;
		// the processor samples the CLA we drive at this edge
		if (busy && !cla_n) cla_cnt <= cla_cnt + 2'd1;

		// the extra data write hold: the clock after a write TA keeps the data
		if (hold_chk && (!d_oe_cpu || d_cpu != hold_d)) begin
			$display("PROTOCOL: write data not held after TA (d_oe=%b d=%08x, was %08x)",
			         d_oe_cpu, d_cpu, hold_d);
			perr <= 1'b1;
		end
		// SAS: asserted in exactly the beat's first sampling clock
		if (busy && cpu_q && (!sas_n) != exp_sas) begin
			$display("PROTOCOL: SAS %0s in a clock of beat %0d of %08x (expected %0s)",
			         sas_n ? "negated" : "asserted", beat, base, exp_sas ? "asserted" : "negated");
			perr <= 1'b1;
		end
		if (!busy && !ts_n && cpu_xfer && !sas_n) begin
			$display("PROTOCOL: SAS asserted with TS");
			perr <= 1'b1;
		end

		if (!busy) begin
			if (!ts_n) take_ts();
		end
		else if (ack) begin
			// the edge on which the processor sampled our termination
			if (wr_pend) begin
				// write data is taken now, the clock TA was sampled in
				if (!EXT)
					for (int i = 0; i < 4; i++)
						if (wr_be[3 - i])
							mem[wr_addr[AW-1:2]][31 - 8*i -: 8] <= d_cpu[31 - 8*i -: 8];
				ev      <= 1'b1;
				ev_err  <= 1'b0;
				ev_rd   <= 1'b0;
				ev_addr <= wr_addr;
				ev_data <= d_cpu;
				ev_be   <= wr_be;
				ev_siz  <= siz_q;
				ev_tt   <= tt_q;
				ev_tm   <= tm_q;
				if (cpu_q && m_hold) begin
					hold_chk <= 1'b1;
					hold_d   <= d_cpu;
				end
			end
			if (ta_n == 1'b0 && tea_n == 1'b1 && tra_n == 1'b1 && burst && beat != 2'd3) begin
				beat  <= beat + 1'd1;
				waits <= pick_waits();
				if (cpu_q) beat_start(ign_clocks(rd_q, 1'b1, !rd_q && m_hold));
			end
			else begin
				busy <= 1'b0;
				if (!ts_n) take_ts();   // back-to-back: TS came with our TA
			end
		end
		else if (cpu_q && ic != 0) begin
			// the next clock is an ignore state (or the hold clock): the
			// processor does not look
			ic <= ic - 5'd1;
			if (ign_junk) begin
				ta_n <= 1'b0; tea_n <= 1'b0; tra_n <= 1'b0; junk <= 1'b1;
			end
			// CLA is sampled in the ignored clocks too
			if (early_cla()) cla_n <= 1'b0;
		end
		else if (waits != 0 || hold) begin
			// wait states (the bench may hold a beat)
			if (waits != 0) waits <= waits - 1'd1;
			if (cpu_q && !sampling) begin sampling <= 1'b1; exp_sas <= 1'b1; end
			if (early_cla()) cla_n <= 1'b0;
		end
		else begin
			// answer this beat
			logic retry;
			logic [3:0] be;
			if (cpu_q && !sampling) begin sampling <= 1'b1; exp_sas <= 1'b1; end
			// the A3:A2 rule: held as driven with TS, or advanced by CLA
			if (cpu_q && siz_q == 2'b11 && inh_left == 2'd0 &&
			    a[3:2] != (cla_mode ? base[3:2] + cla_cnt : base[3:2])) begin
				$display("PROTOCOL: A3:A2 = %0d in beat %0d of the line at %08x (CLA %s, %0d sampled)",
				         a[3:2], beat, base, cla_mode ? "on" : "off", cla_cnt);
				perr <= 1'b1;
			end
			// a burst-inhibited line's follow-up transfers may not be retried
			// (an interrupt acknowledge may be retried too; the LPSTOP and
			// breakpoint acknowledges, TM 0, are not)
			retry = (beat == 2'd0) && (inh_left == 2'd0) && !(tt_q == 2'd3 && tm_q == 3'd0) &&
			        (retry_req || ((retry_pct > 0) && (($urandom % 100) < retry_pct)));
			be = lanes(siz_q, base[1:0]);
			ack <= 1'b1;
			if (tea_req || (!inmem && tt_q != 2'd3)) begin
				tea_n  <= 1'b0;
				inh_left <= 2'd0;        // (a bus error ends the line)
				ev     <= 1'b1;
				ev_err <= 1'b1;
				ev_rd   <= rd_q;
				ev_addr <= baddr;
				ev_data <= rd_q ? rword : d_cpu;
				ev_be   <= be;
				ev_siz  <= siz_q;
				ev_tt   <= tt_q;
				ev_tm   <= tm_q;
			end
			else if (retry) begin
				if (m_native && cpu_q) tra_n <= 1'b0;
				else begin ta_n <= 1'b0; tea_n <= 1'b0; end
			end
			else begin
				ta_n <= 1'b0;
				if (tt_q == 2'd3) begin
					// interrupt or breakpoint acknowledge
					if (iack_vector == 8'd0) avec_n <= 1'b0;
					d_mem <= {24'd0, iack_vector};
				end
				else if (rd_q) begin
					d_mem <= rword;
				end
				if (rd_q || tt_q == 2'd3) begin
					ev      <= 1'b1;
					ev_err  <= 1'b0;
					ev_rd   <= rd_q;
					ev_addr <= baddr;
					ev_data <= rword;
					ev_be   <= be;
					ev_siz  <= siz_q;
					ev_tt   <= tt_q;
					ev_tm   <= tm_q;
				end
				else begin
					wr_pend <= 1'b1;
					wr_addr <= baddr;
					wr_be   <= be;
				end
				if (inh_left != 2'd0) inh_left <= inh_left - 2'd1;
				if (beat == 2'd0 && siz_q == 2'b11 && (tbi_mode == 2'd1 ||
				    (tbi_mode == 2'd2 && ($urandom % 2) == 0))) begin
					tbi_n <= 1'b0;
					burst <= 1'b0;
					inh_left <= 2'd3;
					ln_base <= base;
					// CLA may come with the TBI: it does not move the
					// long words that finish the line
					if (cla_mode && cpu_q && ($urandom % 2) == 0) cla_n <= 1'b0;
				end
				else if (cla_mode && cpu_q && burst && beat != 2'd3) begin
					cla_n <= 1'b0;
					cla_q <= 1'b1;
				end
				if (beat == 2'd0) tci_n <= !tci_req;
			end
		end
	end
end

endmodule
