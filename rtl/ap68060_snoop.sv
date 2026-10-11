//--------------------------------------------------------------------------//
// AP68060 - pipelined MC68060                                              //
//                                                                          //
// ap68060_snoop.sv - bus snooper (MC68060UM 2.6, 5.6, 7.12)                 //
//                                                                          //
// While another master owns the bus, a transfer it starts (TS) with       //
// SNOOP asserted and TT1 = 0 (a normal or MOVE16 transfer) is snooped:    //
// the line at A31-A4 is invalidated in both caches, whatever the          //
// direction, size or the line's state -- a dirty line's data is lost (the //
// 68060 needs shared pages write-through or cache-inhibited).  Nothing is //
// driven on the bus: no memory inhibit, no intervention, no termination   //
// needed for the snooped transfer.  A3:A0 do not matter.                  //
//                                                                          //
// The invalidations wait in queues (the caches are busy with the          //
// processor's own work in between): the data cache's through the DMU's    //
// snoop port, the instruction cache's through the fetch unit's            //
// invalidation request.  A line already queued is not queued again; a     //
// full instruction-cache queue turns into an invalidation of every line   //
// (more than the 68060 drops, never less: its contents are only copies),  //
// a full data-cache queue holds the oldest entries and records the        //
// overflow (dc_ovf) for the bench, as dropping one would be wrong.        //
//--------------------------------------------------------------------------//

module ap68060_snoop
	import ap68060_pkg::*;
#(
	parameter int DQ = 8            // data cache queue depth
)(
	input  logic        clk,
	input  logic        nreset,
	input  logic        bclk_en,
	input  logic        own,            // BIU: we drive the bus

	// the bus, as the alternate master drives it
	input  logic [31:0] a_i,
	input  logic        ts_n_i,
	input  logic  [1:0] tt_i,
	input  logic        snoop_n,

	// data cache: the DMU's snoop port (invalidate what is found)
	output logic        dc_req,
	output logic [31:0] dc_pa,
	input  logic        dc_look,
	input  logic        dc_hit,
	output logic        dc_inv,
	output logic        dc_ovf,         // (simulation) the queue overflowed
	output logic        busy,           // invalidations still to be made

	// instruction cache: line invalidation in the fetch unit
	output logic        ic_req,
	output logic [31:0] ic_pa,
	output logic        ic_all,         // ... or every line (queue overflow)
	input  logic        ic_done
);

// a snooped transfer starts on this bus clock edge
wire        snoop = bclk_en && !own && !ts_n_i && !snoop_n && !tt_i[1];
wire [27:0] nl    = a_i[31:4];

//--------------------------------------------------------------------------
// data cache queue
//--------------------------------------------------------------------------
logic [27:0] dq [DQ];
logic [$clog2(DQ):0] dn;
logic        dbusy;             // the head is being looked up

always_ff @(posedge clk) begin
	dc_inv <= 1'b0;
	if (!nreset) begin
		dn <= '0; dbusy <= 1'b0; dc_req <= 1'b0; dc_pa <= '0; dc_ovf <= 1'b0;
		for (int i = 0; i < DQ; i++) dq[i] <= '0;
	end
	else begin
		logic [$clog2(DQ):0] n;
		logic dup, pop;
		n = dn;
		pop = dbusy && dc_look;
		// the head's lookup: invalidate on a hit, then the next entry
		if (pop) begin
			dc_inv <= dc_hit;
			dc_req <= 1'b0;
			dbusy  <= 1'b0;
			for (int i = 0; i < DQ - 1; i++) dq[i] <= dq[i + 1];
			n = n - 1'd1;
		end
		if (snoop) begin
			// (a head whose lookup ends now does not cover a new snoop)
			dup = 1'b0;
			for (int i = 0; i < DQ; i++)
				if (i < dn && !(pop && i == 0) && dq[i] == nl) dup = 1'b1;
			if (!dup) begin
				if (n < DQ) begin
					dq[n] <= nl;
					n = n + 1'd1;
				end
				else dc_ovf <= 1'b1;
			end
		end
		dn <= n;
		// the head's lookup starts at once -- a snoop into an empty queue
		// in the clock it is seen (a snoop has priority, 5.6).  Not in the
		// clock a lookup ends: the DMU's port waits for the request to drop.
		if (!dbusy && n != 0) begin
			dc_req <= 1'b1;
			dc_pa  <= {(dn != 0) ? dq[0] : nl, 4'd0};
			dbusy  <= 1'b1;
		end
	end
end

// the data cache's invalidations hold the processor's next transfer (a
// push of a line a snoop hit must not go out); the instruction cache's do
// not, the fetch unit making them only outside a line fill
assign busy = (dn != 0) || dc_req || dbusy || snoop;

//--------------------------------------------------------------------------
// instruction cache queue: two entries, overflow -> every line
//--------------------------------------------------------------------------
logic        iq_v1, iq_all;
logic [27:0] iq_l1;
always_ff @(posedge clk) begin
	if (!nreset) begin
		ic_req <= 1'b0; ic_pa <= '0; ic_all <= 1'b0;
		iq_v1 <= 1'b0; iq_all <= 1'b0; iq_l1 <= '0;
	end
	else begin
		logic        v0, a0, v1, ov;
		logic [27:0] l0, l1;
		v0 = ic_req; a0 = ic_all; l0 = ic_pa[31:4];
		v1 = iq_v1;  ov = iq_all; l1 = iq_l1;
		if (ic_done) begin
			// the head is done: the second entry moves up
			v0 = v1 || ov; a0 = ov; l0 = l1;
			v1 = 1'b0; ov = 1'b0;
		end
		if (snoop) begin
			if (!v0) begin v0 = 1'b1; a0 = 1'b0; l0 = nl; end
			else if (a0 || l0 == nl || ov || (v1 && l1 == nl)) ;   // covered
			else if (!v1) begin v1 = 1'b1; l1 = nl; end
			else ov = 1'b1;
		end
		ic_req <= v0; ic_all <= a0; ic_pa <= {l0, 4'd0};
		iq_v1  <= v1; iq_all <= ov; iq_l1 <= l1;
	end
end

endmodule
