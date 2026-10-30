//--------------------------------------------------------------------------//
// AP68060 - pipelined MC68040                                            //
//                                                                          //
// ap68060_atc.sv - address translation cache (MC68060UM 4.3)                //
//                                                                          //
// 64 entries, four-way set associative, 16 sets indexed by LA15-12 (4K     //
// pages) or LA16-13 (8K pages).  The tag is FC2 and the logical page; the  //
// entry holds the physical page and the page attributes.  Valid and global //
// bits are flip-flops (PFLUSHA/PFLUSHAN in one cycle); tags and entries    //
// are MLAB (asynchronous read).                                            //
//                                                                          //
// Lookup: la/fc2 are the caller's registered address; the four ways are   //
// compared here and the hit way's entry returned.  Replacement takes an    //
// existing entry for the same page, else the first invalid way, else the  //
// way a 2-bit counter points to (the counter advances on every miss fill).//
// In the 1/2-cache mode (TCR FOTC/FITC) only ways 0 and 1 take new         //
// entries: 32 of them.                                                     //
// PFLUSH by page reads the set through the lookup port: the caller holds  //
// the lookup inputs on the page for one cycle while flush_page is high.   //
//--------------------------------------------------------------------------//

module ap68060_atc
	import ap68060_pkg::*;
#(
	// an install looks for its page among the set's entries (a data
	// access's M-bit update rewrites its own entry).  0: no copy for the
	// search -- the caller says whether the page is there (wsame) and in
	// which way (the hit vector of the lookup that started the search)
	parameter bit SAME_SEARCH = 1,
	parameter bit SET_IN = 0           // the first port's set from the set input
)
(
	input  logic        clk,
	input  logic        nreset,
	input  logic        p8k,           // TC.P: 8 Kbyte pages
	input  logic        half,          // TC.FOTC/FITC: 32 entries (ways 0, 1)

	// lookup
	input  logic [31:0] la,
	input  logic  [3:0] set,           // (SET_IN) la's set, registered with it
	input  logic        set_v,         // ... and that the port is in use (check)
	input  logic        fc2,
	output logic        hit,
	output atce_t       ent,
	output logic  [3:0] hv,            // the hit way (one-hot), and every way's
	output logic [19:0] wpa [4],       // physical page: a caller's tag compares
	                                   // run against all four in parallel
	// a second lookup port (the DMU's DC1: its own copy of the entries, so
	// no address multiplexer in front of it): its set is given registered
	input  logic [31:0] la_d,
	input  logic        fc2_d,
	input  logic  [3:0] set_d,
	output logic  [3:0] hv_d,
	output logic [19:0] wpa_d [4],
	output atce_t       ent_d [4],     // ... and every way's entry

	// install (one cycle)
	input  logic        wr,
	input  logic [31:0] wla,
	input  logic        wfc2,
	input  atce_t       went,
	input  logic        wsame,         // (SAME_SEARCH 0) the page is in the set ...
	input  logic  [3:0] wsame_hv,      // ... in this way (one-hot)

	// flushes (one cycle each)
	input  logic        flush_all,      // PFLUSHA / PFLUSHAN
	input  logic        flush_page,     // PFLUSH / PFLUSHN (la/fc2 = the page)
	input  logic        flush_nonglobal // the N variants spare global entries
);

typedef struct packed {
	logic        fc2;
	logic [19:0] lpn;     // LA31-12 (bit 12 ignored with 8K pages)
	atce_t       e;
} slot_t;

localparam int SW = $bits(slot_t);

logic [3:0] v   [16];     // valid, per set and way
logic [3:0] g   [16];     // global
logic [1:0] rr;           // replacement counter
logic       fp_v;         // a page flush's clear: pending, its set and ways
logic [3:0] fp_s, fp_m;

// (SET_IN: the caller registers the first port's set with la)
wire [3:0] lset = SET_IN ? set : (p8k ? la[16:13] : la[15:12]);
// synthesis translate_off
always_ff @(posedge clk)
	if (SET_IN && nreset && set_v && set !== (p8k ? la[16:13] : la[15:12])) begin
		$display("FAIL: ATC: the first port's set %0d, its address's %0d", set, p8k ? la[16:13] : la[15:12]);
		$finish;
	end
// synthesis translate_on
wire [3:0] wset = p8k ? wla[16:13] : wla[15:12];

slot_t s [4];
slot_t ws [4];
// synthesis translate_off
logic [$bits(slot_t)-1:0] wqs [4];      // SAME_SEARCH 0: the simulation's copy
// synthesis translate_on
slot_t sd [4];

genvar w;
generate
	for (w = 0; w < 4; w++) begin : g_way
		logic [SW-1:0] q, wq;
		ap68060_lutram #(.AW(4), .DW(SW)) ram (
			.clk(clk), .we(wr && wway == 2'(w)), .waddr(wset),
			.wdata({wfc2, wla[31:12], went}), .raddr(lset), .q(q)
		);
		if (SAME_SEARCH) begin : g_ws
			ap68060_lutram #(.AW(4), .DW(SW)) ram_w (
				.clk(clk), .we(wr && wway == 2'(w)), .waddr(wset),
				.wdata({wfc2, wla[31:12], went}), .raddr(wset), .q(wq)
			);
		end
		else begin : g_nows
			assign wq = '0;
			// synthesis translate_off
			// (the copy, in simulation only: an install that finds its page
			// already there would duplicate it)
			ap68060_lutram #(.AW(4), .DW(SW)) ram_ws (
				.clk(clk), .we(wr && wway == 2'(w)), .waddr(wset),
				.wdata({wfc2, wla[31:12], went}), .raddr(wset), .q(wqs[w])
			);
			// synthesis translate_on
		end
		logic [SW-1:0] dq;
		ap68060_lutram #(.AW(4), .DW(SW)) ram_d (
			.clk(clk), .we(wr && wway == 2'(w)), .waddr(wset),
			.wdata({wfc2, wla[31:12], went}), .raddr(set_d), .q(dq)
		);
		assign s[w]  = q;
		assign ws[w] = wq;
		assign sd[w] = dq;
	end
endgenerate

function automatic logic tag_eq(input slot_t t, input logic [31:0] a, input logic f2,
                                input logic p8);
	tag_eq = (t.fc2 == f2) && (t.lpn[19:1] == a[31:13]) && (p8 || (t.lpn[0] == a[12]));
endfunction

always_comb for (int i = 0; i < 4; i++) wpa[i] = s[i].e.pa;
always_comb
	for (int i = 0; i < 4; i++) begin
		hv_d[i]  = v[set_d][i] && tag_eq(sd[i], la_d, fc2_d, p8k);
		wpa_d[i] = sd[i].e.pa;
		ent_d[i] = sd[i].e;
	end
always_comb begin
	for (int i = 0; i < 4; i++) hv[i] = v[lset][i] && tag_eq(s[i], la, fc2, p8k);
	hit = |hv;
	// at most one way holds a page (install reuses its way): AND-OR, no
	// priority chain
	ent = '0;
	for (int i = 0; i < 4; i++) ent = ent | (hv[i] ? s[i].e : '0);
end
// synthesis translate_off
always_ff @(posedge clk)
	if (nreset && !$onehot0(hv)) begin
		$display("FAIL: ATC: more than one way hits (set %0d, ways %b)", lset, hv);
		$finish;
	end
// synthesis translate_on

// install way: the same page if present (an M-bit update), else invalid, else rr
logic [1:0] wway;
always_comb begin
	logic found;
	found = 1'b0;
	wway  = rr;
	if (SAME_SEARCH) begin
		for (int i = 3; i >= 0; i--)
			if (v[wset][i] && tag_eq(ws[i], wla, wfc2, p8k)) begin wway = 2'(i); found = 1'b1; end
	end
	else if (wsame) begin
		found = 1'b1;
		wway  = wsame_hv[3] ? 2'd3 : wsame_hv[2] ? 2'd2 : wsame_hv[1] ? 2'd1 : 2'd0;
	end
	if (!found) begin
		if (half) wway = {1'b0, rr[0]};
		for (int i = 3; i >= 0; i--) if (!v[wset][i] && !(half && i >= 2)) wway = 2'(i);
	end
end

// synthesis translate_off
always_ff @(posedge clk)
	if (!SAME_SEARCH && wr)
		for (int i = 0; i < 4; i++)
			if ((v[wset][i] && tag_eq(slot_t'(wqs[i]), wla, wfc2, p8k)) != (wsame && wsame_hv[i])) begin
				$display("FAIL: ATC: install of %h: the page %s way %0d, the caller says %b/%b",
				         wla, (v[wset][i] && tag_eq(slot_t'(wqs[i]), wla, wfc2, p8k)) ? "is in" : "is not in",
				         i, wsame, wsame_hv);
				$finish;
			end
// synthesis translate_on

// synthesis translate_off
always_ff @(posedge clk)
	if (fp_v && (wr || flush_all)) begin
		$display("FAIL: ATC: an install or a flush of all in the clock a page flush clears");
		$finish;
	end
// synthesis translate_on

// ATC entries survive RSTI (MC68040UM 3.6.1): only configuration clears them
initial begin
	for (int i = 0; i < 16; i++) begin v[i] = 4'd0; g[i] = 4'd0; end
	rr = 2'd0;
	fp_v = 1'b0;
end

// the install's way as each set would choose it, from that set's own valid
// bits (wway's rule): the valid and global bits are written without the
// read of the install set's bits through the set select
function automatic logic [3:0] pick(input logic [3:0] vv);
	logic [3:0] p;
	if (wsame) p = wsame_hv;
	else begin
		p = 4'd0;
		if (half) p[{1'b0, rr[0]}] = 1'b1;
		else      p[rr] = 1'b1;
		for (int i = 3; i >= 0; i--)
			if (!vv[i] && !(half && i >= 2)) p = 4'b0001 << i;
	end
	pick = p;
endfunction
// synthesis translate_off
always_ff @(posedge clk)
	if (wr && pick(v[wset]) != (4'b0001 << wway)) begin
		$display("FAIL: ATC: the set's own install way %b, wway %0d", pick(v[wset]), wway);
		$finish;
	end
// synthesis translate_on

always_ff @(posedge clk) begin
	if (wr) begin
		for (int k = 0; k < 16; k++)
			if (wset == 4'(k)) begin
				v[k] <= v[k] | pick(v[k]);
				g[k] <= (g[k] & ~pick(v[k])) | (pick(v[k]) & {4{went.g}});
			end
		if (half ? (v[wset][1:0] == 2'b11) : (v[wset] == 4'hF)) rr <= rr + 2'd1;
	end
	if (flush_all)
		for (int i = 0; i < 16; i++)
			v[i] <= flush_nonglobal ? (v[i] & g[i]) : 4'd0;
	// (a page flush clears the entries it found a clock later, from
	// registers: PFLUSH serializes, so nothing that survives it looks up
	// in between)
	fp_v <= flush_page;
	fp_s <= lset;
	for (int i = 0; i < 4; i++)
		fp_m[i] <= v[lset][i] && tag_eq(s[i], la, fc2, p8k) && !(flush_nonglobal && g[lset][i]);
	if (fp_v)
		for (int k = 0; k < 16; k++)
			if (fp_s == 4'(k))
				v[k] <= (flush_all ? (flush_nonglobal ? (v[k] & g[k]) : 4'd0) : v[k]) & ~fp_m;
end

endmodule
