//--------------------------------------------------------------------------//
// ap060_tg68k_compat.v - the AP68060 in TG68K's place in cpu_wrapper       //
//                                                                          //
// The processor runs on clk_cpu (pll_cpu, unrelated to cpu_wrapper's clk)  //
// with BCLK = CLK.  Its bus is answered in two halves:                     //
//                                                                          //
//   clk_cpu  the 68060 bus (MC68060UM section 7): a transfer is latched on //
//            the clock TS is sampled, its write data the clock after (the  //
//            processor drives D31-D0 from then until TA); acknowledge      //
//            transfers (TT = 3) end in TA with AVEC (Minimig's interrupts   //
//            are autovectored): an interrupt acknowledge (a read, TM = the  //
//            level) first runs TG68K's acknowledge read on the Minimig bus, //
//            $FFFFFFF0 + level * 2 -- the Action Replay/HRTmon cartridge    //
//            arms on level 7's and drops its NMI request with it; the       //
//            breakpoint acknowledge and LPSTOP are answered here.  The rest //
//            go to the other half with a request                           //
//            toggle, and TA ends them the clock after its answer toggle     //
//            arrives.  A line (SIZ = line: cache fills, pushes, MOVE16)    //
//            gets TBI with that TA, so the processor runs the rest of it as //
//            long-word transfers (7.7.2); reads outside Fast RAM and       //
//            Kickstart ROM get TCI: chip RAM, I/O and the RTG frame buffer //
//            are never cached, so chipset DMA cannot leave a stale line in  //
//            the 68060                                                      //
//   clk      cpu_wrapper's 16-bit accesses, as TG68KdotC_Kernel makes them: //
//            a byte or word is one, a long word two (A, then A + 2, with    //
//            longword high on the first, as TG68K does: Gayle's 32-bit IDE //
//            data port latches it there); an access completes on the clock //
//            clkena_in is high, the next one or the idle bus state is       //
//            presented from the next clock                                  //
//                                                                          //
// The halves cross with toggles through three-stage synchronisers; each    //
// side holds its fields (address, size, write data; read data) steady from  //
// its toggle until the other side answers it, so only the toggles need     //
// synchronising (Minimig.sdc: the clocks are asynchronous).  IPL, reset,   //
// RSTO, VBR and the Fast RAM windows cross through synchronisers of their   //
// own; the last two change only by MOVEC and autoconfiguration.             //
//                                                                          //
// The 68060 bus is aligned: a long word never crosses a long-word         //
// boundary, a word never an odd address.  Read data goes back on both    //
// halves of D31-D0, so a byte or word reaches the lanes its address      //
// selects.  There is one bus master: the arbitration inputs grant the    //
// bus and the bus-busy inputs see the processor's own outputs.  No        //
// transfer ends in TEA; Minimig has no bus error.  During reset IPL2-IPL0 //
// and D15-D0 carry the reset modes (7.14): all negated / zero.             //
//--------------------------------------------------------------------------//

module ap060_tg68k_compat #(
	parameter NO_FPU = 1               // the MC68LC060
)(
	input             clk,
	input             clk_cpu,         // the processor's clock, unrelated to clk
	input             nreset,          // active low, cpu_wrapper's reset (clk)
	input             clkena_in,       // the current access completes this clock
	input      [15:0] data_in,
	input       [2:0] ipl,             // active low
	output reg [31:0] addr_out,
	output reg [15:0] data_write,
	output reg        nwr,
	output reg        nuds,
	output reg        nlds,
	output            nresetout,
	output reg        longword,
	output reg  [1:0] busstate,        // 0: fetch code, 1: no memaccess, 2: read data, 3: write data
	output      [3:0] cacr_out,
	output            d_cache_out,
	output     [31:0] vbr_out,

	// the Fast RAM windows as autoconfiguration placed them (cachable)
	input             z2ram_ena,
	input       [4:0] z3ram_base0,
	input             z3ram_ena0,
	input       [3:0] z3ram_base1,
	input             z3ram_ena1
);

localparam BS_FETCH = 2'b00, BS_IDLE = 2'b01, BS_READ = 2'b10, BS_WRITE = 2'b11;
localparam SIZ_L = 2'b00, SIZ_B = 2'b01, SIZ_W = 2'b10, SIZ_LINE = 2'b11;

//--------------------------------------------------------------------------
// synchronisers into clk_cpu
//--------------------------------------------------------------------------
reg  [2:0] nreset_c = 3'b000;          // reset, asserted at once, released synchronised
always @(posedge clk_cpu or negedge nreset)
	if (!nreset) nreset_c <= 3'b000;
	else         nreset_c <= {nreset_c[1:0], 1'b1};
wire nrst_c = nreset_c[2];

reg  [2:0] ipl_c1 = 3'b111, ipl_c = 3'b111;
reg  [1:0] z2e_c = 0, z3e0_c = 0, z3e1_c = 0;
reg  [4:0] z3b0_c1 = 0, z3b0_c = 0;
reg  [3:0] z3b1_c1 = 0, z3b1_c = 0;
always @(posedge clk_cpu) begin
	ipl_c1  <= ipl;          ipl_c  <= ipl_c1;
	z2e_c   <= {z2e_c[0], z2ram_ena};
	z3e0_c  <= {z3e0_c[0], z3ram_ena0};
	z3e1_c  <= {z3e1_c[0], z3ram_ena1};
	z3b0_c1 <= z3ram_base0;  z3b0_c <= z3b0_c1;
	z3b1_c1 <= z3ram_base1;  z3b1_c <= z3b1_c1;
end

//--------------------------------------------------------------------------
// the processor
//--------------------------------------------------------------------------
wire [31:0] a_o, d_o, cacr, vbr;
wire        d_oe, rw_n, ts_n, rsto_n;
wire  [1:0] siz, tt;
wire  [2:0] tm;
wire  [3:0] bs_n;
wire        bb_n_o, bb_oe, btt_n_o, btt_oe;
reg  [31:0] d_i;
reg         ta_n, tbi_n, tci_n, avec_n;

ap68060 #(.NO_FPU(NO_FPU)) cpu (
	.clk(clk_cpu), .bclk_en(1'b1), .rsti_n(nrst_c),
	.a_o(a_o), .a_oe(), .a_i(a_o), .ts_n_i(1'b1), .tt_i(tt), .snoop_n(1'b1),
	.d_i(nrst_c ? d_i : 32'd0), .d_o(d_o), .d_oe(d_oe),
	.rw_n(rw_n), .siz(siz), .bs_n(bs_n), .tt(tt), .tm(tm), .tln(), .upa(),
	.ciout_n(), .lock_n(), .locke_n(), .lock_oe(),
	.ts_n(ts_n), .tip_n(), .tip_oe(), .sas_n(), .sas_oe(),
	.ta_n(ta_n), .tea_n(1'b1), .tra_n(1'b1), .tci_n(tci_n), .tbi_n(tbi_n), .cla_n(1'b1),
	.ipl_n(nrst_c ? ipl_c : 3'b111), .avec_n(avec_n),
	.br_n(), .bg_n(1'b0), .bgr_n(1'b1),
	.bb_n_i(bb_oe ? bb_n_o : 1'b1), .bb_n_o(bb_n_o), .bb_oe(bb_oe),
	.btt_n_i(btt_oe ? btt_n_o : 1'b1), .btt_n_o(btt_n_o), .btt_oe(btt_oe),
	.rsto_n(rsto_n), .cdis_n(1'b1), .mdis_n(1'b1), .ipend_n(), .pst(),
	.dbg_pc(), .dbg_retire(), .dbg_halted(), .dbg_vbr(vbr), .dbg_cacr(cacr)
);

// RSTO and VBR into clk (VBR: MOVEC only; nmi_addr in cpu_wrapper)
reg  [1:0] rsto_k = 2'b11;
reg [31:0] vbr_k1 = 0, vbr_k = 0;
always @(posedge clk) begin
	rsto_k <= {rsto_k[0], rsto_n};
	vbr_k1 <= vbr;  vbr_k <= vbr_k1;
end
assign nresetout = rsto_k[1];
assign vbr_out   = vbr_k;

// cpu_wrapper's cache controls (TG68K's CACR) drive the memory controllers'
// own caches (cpu_cache_new in sdram_ctrl and ddram_ctrl).  Those are always
// on here, not tied to the 68060's CACR: a controller cache with its data
// side off neither fills nor updates its lines on stores, so turning it off
// and on again (exec and 68060.library switch EDC) would bring back lines
// older than memory -- and on a 68060 nothing can clear it (TG68K's CACR
// clear bit, cacr_out[3], has no 68060 counterpart).  They snoop chipset
// DMA and see every processor store, so they need no management.
assign cacr_out    = 4'b0001;
assign d_cache_out = 1'b1;

//--------------------------------------------------------------------------
// clk_cpu: the 68060 bus
//--------------------------------------------------------------------------
localparam C_IDLE = 2'd0, C_DATA = 2'd1, C_WAIT = 2'd2, C_ACK = 2'd3;
reg   [1:0] cst;
reg  [31:0] xa;                    // address as driven with TS     } held from the
reg   [1:0] xsiz;                  //                                } request toggle
reg         xrd;                   //                                } until its
reg         xcode;                 // TM: user / supervisor code     } answer
reg         xiack;                 // an interrupt acknowledge       }
reg  [31:0] wd;                    // write data                     }
reg         p_tgl = 1'b0;          // request toggle
reg         k_tgl = 1'b0;          // answer toggle (clk side)
reg  [31:0] k_rd;                  // read data (clk side, held from its toggle)
reg   [2:0] k_tgl_c = 3'b000;      // the answer toggle, synchronised
always @(posedge clk_cpu) k_tgl_c <= {k_tgl_c[1:0], k_tgl};
wire        answered = (k_tgl_c[2] == p_tgl);

// cachable: Fast RAM (Zorro II $200000-$9FFFFF, the Zorro III windows) and
// Kickstart ROM ($F80000-$FFFFFF, $E00000-$E7FFFF)
wire c_z2   = z2e_c[1] && (xa[31:24] == 8'h00) && (xa[23] ^ |xa[22:21]);
wire c_z30  = z3e0_c[1] && (xa[31:27] == z3b0_c);
wire c_z31  = z3e1_c[1] && (xa[31:28] == z3b1_c);
wire c_rom  = (xa[31:19] == 13'h001F) || (xa[31:19] == 13'h001C);
wire x_cach = c_z2 | c_z30 | c_z31 | c_rom;

// a transfer driven with TS this clock
task take_ts;
	begin
		xa    <= a_o;
		xsiz  <= siz;
		xrd   <= rw_n;
		xcode <= (tm == 3'd2) || (tm == 3'd6);
		xiack <= 1'b0;
		if (tt == 2'd3 && rw_n && tm != 3'd0) begin
			// interrupt acknowledge: TG68K's acknowledge read, a word at
			// $FFFFFFF0 + level * 2, then TA with AVEC
			xa    <= {28'hFFFFFFF, tm, 1'b0};
			xsiz  <= SIZ_W;
			xcode <= 1'b0;
			xiack <= 1'b1;
			cst   <= C_DATA;
		end
		else if (tt == 2'd3) begin
			// breakpoint acknowledge, LPSTOP: answered here
			ta_n   <= 1'b0;
			avec_n <= 1'b0;
			d_i    <= 32'd0;
			cst    <= C_ACK;
		end
		else cst <= C_DATA;
	end
endtask

always @(posedge clk_cpu or negedge nrst_c) begin
	if (!nrst_c) begin
		cst    <= C_IDLE;
		ta_n   <= 1'b1;
		tbi_n  <= 1'b1;
		tci_n  <= 1'b1;
		avec_n <= 1'b1;
		d_i    <= 32'd0;
		p_tgl  <= 1'b0;            // (and k_tgl is 0 in reset: nothing owed)
	end
	else begin
		ta_n   <= 1'b1;
		tbi_n  <= 1'b1;
		tci_n  <= 1'b1;
		avec_n <= 1'b1;
		case (cst)
			C_IDLE:
				if (!ts_n) take_ts;
			C_DATA: begin
				// the processor drives the write data from this clock
				wd    <= d_o;
				p_tgl <= ~p_tgl;
				cst   <= C_WAIT;
			end
			C_WAIT:
				if (answered) begin
					d_i    <= xiack ? 32'd0 : k_rd;
					ta_n   <= 1'b0;
					avec_n <= !xiack;
					tbi_n  <= !(xsiz == SIZ_LINE);
					tci_n  <= !(xrd && !x_cach);
					cst    <= C_ACK;
				end
			C_ACK:
				// TA is out this clock; a transfer may start with it
				if (!ts_n) take_ts;
				else cst <= C_IDLE;
		endcase
	end
end

//--------------------------------------------------------------------------
// clk: cpu_wrapper's 16-bit accesses
//--------------------------------------------------------------------------
reg   [2:0] p_tgl_k = 3'b000;      // the request toggle, synchronised
always @(posedge clk) p_tgl_k <= {p_tgl_k[1:0], p_tgl};
wire        req = (p_tgl_k[2] != k_tgl);

localparam K_IDLE = 2'd0, K_ACC = 2'd1, K_DONE = 2'd2;
reg   [1:0] kst;
reg         word1;                 // the second word of a long word
wire        x_long = (xsiz == SIZ_L) || (xsiz == SIZ_LINE);

// the 16-bit access for word w of the transfer
task present;
	input w;
	begin
		addr_out   <= (xsiz == SIZ_B) ? xa : {xa[31:2], (x_long ? w : xa[1]), 1'b0};
		busstate   <= !xrd ? BS_WRITE : xcode ? BS_FETCH : BS_READ;
		nwr        <= xrd;
		nuds       <= (xsiz == SIZ_B) && xa[0];
		nlds       <= (xsiz == SIZ_B) && !xa[0];
		longword   <= x_long && !w;
		data_write <= x_long ? (w ? wd[15:0] : wd[31:16]) : (xa[1] ? wd[15:0] : wd[31:16]);
		word1      <= w;
	end
endtask

always @(posedge clk or negedge nreset) begin
	if (!nreset) begin
		kst        <= K_IDLE;
		busstate   <= BS_IDLE;
		nwr        <= 1'b1;
		nuds       <= 1'b1;
		nlds       <= 1'b1;
		longword   <= 1'b0;
		addr_out   <= 32'd0;
		data_write <= 16'd0;
		word1      <= 1'b0;
		k_tgl      <= 1'b0;
	end
	else case (kst)
		K_IDLE:
			if (req) begin
				present(1'b0);
				kst <= K_ACC;
			end
		K_ACC:
			if (clkena_in) begin
				if (x_long && !word1) begin
					// the first word: high half, then the low one
					k_rd[31:16] <= data_in;
					present(1'b1);
				end
				else begin
					k_rd[15:0] <= data_in;
					if (!x_long) k_rd[31:16] <= data_in;
					busstate <= BS_IDLE;
					nwr      <= 1'b1;
					nuds     <= 1'b1;
					nlds     <= 1'b1;
					longword <= 1'b0;
					kst      <= K_DONE;
				end
			end
		K_DONE: begin
			// the read data is in k_rd: answer
			k_tgl <= ~k_tgl;
			kst   <= K_IDLE;
		end
		default: kst <= K_IDLE;
	endcase
end

endmodule
