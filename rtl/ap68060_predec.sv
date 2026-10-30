//--------------------------------------------------------------------------//
// AP68060 - pipelined MC68040                                            //
//                                                                          //
// ap68060_predec.sv - predecode of one instruction queue word              //
//                                                                          //
// Treats the word as an operation word and works out everything D1 needs  //
// to size the instruction without decoding it: legality, decoder entry,   //
// size, and the instruction length assuming brief index extension words. //
// D1 checks bit 8 of the words at the index extension positions (a full-  //
// format extension) and the slow flag (FPU immediates) and takes its      //
// multi-cycle path for those.                                             //
//                                                                          //
// Three stages, a register after each (no enable: the fetch pipeline      //
// carries its words' valid bits alongside, and never holds):              //
//   P1  the decoder's entry: every pattern matched in parallel, each      //
//       excluded by the more specific ones overlapping it (one-hot); and  //
//       whether any matches (from the word: not an OR of the entries)     //
//   P2  the entry's attributes (OR trees over the one-hot selection), and //
//       what comes from the word alone: its EA mode indexes and their     //
//       extension lengths, the immediate kinds' lengths                    //
//   P3  the lengths, legality and the class                                //
// pd is the predecode of the word presented three clocks before.          //
//--------------------------------------------------------------------------//

module ap68060_predec
	import ap68060_pkg::*, ap68060_upkg::*;
(
	input  logic        clk,
	input  logic [15:0] op,
	output pd_t         pd
);

`include "gen/ap68060_dec_pla.svh"

function automatic logic [3:0] ea_idx(input logic [2:0] m, input logic [2:0] r);
	if (m != 3'd7) ea_idx = {1'b0, m};
	else if (r <= 3'd4) ea_idx = 4'd7 + {1'b0, r};
	else ea_idx = EM_NONE;
endfunction

// extension words of an EA with a brief index extension
function automatic logic [2:0] ea_brief_len(input logic [3:0] i, input logic [1:0] sz);
	case (i)
		EM_AD16, EM_ABSW, EM_PC16, EM_AX, EM_PCX: ea_brief_len = 3'd1;
		EM_ABSL: ea_brief_len = 3'd2;
		EM_IMM:  ea_brief_len = (sz == SZ_L) ? 3'd2 : 3'd1;
		default: ea_brief_len = 3'd0;
	endcase
endfunction

//--------------------------------------------------------------------------
// P1
//--------------------------------------------------------------------------
logic [NENT-1:0] s1;
logic     [15:0] op1;
logic            m1;           // some pattern matches (not the OR of s1: P2's)
always_ff @(posedge clk) begin
	s1  <= dec_sel(op);
	m1  <= dec_match(op);
	op1 <= op;
end

//--------------------------------------------------------------------------
// P2
//--------------------------------------------------------------------------
pla_t        a2;
logic        m2;
logic [15:0] op2;
logic  [3:0] i0r, i1r;        // the EA fields' mode indexes
logic  [2:0] l0l, l0n, l1r;   // ... their extension words (EA0 long / shorter)
logic  [2:0] nim5, nim6, nim7; // the immediate lengths that depend on the word
always_ff @(posedge clk) begin
	logic [3:0] i0, i1;
	a2   <= ent_attr_oh(s1);
	m2   <= m1;
	op2  <= op1;
	i0    = ea_idx(op1[5:3], op1[2:0]);
	i1    = ea_idx(op1[8:6], op1[11:9]);
	i0r  <= i0;
	i1r  <= i1;
	l0l  <= ea_brief_len(i0, SZ_L);
	l0n  <= ea_brief_len(i0, SZ_W);
	l1r  <= ea_brief_len(i1, SZ_W);       // (EA1 is never an immediate)
	nim5 <= (op1[7:0] == 8'h00) ? 3'd1 : (op1[7:0] == 8'hFF) ? 3'd2 : 3'd0;   // Bcc
	nim6 <= (op1[2:0] == 3'd2) ? 3'd1 : (op1[2:0] == 3'd3) ? 3'd2 : 3'd0;     // TRAPcc
	nim7 <= op1[6] ? 3'd2 : 3'd1;                                             // FBcc
end

//--------------------------------------------------------------------------
// P3
//--------------------------------------------------------------------------
always_ff @(posedge clk) begin
	pla_t a;
	logic [3:0] i0, i1;
	logic [2:0] nimm_l, nimm_n, l0_l, l0_n, l1, b_l, b_n;
	logic [1:0] sz;
	logic       isl, legal;
	a  = a2;
	i0 = a.ea0v ? i0r : EM_NONE;
	i1 = a.ea1v ? i1r : EM_NONE;
	sz = (a.szc == 2'd3) ? op2[7:6] : a.szc;
	// the lengths are formed for a long and for a shorter size side by
	// side, and the size picks one last (the size comes out of the table)
	isl = (a.szc == 2'd3) ? (op2[7:6] == SZ_L) : (a.szc == SZ_L);
	case (a.immk)
		3'd1:       begin nimm_l = 3'd2; nimm_n = 3'd1; end
		3'd2, 3'd3: begin nimm_l = 3'd1; nimm_n = 3'd1; end
		3'd4:       begin nimm_l = 3'd2; nimm_n = 3'd2; end
		3'd5:       begin nimm_l = nim5; nimm_n = nim5; end
		3'd6:       begin nimm_l = nim6; nimm_n = nim6; end
		3'd7:       begin nimm_l = nim7; nimm_n = nim7; end
		default:    begin nimm_l = 3'd0; nimm_n = 3'd0; end
	endcase
	l0_l = a.ea0v ? l0l : 3'd0;
	l0_n = a.ea0v ? l0n : 3'd0;
	l1   = a.ea1v ? l1r : 3'd0;
	b_l  = 3'd1 + {1'b0, a.nfix} + nimm_l;
	b_n  = 3'd1 + {1'b0, a.nfix} + nimm_n;
	legal = m2 &&
	        (!a.ea0v || (i0 != EM_NONE && a.ea0m[i0])) &&
	        (!a.ea1v || (i1 != EM_NONE && a.ea1m[i1]));
	pd.legal <= legal;
	pd.ent   <= a.ent;
	pd.sz    <= sz;
	pd.b     <= isl ? b_l : b_n;
	pd.p1    <= isl ? (b_l + l0_l) : (b_n + l0_n);
	pd.tot   <= isl ? ({1'b0, b_l} + ({1'b0, l0_l} + {1'b0, l1}))
	                : ({1'b0, b_n} + ({1'b0, l0_n} + {1'b0, l1}));
	pd.i0    <= i0;
	pd.i1    <= i1;
	pd.cls   <= a.cls;
	pd.priv  <= a.priv;
	pd.jc0   <= a.jc0;
	pd.nfix  <= a.nfix;
	pd.bzx   <= (a.immk == 3'd2) || (a.immk == 3'd1 && sz == SZ_B);
	pd.fea   <= a.fea;
	// an illegal word: D1 takes one word and the exception; b, p1 and tot
	// are not used for it
	pd.x0    <= legal && ((i0 == EM_AX) || (i0 == EM_PCX));
	pd.x1    <= legal && ((i1 == EM_AX) || (i1 == EM_PCX));
	pd.slow  <= legal && (a.cls == CL_FPU_GEN) && (i0 == EM_IMM);
	pd.simple <= legal && !((a.cls == CL_FPU_GEN) && (i0 == EM_IMM)) &&
	             !((i0 == EM_AX) || (i0 == EM_PCX)) && !((i1 == EM_AX) || (i1 == EM_PCX)) &&
	             ((isl ? ({1'b0, b_l} + ({1'b0, l0_l} + {1'b0, l1}))
	                   : ({1'b0, b_n} + ({1'b0, l0_n} + {1'b0, l1}))) <= 4'd7);
end

// synthesis translate_off
// the pipeline against the one-clock decode of the same word
logic [15:0] opq;
logic  [2:0] warm = 3'd0;        // (the stages hold words presented)
always_ff @(posedge clk) begin
	opq <= op2;
	if (warm != 3'd4) warm <= warm + 3'd1;
end
always_ff @(posedge clk) if (warm == 3'd4) begin
	logic [NENT-1:0] s;
	pla_t a;
	logic [3:0] i0, i1;
	logic l;
	s  = dec_sel(opq);
	a  = ent_attr_oh(s);
	i0 = a.ea0v ? ea_idx(opq[5:3], opq[2:0]) : EM_NONE;
	i1 = a.ea1v ? ea_idx(opq[8:6], opq[11:9]) : EM_NONE;
	l  = a.match && (!a.ea0v || (i0 != EM_NONE && a.ea0m[i0])) &&
	     (!a.ea1v || (i1 != EM_NONE && a.ea1m[i1]));
	if (pd.legal != l || (l && (pd.ent != a.ent || pd.i0 != i0 || pd.i1 != i1)) ||
	    (|dec_sel(opq)) != dec_match(opq)) begin
		$display("FAIL: predec: word %h: legal %b/%b entry %0d/%0d", opq, pd.legal, l, pd.ent, a.ent);
		$finish;
	end
end
// synthesis translate_on

endmodule
