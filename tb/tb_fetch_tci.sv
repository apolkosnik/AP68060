`timescale 1ns/1ps
//--------------------------------------------------------------------------//
// AP68060 test bench                                                     //
//                                                                          //
// tb_fetch_tci.sv - the fetch unit alone: a TCI instruction line is not  //
// kept (MC68060UM 5.7.1).  The line at $2000 is fetched with TCI (+tci) or //
// cachable, its opcode $7001; memory then holds $7002.  After a redirect //
// to the line (default), a sequential request (+sequential), a redirect //
// on the fill's completion clock (+race), or an invalidation first      //
// (+invalidate), a TCI line must come from a new transfer ($7002); the   //
// cachable control hits the cache ($7001, one transfer).  (From the       //
// October 12 core audit's probe.)                                         //
//--------------------------------------------------------------------------//
module tb_fetch_tci;
import ap68060_pkg::*;
logic        clk;
logic        nreset;
logic        redir_v;
logic [31:0] redir_pc;
logic        d_redir_v;
logic [31:0] d_redir_pc;
logic        stop;
logic        smode;
logic [31:0] cacr;
logic [31:0] tc;
logic [31:0] itt0;
logic [31:0] itt1;
logic        cdis;
logic        mdis;
logic [15:0] win [8];
logic  [7:0] win_flt;
logic  [7:0] win_fdem;
logic  [7:0] win_fatc;
logic  [7:0] win_bt;
logic [31:0] bt_tgt;
pd_t         pd0;
logic  [3:0] qcnt;
logic [31:0] qpc;
logic        cs_fire;
logic        cr_fire;
logic  [2:0] cr_use;
logic        q_odd;
logic        bc_clr;
logic        btb_we;
logic [BTB_AW-1:0] btb_wi;
logic        btb_wv;
logic [BTB_TW-1:0] btb_wtag;
logic        btb_wslot;
logic  [1:0] btb_wkind;
logic [30:0] btb_wtgt;
logic [30:0] d_ras;
logic  [2:0] d_ras_tp;
logic  [3:0] d_ras_n;
logic        iw_req;
logic [31:0] iw_va;
logic        iw_fc2;
logic        iw_done;
atce_t       iw_ent;
logic [31:0] iw_cause;
logic [31:0] if_cause;
logic        iatc_wr;
logic [31:0] iatc_wla;
logic        iatc_wfc2;
atce_t       iatc_went;
logic        iatc_flush_all;
logic        iatc_flush_page;
logic        iatc_flush_ng;
logic [31:0] iatc_flush_la;
logic        iatc_flush_fc2;
logic        ic_inv;
logic  [1:0] ic_inv_scope;
logic [31:0] ic_inv_pa;
logic        ic_inv_done;
logic        sn_inv;
logic [31:0] sn_inv_pa;
logic        sn_inv_all;
logic        sn_inv_done;
logic        sn_ihit;
logic        b_req;
busreq_t     b_breq;
logic        b_gnt;
logic        b_done;
logic        b_err;
logic        b_rvalid;
logic [31:0] b_rdata;
logic  [1:0] b_rbeat;
logic        b_rtci;
ap68060_fetch dut (.*);
always #5 clk = ~clk;
task tick; @(posedge clk); #1; @(negedge clk); endtask
int reads;
bit inhibit;
task fill(input logic [15:0] opcode);
 int timeout;
 logic [1:0] startbeat;
 timeout=0;
 while (!b_req && timeout<100) begin tick(); timeout++; end
 if (!b_req) $fatal(1,"FAIL expected a bus request");
 if (b_breq.addr[31:4] != 28'h200) $fatal(1,"FAIL unrelated fill addr=%h",b_breq.addr);
 startbeat=b_breq.addr[3:2]; reads++;
 stop=1; b_gnt=1; tick(); b_gnt=0;
 for(int beat=0;beat<4;beat++) begin
  b_rvalid=1; b_rbeat=startbeat+2'(beat); b_rtci=inhibit && beat==0;
  b_rdata={opcode,16'h4e71}; tick();
 end
 b_rvalid=0; b_rtci=0; b_done=1;
 if ($test$plusargs("race") && reads==1) begin redir_v=1; redir_pc=32'h2000; end
 tick(); b_done=0; redir_v=0;
 tick(); stop=0; tick(); stop=1;
 repeat(6) tick();
endtask
initial begin
clk = '0;
nreset = '0;
redir_v = '0;
redir_pc = '0;
d_redir_v = '0;
d_redir_pc = '0;
stop = '0;
smode = '0;
cacr = '0;
tc = '0;
itt0 = '0;
itt1 = '0;
cdis = '0;
mdis = '0;
cs_fire = '0;
cr_fire = '0;
cr_use = '0;
bc_clr = '0;
btb_we = '0;
btb_wi = '0;
btb_wv = '0;
btb_wtag = '0;
btb_wslot = '0;
btb_wkind = '0;
btb_wtgt = '0;
d_ras = '0;
d_ras_tp = '0;
d_ras_n = '0;
iw_done = '0;
iw_ent = '0;
iw_cause = '0;
iatc_wr = '0;
iatc_wla = '0;
iatc_wfc2 = '0;
iatc_went = '0;
iatc_flush_all = '0;
iatc_flush_page = '0;
iatc_flush_ng = '0;
iatc_flush_la = '0;
iatc_flush_fc2 = '0;
ic_inv = '0;
ic_inv_scope = '0;
ic_inv_pa = '0;
sn_inv = '0;
sn_inv_pa = '0;
sn_inv_all = '0;
b_gnt = '0;
b_done = '0;
b_err = '0;
b_rvalid = '0;
b_rdata = '0;
b_rbeat = '0;
b_rtci = '0;
 stop=1; reads=0; inhibit=$test$plusargs("tci");
 repeat(4) tick(); nreset=1; cacr=32'h00008000; smode=1;
 repeat(150) tick();
 redir_v=1; redir_pc=32'h2000; tick(); redir_v=0; stop=0;
 fill(16'h7001);
 if ($test$plusargs("race")) begin
  for (int k = 0; k < 4 && (qcnt == 0 || win[0] != 16'h7002); k++) begin
   if (b_req) fill(16'h7002);
   else begin stop=0; tick(); stop=1; repeat(5) tick(); end
  end
  $display("RACE data=%h reads=%0d",win[0],reads);
  if (win[0]!=16'h7002 || reads<2) $display("FAIL TCI completion and redirect retain stale data");
  else $display("PASS TCI completion/redirect");
  $finish;
 end else begin
 if (qcnt==0 || win[0]!=16'h7001) $fatal(1,"FAIL first fetch qcnt=%d data=%h",qcnt,win[0]);
 $display("FIRST data=%h reads=%0d lb_valid=%b",win[0],reads,dut.lb_v);
 if ($test$plusargs("invalidate")) begin
  ic_inv=1; ic_inv_scope=1; ic_inv_pa=32'h2000;
  // (a fill the unit has asked for meanwhile goes first: answered)
  while(!ic_inv_done) begin if (b_req) fill(16'h7002); else tick(); end
  ic_inv=0; repeat(3) tick();
 end
 // A new sequential or redirected IFU request after burst completion and a memory update.
 if ($test$plusargs("sequential")) begin cr_fire=1; cr_use=2; tick(); cr_fire=0; end
 else begin redir_v=1; redir_pc=32'h2000; tick(); redir_v=0; end
 stop=0; tick(); stop=1;
 repeat(5) tick();
 // (the unit may have asked for the next long word already: every request
 // is answered from memory as it now is, until words arrive)
 for (int k = 0; k < 4 && qcnt == 0; k++) begin
  if (b_req) fill(16'h7002);
  else begin stop=0; tick(); stop=1; repeat(5) tick(); end
 end
 if (qcnt==0) $fatal(1,"FAIL no words after redirect");
 $display("SECOND data=%h reads=%0d lb_valid=%b",win[0],reads,dut.lb_v);
 if (inhibit || $test$plusargs("invalidate")) begin
  if (win[0]!=16'h7002 || reads<2) $display("FAIL completed TCI fill reused: expected data=7002 from a new transfer");
  else $display("PASS TCI/invalidated refill");
 end else begin
  if (win[0]!=16'h7001 || reads!=1) $display("FAIL cacheable control");
  else $display("PASS cacheable hit control");
 end
 $finish;
 end
end
initial begin #100000; $fatal(1,"FAIL timeout"); end
endmodule
