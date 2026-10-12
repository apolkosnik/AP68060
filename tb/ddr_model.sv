//--------------------------------------------------------------------------//
// AP68060 test bench                                                       //
//                                                                          //
// ddr_model.sv - a 64-bit Avalon-MM memory standing in for the DDRAM       //
// interface behind Fast RAM: random waitrequest, a random read latency,   //
// burst reads returned one beat per clock, writes with byte enables.  One //
// command at a time, as ap030_fastram_be issues them.  Initialised to a   //
// pattern of the word address.                                             //
//--------------------------------------------------------------------------//

module ddr_model #(
	parameter int AW = 17               // 2**AW 64-bit words
)
(
	input  logic        clk,
	input  logic [28:0] address,
	input  logic  [7:0] burstcount,
	input  logic        read,
	input  logic        write,
	input  logic [63:0] writedata,
	input  logic  [7:0] byteenable,
	output logic        waitrequest,
	output logic [63:0] readdata,
	output logic        readdatavalid
);

logic [63:0] mem [0:(1 << AW) - 1];
logic  [4:0] lat;
logic  [7:0] left;
logic [AW-1:0] ra;
logic        rbusy;

initial begin
	for (int i = 0; i < (1 << AW); i++) mem[i] = {32'hFA570000 | (i[15:0]), 32'h5EED0000 | i[15:0]};
	waitrequest = 1'b1; readdatavalid = 1'b0; rbusy = 1'b0;
end

always_ff @(posedge clk) begin
	readdatavalid <= 1'b0;
	waitrequest   <= ($urandom % 4) == 0;
	if (rbusy) begin
		if (lat != 0) lat <= lat - 1'd1;
		else begin
			readdatavalid <= 1'b1;
			readdata      <= mem[ra];
			ra            <= ra + 1'd1;
			left          <= left - 1'd1;
			if (left == 8'd1) rbusy <= 1'b0;
		end
	end
	else if (!waitrequest && write) begin
		for (int b = 0; b < 8; b++)
			if (byteenable[b]) mem[address[AW-1:0]][8*b +: 8] <= writedata[8*b +: 8];
	end
	else if (!waitrequest && read) begin
		rbusy <= 1'b1;
		ra    <= address[AW-1:0];
		left  <= burstcount;
		lat   <= 5'(3 + ($urandom % 12));
	end
	// (no command is taken while a read is returning its beats)
	if (rbusy || (!waitrequest && read)) waitrequest <= 1'b1;
end

endmodule
