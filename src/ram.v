/// sta-blackbox

module RAM16 (
    input wire CLK,
    input wire [3:0] WE0,
    input wire EN0,
    input wire [1:0] A0,
    input wire [31:0] Di0,
    output wire [31:0] Do0
);
  reg [31:0] RAM[3:0];

  always @(posedge CLK)
    if (EN0) begin
      if (WE0[0]) RAM[A0][7:0] <= Di0[7:0];
      if (WE0[1]) RAM[A0][15:8] <= Di0[15:8];
      if (WE0[2]) RAM[A0][23:16] <= Di0[23:16];
      if (WE0[3]) RAM[A0][31:24] <= Di0[31:24];
    end

  assign Do0 = RAM[A0] & {32{EN0}};

endmodule
