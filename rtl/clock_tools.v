`default_nettype none

module clock_controller #(
    parameter c_SLOW_CLOCK_BITS = 0
)
(
    input wire i_Clk,
    input wire i_Rst,
    output wire o_Clk,
    output wire o_RstN
);


    reg [c_SLOW_CLOCK_BITS-1:0] r_slow_clk_cntr = 0;

    always @(posedge i_Clk) begin
        r_slow_clk_cntr <= r_slow_clk_cntr + 1;
    end

    assign o_Clk = r_slow_clk_cntr[c_SLOW_CLOCK_BITS-1];

endmodule