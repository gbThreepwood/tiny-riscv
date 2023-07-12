`default_nettype none
`include "clock_tools.v"

module tiny_riscv_top(
    input  i_Clk,
    input  i_UART_RX,
    output o_UART_TX, 

    input i_Switch_1,
    input i_Switch_2,
    input i_Switch_3,
    input i_Switch_4,

    output o_LED_1,
    output o_LED_2,
    output o_LED_3,
    output o_LED_4,

    // Segment 1
    output o_Segment1_A,
    output o_Segment1_B,
    output o_Segment1_C,
    output o_Segment1_D,
    output o_Segment1_E,
    output o_Segment1_F,
    output o_Segment1_G,

    // Segment 2
    output o_Segment2_A,
    output o_Segment2_B,
    output o_Segment2_C,
    output o_Segment2_D,
    output o_Segment2_E,
    output o_Segment2_F,
    output o_Segment2_G,
    
    // VGA
    output o_VGA_HSync,
    output o_VGA_VSync,
    output o_VGA_Red_0,
    output o_VGA_Red_1,
    output o_VGA_Red_2,
    output o_VGA_Grn_0,
    output o_VGA_Grn_1,
    output o_VGA_Grn_2,
    output o_VGA_Blu_0,
    output o_VGA_Blu_1,
    output o_VGA_Blu_2 

);

    wire w_internal_Clock;


    tiny_riscv_processor processor_inst (
        .i_Clk(w_internal_Clock),
        .i_Rst_N(1'b1)
    );


    //reg [31:0] r_PC = 0; // Program counter

    //reg [31:0] r_instr = 0;


    //always @(posedge w_internal_Clock) begin
    //    o_LEDs <= memory[r_PC];

    //    if(i_Reset || r_PC == 20) begin
    //        r_PC <= 0;
    //    end
    //    else begin
    //        r_PC <= r_PC + 1;
    //    end
    //end


    clock_controller #(
        .c_SLOW_CLOCK_BITS(21)
    )
    clock_controller_inst
    (
        .i_Clk(i_Clk),
        .i_Rst(1'b0),
        .o_Clk(w_internal_Clock),
        .o_RstN()
    );

endmodule
