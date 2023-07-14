`default_nettype none
//`include "hw_specific/clock_tools.v"

module tiny_riscv_top(
    input  i_Clk,
    input  i_UART_RX,
    output o_UART_TX, 

    input wire [3:0] i_Switch,
    output wire [3:0] o_LED,

    output wire [6:0] o_Segment1,
    output wire [6:0] o_Segment2,
   
    output o_VGA_HSync,
    output o_VGA_VSync,
    output wire [2:0] o_VGA_Red,
    output wire [2:0] o_VGA_Grn,
    output wire [2:0] o_VGA_Blu,

    inout wire [7:0] io_PMOD
);

    assign o_Segment1[6] = i_Switch[1];

    wire w_internal_Clock;

    wire [31:0] w_mem_addr;
    wire [31:0] w_mem_data;
    wire w_mem_read_strobe;

    tiny_riscv_memory memory_inst (
        .i_Clk(w_internal_Clock),
        .i_mem_addr(w_mem_addr),
        .i_read_strobe(w_mem_read_strobe),
        .o_mem_data(w_mem_data)
    );


    wire [31:0] w_Reg;
    assign o_LED = w_Reg[3:0];

    tiny_riscv_processor processor_inst (
        .i_Clk(w_internal_Clock),
        .i_Rst_N(!i_Switch[0]),
        .o_Reg(w_Reg),
        .o_mem_addr(w_mem_addr),
        .i_mem_data(w_mem_data),
        .o_read_strobe(w_mem_read_strobe)
    );


    //reg [31:0] r_PC = 0; // Program counter

    //reg [31:0] r_instr = 0;

    //always @(posedge w_internal_Clock) begin
    //    o_LED[2] = ~o_LED[2];
    //end

    //always @(posedge w_internal_Clock) begin
    //    o_LEDs <= memory[r_PC];

    //    if(i_Reset || r_PC == 20) begin
    //        r_PC <= 0;
    //    end
    //    else begin
    //        r_PC <= r_PC + 1;
    //    end
    //end


    `ifdef TESTBENCH
    assign w_internal_Clock = i_Clk;
    `else
    clock_controller #(
        //.c_SLOW_CLOCK_BITS(21)
    )
    clock_controller_inst
    (
        .i_Clk(i_Clk),
        .i_Rst(1'b0),
        .o_Clk(w_internal_Clock),
        .o_RstN()
    );
    `endif

endmodule
