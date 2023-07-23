`default_nettype none
//`include "hw_specific/clock_tools.v"

module tiny_riscv_top(
    input  i_Clk,
    input  i_UART_RX,
    output o_UART_TX,

    input wire [3:0] i_Switch,
    output reg [3:0] o_LED,

    output wire [6:0] o_Segment1,
    output wire [6:0] o_Segment2,

    output o_VGA_HSync,
    output o_VGA_VSync,
    output wire [2:0] o_VGA_Red,
    output wire [2:0] o_VGA_Grn,
    output wire [2:0] o_VGA_Blu,

    inout wire [7:0] io_PMOD
);

    wire w_reset_N = !i_Switch[0];

    initial begin
        o_LED = 4'b0000;
    end

    assign o_Segment1[6] = i_Switch[1];

    //wire w_internal_Clock;

    wire [31:0] w_mem_addr;
    wire [29:0] w_mem_word_addr = w_mem_addr[31:2];

    wire [31:0] w_mem_read_data;
    wire [31:0] w_mem_RAM_read_data;
    wire w_mem_read_strobe;

    wire [31:0] w_mem_write_data;
    wire [3:0] w_mem_write_mask;

    wire w_is_periph_req = w_mem_addr[22];
    wire w_is_memory_req = !w_is_periph_req;

    wire w_memory_write_strobe = |w_mem_write_mask; // Any high bit in the write mask is considered a write strobe

    tiny_riscv_memory memory_inst (
        .i_Clk(i_Clk),
        .i_mem_addr(w_mem_addr),
        .i_read_strobe(w_mem_read_strobe & w_is_memory_req),
        .o_mem_data(w_mem_RAM_read_data),
        .i_mem_write_data(w_mem_write_data),
        .i_mem_write_mask(w_mem_write_mask & {4{w_is_memory_req}})
    );

    //wire [31:0] w_Reg;
    //assign o_LED = w_Reg[3:0];

    tiny_riscv_processor processor_inst (
        .i_Clk(i_Clk),
        .i_Rst_N(w_reset_N),
        //.o_Reg(w_Reg),
        .o_mem_addr(w_mem_addr),
        .i_mem_data(w_mem_read_data),
        .o_read_strobe(w_mem_read_strobe),
        .o_mem_write_data(w_mem_write_data),
        .o_mem_write_mask(w_mem_write_mask)
    );

    localparam s_PERIPH_LED_BIT = 0;
    localparam s_PERIPH_UART_DATA_BIT = 1;
    localparam s_PERIPH_UART_CTRL_BIT = 2;
    localparam s_PERIPH_7SEG1_DATA_BIT = 3;
    localparam s_PERIPH_7SEG2_DATA_BIT = 4;

    always @(posedge i_Clk) begin

        if(w_is_periph_req & w_memory_write_strobe & w_mem_word_addr[s_PERIPH_LED_BIT]) begin
            o_LED <= w_mem_write_data[3:0];
        end

        //if(w_is_periph_req & w_memory_write_strobe & w_mem_word_addr[s_PERIPH_7SEG1_DATA_BIT]) begin
        //    o_Segment1 <= w_mem_write_data[6:0];
        //end

        //if(w_is_periph_req & w_memory_write_strobe & w_mem_word_addr[s_PERIPH_7SEG2_DATA_BIT]) begin
        //    o_Segment2 <= w_mem_write_data[6:0];
        //end
    end

    wire w_UART_start_tx = w_is_periph_req & w_memory_write_strobe & w_mem_word_addr[s_PERIPH_UART_DATA_BIT];
    wire w_UART_ready;

    reg [7:0] r_test_data = "Y";

    wire [7:0] w_uart_data = !i_Switch[2] ? w_mem_write_data[7:0] : r_test_data;

    uart_tx #(
        .CLKS_PER_BIT(217)
    ) uart_tx_inst (
        .i_Rst_L(w_reset_N),
        .i_Clk(i_Clk),
        //.i_TX_Byte(w_mem_write_data[7:0]),
        .i_TX_Byte(w_uart_data),
        .i_TX_Start(w_UART_start_tx),      // Initiate TX of the data in the TX register
        .o_TX_InProgress(),
        .o_TX_Done(w_UART_ready),
        .o_TX_Serial(o_UART_TX)
    );

    wire [31:0] w_periph_read_data = w_mem_word_addr[s_PERIPH_UART_CTRL_BIT] ? {22'b0, !w_UART_ready, 9'b0} : 32'b0;

    assign w_mem_read_data = w_is_memory_req ? w_mem_RAM_read_data : w_periph_read_data;

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


    //`ifdef TESTBENCH
    //assign w_internal_Clock = i_Clk;
    //`else
    //clock_controller #(
    //    //.c_SLOW_CLOCK_BITS(21)
    //)
    //clock_controller_inst
    //(
    //    .i_Clk(i_Clk),
    //    .i_Rst(1'b0),
    //    .o_Clk(w_internal_Clock),
    //    .o_RstN()
    //);
    //`endif

    `ifdef TESTBENCH

    always @(posedge i_Clk) begin
        if(w_UART_start_tx) begin
            //$display("UART output:");
            $write("%c", w_mem_write_data[7:0]);
            $fflush(32'h8000_0001);
            //$display("\r\n");
        end
    end

    `endif

endmodule
