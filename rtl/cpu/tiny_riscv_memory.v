`default_nettype none

module tiny_riscv_memory(
    input wire i_Clk,
    input wire [31:0] i_mem_addr,
    input wire i_read_strobe,
    output reg [31:0] o_mem_data,
    input wire [31:0] i_mem_write_data,
    input wire [3:0]  i_mem_write_mask
);

    // The HX1K has 16 4k memory blocks
    // This gives a total of 8kB of memory, but not all of it is available for
    // us to use as RAM.
    //
    // If we use 12 blocks:
    // 12*4k/8 = 6kB.
    // 
    reg [31:0] r_memory[0:1535];

    //initial begin
    //    $readmemh("../../firmware/firmware.mem",r_memory);
    //end

    `include "rtl/tools/riscv_assembly.v"

    integer L0_   = 12;
    integer L1_   = 20;
    integer L2_   = 52;
    integer L3_   = 104;
    integer wait_ = 152;
    integer wait_L0_ = 160;
    integer putc_ = 172;
    integer putc_L0_ = 180;
   
    initial begin
       // The memory addresses refer to single bytes, but
       // 4 bytes are read at the same time.
       //                403     402    401    400
       r_memory[100] = {8'h04, 8'h03, 8'h02, 8'h01};
       //                407    406    405    404 
       r_memory[101] = {8'h08, 8'h07, 8'h06, 8'h05};

       r_memory[102] = {8'h0C, 8'h0B, 8'h0A, 8'h09};
       r_memory[103] = {8'h00, 8'h0F, 8'h0E, 8'h0D};

    end

    // We read 4 bytes (one word) at each read strobe, thus the two LSBs are ignored.
    wire [31:0] w_word_address = i_mem_addr[31:2];

    always @(posedge i_Clk) begin
        if(i_read_strobe) begin
            o_mem_data <= r_memory[w_word_address];         
        end

        if(i_mem_write_mask[0]) r_memory[w_word_address][7:0]    <= i_mem_write_data[7:0];
        if(i_mem_write_mask[1]) r_memory[w_word_address][15:8]   <= i_mem_write_data[15:8];
        if(i_mem_write_mask[2]) r_memory[w_word_address][23:16]  <= i_mem_write_data[23:16];
        if(i_mem_write_mask[3]) r_memory[w_word_address][31:24]  <= i_mem_write_data[31:24];
    end

endmodule
