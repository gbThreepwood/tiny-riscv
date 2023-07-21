`default_nettype none

module tiny_riscv_memory(
    input wire i_Clk,
    input wire [31:0] i_mem_addr,
    input wire i_read_strobe,
    output reg [31:0] o_mem_data,
    input wire [31:0] i_mem_write_data,
    input wire [3:0]  i_mem_write_mask
);

    function [31:0] changeEndian;   //transform data from the memory to opposite endian form
        input [31:0] value;
        changeEndian = {value[7:0], value[15:8], value[23:16], value[31:24]};
    endfunction

    // The HX1K has 16 4k memory blocks
    // This gives a total of 8kB of memory, but not all of it is available for
    // us to use as RAM.
    //
    // If we use 12 blocks:
    // 12*4k/8 = 6kB.
    // 
    reg [31:0] r_memory[0:1535];
    //integer idx;
    initial begin
        //$readmemh("firmware/assembly/ledblink.mem",r_memory);
        $readmemh("firmware/assembly/hello/hello_world.mem",r_memory);

        //for (idx = 0; idx < 10; idx = idx + 1) $dumpvars(1, r_memory[idx]);
    end

    


    //initial begin
    //   // The memory addresses refer to single bytes, but
    //   // 4 bytes are read at the same time.
    //   //                403     402    401    400
    //   r_memory[100] = {8'h04, 8'h03, 8'h02, 8'h01};
    //   //                407    406    405    404 
    //   r_memory[101] = {8'h08, 8'h07, 8'h06, 8'h05};

    //   r_memory[102] = {8'h0C, 8'h0B, 8'h0A, 8'h09};
    //   r_memory[103] = {8'h00, 8'h0F, 8'h0E, 8'h0D};

    //end

    // We read 4 bytes (one word) at each read strobe, thus the two LSBs are ignored.
    wire [29:0] w_word_address = i_mem_addr[31:2];

    always @(posedge i_Clk) begin
        if(i_read_strobe) begin
            // We have to change the endianess since the data is stored as little endian (in accordance with the RISC-V standard)
            // TODO: investigate if there are more elegant ways of implementing this (perhaps in the CPU module)
            o_mem_data <= changeEndian(r_memory[w_word_address[10:0]]);         
        end

        if(i_mem_write_mask[0]) r_memory[w_word_address[10:0]][7:0]    <= i_mem_write_data[7:0];
        if(i_mem_write_mask[1]) r_memory[w_word_address[10:0]][15:8]   <= i_mem_write_data[15:8];
        if(i_mem_write_mask[2]) r_memory[w_word_address[10:0]][23:16]  <= i_mem_write_data[23:16];
        if(i_mem_write_mask[3]) r_memory[w_word_address[10:0]][31:24]  <= i_mem_write_data[31:24];
    end

endmodule
