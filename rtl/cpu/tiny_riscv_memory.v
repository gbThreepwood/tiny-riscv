`default_nettype none

module tiny_riscv_memory(
    input wire i_Clk,
    input wire [31:0] i_mem_addr,
    input wire i_read_strobe,
    output reg [31:0] o_mem_data
);

    reg [31:0] r_memory[0:255];

    initial begin
        $readmemh("firmware.hex",r_memory);
    end


    always @(posedge i_Clk) begin
        if(i_read_strobe) begin
            o_mem_data <= r_memory[i_mem_addr[31:2]];
        end
    end

endmodule
