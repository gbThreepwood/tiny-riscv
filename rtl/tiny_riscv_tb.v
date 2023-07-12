`default_nettype none
`define TESTBENCH

module tiny_riscv_tb();

    reg r_Clk;
    wire [4:0] w_LEDs;
    reg [4:0] w_LEDs_prev;

    //tiny_riscv_top tiny_riscv_top_inst (
    //    .i_Reset(),
    //    .i_Clk(r_Clk),
    //    .o_LEDs(w_LEDs)
    //);
    
    tiny_riscv_processor processor_inst (
        .i_Clk(r_Clk),
        .i_Rst_N(1'b1)
    );

    initial begin

        r_Clk = 1'b0;

        #100 $finish;
    end


    always
        #1 r_Clk = ~r_Clk;
    
        //forever begin
        //    #1 r_Clk = ~r_Clk;

        //    if(w_LEDs != w_LEDs_prev) begin
        //        $display("LEDs = %b", w_LEDs);
        //    end
        //    w_LEDs_prev <= w_LEDs;

        //end


    initial 
        begin
          $dumpfile("dump.vcd");
          $dumpvars(0);
        end


endmodule
