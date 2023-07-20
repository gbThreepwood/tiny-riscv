`default_nettype none
`define TESTBENCH

module tiny_riscv_tb();

    reg r_Clk;
    reg r_ResetN;

    wire [3:0] w_LEDs;
    reg [3:0] w_LEDs_prev;

    reg [3:0] r_Switch = 4'b0000;

    tiny_riscv_top tiny_riscv_top_inst (
        .i_Clk(r_Clk),
        .o_LED(w_LEDs),
        .i_Switch(r_Switch)
    );
    
    initial begin

        r_Clk = 1'b0;

        r_ResetN <= 1'b0;
        #10 r_ResetN <= 1'b1;
        #1500 $display("Explicitly calling finish from testbench.");
        #1 $finish;
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

    reg [4095:0] vcdfile;

    initial 
        begin
            if ($value$plusargs("vcd=%s", vcdfile)) begin
                $dumpfile(vcdfile);
                $dumpvars(0);
            end
        end


endmodule
