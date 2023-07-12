
`default_nettype none

module tiny_riscv_processor(
    input wire i_Clk,
    input wire i_Rst_N
    //input wire [31:0] memory
);

    reg [31:0] r_PC;
    reg [31:0] r_instr;

    reg [31:0] r_RegisterBank[0:31];


    reg [31:0] r_memory [0:255];

    initial begin
        r_PC = 0;

        r_instr     = 32'b0000000_00000_00000_000_00000_0110011;
        
        r_memory[0] = 32'b0000000_00000_00000_000_00001_0010011;
        r_memory[1] = 32'b0000000_00000_00000_000_00001_1101111;
        r_memory[2] = 32'b0000000_00000_00000_000_00001_0110111;
    end




    // Checks for all the different possible instruction types
    wire w_is_fence_instr          =  r_instr[6:0] == 7'b000_1111;

    wire w_is_ecall_instr          =  r_instr[6:0] == 7'b111_0011;
    wire w_is_ebreak_instr         =  r_instr[6:0] == 7'b111_0011;
    wire w_is_system_instr         =  r_instr[6:0] == 7'b111_0011;

    wire w_is_ALU_reg_instr        =  r_instr[6:0] == 7'b011_0011;
    wire w_is_ALU_imm_instr        =  r_instr[6:0] == 7'b001_0011;
    wire w_is_store_instr          =  r_instr[6:0] == 7'b010_0011;
    wire w_is_load_instr           =  r_instr[6:0] == 7'b000_0011;
    wire w_is_branch_instr         =  r_instr[6:0] == 7'b110_0011;
    wire w_is_jump_link_r_instr    =  r_instr[6:0] == 7'b110_0111;
    wire w_is_jump_link_instr      =  r_instr[6:0] == 7'b110_1111;
    wire w_is_add_ui_pc_instr      =  r_instr[6:0] == 7'b001_0111;
    wire w_is_load_ui_instr        =  r_instr[6:0] == 7'b011_0111;

    // Obtain the register parameters for a given instruction
    wire [4:0] w_rs1_param = r_instr[19:15];
    wire [4:0] w_rs2_param = r_instr[24:20];
    wire [4:0] w_rd_param  = r_instr[11:7];

    // Obtain the instruction parameter which distinguishes the various operations which
    // are supported under a given instruction type.
    wire [2:0] w_funct3 = r_instr[14:12];
    wire [6:0] w_funct7 = r_instr[31:25];

    // Obtain the immediate values for the various instructions
    // The I, S, B, and J type instructions do sign expansion by repeating the MSB of the immediate
    // value the required number of times to fill the entire 32 bit register.
    wire [31:0] w_U_type_imm = {r_instr[31:12], {12{1'b0}}}; // Immediate value is left adjusted, the remaining bits are set to zero
    wire [31:0] w_I_type_imm = {{21{r_instr[31]}}, r_instr[30:20]};
    wire [31:0] w_S_type_imm = {{21{r_instr[31]}}, r_instr[30:25], r_instr[11:7]};
    wire [31:0] w_B_type_imm = {{20{r_instr[31]}}, r_instr[7], r_instr[30:25], r_instr[11:8], 1'b0};
    wire [31:0] w_J_type_imm = {{12{r_instr[31]}}, r_instr[19:12], r_instr[20], r_instr[30:21], 1'b0};


    always @(posedge i_Clk) begin
        if(!i_Rst_N) begin
            r_PC <= 0;
        end
        else if (!w_is_system_instr) begin
            r_instr <= r_memory[r_PC];
            r_PC <= r_PC + 1;
        end
    end


    localparam STATE_FETCH_INSTR = 0;
    localparam STATE_FETCH_REGS  = 1;
    localparam STATE_EXECUTE     = 2;

    reg [1:0] r_state;
    reg [31:0] rs1, rs2;

    wire [31:0] w_ALU_in_1 = rs1;
    wire [31:0] w_ALU_in_2 = w_is_ALU_reg_instr ? rs2 : w_I_type_imm;
    reg [31:0] r_ALU_out;

    wire [31:0] w_return_data;
    wire w_return_en;


    always @(posedge i_Clk) begin
        case(r_state)
            STATE_FETCH_INSTR: begin
                r_instr <= r_memory[r_PC];
                r_state <= STATE_FETCH_REGS;
            end
            STATE_FETCH_REGS: begin
                rs1 <= r_RegisterBank[w_rs1_param];
                rs2 <= r_RegisterBank[w_rs2_param];

                r_state <= STATE_EXECUTE; 
            end
            STATE_EXECUTE: begin
                r_PC <= r_PC + 1;
                r_state <= STATE_FETCH_INSTR;
            end
        endcase
    end

    always @(posedge i_Clk) begin
        if(w_return_en && w_rd_param != 0) begin
            r_RegisterBank[w_rd_param] <= w_return_data;
        end
    end

    wire [4:0] w_shift_amount = w_is_ALU_reg_instr ? rs2[4:0] : r_instr[24:20];

    always @(*) begin
        case (w_funct3)
            // Bit 5 of w_funct7 is 1 for sub and 0 for add.
            3'b000: r_ALU_out = (w_funct7[5] & r_instr[5]) ? (w_ALU_in_1 - w_ALU_in_2) : (w_ALU_in_1 + w_ALU_in_2);
        endcase
    end

    `ifdef TESTBENCH
        always @(posedge i_Clk) begin
            $display("PC = %0d", r_PC);

            //if(w_is_ALU_reg_instr) begin
            //    $display("ALUreg rd=%d, rs1=%d, rs2=%d, funct3=%b, funct7=%b", w_rd_param, w_rs1_param, w_rs2_param, w_funct3, w_funct7);
            //end

            case(1'b1) // The statements will be executed if they are equal to 1'b1 (if they are true)
                w_is_ALU_reg_instr:     $display("ALUreg rd=%d, rs1=%d, rs2=%d, funct3=%b, funct7=%b", w_rd_param, w_rs1_param, w_rs2_param, w_funct3, w_funct7);
                w_is_ALU_imm_instr:     $display("ALUimm rd=%d, rs1=%d, imm=%0d, funct3=%b", w_rd_param, w_rs1_param, w_I_type_imm, w_funct3);
                w_is_branch_instr:      $display("BRANCH instruction");
                w_is_jump_link_instr:   $display("JAL (Jump and link) instruction");
                w_is_jump_link_r_instr: $display("JALR (jump and link register) instruction");
                w_is_add_ui_pc_instr:   $display("AUIPC (add upper immediate) instruction");
                w_is_load_ui_instr:     $display("LUI (load upper immediate) instruction");
                w_is_load_instr:        $display("LOAD instruction");
                w_is_store_instr:       $display("STORE instruction");
                w_is_system_instr:      $display("SYSTEM instruction");
            endcase
        end
    `endif

endmodule