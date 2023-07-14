
`default_nettype none

module tiny_riscv_processor(
    input wire i_Clk,
    input wire i_Rst_N,
    output wire [31:0] o_Reg,
    output wire [31:0] o_mem_addr,
    input wire [31:0] i_mem_data,
    output wire o_read_strobe
);

    //always @(posedge i_Clk) begin
    //    o_Reg <= ~o_Reg;
    //end



    reg [31:0] r_PC = 0;
    wire [31:0] w_next_PC;

    reg [31:0] r_instr;

    reg [31:0] r_RegisterBank[0:31];

    reg [31:0] rs1 = 0, rs2 = 0;
    assign o_Reg = r_RegisterBank[10];

    wire [31:0] w_return_data;
    wire w_return_en;

    reg [31:0] r_memory [0:255];

//`ifdef TESTBENCH
    integer i;
    initial begin
        //r_RegisterBank <= '{default: '0};
        for(i = 0; i < 32; i = i + 1) begin
            r_RegisterBank[i] <= 0;
        end
    end
//`endif


    // Checks for all the different possible instruction types
    wire w_is_fence_instr          =  r_instr[6:0] == 7'b000_1111;

    wire w_is_ecall_instr          =  r_instr[6:0] == 7'b111_0011;
    wire w_is_ebreak_instr         =  r_instr[6:0] == 7'b111_0011;
    wire w_is_system_instr         =  r_instr[6:0] == 7'b111_0011;

    wire w_is_ALU_reg_instr        =  r_instr[6:0] == 7'b011_0011; // rd <= rs1 OP rs2
    wire w_is_ALU_imm_instr        =  r_instr[6:0] == 7'b001_0011; // rd <= rs1 OP imm
    wire w_is_store_instr          =  r_instr[6:0] == 7'b010_0011; // mem[rs1 + imm] <= rs2
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


    //always @(posedge i_Clk) begin
    //    if(!i_Rst_N) begin
    //        r_PC <= 0;
    //    end
    //    else if (!w_is_system_instr) begin
    //        r_instr <= r_memory[r_PC];
    //        r_PC <= r_PC + 1;
    //    end
    //end


    localparam STATE_FETCH_INSTR = 0;
    localparam STATE_WAIT_INSTR  = 1;
    localparam STATE_FETCH_REGS  = 2;
    localparam STATE_EXECUTE     = 3;

    // Execution state register
    reg [1:0] r_state = STATE_FETCH_INSTR;

    always @(posedge i_Clk) begin

        if(!i_Rst_N) begin // Reset is active low
            r_PC <= 0;
            r_state <= STATE_FETCH_INSTR;
        end
        else begin

            // Check if the desitantion register is not register 0. If register 0 is used the write should have no effect.
            // Also check if we are in EXECUTE state and the current instruction is an ALU instruction, indicating that we should update a register.

            //if(((r_state == STATE_EXECUTE) && (w_is_ALU_reg_instr || w_is_ALU_imm_instr)) && (w_rd_param != 0)) begin
            //    r_RegisterBank[w_rd_param] <= w_return_data;

            //    `ifdef TESTBENCH
            //    $display("x%0d <= %b", w_rd_param, w_return_data);
            //    `endif
            //end

            if(w_return_en && (w_rd_param != 0)) begin
                r_RegisterBank[w_rd_param] <= w_return_data;

                `ifdef TESTBENCH
                $display("x%0d <= %b", w_rd_param, w_return_data);
                `endif

            end

            case(r_state)
                STATE_FETCH_INSTR: begin
                    r_state <= STATE_WAIT_INSTR;
                end
                STATE_WAIT_INSTR: begin
                    //r_instr <= r_memory[r_PC[31:2]]; //r_memory[r_PC];
                    r_instr <= i_mem_data;
                    r_state <= STATE_FETCH_REGS;
                end
                STATE_FETCH_REGS: begin
                    rs1 <= r_RegisterBank[w_rs1_param];
                    rs2 <= r_RegisterBank[w_rs2_param];

                    r_state <= STATE_EXECUTE; 
                end
                STATE_EXECUTE: begin
                    if(!w_is_system_instr) begin
                        //r_PC <= r_PC + 4; // Each instruction is 4 bytes long.
                        r_PC <= w_next_PC; // In case of jumps however the next PC value might not be current value + 4.
                    end
                    r_state <= STATE_FETCH_INSTR;

                    `ifdef TESTBENCH
                    //$display("STATE_EXECUTE");
                    if(w_is_system_instr) begin 
                        $display("Finishing simulation due to SYSTEM instruction");
                        $finish();
                    end
                    `endif
                end
            endcase
        end
    end


    wire [31:0] w_ALU_in_1 = rs1;
    wire [31:0] w_ALU_in_2 = w_is_ALU_reg_instr ? rs2 : w_I_type_imm;
    reg [31:0] r_ALU_out = 0;
    wire [4:0] w_shift_amount = w_is_ALU_reg_instr ? rs2[4:0] : r_instr[24:20];

    assign o_mem_addr = r_PC;
    assign o_read_strobe = (r_state == STATE_FETCH_INSTR);

    // ALU:
    always @(*) begin
        case (w_funct3)
            // Bit 5 of w_funct7 is 1 for sub and 0 for add. Furthermore there is no subtract immediate instruction, and hence we only do subtract if bit 5 of the instruction register is set. Failure to check for this bit could lead to subtraction happening at random depending on the immediate value.
            3'b000: r_ALU_out = (w_funct7[5] & r_instr[5]) ? (w_ALU_in_1 - w_ALU_in_2) : (w_ALU_in_1 + w_ALU_in_2); // ADD/SUB - Addition or subtraction

            3'b001: r_ALU_out = (w_ALU_in_1 << w_shift_amount); // SLL/SLLI - Left shift
            
            3'b010: r_ALU_out = ($signed(w_ALU_in_1)) < ($signed(w_ALU_in_2)); // Signed comparison ($signed() evaluate the input expression and return a value with the same size and value of the input expression and signed datatype)
            
            3'b011: r_ALU_out = (w_ALU_in_1 < w_ALU_in_2); // SLTU/SLTIU - Unsigned comparison
            
            3'b100: r_ALU_out = (w_ALU_in_1 ^ w_ALU_in_2); // XOR
            
            3'b101: r_ALU_out = w_funct7[5] ? ($signed(w_ALU_in_1) >>> w_shift_amount) : (w_ALU_in_1 >> w_shift_amount); // SRA/SRAI (arithmetic) SRL/SRLI (logical) - Logical (zero-fill) or arithmetic (sticky) right shift
            
            3'b110: r_ALU_out = (w_ALU_in_1 | w_ALU_in_2); // OR
            
            3'b111: r_ALU_out = (w_ALU_in_1 & w_ALU_in_2); // AND
        endcase
    end


    reg r_branch_criterion_fulfilled;


    always @(*) begin
        case(w_funct3)
            3'b000: r_branch_criterion_fulfilled = (rs1 == rs2);
            3'b001: r_branch_criterion_fulfilled = (rs1 != rs2);
            3'b100: r_branch_criterion_fulfilled = ($signed(rs1) < $signed(rs2));
            3'b101: r_branch_criterion_fulfilled = ($signed(rs1) >= $signed(rs2));
            3'b110: r_branch_criterion_fulfilled = (rs1 < rs2);
            3'b111: r_branch_criterion_fulfilled = (rs1 >= rs2);
            default: r_branch_criterion_fulfilled = 1'b0;
        endcase
    end

    assign w_return_en = (r_state == STATE_EXECUTE) && 
                         ( w_is_ALU_reg_instr     || 
                           w_is_ALU_imm_instr     ||
                           w_is_jump_link_instr   ||
                           w_is_jump_link_r_instr ||
                           w_is_load_ui_instr     ||
                           w_is_add_ui_pc_instr
                         );

    assign w_return_data = (w_is_jump_link_instr || w_is_jump_link_r_instr) ? r_PC + 4 : // The jump and link instructions store the PC (+4) value before jumping
                            w_is_load_ui_instr ? w_U_type_imm :                          // Load 20 immediate bits left adjusted in the destination register
                            w_is_add_ui_pc_instr ? (r_PC + w_U_type_imm) :               // 
                            r_ALU_out;

    assign w_next_PC =  (w_is_branch_instr && r_branch_criterion_fulfilled) ? r_PC + w_B_type_imm : 
                        w_is_jump_link_instr                                ? r_PC + w_J_type_imm : 
                        w_is_jump_link_r_instr                              ? rs1 + w_I_type_imm :
                        r_PC + 4;



    `ifdef TESTBENCH
        always @(posedge i_Clk) begin

            if(r_state == STATE_EXECUTE) begin
                
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
                    //w_is_ebreak_instr:
                    //w_is_ecall_instr:
                    w_is_system_instr:      $display("SYSTEM instruction");
                endcase
            end
        end
    `endif

endmodule
