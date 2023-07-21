////////////////////////////////////////////////////////////
// Simple RISCV implementation
//
// Mostly for fun, probably not the best choice if you are
// looking for a reliable CPU for your next project, but feel
// free to try it under the GPL v3 terms.
//
// Copyleft, Eirik Haustveit, 2023
////////////////////////////////////////////////////////////

// No optimizations: 1137
// Subtraction for comparison: 1093
// Reuse comparison for branching 1097???
// Reuse ALU adder for jump 968
// Single bit shifter for three operations: 840

`default_nettype none

module tiny_riscv_processor(
    input wire i_Clk,
    input wire i_Rst_N,
    //output reg [31:0] o_Reg,
    output wire [31:0] o_mem_addr,
    input wire [31:0] i_mem_data,
    output wire o_read_strobe,
    output wire [31:0] o_mem_write_data,
    output wire [3:0]  o_mem_write_mask
);

    assign o_mem_addr = (r_state == STATE_WAIT_INSTR || r_state == STATE_FETCH_INSTR) ? r_PC : w_load_or_store_address;
    assign o_read_strobe = (r_state == STATE_FETCH_INSTR || r_state == STATE_LOAD_DATA);

    assign o_mem_write_mask = {4{r_state == STATE_STORE_DATA}} & r_store_write_mask;
    assign o_mem_write_data = r_mem_write_data;

    reg [31:0] r_PC = 0;

    reg [31:0] r_instr;


`ifdef TESTBENCH
    integer i;
    initial begin
        //r_RegisterBank <= '{default: '0};
        for(i = 0; i < 32; i = i + 1) begin
            r_RegisterBank[i] = 0;
        end
    end
`endif

    ////////////////////////////////////////////////////////////
    // Instruction decoding
    ////////////////////////////////////////////////////////////

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


    // Memory for the work registers of the CPU
    reg [31:0] r_RegisterBank[0:31];

    // Registers to hold the data of the currently selected work registers
    // (for the currently executed operation)
    reg [31:0] rs1 = 0;
    reg [31:0] rs2 = 0;


    ////////////////////////////////////////////////////////////
    // Load and store
    ////////////////////////////////////////////////////////////

    // The load instructions are of type I, while store are of type S.
    wire [31:0] w_load_or_store_address = rs1 + (w_is_store_instr ? w_S_type_imm : w_I_type_imm);

    // RISC-V is little endian (the little end comes first)
    // This means that it stores the least significant byte of a
    // word at the smallest memory address.
    // This has implications for how we determine which word and byte to load
    wire [15:0] w_load_halfword = w_load_or_store_address[1] ? i_mem_data[31:16] : i_mem_data[15:0];         // The upper or lower part of the word
    
    reg [7:0] w_load_byte;//      = w_load_or_store_address[0] ? w_load_halfword[15:8] : w_load_halfword[7:0]; // One of the 4 bytes in the word

    always @(*) begin
        case(w_load_or_store_address[1:0])
            2'b00: w_load_byte = i_mem_data[7:0];
            2'b01: w_load_byte = i_mem_data[15:8];
            2'b10: w_load_byte = i_mem_data[23:16];
            2'b11: w_load_byte = i_mem_data[31:24];
        endcase
    end

    reg [31:0] w_load_data;

    always @(*) begin
        case(w_funct3)
            3'b000: w_load_data = {{25{w_load_byte[7]}}, w_load_byte[6:0]};             // Load byte with sign extension
            3'b001: w_load_data = {{17{w_load_halfword[15]}}, w_load_halfword[14:0]};   // Load halfword with sign extension
            3'b010: w_load_data = i_mem_data;                                           // Load word
            3'b100: w_load_data = {{24{1'b0}},w_load_byte};                             // Load byte without sign extension 
            3'b101: w_load_data = {{16{1'b0}},w_load_halfword};                         // Load halfword without sign extension
            default: w_load_data = 0;
        endcase
    end

    // We have three store instructions
    // SB(rs2,rs1,imm) first 8 bytes of rs2 stored at address rs1+imm
    // SH(rs2,rs1,imm) first 16 bytes of rs2 stored at address rs1+imm
    // SW(rs2,rs1,imm) rs2 stored at address rs1+imm
    //
    // For the SB (Store byte), and SH (Store halfword) instructions we
    // have to look at the two LSB of the address to determine where in
    // the word the data should be placed.
    // The write mask ensures than only the intended part is actually
    // written.
    //
    // E.g.:
    // data = AABBCCDD
    //
    // LB:
    // 00 -> xxxxxxDD
    // 01 -> xxxxDDxx
    // 10 -> xxDDxxxx
    // 11 -> DDxxxxxx
    //
    // LH:
    // 00 -> xxxxCCDD
    // 10 -> CCDDxxxx
    //
    // LW:
    // 00 -> AABBCCDD

    reg [31:0] r_mem_write_data;

    always @(*) begin
       
        case (w_load_or_store_address[1:0])
            2'b00: r_mem_write_data[31:0] = {rs2[31:0]};
            2'b01: r_mem_write_data[31:0] = {rs2[31:24] ,rs2[23:16] ,rs2[7:0], rs2[7:0]};
            2'b10: r_mem_write_data[31:0] = {rs2[15:8] ,rs2[7:0] ,rs2[15:8] ,rs2[7:0]};
            2'b11: r_mem_write_data[31:0] = {rs2[7:0] ,rs2[7:0] ,rs2[7:0], rs2[7:0]};
        endcase
        
    end

    reg [3:0] w_byte_write_mask = 0;
    always @(*) begin
        case(w_load_or_store_address[1:0])
            2'b00 : w_byte_write_mask = 4'b0001;
            2'b01 : w_byte_write_mask = 4'b0010;
            2'b10 : w_byte_write_mask = 4'b0100;
            2'b11 : w_byte_write_mask = 4'b1000;
        endcase
    end

    wire [3:0] w_half_word_write_mask = w_load_or_store_address[1] ? 4'b1100 : 4'b0011;
    
    reg [3:0] r_store_write_mask;
    always @(*) begin
        case(w_funct3)
            3'b000: r_store_write_mask = w_byte_write_mask;      // Store byte
            3'b001: r_store_write_mask = w_half_word_write_mask; // Store halfword
            3'b010: r_store_write_mask = 4'b1111;                // Store word
            default: r_store_write_mask = 0;
        endcase
    end

    ////////////////////////////////////////////////////////////
    // State machine
    ////////////////////////////////////////////////////////////

    localparam STATE_FETCH_INSTR = 0;
    localparam STATE_WAIT_INSTR  = 1;
    localparam STATE_FETCH_REGS  = 2;
    localparam STATE_EXECUTE     = 3;
    localparam STATE_LOAD_DATA   = 4;
    localparam STATE_WAIT_DATA   = 5;
    localparam STATE_STORE_DATA  = 6;

    // Execution state register
    reg [2:0] r_state = STATE_FETCH_INSTR;

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

                `ifdef VERBOSE_TESTBENCH
                $display("x%0d <= %b", w_rd_param, w_return_data);
                `endif

                //if(w_rd_param == 10) begin
                //    o_Reg <= w_return_data;
                //end
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

                    if(w_is_load_instr)
                        r_state <= STATE_LOAD_DATA;
                    else if(w_is_store_instr)
                        r_state <= STATE_STORE_DATA;
                    else
                        r_state <= STATE_FETCH_INSTR;

                    //r_state <= w_is_load_instr ? STATE_LOAD_DATA : STATE_FETCH_INSTR;

                    `ifdef TESTBENCH
                    //$display("STATE_EXECUTE");
                    if(w_is_system_instr) begin 
                        $display("Finishing simulation due to SYSTEM instruction");
                        $finish();
                    end
                    `endif
                end
                STATE_LOAD_DATA: begin
                    r_state <= STATE_WAIT_DATA;
                end
                STATE_WAIT_DATA: begin
                    r_state <= STATE_FETCH_INSTR;
                end
                STATE_STORE_DATA: begin
                    r_state <= STATE_FETCH_INSTR;
                end
            endcase
        end
    end

    ////////////////////////////////////////////////////////////
    // ALU
    ////////////////////////////////////////////////////////////

    wire [31:0] w_ALU_in_1 = rs1;
    
    wire [31:0] w_ALU_in_2 = (w_is_ALU_reg_instr | w_is_branch_instr) ? rs2 : w_I_type_imm;

    reg [31:0] r_ALU_out;// = 0;
    wire [4:0] w_shift_amount = w_is_ALU_reg_instr ? rs2[4:0] : r_instr[24:20];


    // Size optimizations for the ALU
    // We reuse subtraction for comparison. In order for this to work we perform a 33 bit subtraction 
    // instead of 32 bit.
    wire [32:0] w_alu_minus = {1'b0, w_ALU_in_1} - {1'b0, w_ALU_in_2};
    wire w_equal                = (w_alu_minus[31:0] == 0); // Inputs must be equal if subracting one from the other gives us zero
    wire w_less_than_unsigned   = w_alu_minus[32];          // Unsigned comparison
    wire w_less_than_signed     = (w_ALU_in_1[31] ^ w_ALU_in_2[31]) ? w_ALU_in_1[31] : w_alu_minus[32]; // Signed comparison
    wire [31:0] w_alu_plus      = w_ALU_in_1 + w_ALU_in_2;

    // The ALU support 3 types of bit shift operations
    // In order to save area on our FPGA we use a single shifter + some tricks to perform all three operations
    // 
    // Arithmetic and logical left shift
    // Left shift is performed by first reversing the order of all bits, then shifting right, and finally reversing
    // the bit order back to the original state.
    // E.g.: left shift of 00001110 two places:
    // Original    Flipped      Right shift     Flipped again
    // 00001110 -> 01110000     -> 00011100     -> 00111000 
    //
    // 
    // Logical right shift:
    //
    //
    // Arithmetic right shift:
    // 
    function [31:0] flipped(input [31:0] in);
        begin
            flipped = { in[0],  in[1],  in[2],  in[3],  in[4],  in[5],
                        in[6],  in[7],  in[8],  in[9],  in[10], in[11],
                        in[12], in[13], in[14], in[15], in[16], in[17],
                        in[18], in[19], in[20], in[21], in[22], in[23],
                        in[24], in[25], in[26], in[27], in[28], in[29],
                        in[30], in[31]
                       };
        end
    endfunction


    wire [31:0] w_shifter_input = (w_funct3 == 3'b001) ? flipped(w_ALU_in_1) : w_ALU_in_1; // The input to the bit shifter should be the flipped version of the acutal input if we want to perform left shift
    
    // Bit 30 of the instruction determines if the shift is arithmetic or logical and bit 31 of the input is the sign bit
    // If both of them are high we perform arithmetic right shift
    // To avoid warnings from sim/synth tools we explicitly truncate away one bit
    wire [32:0] w_bit_shifter_temp = $signed({r_instr[30] & w_ALU_in_1[31], w_shifter_input}) >>> w_shift_amount; //w_ALU_in_2[4:0];
    wire [31:0] w_bit_shifter = w_bit_shifter_temp[31:0];

    // Flip the result if the operation was a left shift (the input was flipped before using the right shifter, and now we flip it back)
    wire [31:0] w_left_shift = flipped(w_bit_shifter);

    // ALU:
    always @(*) begin
        case (w_funct3)
            3'b000: r_ALU_out = (w_funct7[5] & r_instr[5]) ? (w_alu_minus[31:0]) : w_alu_plus; // ADD/SUB - Addition or subtraction
            3'b001: r_ALU_out = w_left_shift; //(w_ALU_in_1 << w_shift_amount); // SLL/SLLI - Left shift
            3'b010: r_ALU_out = {31'b0, w_less_than_signed};
            3'b011: r_ALU_out = {31'b0, w_less_than_unsigned}; // SLTU/SLTIU - Unsigned comparison
            3'b100: r_ALU_out = (w_ALU_in_1 ^ w_ALU_in_2); // XOR
            3'b101: r_ALU_out = w_bit_shifter; //w_funct7[5] ? ($signed(w_ALU_in_1) >>> w_shift_amount) : ($signed(w_ALU_in_1) >> w_shift_amount); // SRA/SRAI (arithmetic) SRL/SRLI (logical) - Logical (zero-fill) or arithmetic (sticky) right shift
            3'b110: r_ALU_out = (w_ALU_in_1 | w_ALU_in_2); // OR
            3'b111: r_ALU_out = (w_ALU_in_1 & w_ALU_in_2); // AND
        endcase
    end



    // ALU:
    //always @(*) begin
    //    case (w_funct3)
    //        // Bit 5 of w_funct7 is 1 for sub and 0 for add. Furthermore there is no subtract immediate instruction, and hence we only do subtract if bit 5 of the instruction register is set. Failure to check for this bit could lead to subtraction happening at random depending on the immediate value.
    //        3'b000: r_ALU_out = (w_funct7[5] & r_instr[5]) ? (w_ALU_in_1 - w_ALU_in_2) : (w_ALU_in_1 + w_ALU_in_2); // ADD/SUB - Addition or subtraction
    //        3'b001: r_ALU_out = (w_ALU_in_1 << w_shift_amount); // SLL/SLLI - Left shift
    //        3'b010: r_ALU_out = ($signed(w_ALU_in_1)) < ($signed(w_ALU_in_2)); // Signed comparison ($signed() evaluate the input expression and return a value with the same size and value of the input expression and signed datatype)
    //        3'b011: r_ALU_out = (w_ALU_in_1 < w_ALU_in_2); // SLTU/SLTIU - Unsigned comparison
    //        3'b100: r_ALU_out = (w_ALU_in_1 ^ w_ALU_in_2); // XOR
    //        3'b101: r_ALU_out = w_funct7[5] ? ($signed(w_ALU_in_1) >>> w_shift_amount) : ($signed(w_ALU_in_1) >> w_shift_amount); // SRA/SRAI (arithmetic) SRL/SRLI (logical) - Logical (zero-fill) or arithmetic (sticky) right shift
    //        3'b110: r_ALU_out = (w_ALU_in_1 | w_ALU_in_2); // OR
    //        3'b111: r_ALU_out = (w_ALU_in_1 & w_ALU_in_2); // AND
    //    endcase
    //end



    ////////////////////////////////////////////////////////////
    // Branching
    ////////////////////////////////////////////////////////////

    reg r_branch_criterion_fulfilled;
    always @(*) begin
        case(w_funct3)
            3'b000: r_branch_criterion_fulfilled = w_equal;
            3'b001: r_branch_criterion_fulfilled = !w_equal;
            3'b100: r_branch_criterion_fulfilled = w_less_than_signed;
            3'b101: r_branch_criterion_fulfilled = !w_less_than_signed;
            3'b110: r_branch_criterion_fulfilled = w_less_than_unsigned;
            3'b111: r_branch_criterion_fulfilled = !w_less_than_unsigned;
            default: r_branch_criterion_fulfilled = 1'b0;
        endcase
    end

    //always @(*) begin
    //    case(w_funct3)
    //        3'b000: r_branch_criterion_fulfilled = (rs1 == rs2);
    //        3'b001: r_branch_criterion_fulfilled = (rs1 != rs2);
    //        3'b100: r_branch_criterion_fulfilled = ($signed(rs1) < $signed(rs2));
    //        3'b101: r_branch_criterion_fulfilled = ($signed(rs1) >= $signed(rs2));
    //        3'b110: r_branch_criterion_fulfilled = (rs1 < rs2);
    //        3'b111: r_branch_criterion_fulfilled = (rs1 >= rs2);
    //        default: r_branch_criterion_fulfilled = 1'b0;
    //    endcase
    //end

    wire [31:0] w_PC_plus4 = r_PC + 4;

    wire [31:0] w_PC_plus_imm = r_PC + ( r_instr[3] ? w_J_type_imm[31:0] :
                                         r_instr[4] ? w_U_type_imm[31:0] :
                                         w_B_type_imm[31:0] );

    wire w_return_en;
    assign w_return_en =    ((r_state == STATE_EXECUTE) && 
                            (w_is_ALU_reg_instr || w_is_ALU_imm_instr || w_is_jump_link_instr || w_is_jump_link_r_instr || w_is_load_ui_instr || w_is_add_ui_pc_instr))  ||
                            (r_state == STATE_WAIT_DATA);

    wire [31:0] w_return_data;
    assign w_return_data = (w_is_jump_link_instr || w_is_jump_link_r_instr) ? (w_PC_plus4)  : // The jump and link instructions store the PC (+4) value before jumping
                            w_is_load_ui_instr ? w_U_type_imm                               : // Load 20 immediate bits left adjusted in the destination register
                            w_is_add_ui_pc_instr ? w_PC_plus_imm                            :
                            w_is_load_instr ? w_load_data                                   : // Load data from memory
                            r_ALU_out;


    //wire [31:0] w_return_data;
    //assign w_return_data = (w_is_jump_link_instr || w_is_jump_link_r_instr) ? (w_PC_plus4) : // The jump and link instructions store the PC (+4) value before jumping
    //                        w_is_load_ui_instr ? w_U_type_imm :                          // Load 20 immediate bits left adjusted in the destination register
    //                        w_is_add_ui_pc_instr ? (r_PC + w_U_type_imm) :               // 
    //                        r_ALU_out;


    wire [31:0] w_next_PC;
    assign w_next_PC =  ((w_is_branch_instr && r_branch_criterion_fulfilled) || w_is_jump_link_instr) ? w_PC_plus_imm : 
                        w_is_jump_link_r_instr                              ? {w_alu_plus[31:1],1'b0} :
                        w_PC_plus4;


    //wire [31:0] w_next_PC;
    //assign w_next_PC =  (w_is_branch_instr && r_branch_criterion_fulfilled) ? r_PC + w_B_type_imm : 
    //                    w_is_jump_link_instr                                ? r_PC + w_J_type_imm : 
    //                    w_is_jump_link_r_instr                              ? rs1 + w_I_type_imm :
    //                    r_PC + 4;


    ////////////////////////////////////////////////////////////
    // Testbench output
    ////////////////////////////////////////////////////////////

    `ifdef VERBOSE_TESTBENCH
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
                    w_is_load_instr:        $display("LOAD addr: %0d. rd=%d, rs1=%d, imm=%0d, funct3=%b", rs1 + w_I_type_imm, w_rd_param, w_rs1_param, w_I_type_imm, w_funct3);
                    w_is_store_instr:       $display("STORE addr: %0d. Data: %0d. rs1=%d, rs2=%d, imm=%0d, funct3=%b", rs1 + w_S_type_imm, rs2, w_rs1_param, w_rs2_param, w_S_type_imm, w_funct3);
                    //w_is_ebreak_instr:
                    //w_is_ecall_instr:
                    w_is_system_instr:      $display("SYSTEM instruction");
                endcase
            end
        end
    `endif

endmodule
