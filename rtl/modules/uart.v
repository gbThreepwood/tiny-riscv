`default_nettype none

module uart_tx #(
    parameter CLKS_PER_BIT = 217
)(
    input wire i_Rst_L,
    input wire i_Clk,
    input wire [7:0] i_TX_Byte,
    input wire i_TX_Start,      // Initiate TX of the data in the TX register
    output reg o_TX_InProgress,
    output reg o_TX_Done,
    output reg o_TX_Serial
);

// States
localparam IDLE         = 3'b000;
localparam TX_START_BIT = 3'b001;
localparam TX_DATA_BITS = 3'b010;
localparam TX_STOP_BIT  = 3'b011;
localparam FINISH       = 3'b100;

reg [2:0] r_TX_State = IDLE;    // Register for the current state
reg [2:0] r_Bit_Index = 0;      // Bit index register
reg [7:0] r_TX_Byte = 0;        // TX data register

// 217 clocks per bit.
// The function (e.g. in Python) floor(log2(217)) + 1 = 8 gives us the number of bits
// required to represent a given number.
// However since we start counting at zero, it is sometimes one less bit
// required for the counting compared to the number of bits required to
// represent the number.
//
// E.g. if CLKS_PER_BIT = 256 we need 8 bits to index the different numbers
// but 9 bits to repersent the acutal value: 0b1_0000_0000.
//
//reg [8:0] r_Clock_Count;
reg [$clog2(CLKS_PER_BIT):0] r_Clock_Count;

initial begin
    o_TX_Serial = 1'b1;
    o_TX_Done = 1'b1;
    o_TX_InProgress = 1'b0;
end

always @(posedge i_Clk or negedge i_Rst_L) begin

    if(~i_Rst_L) begin
        r_Clock_Count <= 0;
        r_Bit_Index <= 0;
        o_TX_Done <= 1'b1;
        o_TX_InProgress <= 1'b0;
        o_TX_Serial <= 1'b1; // Drive UART TX line high when idle
        r_TX_State <= IDLE;
    end
    else begin
        case (r_TX_State)
        IDLE:
            begin

                o_TX_Serial <= 1'b1; // Drive line high for idle condition
                r_Clock_Count <= 0;
                r_Bit_Index <= 0;

                if(i_TX_Start == 1'b1) begin
                    r_TX_Byte <= i_TX_Byte; // Copy the data from the input to the TX register

                    o_TX_InProgress <= 1'b1;
                    o_TX_Done <= 1'b0;

                    r_TX_State <= TX_START_BIT;
                end
            end
        TX_START_BIT:
            begin

                o_TX_Serial <= 1'b0; // Set line low to begin a start bit

                if(r_Clock_Count == (CLKS_PER_BIT - 1)) begin // Wait for one bit duration
                    r_Clock_Count <= 0;
                    r_TX_State <= TX_DATA_BITS;
                end
                else begin
                   r_Clock_Count <= r_Clock_Count + 1;
                end

            end
        TX_DATA_BITS:
            begin

                o_TX_Serial <= r_TX_Byte[r_Bit_Index];

                if(r_Clock_Count == (CLKS_PER_BIT - 1)) begin // Wait for one bit duration
                    r_Clock_Count <= 0;

                    if (r_Bit_Index < 7) begin
                        r_Bit_Index <= r_Bit_Index + 1;
                    end
                    else begin
                        //r_Bit_Index <= 1'b0;
                        r_TX_State <= TX_STOP_BIT;
                    end
                end
                else begin
                   r_Clock_Count <= r_Clock_Count + 1;
                end

            end
        TX_STOP_BIT:
            begin

                o_TX_Serial <= 1'b1; // Set the serial line high to indicate a stop bit

                if(r_Clock_Count == (CLKS_PER_BIT - 1)) begin // Wait for one bit duration
                    r_Clock_Count <= 0;
                    r_TX_State <= FINISH;
                end
                else begin
                   r_Clock_Count <= r_Clock_Count + 1;
                end

            end
        FINISH:
            begin

                o_TX_InProgress <= 1'b0;
                o_TX_Done <= 1'b1;
                r_TX_State <= IDLE;

            end
        default:
            r_TX_State <= IDLE;
        endcase
    end

end


endmodule
