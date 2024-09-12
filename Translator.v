`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Michael Rosales
// 
// Create Date: 05/02/2024 03:04:28 PM
// Design Name: 
// Module Name: Morse Code Translator
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module Translator(
    input clk,
    input rreset, rdot, rdash, rnone,
    output ampPWM,
    output ampSD,
    input UART_RX,
    output UART_TX,
    input [15:2] SW,
    input mode,decode,
    output LED,
    output [4:0] FSMLED
    );
    wire [7:0] sinpos1, envout;
    wire [7:0] rx_byte;
    wire start, done;
    wire [10:0] freq;
    wire pwm;
    wire [9:0] character, char;
    wire rx_dv;  
    wire tx_ready;  
    wire tx_dv; 
    wire [7:0] tx_byte;  
    assign freq = 600;
    assign ampPWM = (pwm) ? 1'bz : 1'b0;
    assign ampSD = 1'b1;
   
    button_pulse u1 (.clk(clk), .raw_button(rreset), .button_pulse(reset));
    button_pulse u2 (.clk(clk), .raw_button(rdot), .button_pulse(dot));
    button_pulse u3 (.clk(clk), .raw_button(rdash), .button_pulse(dash));
    button_pulse u4 (.clk(clk), .raw_button(rnone), .button_pulse(none));
    uart_tx_vlog tx0 (clk, tx_dv, tx_byte, UART_TX, tx_done, tx_ready);
    uart_rx_vlog rx0 (clk, UART_RX, rx_dv, rx_byte);
    rx_parse_note rxp (clk, reset, rx_dv, rx_byte, start, character);
    tx_parse print (.clk(clk),.decode(decode), .done(done), .reset(reset),.tx_dv(tx_dv), .tx_byte(tx_byte), .tx_ready(tx_ready),.morsecode(char),.char_morsecode(character),.start_send(start));
    morse_fsm morse (clk, reset, decode, dot, dash, none, char, character, {2'b00, SW[10:2]}, SW[15:11], SW[15:11], start, envout, mode, FSMLED, LED, done);
    sine_gen sigen(clk, reset, (freq), envout, sinpos1);
    pwm_gen pwmg(clk, reset, sinpos1, pwm);
    
endmodule

module morse_fsm(clk, reset, decode, dot, dash, none, char, character, N_i, begin_i, end_i, start, envout, mode, fsmled, led, done);
input clk, reset;
   input start, decode, dot, dash, none;
   input [9:0] character;
   input [5:0] begin_i, end_i;
   input [10:0] N_i;
   input mode;
   output reg [7:0] envout;
   output reg [4:0] fsmled;
   output reg led, done;
   output reg [9:0] char;
   reg running;
   reg [7:0]  beginv, endv;
   reg [10:0] N;
   reg [10:0] Ncnt;
   reg [15:0] clkcnt;
   wire [31:0] m, mtop, mx, envout_calc;
   reg dstart, mstart1;
   wire dcomp, dovf, mcomp1, movf1;

   qadd #(20,32) subm ({1'b0, {3'b0,endv}, 20'b0}, {1'b1, {3'b0,beginv}, 20'b0}, mtop);
   qdiv #(20,32) mcalc (mtop, {1'b0, N, 20'b0}, dstart, clk, m, dcomp, dovf);

   qmults #(20,32) mxcalc (m, {1'b0, Ncnt, 20'b0}, mstart1, clk, mx, mcomp1, movf1);
   qadd #(20,32) addb (mx, {1'b0, {3'b0,beginv}, 20'b0}, envout_calc);
   
    parameter IDLE = 0;
    parameter CHECK1 = 1; // checks char bits 9-8
    parameter SPACE1 = 2;
    parameter CHECK2 = 3; // checks char bits 7-6
    parameter SPACE2 = 4;
    parameter CHECK3 = 5; // checks char bits 5-4
    parameter SPACE3 = 6;
    parameter CHECK4 = 7; // checks char bits 3-2
    parameter SPACE4 = 8;
    parameter CHECK5 = 9; // checks char bits 1-0
    parameter SPACE5 = 10;
    parameter TRANS1 = 11;
    parameter TRANS2 = 12; 
    parameter TRANS3 = 13;
    parameter TRANS4 = 14; 
    parameter TRANS5 = 15;
    
    
   reg [3:0] state;
   reg [3:0] next_state;
   reg [7:0] next_endv, next_beginv;
   reg [10:0] next_N;
   reg [9:0] next_char;
   reg next_running;
   
   always@(posedge clk or posedge reset)
     begin
        if (reset) begin
            state <= IDLE;
            envout <= 0;
            clkcnt <= 0;
            Ncnt <= 0;
            dstart <= 0;
            mstart1 <= 0;
            char <= 0;
            
        end else begin
            dstart <= 0;
            mstart1 <= 0;
            state <= next_state;
            endv <= next_endv;
            beginv <= next_beginv;
            N <= next_N;
            running <= next_running;
            char <= next_char;
            if(start) begin
                Ncnt <= 0;
                clkcnt <= 0;
                //envout <= beginv;     
            end else if (running) begin
                clkcnt <= clkcnt + 1;
                if (clkcnt == 16'hFFFF) begin
                   if (Ncnt >= N) begin
                      envout <= endv;
                      clkcnt <= 0;    
                      Ncnt <= 0;
                      
                   end else begin
                      envout <= envout_calc[27:20];
                      Ncnt <= Ncnt + 1;
                   end
                end
                if (clkcnt == 16'h8100) begin
                   mstart1 <= 1;  // recalculte output
                end
                if (Ncnt == 0 && clkcnt == 16'h0001) begin 
                   dstart <= 1;   // recalculate m once per envelope start
                end
            end 
	     end
     end
     
     always @(*) begin
     next_state = state;
     next_endv = endv;
     next_beginv = beginv;
     next_N = N;
     next_running = running;
     next_char = char;
      
     if (reset)
        next_state = IDLE;
     else begin
     case(state)
        IDLE: begin
            fsmled = 5'b00000;
            led = 1'b0;
            next_running = 0;
            next_beginv = 0;
            next_endv = 0;
            next_N = 0;
            if (start) begin
                next_running = 1;
                next_state = CHECK1;
            end
            else if(decode)
                next_state = TRANS1;
        end
       TRANS1: begin
           fsmled = char[9:5];
           if (reset)
               next_state = IDLE;
           else if(dot || dash || none) begin
               if(dot)next_char[1:0] = 2'b01;
               else if(dash)next_char[1:0] = 2'b11;
               else if(none)next_char[1:0] =2'b00;
               next_state = TRANS2; // Move to next TRANS state
           end
       end
       TRANS2: begin
           fsmled = 5'b00010;
           done = 0;
           if (reset)
               next_state = IDLE;
           else if(dot || dash || none) begin
               if(dot) begin
                   next_char[3:2] = 2'b01;
               end
               else if(dash) begin
                   next_char[3:2] = 2'b11;
               end
               else if(none)next_char[3:2] = 2'b00;
               next_state = TRANS3; // Move to next TRANS state
           end
       end
       TRANS3: begin
           fsmled = 5'b00100;
           if (reset)
               next_state = IDLE;
           else if(dot || dash || none) begin
               if(dot) begin
                   next_char[5:4] = 2'b01;
               end
               else if(dash) begin
                   next_char[5:4] = 2'b11;
               end
               else if(none)next_char[5:4] =2'b00;
               next_state = TRANS4; // Move to next TRANS state
           end
       end
       TRANS4: begin
           fsmled = 5'b01000;
           if (reset)
               next_state = IDLE;
           else if(dot || dash || none) begin
               if(dot) begin
                   next_char[7:6] = 2'b01;
               end
               else if(dash) begin
                   next_char[7:6] = 2'b11;
               end
               else if(none)next_char[7:6] =2'b00;
               next_state = TRANS5; // Move to next TRANS state
           end
       end
       TRANS5: begin
           fsmled = 5'b10000;
           if (reset)
               next_state = IDLE;
           else if(dot || dash || none) begin
               if(dot) begin
                   next_char[9:8] = 2'b01;
               end
               else if(dash) begin
                   next_char[9:8] = 2'b11;
               end
               else if(none)next_char[9:8] =2'b00;
               done = 1;
               next_state = IDLE; // Return to IDLE state
           end
           
       end
 
        CHECK1: begin   // bits 9-8
            fsmled = 5'b00001;
            led = (mode == 1'b1)? 1'b1 : 1'b0;
        if(start)
        next_state = CHECK1;
        else begin
            if(character[9:8] == 2'b11)begin
                next_N = N_i*3;
                next_endv = (mode == 1'b1)? 8'h00 : {end_i, 2'b00};
                next_beginv = (mode == 1'b1)? 8'h00 : {begin_i, 2'b00}; 
                    if (clkcnt == 16'hFFFF && Ncnt >= next_N)
                        next_state = SPACE1;  
            end else if(character[9:8] == 2'b01)begin
                next_N = N_i;
                next_endv = (mode == 1'b1)? 8'h00 : {end_i, 2'b00};
                next_beginv = (mode == 1'b1)? 8'h00 : {begin_i, 2'b00};
                    if (clkcnt == 16'hFFFF && Ncnt >= next_N)
                        next_state = SPACE1;
            end else if(character[9:8] == 2'b00) begin
                next_state = CHECK2;
            end 
        end
        end
        SPACE1:begin
            fsmled = 5'b00000;
            led = 1'b0;
            next_N = N_i;
            next_endv = 8'h00;
            next_beginv = 8'h00; 
                if (clkcnt == 16'hFFFF && Ncnt >= next_N)
                    next_state = CHECK2;  
        end
        CHECK2: begin // bits 7-6
            fsmled = 5'b00010;
            led = (mode == 1'b1)? 1'b1 : 1'b0;
        if(start)
        next_state = CHECK1;
        else begin
            if(character[7:6] == 2'b11)begin
                next_N = N_i*3;
                next_endv = (mode == 1'b1)? 8'h00 : {end_i, 2'b00};
                next_beginv = (mode == 1'b1)? 8'h00 : {begin_i, 2'b00}; 
                    if (clkcnt == 16'hFFFF && Ncnt >= next_N)
                        next_state = SPACE2;  
            end else if(character[7:6] == 2'b01)begin
                next_N = N_i;
                next_endv = (mode == 1'b1)? 8'h00 : {end_i, 2'b00};
                next_beginv = (mode == 1'b1)? 8'h00 : {begin_i, 2'b00};
                    if (clkcnt == 16'hFFFF && Ncnt >= next_N)
                        next_state = SPACE2;
            end else if(character[7:6] == 2'b00) begin
                next_state = CHECK3;
            end 
        end
        end
       SPACE2:begin
       fsmled = 5'b00011;
       led = 1'b0;
            next_N = N_i;
            next_endv = 8'h00;
            next_beginv = 8'h00; 
                if (clkcnt == 16'hFFFF && Ncnt >= next_N)
                    next_state = CHECK3;  
        end
        CHECK3: begin // bits 5-4
            fsmled = 5'b00100;
            led = (mode == 1'b1)? 1'b1 : 1'b0;
        if(start) 
            next_state = CHECK1;
        else begin
            if(character[5:4] == 2'b11)begin
                next_N = N_i*3;
                next_endv = (mode == 1'b1)? 8'h00 : {end_i, 2'b00};
                next_beginv = (mode == 1'b1)? 8'h00 : {begin_i, 2'b00}; 
                    if (clkcnt == 16'hFFFF && Ncnt >= next_N)
                        next_state = SPACE3;  
            end else if(character[5:4] == 2'b01)begin
                next_N = N_i;
                next_endv = (mode == 1'b1)? 8'h00 : {end_i, 2'b00};
                next_beginv = (mode == 1'b1)? 8'h00 : {begin_i, 2'b00};
                    if (clkcnt == 16'hFFFF && Ncnt >= next_N)
                        next_state = SPACE3;  
            end else if(character[5:4] == 2'b00) begin
                next_state = CHECK4;
            end 
        end
        end
        SPACE3:begin
            fsmled = 5'b00101;
            led = 1'b0;
            next_N = N_i;
            next_endv = 8'h00;
            next_beginv = 8'h00; 
                if (clkcnt == 16'hFFFF && Ncnt >= next_N)
                    next_state = CHECK4;
        end
        CHECK4: begin //bits 3-2
            fsmled = 5'b01000;
            led = (mode == 1'b1)? 1'b1 : 1'b0;
        if(start)
            next_state = CHECK1;
        else begin
            if(character[3:2] == 2'b11)begin
                next_N = N_i*3;
                next_endv = (mode == 1'b1)? 8'h00 : {end_i, 2'b00};
                next_beginv = (mode == 1'b1)? 8'h00 : {begin_i, 2'b00}; 
                if (clkcnt == 16'hFFFF && Ncnt >= next_N)
                next_state = SPACE4;
            end else if(character[3:2] == 2'b01)begin
                next_N = N_i;
                next_endv = (mode == 1'b1)? 8'h00 : {end_i, 2'b00};
                next_beginv = (mode == 1'b1)? 8'h00 : {begin_i, 2'b00};
                if (clkcnt == 16'hFFFF && Ncnt >= next_N)
                next_state = SPACE4;
            end else if(character[3:2] == 2'b00) begin
                next_state = CHECK5;  
            end
        end
        end
        SPACE4: begin
        fsmled = 5'b01001;
        led = 1'b0;
            next_N = N_i;
            next_endv = 8'h00;
            next_beginv = 8'h00; 
                if (clkcnt == 16'hFFFF && Ncnt >= next_N)
                    next_state = CHECK5;
        end
        CHECK5: begin // bits 1-0
            fsmled = 5'b10000;
            led = (mode == 1'b1)? 1'b1 : 1'b0;
        if(start)
            next_state = CHECK1;
        else begin
            if(character[1:0] == 2'b11)begin
                next_N = N_i*3;
                next_endv = (mode == 1'b1)? 8'h00 : {end_i, 2'b00};
                next_beginv = (mode == 1'b1)? 8'h00 : {begin_i, 2'b00}; 
                    if (clkcnt == 16'hFFFF && Ncnt >= next_N)begin
                        next_state = SPACE5;   
                    end
            end else if(character[1:0] == 2'b01)begin
                next_N = N_i;
                next_endv = (mode == 1'b1)? 8'h00 : {end_i, 2'b00};
                next_beginv = (mode == 1'b1)? 8'h00 : {begin_i, 2'b00};
                    if (clkcnt == 16'hFFFF && Ncnt >= next_N)begin
                        
                        next_state = SPACE5;
                    end
            end else if(character[1:0] == 2'b00) begin
                next_state = IDLE;
            end 
        end
        end
        SPACE5: begin
        fsmled = 5'b11000;
        led = 1'b0;
            next_N = N_i;
            next_endv = 8'h00;
            next_beginv = 8'h00; 
                if (clkcnt == 16'hFFFF && Ncnt >= next_N)begin
                    next_running = 0;
                    next_state = IDLE;
                    end
        end

        default: begin
        end
     endcase
     
     end
     end
endmodule

module rx_parse_note(
   input clk,
   input reset, 
   input rx_dv,
   input [7:0] rx_byte, 
   output reg start,
   output reg [9:0] morsecode
   );
   
   // 11 dash
   // 10 slash
   // 01 dot
   // 00 clear

    always @(posedge clk or posedge reset)
    begin
        if (reset) begin
           start <= 0;
        end else begin
           if (rx_dv) begin
             start <= 1;
             case (rx_byte)
               "A": morsecode <= 10'b0111000000;
               "B": morsecode <= 10'b1101010100;
               "C": morsecode <= 10'b1101110100;
               "D": morsecode <= 10'b1101010000;
               "E": morsecode <= 10'b0100000000;
               "F": morsecode <= 10'b0101110100;
               "G": morsecode <= 10'b1111010000;
               "H": morsecode <= 10'b0101010100;
               "I": morsecode <= 10'b0101000000;
               "J": morsecode <= 10'b0111111100;
               "K": morsecode <= 10'b1101110000;
               "L": morsecode <= 10'b0111010100;
               "M": morsecode <= 10'b1111000000;
               "N": morsecode <= 10'b1101000000;
               "O": morsecode <= 10'b1111110000;
               "P": morsecode <= 10'b0111110100;
               "Q": morsecode <= 10'b1111011100;
               "R": morsecode <= 10'b0111010000;
               "S": morsecode <= 10'b0101010000;
               "T": morsecode <= 10'b1100000000;
               "U": morsecode <= 10'b0101110000;
               "V": morsecode <= 10'b0101011100;
               "W": morsecode <= 10'b0111110000;
               "X": morsecode <= 10'b1101011100;
               "Y": morsecode <= 10'b1101111100;
               "Z": morsecode <= 10'b1111010100;
               "0": morsecode <= 10'b1111111111;
               "1": morsecode <= 10'b0111111111;
               "2": morsecode <= 10'b0101111111;
               "3": morsecode <= 10'b0101011111;
               "4": morsecode <= 10'b0101010111;
               "5": morsecode <= 10'b0101010101;
               "6": morsecode <= 10'b1101010101;
               "7": morsecode <= 10'b1111010101;
               "8": morsecode <= 10'b1111110101;
               "9": morsecode <= 10'b1111111101;
               //8'h20: morsecode <= 10'b1000000000; // space
             endcase  
           end else begin
             start <= 0;
           end
        end
    end

endmodule

module tx_parse(
   input clk, decode, done,
   input reset,
   output tx_dv,  // Changed to reg since we're driving it in always block
   output reg [7:0] tx_byte, 
   input tx_ready, 
   input [9:0] morsecode, char_morsecode,
   input start_send // New input to trigger sending
);

   reg [3:0] char_cnt = 0;
   reg prev_tx_ready;

       always @(*)
        case(char_cnt)
            8: tx_byte = (morsecode == 10'b0101100000)? "A" :
                            (morsecode == 10'b1001010100)? "B" :
                            (morsecode == 10'b1001100100)? "C" :
                            (morsecode == 10'b1001010000)? "D" :
                            (morsecode == 10'b0100000000)? "E" :
                            (morsecode == 10'b0100110100)? "F" :
                            (morsecode == 10'b1101010000)? "G" :
                            (morsecode == 10'b0101010100)? "H" :
                            (morsecode == 10'b0101000000)? "I" :
                            (morsecode == 10'b0111110000)? "J" :
                            (morsecode == 10'b1011100000)? "K" :
                            (morsecode == 10'b0101010100)? "L" :
                            (morsecode == 10'b1101000000)? "M" :
                            (morsecode == 10'b1010000000)? "N" :
                            (morsecode == 10'b1110100000)? "O" :
                            (morsecode == 10'b0101110100)? "P" :
                            (morsecode == 10'b1101011100)? "Q" :
                            (morsecode == 10'b0101010000)? "R" :
                            (morsecode == 10'b0101010000)? "S" :
                            (morsecode == 10'b1000000000)? "T" :
                            (morsecode == 10'b0101011000)? "U" :
                            (morsecode == 10'b0101011100)? "V" :
                            (morsecode == 10'b0111010000)? "W" :
                            (morsecode == 10'b1001101000)? "X" :
                            (morsecode == 10'b1011101000)? "Y" :
                            (morsecode == 10'b1101101000)? "Z" :
                            (morsecode == 10'b1111110000)? "0" :
                            (morsecode == 10'b0111110000)? "1" :
                            (morsecode == 10'b0101110000)? "2" :
                            (morsecode == 10'b0101010000)? "3" :
                            (morsecode == 10'b0101001000)? "4" :
                            (morsecode == 10'b0101000100)? "5" :
                            (morsecode == 10'b1001010000)? "6" :
                            (morsecode == 10'b1101010000)? "7" :
                            (morsecode == 10'b1111010000)? "8" :
                            (morsecode == 10'b1111100000)? "9" :
                            8'h20; // for unrecognized
            7: tx_byte = (char_morsecode[9:8] == 2'b11)? 8'h2D : (char_morsecode[9:8] == 2'b01)? 8'h2E : 8'h20;
            6: tx_byte = (char_morsecode[7:6] == 2'b11)? 8'h2D : (char_morsecode[7:6] == 2'b01)? 8'h2E : 8'h20;
            5: tx_byte = (char_morsecode[5:4] == 2'b11)? 8'h2D : (char_morsecode[5:4] == 2'b01)? 8'h2E : 8'h20;
            4: tx_byte = (char_morsecode[3:2] == 2'b11)? 8'h2D : (char_morsecode[3:2] == 2'b01)? 8'h2E : 8'h20;
            3: tx_byte = (char_morsecode[1:0] == 2'b11)? 8'h2D : (char_morsecode[1:0] == 2'b01)? 8'h2E : 8'h20;
            2: tx_byte = 8'h0D;  // carriage return
            1: tx_byte = 8'h0A;  // line feed
            default: tx_byte = 0;
        endcase
assign tx_dv = (tx_ready && (char_cnt || done != 0));

    always @(posedge clk or posedge reset)
    begin
        if (reset) begin
            char_cnt <= 0;
            prev_tx_ready <= 0;
        end else if (start_send && char_cnt == 0) begin
            if(decode)
                char_cnt <= 8; // Start sending characters   
            else
                char_cnt <= 7;
        end else if (tx_ready && prev_tx_ready && char_cnt != 0) begin
            char_cnt <= char_cnt - 1;
            if(decode)begin
                if(char_cnt == 7 )
                char_cnt <= 2;
            end
            else
            char_cnt <= char_cnt - 1;     
        end
        prev_tx_ready <= tx_ready;
    end
endmodule

module uart_tx_vlog 
  (
   input       i_Clock,           // master clock
   input       i_Tx_DV,           // data valid - set high to send byte
   input [7:0] i_Tx_Byte,         // byte to send
   output reg  o_Tx_Serial,       // UART serial TX wire
   output      o_Tx_Done,         // done signal, goes high for 2 clk cycles when finished sending
   output      o_Tx_Ready         // when high, UART TX is ready to send (not busy)
   );
  
  localparam s_IDLE         = 3'b000;
  localparam s_TX_START_BIT = 3'b001;
  localparam s_TX_DATA_BITS = 3'b010;
  localparam s_TX_STOP_BIT  = 3'b011;
  localparam s_CLEANUP      = 3'b100;
  localparam CLKS_PER_BIT   = 14'b10100010110000;   // hard coded to 9600 baud from the 100 MHz master clock
                                                    // i.e. 100 MHz / 9600 Hz = 10416 = 14'b10100010110000
   
  reg [2:0]    r_SM_Main     = 0;
  reg [13:0]    r_Clock_Count = 0;
  reg [2:0]    r_Bit_Index   = 0;
  reg [7:0]    r_Tx_Data     = 0;
  reg          r_Tx_Done     = 0;
  reg          r_Tx_Active   = 0;
     
  always @(posedge i_Clock)
    begin
       
      case (r_SM_Main)
        s_IDLE :
          begin
            o_Tx_Serial   <= 1'b1;         // Drive Line High for Idle
            r_Tx_Done     <= 1'b0;
            r_Clock_Count <= 0;
            r_Bit_Index   <= 0;
             
            if (i_Tx_DV == 1'b1)
              begin
                r_Tx_Active <= 1'b1;
                r_Tx_Data   <= i_Tx_Byte;
                r_SM_Main   <= s_TX_START_BIT;
              end
            else
              r_SM_Main <= s_IDLE;
          end // case: s_IDLE
         
         
        // Send out Start Bit. Start bit = 0
        s_TX_START_BIT :
          begin
            o_Tx_Serial <= 1'b0;
             
            // Wait CLKS_PER_BIT-1 clock cycles for start bit to finish
            if (r_Clock_Count < CLKS_PER_BIT-1)
              begin
                r_Clock_Count <= r_Clock_Count + 1;
                r_SM_Main     <= s_TX_START_BIT;
              end
            else
              begin
                r_Clock_Count <= 0;
                r_SM_Main     <= s_TX_DATA_BITS;
              end
          end // case: s_TX_START_BIT
         
         
        // Wait CLKS_PER_BIT-1 clock cycles for data bits to finish         
        s_TX_DATA_BITS :
          begin
            o_Tx_Serial <= r_Tx_Data[r_Bit_Index];
             
            if (r_Clock_Count < CLKS_PER_BIT-1)
              begin
                r_Clock_Count <= r_Clock_Count + 1;
                r_SM_Main     <= s_TX_DATA_BITS;
              end
            else
              begin
                r_Clock_Count <= 0;
                 
                // Check if we have sent out all bits
                if (r_Bit_Index < 7)
                  begin
                    r_Bit_Index <= r_Bit_Index + 1;
                    r_SM_Main   <= s_TX_DATA_BITS;
                  end
                else
                  begin
                    r_Bit_Index <= 0;
                    r_SM_Main   <= s_TX_STOP_BIT;
                  end
              end
          end // case: s_TX_DATA_BITS
         
         
        // Send out Stop bit.  Stop bit = 1
        s_TX_STOP_BIT :
          begin
            o_Tx_Serial <= 1'b1;
             
            // Wait CLKS_PER_BIT-1 clock cycles for Stop bit to finish
            if (r_Clock_Count < CLKS_PER_BIT-1)
              begin
                r_Clock_Count <= r_Clock_Count + 1;
                r_SM_Main     <= s_TX_STOP_BIT;
              end
            else
              begin
                r_Tx_Done     <= 1'b1;
                r_Clock_Count <= 0;
                r_SM_Main     <= s_CLEANUP;
                r_Tx_Active   <= 1'b0;
              end
          end // case: s_Tx_STOP_BIT
         
         
        // Stay here 1 clock
        s_CLEANUP :
          begin
            r_Tx_Done <= 1'b1;
            r_SM_Main <= s_IDLE;
          end
         
         
        default :
          r_SM_Main <= s_IDLE;
         
      endcase
    end
 
  assign o_Tx_Done   = r_Tx_Done;
  assign o_Tx_Ready = ~r_Tx_Active;  
endmodule

module uart_rx_vlog 
  (
   input        i_Clock,           // master clock
   input        i_Rx_Serial,       // UART serial RX wire
   output       o_Rx_DV,           // data valid - goes high for 1 clk cycle when data byte has been received
   output [7:0] o_Rx_Byte          // byte received
   );

  localparam CLKS_PER_BIT = 14'b10100010110000;   
  localparam s_IDLE         = 3'b000;
  localparam s_RX_START_BIT = 3'b001;
  localparam s_RX_DATA_BITS = 3'b010;
  localparam s_RX_STOP_BIT  = 3'b011;
  localparam s_CLEANUP      = 3'b100;
   
  reg           r_Rx_Data_R = 1'b1;
  reg           r_Rx_Data   = 1'b1;
   
  reg [13:0]     r_Clock_Count = 0;
  reg [2:0]     r_Bit_Index   = 0; //8 bits total
  reg [7:0]     r_Rx_Byte     = 0;
  reg           r_Rx_DV       = 0;
  reg [2:0]     r_SM_Main     = 0;
   
  // Purpose: Double-register the incoming data.
  // This allows it to be used in the UART RX Clock Domain.
  // (It removes problems caused by metastability)
  always @(posedge i_Clock)
    begin
      r_Rx_Data_R <= i_Rx_Serial;
      r_Rx_Data   <= r_Rx_Data_R;
    end
   
   
  // Purpose: Control RX state machine
  always @(posedge i_Clock)
    begin
       
      case (r_SM_Main)
        s_IDLE :
          begin
            r_Rx_DV       <= 1'b0;
            r_Clock_Count <= 0;
            r_Bit_Index   <= 0;
             
            if (r_Rx_Data == 1'b0)          // Start bit detected
              r_SM_Main <= s_RX_START_BIT;
            else
              r_SM_Main <= s_IDLE;
          end
         
        // Check middle of start bit to make sure it's still low
        s_RX_START_BIT :
          begin
            if (r_Clock_Count == (CLKS_PER_BIT-1)/2)
              begin
                if (r_Rx_Data == 1'b0)
                  begin
                    r_Clock_Count <= 0;  // reset counter, found the middle
                    r_SM_Main     <= s_RX_DATA_BITS;
                  end
                else
                  r_SM_Main <= s_IDLE;
              end
            else
              begin
                r_Clock_Count <= r_Clock_Count + 1;
                r_SM_Main     <= s_RX_START_BIT;
              end
          end // case: s_RX_START_BIT
         
         
        // Wait CLKS_PER_BIT-1 clock cycles to sample serial data
        s_RX_DATA_BITS :
          begin
            if (r_Clock_Count < CLKS_PER_BIT-1)
              begin
                r_Clock_Count <= r_Clock_Count + 1;
                r_SM_Main     <= s_RX_DATA_BITS;
              end
            else
              begin
                r_Clock_Count          <= 0;
                r_Rx_Byte[r_Bit_Index] <= r_Rx_Data;
                 
                // Check if we have received all bits
                if (r_Bit_Index < 7)
                  begin
                    r_Bit_Index <= r_Bit_Index + 1;
                    r_SM_Main   <= s_RX_DATA_BITS;
                  end
                else
                  begin
                    r_Bit_Index <= 0;
                    r_SM_Main   <= s_RX_STOP_BIT;
                  end
              end
          end // case: s_RX_DATA_BITS
     
     
        // Receive Stop bit.  Stop bit = 1
        s_RX_STOP_BIT :
          begin
            // Wait CLKS_PER_BIT-1 clock cycles for Stop bit to finish
            if (r_Clock_Count < CLKS_PER_BIT-1)
              begin
                r_Clock_Count <= r_Clock_Count + 1;
                r_SM_Main     <= s_RX_STOP_BIT;
              end
            else
              begin
                r_Rx_DV       <= 1'b1;
                r_Clock_Count <= 0;
                r_SM_Main     <= s_CLEANUP;
              end
          end // case: s_RX_STOP_BIT
     
         
        // Stay here 1 clock
        s_CLEANUP :
          begin
            r_SM_Main <= s_IDLE;
            r_Rx_DV   <= 1'b0;
          end
         
         
        default :
          r_SM_Main <= s_IDLE;
         
      endcase
    end   
   
  assign o_Rx_DV   = r_Rx_DV;
  assign o_Rx_Byte = r_Rx_Byte;
   
endmodule // uart_rx

module button_pulse(
		    input clk,
		    input raw_button,
		    output button_pulse
		    );
    
    localparam N = 3;
    
    reg [N - 1:0] Q_reg;
    
    always @(posedge clk)
    begin
        Q_reg <= {Q_reg[N - 2:0], raw_button};
    end
    
    assign button_pulse = (&Q_reg[N - 2:0]) & ~Q_reg[N-1];
endmodule

module pwm_gen(clk, reset, inp, pwm); 
   input clk, reset;
   input [7:0] inp;
   output pwm;

   reg [7:0] pwmcnt;
   wire	       pwm;

   assign pwm = (pwmcnt < inp) ? 1 : 0;

  always@(posedge clk or posedge reset)
     begin
         if (reset) begin
	        pwmcnt <= 0;
         end else begin
	        pwmcnt <= pwmcnt + 1;
	     end
     end

endmodule

module sine_gen(clk, reset, N, ampl, sinpos);
   input clk, reset;
   input [10:0] N;  
   input [7:0] ampl;
   output [7:0] sinpos;

   wire [31:0] sin_out, cos_out;
   reg [31:0] sin_r, cos_r;
   reg [10:0] Ncnt;
   reg [7:0] pwmcnt;
   wire [31:0] a, delsin, delcos, sinfpos;
   wire [7:0] sinpos;

   reg	       dstart, mstart1, mstart2;
   wire	       dcomp, dovf, mcomp1, movf1, mcomp2, movf2;
   wire [6:0]  ampl_div_2;

   assign ampl_div_2 = ampl >> 1;  // the sine amplitude is 1/2 the peak-to-peak value

// a = (2 * Pi) / N
//   2 * Pi shifted left by 20 bits = 6588397 = 0x006_487ED in 20,32 format
//   N in 20,32 format: {1'b0, N, 20'b0}   

   qdiv #(20,32) acalc (32'h006_487ED, {1'b0, N, 20'b0}, dstart, clk, a, dcomp, dovf);

//   sin_out = sin_r + a*(cos_r);
//   cos_out = cos_r - a*(sin_r);
   
// its a bit wasteful to use two multipliers, we could multiplex into one,
// but we have plenty of FPGA resouces on this chip...   

   qmults #(20,32) dscalc (a, cos_r, mstart1, clk, delsin, mcomp1, movf1);

   qmults #(20,32) dccalc (a, sin_r, mstart2, clk, delcos, mcomp2, movf2);

   qadd #(20,32) adds (sin_r, delsin, sin_out);

   qadd #(20,32) subc (cos_r, {~delcos[31],delcos[30:0]}, cos_out);

// sin function is positive and negative
// add an offset so the sinpos is always positive
   
   qadd #(20,32) addos (sin_r, {1'b0, {4'b0,ampl_div_2} + 11'h1, 20'b0}, sinfpos);
   
   assign sinpos = sinfpos[31] ? 0 : sinfpos[27:20];  // output the integer part, clip to zero if negative
   
   always@(posedge clk or posedge reset)
     begin
         if (reset) begin
            sin_r <= 32'h000_00000;
            cos_r <= {1'b0, {4'b0,ampl_div_2}, 20'b0};
	        Ncnt <= 0;
	        pwmcnt <= 0;
	        dstart <= 0;
	        mstart1 <= 0;
	        mstart2 <= 0;
         end else begin
	        dstart <= 0;
	        mstart1 <= 0;
	        mstart2 <= 0;
	        pwmcnt <= pwmcnt + 1;

	      if (pwmcnt == 8'hFF) begin
	       if (Ncnt >= N-1) begin     
		      sin_r <= 32'h000_00000; // reset sin & cos for the first cycle,
		      cos_r <= {1'b0, {4'b0,ampl_div_2}, 20'b0}; //   so we don't accumulate errors
		      Ncnt <= 0;
	       end else begin
		      sin_r <= sin_out;
		      cos_r <= cos_out;
		      Ncnt <= Ncnt + 1;
	       end
	      end

	    if (pwmcnt == 8'h81) begin
	       mstart1 <= 1;  // recalculte cos & sin
	       mstart2 <= 1;
	    end

 	    if (Ncnt == 0 && pwmcnt == 8'h01) begin 
	       dstart <= 1;   // recalculate a once per audio cycle
	    end
	 end 
     end
endmodule

module qadd #(
	//Parameterized values
	parameter Q = 15,
	parameter N = 32
	)
	(
    input [N-1:0] a,
    input [N-1:0] b,
    output [N-1:0] c
    );

reg [N-1:0] res;

assign c = res;

always @(a,b) begin
	// both negative or both positive
	if(a[N-1] == b[N-1]) begin						//	Since they have the same sign, absolute magnitude increases
		res[N-2:0] = a[N-2:0] + b[N-2:0];		//		So we just add the two numbers
		res[N-1] = a[N-1];							//		and set the sign appropriately...  Doesn't matter which one we use, 
															//		they both have the same sign
															//	Do the sign last, on the off-chance there was an overflow...  
		end												//		Not doing any error checking on this...
	//	one of them is negative...
	else if(a[N-1] == 0 && b[N-1] == 1) begin		//	subtract a-b
		if( a[N-2:0] > b[N-2:0] ) begin					//	if a is greater than b,
			res[N-2:0] = a[N-2:0] - b[N-2:0];			//		then just subtract b from a
			res[N-1] = 0;										//		and manually set the sign to positive
			end
		else begin												//	if a is less than b,
			res[N-2:0] = b[N-2:0] - a[N-2:0];			//		we'll actually subtract a from b to avoid a 2's complement answer
			if (res[N-2:0] == 0)
				res[N-1] = 0;										//		I don't like negative zero....
			else
				res[N-1] = 1;										//		and manually set the sign to negative
			end
		end
	else begin												//	subtract b-a (a negative, b positive)
		if( a[N-2:0] > b[N-2:0] ) begin					//	if a is greater than b,
			res[N-2:0] = a[N-2:0] - b[N-2:0];			//		we'll actually subtract b from a to avoid a 2's complement answer
			if (res[N-2:0] == 0)
				res[N-1] = 0;										//		I don't like negative zero....
			else
				res[N-1] = 1;										//		and manually set the sign to negative
			end
		else begin												//	if a is less than b,
			res[N-2:0] = b[N-2:0] - a[N-2:0];			//		then just subtract a from b
			res[N-1] = 0;										//		and manually set the sign to positive
			end
		end
	end
endmodule

module qdiv #(
	//Parameterized values
	parameter Q = 15,
	parameter N = 32
	)
	(
	input 	[N-1:0] i_dividend,
	input 	[N-1:0] i_divisor,
	input 	i_start,
	input 	i_clk,
	output 	[N-1:0] o_quotient_out,
	output 	o_complete,
	output	o_overflow
	);
 
	reg [2*N+Q-3:0]	reg_working_quotient;	//	Our working copy of the quotient
	reg [N-1:0] 		reg_quotient;				//	Final quotient
	reg [N-2+Q:0] 		reg_working_dividend;	//	Working copy of the dividend
	reg [2*N+Q-3:0]	reg_working_divisor;		// Working copy of the divisor
 
	reg [N-1:0] 			reg_count; 		//	This is obviously a lot bigger than it needs to be, as we only need 
													//		count to N-1+Q but, computing that number of bits requires a 
													//		logarithm (base 2), and I don't know how to do that in a 
													//		way that will work for everyone
										 
	reg					reg_done;			//	Computation completed flag
	reg					reg_sign;			//	The quotient's sign bit
	reg					reg_overflow;		//	Overflow flag
 
	initial reg_done = 1'b1;				//	Initial state is to not be doing anything
	initial reg_overflow = 1'b0;			//		And there should be no woverflow present
	initial reg_sign = 1'b0;				//		And the sign should be positive

	initial reg_working_quotient = 0;	
	initial reg_quotient = 0;				
	initial reg_working_dividend = 0;	
	initial reg_working_divisor = 0;		
 	initial reg_count = 0; 		

 
	assign o_quotient_out[N-2:0] = reg_quotient[N-2:0];	//	The division results
	assign o_quotient_out[N-1] = reg_sign;						//	The sign of the quotient
	assign o_complete = reg_done;
	assign o_overflow = reg_overflow;
 
	always @( posedge i_clk ) begin
		if( reg_done && i_start ) begin										//	This is our startup condition
			//  Need to check for a divide by zero right here, I think....
			reg_done <= 1'b0;												//	We're not done			
			reg_count <= N+Q-1;											//	Set the count
			reg_working_quotient <= 0;									//	Clear out the quotient register
			reg_working_dividend <= 0;									//	Clear out the dividend register 
			reg_working_divisor <= 0;									//	Clear out the divisor register 
			reg_overflow <= 1'b0;										//	Clear the overflow register

			reg_working_dividend[N+Q-2:Q] <= i_dividend[N-2:0];				//	Left-align the dividend in its working register
			reg_working_divisor[2*N+Q-3:N+Q-1] <= i_divisor[N-2:0];		//	Left-align the divisor into its working register

			reg_sign <= i_dividend[N-1] ^ i_divisor[N-1];		//	Set the sign bit
			end 
		else if(!reg_done) begin
			reg_working_divisor <= reg_working_divisor >> 1;	//	Right shift the divisor (that is, divide it by two - aka reduce the divisor)
			reg_count <= reg_count - 1;								//	Decrement the count

			//	If the dividend is greater than the divisor
			if(reg_working_dividend >= reg_working_divisor) begin
				reg_working_quotient[reg_count] <= 1'b1;										//	Set the quotient bit
				reg_working_dividend <= reg_working_dividend - reg_working_divisor;	//		and subtract the divisor from the dividend
				end
 
			//stop condition
			if(reg_count == 0) begin
				reg_done <= 1'b1;										//	If we're done, it's time to tell the calling process
				reg_quotient <= reg_working_quotient;			//	Move in our working copy to the outside world
				if (reg_working_quotient[2*N+Q-3:N]>0)
					reg_overflow <= 1'b1;
					end
			else
				reg_count <= reg_count - 1;	
			end
		end
endmodule

module qmults#(
	//Parameterized values
	parameter Q = 15,
	parameter N = 32
	)
	(
	input 	[N-1:0]  i_multiplicand,
	input 	[N-1:0]	i_multiplier,
	input 	i_start,
	input 	i_clk,
	output 	[N-1:0] o_result_out,
	output 	o_complete,
	output	o_overflow
	);

	reg [2*N-2:0]	reg_working_result;		//	a place to accumulate our result
	reg [2*N-2:0]	reg_multiplier_temp;		//	a working copy of the multiplier
	reg [N-1:0]		reg_multiplicand_temp;	//	a working copy of the umultiplicand
	
	reg [N-1:0] 			reg_count; 		//	This is obviously a lot bigger than it needs to be, as we only need 
												//		count to N, but computing that number of bits requires a 
												//		logarithm (base 2), and I don't know how to do that in a 
												//		way that will work for every possibility
										 
	reg					reg_done;		//	Computation completed flag
	reg					reg_sign;		//	The result's sign bit
	reg					reg_overflow;	//	Overflow flag
 
	initial reg_done = 1'b1;			//	Initial state is to not be doing anything
	initial reg_overflow = 1'b0;		//		And there should be no woverflow present
	initial reg_sign = 1'b0;			//		And the sign should be positive
	
	assign o_result_out[N-2:0] = reg_working_result[N-2+Q:Q];	//	The multiplication results
	assign o_result_out[N-1] = reg_sign;								//	The sign of the result
	assign o_complete = reg_done;											//	"Done" flag
	assign o_overflow = reg_overflow;									//	Overflow flag
	
	always @( posedge i_clk ) begin
		if( reg_done && i_start ) begin										//	This is our startup condition
			reg_done <= 1'b0;														//	We're not done			
			reg_count <= 0;														//	Reset the count
			reg_working_result <= 0;											//	Clear out the result register
			reg_multiplier_temp <= 0;											//	Clear out the multiplier register 
			reg_multiplicand_temp <= 0;										//	Clear out the multiplicand register 
			reg_overflow <= 1'b0;												//	Clear the overflow register

			reg_multiplicand_temp <= i_multiplicand[N-2:0];				//	Load the multiplicand in its working register and lose the sign bit
			reg_multiplier_temp <= i_multiplier[N-2:0];					//	Load the multiplier into its working register and lose the sign bit

			reg_sign <= i_multiplicand[N-1] ^ i_multiplier[N-1];		//	Set the sign bit
			end 

		else if (!reg_done) begin
			if (reg_multiplicand_temp[reg_count] == 1'b1)								//	if the appropriate multiplicand bit is 1
				reg_working_result <= reg_working_result + reg_multiplier_temp;	//		then add the temp multiplier
	
			reg_multiplier_temp <= reg_multiplier_temp << 1;						//	Do a left-shift on the multiplier
			reg_count <= reg_count + 1;													//	Increment the count

			//stop condition
			if(reg_count == N) begin
				reg_done <= 1'b1;										//	If we're done, it's time to tell the calling process
				if (reg_working_result[2*N-2:N-1+Q] > 0)			// Check for an overflow
					reg_overflow <= 1'b1;
//			else
//				reg_count <= reg_count + 1;													//	Increment the count
				end
			end
		end
endmodule