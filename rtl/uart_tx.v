`timescale 1ns/1ps
module uart_tx #(parameter DIV=868)(input wire clk, reset, send, input wire [7:0] data,
               output reg tx, output reg busy);
    // 100 MHz / 115200 = 868 clock ticks per UART bit.
    reg [9:0] ticks;
    reg [3:0] bits;
    reg [9:0] frame;
    always @(posedge clk) begin
        if (reset) begin tx<=1; busy<=0; ticks<=0; bits<=0; frame<=10'h3ff; end
        else if (!busy) begin
            tx<=1;
            if (send) begin
                frame<={1'b1,data,1'b0}; bits<=0; ticks<=0;
                busy<=1; tx<=0;
            end
        end else if (ticks==DIV-1) begin
            ticks<=0;
            if (bits==9) begin busy<=0; tx<=1; end
            else begin bits<=bits+1'b1; tx<=frame[bits+1'b1]; end
        end else ticks<=ticks+1'b1;
    end
endmodule
