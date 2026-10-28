`timescale 1ns/1ps
module eth_udp_rx #(parameter [15:0] UDP_PORT=16'd5006)(
    input wire mii_rx_clk, input wire [3:0] mii_rxd, input wire mii_rx_dv,
    input wire clk, output reg cmd_valid=0, output wire cmd_add,
    output wire [5:0] cmd_cell);
    // Receives cell edits from the PC over MII. A command is a UDP/IPv4
    // packet to UDP_PORT whose payload starts with 5A, op (1 add, 0 remove),
    // cell. Frames with a bad Ethernet CRC or anything else are ignored.
    // cmd_valid is a one-cycle pulse in the clk domain.

    function [31:0] crc_nibble(input [31:0] c, input [3:0] d);
        integer i;
        begin
            crc_nibble=c;
            for(i=0;i<4;i=i+1)
                crc_nibble=(crc_nibble[0]^d[i]) ? (crc_nibble>>1)^32'hedb88320
                                                 : crc_nibble>>1;
        end
    endfunction

    // RX_CLK side
    reg [3:0] rxd=0;
    reg dv=0;
    localparam HUNT=0, BODY=1, DROP=2;
    reg [1:0] state=HUNT;
    reg [31:0] crc=0;
    reg [3:0] low=0;
    reg high_half=0, match=0;
    reg [6:0] index=0; // byte index from the destination MAC; stops at 127
    reg [7:0] op=0;
    reg [5:0] target=0;
    reg rx_add=0, rx_toggle=0;
    reg [5:0] rx_cell=0;
    wire [7:0] rx_byte={rxd,low};
    assign cmd_add=rx_add;
    assign cmd_cell=rx_cell;
    always @(posedge mii_rx_clk) begin
        rxd<=mii_rxd; dv<=mii_rx_dv;
        if(!dv) begin
            // The CRC over data + FCS leaves a fixed residue.
            if(state==BODY && match && index>=48 && crc==32'hdebb20e3 && op<2) begin
                rx_add<=op[0]; rx_cell<=target; rx_toggle<=~rx_toggle;
            end
            state<=HUNT;
        end else case(state)
            HUNT: // preamble nibbles are 5, the SFD ends with D
                if(rxd==4'hd) begin
                    state<=BODY; crc<=32'hffffffff; high_half<=0;
                    index<=0; match<=1;
                end else if(rxd!=4'h5) state<=DROP;
            BODY: begin
                crc<=crc_nibble(crc,rxd);
                high_half<=~high_half;
                if(!high_half) low<=rxd; // low nibble arrives first
                else begin
                    case(index)
                        12: if(rx_byte!=8'h08) match<=0; // IPv4
                        13: if(rx_byte!=8'h00) match<=0;
                        14: if(rx_byte!=8'h45) match<=0; // 20-byte IP header
                        23: if(rx_byte!=8'd17) match<=0; // UDP
                        36: if(rx_byte!=UDP_PORT[15:8]) match<=0;
                        37: if(rx_byte!=UDP_PORT[7:0]) match<=0;
                        42: if(rx_byte!=8'h5a) match<=0;
                        43: op<=rx_byte;
                        44: if(rx_byte>63) match<=0; else target<=rx_byte[5:0];
                    endcase
                    if(index!=127) index<=index+1'b1;
                end
            end
            DROP: ;
        endcase
    end

    // clk side: rx_add/rx_cell are stable long before the toggle arrives.
    reg [2:0] toggle_sync=0;
    always @(posedge clk) begin
        toggle_sync<={toggle_sync[1:0],rx_toggle};
        cmd_valid<=toggle_sync[2]^toggle_sync[1];
    end
endmodule
