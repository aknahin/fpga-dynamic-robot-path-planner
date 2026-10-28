`timescale 1ns/1ps
module eth_udp_tx #(parameter [47:0] SRC_MAC=48'h02_00_00_00_00_01,
                    parameter [31:0] SRC_IP={8'd192,8'd168,8'd10,8'd2},
                    parameter [31:0] DST_IP=32'hffffffff,
                    parameter [15:0] UDP_PORT=16'd5005)(
    input wire clk, send, input wire [167:0] payload, output wire ready,
    input wire mii_tx_clk, output reg [3:0] mii_txd=0, output reg mii_tx_en=0,
    output reg sent_toggle=0);
    // Sends the 21-byte payload as one UDP broadcast frame over MII
    // (TX only, no ARP). payload[7:0] is the first byte on the wire.
    // MII sends the low nibble of each byte first; TX_CLK comes from the PHY
    // (25 MHz at 100 Mb/s, 2.5 MHz at 10 Mb/s).

    // clk side: hold the payload until the TX_CLK side acknowledges it.
    reg req=0;
    reg [1:0] ack_sync=0;
    reg [167:0] held=0;
    reg ack=0;
    assign ready=req==ack_sync[1];
    always @(posedge clk) begin
        ack_sync<={ack_sync[0],ack};
        if(send && ready) begin held<=payload; req<=~req; end
    end

    // Header: Ethernet (14) + IPv4 (20) + UDP (8) = 42 bytes, written
    // first-byte-first, then reversed so byte 0 sits in bits [7:0].
    localparam [15:0] IP_LEN=16'd49, UDP_LEN=16'd29;
    wire [19:0] ip_sum=16'h4500+IP_LEN+16'h4000+16'h4011+
        SRC_IP[31:16]+SRC_IP[15:0]+DST_IP[31:16]+DST_IP[15:0];
    wire [16:0] ip_fold=ip_sum[15:0]+ip_sum[19:16];
    wire [15:0] ip_check=~(ip_fold[15:0]+ip_fold[16]);
    wire [335:0] header_be={48'hffffffffffff,SRC_MAC,16'h0800,
        16'h4500,IP_LEN,16'h0000,16'h4000,8'd64,8'd17,ip_check,SRC_IP,DST_IP,
        UDP_PORT,UDP_PORT,UDP_LEN,16'h0000};
    wire [335:0] header;
    genvar g;
    generate for(g=0;g<42;g=g+1) begin: swap
        assign header[g*8 +: 8]=header_be[(41-g)*8 +: 8];
    end endgenerate

    function [31:0] crc_nibble(input [31:0] c, input [3:0] d);
        integer i;
        begin
            crc_nibble=c;
            for(i=0;i<4;i=i+1)
                crc_nibble=(crc_nibble[0]^d[i]) ? (crc_nibble>>1)^32'hedb88320
                                                 : crc_nibble>>1;
        end
    endfunction

    // TX_CLK side
    localparam IDLE=0, PREAMBLE=1, DATA=2, FCS=3, GAP=4;
    reg [2:0] state=IDLE;
    reg [1:0] req_sync=0;
    reg [503:0] shift=0;
    reg [31:0] crc=0;
    reg [6:0] count=0;
    always @(posedge mii_tx_clk) begin
        req_sync<={req_sync[0],req};
        case(state)
            IDLE: begin
                mii_tx_en<=0; mii_txd<=0;
                if(req_sync[1]!=ack) begin
                    shift<={held,header}; ack<=req_sync[1];
                    count<=0; state<=PREAMBLE;
                end
            end
            PREAMBLE: begin // 15 x 5, then D (bytes 55 x7, D5)
                mii_tx_en<=1; mii_txd<=count==15 ? 4'hd : 4'h5;
                crc<=32'hffffffff;
                if(count==15) begin count<=0; state<=DATA; end
                else count<=count+1'b1;
            end
            DATA: begin // 63 bytes = 126 nibbles
                mii_txd<=shift[3:0]; shift<=shift>>4;
                crc<=crc_nibble(crc,shift[3:0]);
                if(count==125) begin count<=0; state<=FCS; end
                else count<=count+1'b1;
            end
            FCS: begin
                mii_txd<=~crc[3:0]; crc<=crc>>4;
                if(count==7) begin count<=0; state<=GAP; sent_toggle<=~sent_toggle; end
                else count<=count+1'b1;
            end
            GAP: begin // at least 12 idle bytes between frames
                mii_tx_en<=0; mii_txd<=0;
                if(count==31) state<=IDLE;
                else count<=count+1'b1;
            end
        endcase
    end
endmodule
