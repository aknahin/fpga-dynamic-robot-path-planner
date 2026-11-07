`timescale 1ns/1ps
module eth_ff_rx #(parameter [15:0] UDP_PORT=16'd5006)(
    input wire mii_rx_clk, input wire [3:0] mii_rxd, input wire mii_rx_dv,
    input wire clk, output reg pkt_valid=0, output wire [1:0] pkt_type,
    output wire [7:0] pkt_frame, output wire [1:0] pkt_cam,
    output wire [255:0] pkt_cols, output wire pkt_add, output wire [7:0] pkt_cell,
    output wire [15:0] pkt_x, pkt_y, pkt_h);
    // Receiver for the flow-field brain (docs/packets_ff.md). Accepts
    // UDP/IPv4 packets to UDP_PORT with a good Ethernet CRC and tells them
    // apart by the first payload byte:
    //   5A edit (type 0): pkt_add, pkt_cell
    //   C0 reset (type 1)
    //   C3 camera strip (type 2): pkt_frame, pkt_cam, pkt_cols. The 64 depth
    //      bytes (2 rows x 32 columns) are never stored: while they stream in,
    //      one register per column keeps the smaller of the two rows, which is
    //      the range of that column (ref/perception.column_ranges).
    //   C2 pose (type 3): pkt_frame, pkt_x, pkt_y, pkt_h
    // pkt_valid is a one-cycle pulse in the clk domain; the other outputs
    // hold until the next packet.
    localparam [1:0] T_EDIT=0, T_RESET=1, T_CAM=2, T_POSE=3;

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
    reg [10:0] index=0;     // byte index from the destination MAC; stops at 2047
    reg [15:0] udp_len=0;
    reg [7:0] kind=0, p1=0, p2=0, p3=0, p4=0, p5=0, p6=0, p7=0;
    reg [7:0] colmin [0:31];
    wire [7:0] rx_byte={rxd,low};
    wire [10:0] pixel=index-11'd45; // C3 depth bytes are frame bytes 45..108
    integer c;

    // Results, held for the clk side.
    reg [1:0] r_type=0, r_cam=0;
    reg [7:0] r_frame=0;
    reg r_add=0, rx_toggle=0;
    reg [7:0] r_cell=0;
    reg [15:0] r_x=0, r_y=0, r_h=0;
    reg [255:0] r_cols=0;
    assign pkt_type=r_type; assign pkt_frame=r_frame; assign pkt_cam=r_cam;
    assign pkt_cols=r_cols; assign pkt_add=r_add; assign pkt_cell=r_cell;
    assign pkt_x=r_x; assign pkt_y=r_y; assign pkt_h=r_h;

    // A packet counts only if the UDP length and the bytes actually received
    // cover its fixed size (n payload bytes + 42 header bytes + 4 FCS bytes).
    wire len_edit =udp_len>=16'd11 && index>=11'd49;   // n=3
    wire len_reset=udp_len>=16'd9  && index>=11'd47;   // n=1
    wire len_cam  =udp_len>=16'd75 && index>=11'd113;  // n=67
    wire len_pose =udp_len>=16'd16 && index>=11'd54;   // n=8

    wire good=state==BODY && match && crc==32'hdebb20e3;
    always @(posedge mii_rx_clk) begin
        rxd<=mii_rxd; dv<=mii_rx_dv;
        if(!dv) begin
            if(good) case(kind)
                8'h5a: if(len_edit && p1<2) begin
                    r_type<=T_EDIT; r_add<=p1[0]; r_cell<=p2; rx_toggle<=~rx_toggle;
                end
                8'hc0: if(len_reset) begin
                    r_type<=T_RESET; rx_toggle<=~rx_toggle;
                end
                8'hc3: if(len_cam && p2<4) begin
                    r_type<=T_CAM; r_frame<=p1; r_cam<=p2[1:0];
                    for(c=0;c<32;c=c+1) r_cols[c*8 +: 8]<=colmin[c];
                    rx_toggle<=~rx_toggle;
                end
                8'hc2: if(len_pose) begin
                    r_type<=T_POSE; r_frame<=p1;
                    r_x<={p3,p2}; r_y<={p5,p4}; r_h<={p7,p6};
                    rx_toggle<=~rx_toggle;
                end
            endcase
            state<=HUNT;
        end else case(state)
            HUNT: // preamble nibbles are 5, the SFD ends with D
                if(rxd==4'hd) begin
                    state<=BODY; crc<=32'hffffffff; high_half<=0;
                    index<=0; match<=1; kind<=0;
                    for(c=0;c<32;c=c+1) colmin[c]<=8'hff;
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
                        38: udp_len[15:8]<=rx_byte;
                        39: udp_len[7:0]<=rx_byte;
                        42: kind<=rx_byte;
                        43: p1<=rx_byte;
                        44: p2<=rx_byte;
                        45: p3<=rx_byte;
                        46: p4<=rx_byte;
                        47: p5<=rx_byte;
                        48: p6<=rx_byte;
                        49: p7<=rx_byte;
                    endcase
                    if(index>=45 && index<109 && rx_byte<colmin[pixel[4:0]])
                        colmin[pixel[4:0]]<=rx_byte;
                    if(index!=2047) index<=index+1'b1;
                end
            end
            DROP: ;
        endcase
    end

    // clk side: the results are stable long before the toggle arrives
    // (the shortest frame takes over 5 us at 100 Mb/s).
    reg [2:0] toggle_sync=0;
    always @(posedge clk) begin
        toggle_sync<={toggle_sync[1:0],rx_toggle};
        pkt_valid<=toggle_sync[2]^toggle_sync[1];
    end
endmodule
