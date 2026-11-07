`timescale 1ns/1ps
module nexys3_top_hil #(parameter HEARTBEAT_TICKS=50000000, PHY_RESET_TICKS=1000000)(
    input wire clk,
    output wire [6:0] led,
    input wire phy_tx_clk, output wire [3:0] phy_txd, output wire phy_tx_en,
    output wire phy_tx_er, output reg phy_rstn=0, output wire phy_mdc,
    input wire phy_rx_clk, input wire [3:0] phy_rxd, input wire phy_rx_dv);
    // Phase 2 top level: the FPGA is the brain of the robot simulated in
    // Webots on the Mac. The Mac streams the overhead depth image and the
    // robot pose over Ethernet (docs/packets.md); hil_brain finds the
    // obstacles, replans with BFS and computes the wheel speeds; each B5
    // reply goes back as a UDP broadcast on port 5005. The last reply is
    // repeated every HEARTBEAT_TICKS so the viewer can start at any time.
    wire pkt_valid, pkt_add;
    wire [1:0] pkt_type, status;
    wire [7:0] pkt_frame, pkt_bits;
    wire [2:0] pkt_band;
    wire [5:0] pkt_cell;
    wire [15:0] pkt_x, pkt_y, pkt_h;
    wire reply_valid, planning;
    wire [191:0] reply;

    eth_hil_rx receiver(.mii_rx_clk(phy_rx_clk),.mii_rxd(phy_rxd),.mii_rx_dv(phy_rx_dv),
        .clk(clk),.pkt_valid(pkt_valid),.pkt_type(pkt_type),.pkt_frame(pkt_frame),
        .pkt_band(pkt_band),.pkt_bits(pkt_bits),.pkt_add(pkt_add),.pkt_cell(pkt_cell),
        .pkt_x(pkt_x),.pkt_y(pkt_y),.pkt_h(pkt_h));
    hil_brain brain(.clk(clk),.pkt_valid(pkt_valid),.pkt_type(pkt_type),
        .pkt_frame(pkt_frame),.pkt_band(pkt_band),.pkt_bits(pkt_bits),
        .pkt_add(pkt_add),.pkt_cell(pkt_cell),.pkt_x(pkt_x),.pkt_y(pkt_y),.pkt_h(pkt_h),
        .reply_valid(reply_valid),.reply(reply),.status_code(status),.planning(planning));

    // Send each new reply; while the transmitter is busy, the request waits.
    reg packet_request=0, have_reply=0;
    reg [191:0] last_reply=0;
    reg [25:0] heartbeat=0;
    wire eth_ready, sent_toggle;
    wire eth_send=packet_request && eth_ready;
    eth_udp_tx #(.PAYLOAD_BYTES(24)) ethernet(.clk(clk),.send(eth_send),.payload(last_reply),
        .ready(eth_ready),.mii_tx_clk(phy_tx_clk),.mii_txd(phy_txd),.mii_tx_en(phy_tx_en),
        .sent_toggle(sent_toggle));

    reg [19:0] phy_reset_timer=0;
    always @(posedge clk) begin
        // Hold the PHY in reset for ~10 ms after configuration.
        if(!phy_rstn) begin
            if(phy_reset_timer==PHY_RESET_TICKS-1) phy_rstn<=1;
            else phy_reset_timer<=phy_reset_timer+1'b1;
        end
        if(eth_send) packet_request<=0;
        if(heartbeat==HEARTBEAT_TICKS-1) begin
            heartbeat<=0;
            if(have_reply) packet_request<=1;
        end else heartbeat<=heartbeat+1'b1;
        if(reply_valid) begin
            last_reply<=reply; have_reply<=1; packet_request<=1; heartbeat<=0;
        end
    end

    // LD1 LD0: brain status (00 waiting for a frame, 01 running, 10 no route,
    // 11 at goal), the same codes as the B5 status byte and Phase 1.
    // LD2: BFS planning. LD3: flips per frame sent. LD4: PHY out of reset.
    // LD5: flips per packet received. LD6: blinks while the 100 MHz clock runs.
    reg rx_flip=0;
    reg [26:0] clk_count=0;
    always @(posedge clk) begin
        clk_count<=clk_count+1'b1;
        if(pkt_valid) rx_flip<=~rx_flip;
    end
    assign led[1:0]=status;
    assign led[2]=planning;
    assign led[3]=sent_toggle;
    assign led[4]=phy_rstn;
    assign led[5]=rx_flip;
    assign led[6]=clk_count[26];
    assign phy_tx_er=1'b0;
    assign phy_mdc=1'b0; // PHY registers are left at their power-on settings
endmodule
