`timescale 1ns/1ps
module tb_nexys3_ff;
    // Plays a vector file into nexys3_top_ff as real Ethernet frames and
    // checks every B6 reply byte for byte against ref/brain_ff.py.
    // The vectors come from ref/make_ff_vectors.py (see that file for the
    // format). It acts as the PHY on both MII sides and also checks the
    // preamble, CRC, IPv4 checksum and UDP header of every frame sent.
    //   +vectors=sim/ff_vectors.txt  +max_frames=N  +show=N
    reg clk=0, tx_clk=0, rx_clk=0;
    reg [3:0] phy_rxd=0;
    reg phy_rx_dv=0;
    wire [6:0] led;
    wire [3:0] phy_txd;
    wire phy_tx_en, phy_tx_er, phy_rstn, phy_mdc;
    nexys3_top_ff #(.HEARTBEAT_TICKS(20000000),.PHY_RESET_TICKS(10)) dut (
        .clk(clk),.led(led),.phy_tx_clk(tx_clk),
        .phy_txd(phy_txd),.phy_tx_en(phy_tx_en),.phy_tx_er(phy_tx_er),
        .phy_rstn(phy_rstn),.phy_mdc(phy_mdc),
        .phy_rx_clk(rx_clk),.phy_rxd(phy_rxd),.phy_rx_dv(phy_rx_dv));
    always #5 clk=~clk;
    always #20 tx_clk=~tx_clk; // 25 MHz, 100 Mb/s
    initial begin #7; forever #20 rx_clk=~rx_clk; end

    // ---- FPGA -> PC: check each frame, keep the latest B5 payload ----
    reg [7:0] tx_bytes [0:255];
    reg [575:0] got;
    integer tx_nibbles=0, replies=0, i, k;
    reg [31:0] tx_crc;
    reg [19:0] sum;
    reg [3:0] low;
    always @(posedge tx_clk) begin
        if(phy_tx_en) begin
            if(tx_nibbles[0]==0) low=phy_txd;
            else tx_bytes[tx_nibbles/2]={phy_txd,low};
            tx_nibbles=tx_nibbles+1;
        end else if(tx_nibbles!=0) begin
            check_reply(tx_nibbles/2);
            tx_nibbles=0;
        end
    end

    task check_reply(input integer n);
        begin
            if(n!=126) $fatal(1,"sent frame has %0d bytes, expected 126",n);
            for(i=0;i<7;i=i+1) if(tx_bytes[i]!=8'h55) $fatal(1,"bad preamble");
            if(tx_bytes[7]!=8'hd5) $fatal(1,"bad SFD");
            tx_crc=32'hffffffff; // CRC over data + FCS leaves the magic residue
            for(i=8;i<n;i=i+1) for(k=0;k<8;k=k+1)
                tx_crc=(tx_crc[0]^tx_bytes[i][k]) ? (tx_crc>>1)^32'hedb88320 : tx_crc>>1;
            if(tx_crc!=32'hdebb20e3) $fatal(1,"bad Ethernet CRC on a sent frame");
            sum=0;
            for(i=22;i<42;i=i+2) sum=sum+{tx_bytes[i],tx_bytes[i+1]};
            sum=sum[15:0]+sum[19:16];
            if(sum[15:0]!=16'hffff) $fatal(1,"bad IPv4 checksum");
            if({tx_bytes[20],tx_bytes[21]}!=16'h0800 || tx_bytes[31]!=17 ||
               {tx_bytes[44],tx_bytes[45]}!=5005 || {tx_bytes[46],tx_bytes[47]}!=80 ||
               tx_bytes[50]!=8'hb6)
                $fatal(1,"bad Ethernet/UDP header or B6 marker");
            for(i=0;i<72;i=i+1) got[i*8 +: 8]=tx_bytes[50+i];
            replies=replies+1;
        end
    endtask

    // ---- PC -> FPGA: one UDP packet as an Ethernet frame ----
    reg [7:0] payload [0:127];
    reg [7:0] frame [0:255];
    reg [31:0] crc;
    integer j, b, flen;
    task send_packet(input integer n, input [15:0] port, input bad_crc);
        begin
            for(j=0;j<256;j=j+1) frame[j]=0;
            for(j=0;j<6;j=j+1) frame[j]=8'hff;
            frame[6]=8'h6c; frame[7]=8'h1f; frame[8]=8'hf7; frame[9]=8'h5d;
            frame[10]=8'hc3; frame[11]=8'h05; frame[12]=8'h08;
            frame[14]=8'h45; {frame[16],frame[17]}=28+n; frame[20]=8'h40;
            frame[22]=8'd64; frame[23]=8'd17;
            frame[26]=8'd192; frame[27]=8'd168; frame[28]=8'd10; frame[29]=8'd1;
            frame[30]=8'd192; frame[31]=8'd168; frame[32]=8'd10; frame[33]=8'd255;
            sum=0;
            for(j=14;j<34;j=j+2) sum=sum+{frame[j],frame[j+1]};
            sum=sum[15:0]+sum[19:16]; sum=sum[15:0]+sum[19:16];
            {frame[24],frame[25]}=~sum[15:0];
            frame[34]=8'hc3; frame[35]=8'h50; // source port 50000
            {frame[36],frame[37]}=port; {frame[38],frame[39]}=8+n;
            for(j=0;j<n;j=j+1) frame[42+j]=payload[j];
            flen=42+n<60 ? 60 : 42+n;
            crc=32'hffffffff;
            for(j=0;j<flen;j=j+1) for(b=0;b<8;b=b+1)
                crc=(crc[0]^frame[j][b]) ? (crc>>1)^32'hedb88320 : crc>>1;
            crc=~crc ^ (bad_crc ? 32'h1 : 32'h0);
            for(j=0;j<4;j=j+1) frame[flen+j]=crc[j*8 +: 8];
            @(negedge rx_clk); phy_rx_dv=1;
            for(j=0;j<15;j=j+1) begin phy_rxd=4'h5; @(negedge rx_clk); end
            phy_rxd=4'hd; @(negedge rx_clk);
            for(j=0;j<flen+4;j=j+1) begin
                phy_rxd=frame[j][3:0]; @(negedge rx_clk);
                phy_rxd=frame[j][7:4]; @(negedge rx_clk);
            end
            phy_rx_dv=0; phy_rxd=0;
            repeat(24) @(negedge rx_clk); // inter-frame gap
        end
    endtask

    // ---- main loop over the vector file ----
    reg [8*256-1:0] path;
    reg [7:0] kind, byte_in;
    reg [575:0] want;
    integer fd, n, r, frames=0, bad=0, show=5, max_frames=0, seq, waited;
    initial begin
        if(!$value$plusargs("vectors=%s", path)) path="sim/ff_vectors.txt";
        if($value$plusargs("show=%d", show)) ;
        if($value$plusargs("max_frames=%d", max_frames)) ;
        fd=$fopen(path,"r");
        if(fd==0) $fatal(1,"cannot open %0s (run ref/make_ff_vectors.py first)", path);
        wait(phy_rstn);
        repeat(100) @(posedge clk);
        forever begin
            r=$fscanf(fd," %c",kind);
            if(r!=1 || kind=="E" || (max_frames!=0 && frames==max_frames)) begin
                if(bad==0)
                    $display("PASS: %0d frames, every B6 reply matches ref/brain_ff.py (%0d frames sent by the FPGA)",
                             frames, replies);
                else $display("FAIL: %0d of %0d replies differ", bad, frames);
                $finish;
            end
            r=$fscanf(fd," %d",n);
            for(j=0;j<n;j=j+1) begin r=$fscanf(fd," %h",byte_in); payload[j]=byte_in; end
            seq=replies;
            send_packet(n, kind=="W" ? 16'd5007 : 16'd5006, kind=="B");
            if(kind=="P" && payload[0]==8'hc2) begin
                r=$fscanf(fd," %c",kind);
                if(kind!="R") $fatal(1,"vector file: expected R after a C2");
                for(j=0;j<72;j=j+1) begin r=$fscanf(fd," %h",byte_in); want[j*8 +: 8]=byte_in; end
                // Wait for the reply to this frame (heartbeats repeat older ones).
                waited=0;
                while(!(replies>seq && got[15:8]==payload[1]) && waited<4000000) begin
                    @(posedge clk); waited=waited+1;
                end
                if(waited==4000000) $fatal(1,"frame %0d: no reply within 40 ms", frames);
                if(got!==want) begin
                    bad=bad+1;
                    if(bad<=show)
                        $display("frame %0d differs:\n  got  %h\n  want %h", frames, got, want);
                end
                frames=frames+1;
                if(frames%200==0) $display("%0d frames checked, %0d differ", frames, bad);
            end
        end
    end
endmodule
