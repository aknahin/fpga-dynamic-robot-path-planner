`timescale 1ns/1ps
module tb_nexys3_lan;
    // Runs the same SW2 scenario as tb_nexys3_dynamic, acts as the PHY on
    // the MII side, and checks every Ethernet frame (preamble, CRC, IPv4
    // checksum, UDP header, packet marker). Frames go to sim/lan_frames.txt.
    // It also sends cell edits into the FPGA, as the PC viewer does.
    reg clk=0, tx_clk=0, rx_clk=0, btn_up=0;
    reg [3:0] phy_rxd=0;
    reg phy_rx_dv=0;
    reg [2:0] sw=3'b111;
    wire [6:0] led;
    wire [3:0] phy_txd;
    wire phy_tx_en, phy_tx_er, phy_rstn, phy_mdc;
    nexys3_top_lan #(.MOVE_TICKS(2000),.DEBOUNCE_TICKS(4),
                     .HEARTBEAT_TICKS(30000),.PHY_RESET_TICKS(10)) dut (
        .clk(clk),.btn_up(btn_up),.sw(sw),.led(led),.phy_tx_clk(tx_clk),
        .phy_txd(phy_txd),.phy_tx_en(phy_tx_en),.phy_tx_er(phy_tx_er),
        .phy_rstn(phy_rstn),.phy_mdc(phy_mdc),
        .phy_rx_clk(rx_clk),.phy_rxd(phy_rxd),.phy_rx_dv(phy_rx_dv));
    always #5 clk=~clk;
    always #20 tx_clk=~tx_clk; // 25 MHz, 100 Mb/s
    initial begin #7; forever #20 rx_clk=~rx_clk; end

    reg [7:0] bytes [0:255];
    integer nibbles=0, frames=0, last_robot=-1, out, i, k;
    reg [31:0] crc;
    reg [19:0] sum;
    reg [3:0] low;
    always @(posedge tx_clk) begin
        if(phy_tx_en) begin
            if(nibbles[0]==0) low=phy_txd;
            else bytes[nibbles/2]={phy_txd,low};
            nibbles=nibbles+1;
        end else if(nibbles!=0) begin
            check_frame(nibbles/2);
            nibbles=0;
        end
    end

    task check_frame(input integer n);
        begin
            if(n!=75) $fatal(1,"frame has %0d bytes, expected 75",n);
            for(i=0;i<7;i=i+1) if(bytes[i]!=8'h55) $fatal(1,"bad preamble");
            if(bytes[7]!=8'hd5) $fatal(1,"bad SFD");
            crc=32'hffffffff; // CRC over data + FCS leaves the magic residue
            for(i=8;i<n;i=i+1) for(k=0;k<8;k=k+1)
                crc=(crc[0]^bytes[i][k]) ? (crc>>1)^32'hedb88320 : crc>>1;
            if(crc!=32'hdebb20e3) $fatal(1,"bad Ethernet CRC");
            sum=0;
            for(i=22;i<42;i=i+2) sum=sum+{bytes[i],bytes[i+1]};
            sum=sum[15:0]+sum[19:16];
            if(sum[15:0]!=16'hffff) $fatal(1,"bad IPv4 checksum");
            if({bytes[20],bytes[21]}!=16'h0800 || bytes[31]!=17 ||
               {bytes[44],bytes[45]}!=5005 || bytes[50]!=8'ha5)
                $fatal(1,"bad Ethernet/UDP header or packet marker");
            if(bytes[51]!=last_robot) begin
                $display("frame %0d: robot=%0d found=%0d length=%0d",
                         frames,bytes[51],bytes[52],bytes[53]);
                last_robot=bytes[51];
            end
            for(i=8;i<n;i=i+1) $fwrite(out,"%02x",bytes[i]);
            $fwrite(out,"\n");
            frames=frames+1;
        end
    endtask

    // One UDP command frame from the PC: 5A, op, cell, padded to 60 bytes.
    reg [7:0] cmd [0:63];
    reg [31:0] cmd_crc;
    integer j, b;
    task send_cmd(input [7:0] op, input [5:0] target, input [15:0] port, input bad_crc);
        begin
            for(j=0;j<64;j=j+1) cmd[j]=0;
            for(j=0;j<6;j=j+1) cmd[j]=8'hff;
            cmd[6]=8'h6c; cmd[7]=8'h1f; cmd[8]=8'hf7; cmd[9]=8'h5d; cmd[10]=8'hc3; cmd[11]=8'h05;
            cmd[12]=8'h08; cmd[14]=8'h45; cmd[17]=8'd31; cmd[20]=8'h40; cmd[22]=8'd64;
            cmd[23]=8'd17; cmd[26]=8'd192; cmd[27]=8'd168; cmd[28]=8'd10; cmd[29]=8'd1;
            cmd[30]=8'd192; cmd[31]=8'd168; cmd[32]=8'd10; cmd[33]=8'd255;
            cmd[34]=port[15:8]; cmd[35]=port[7:0]; cmd[36]=port[15:8]; cmd[37]=port[7:0];
            cmd[39]=8'd11; cmd[42]=8'h5a; cmd[43]=op; cmd[44]=target;
            cmd_crc=32'hffffffff;
            for(j=0;j<60;j=j+1) for(b=0;b<8;b=b+1)
                cmd_crc=(cmd_crc[0]^cmd[j][b]) ? (cmd_crc>>1)^32'hedb88320 : cmd_crc>>1;
            cmd_crc=~cmd_crc ^ (bad_crc ? 32'h1 : 32'h0);
            for(j=0;j<4;j=j+1) cmd[60+j]=cmd_crc[j*8 +: 8];
            @(negedge rx_clk); phy_rx_dv=1;
            for(j=0;j<15;j=j+1) begin phy_rxd=4'h5; @(negedge rx_clk); end
            phy_rxd=4'hd; @(negedge rx_clk);
            for(j=0;j<64;j=j+1) begin
                phy_rxd=cmd[j][3:0]; @(negedge rx_clk);
                phy_rxd=cmd[j][7:4]; @(negedge rx_clk);
            end
            phy_rx_dv=0; phy_rxd=0;
            repeat(30) @(negedge rx_clk);
        end
    endtask

    // A cell on the remaining route, close to the goal.
    function integer cell_ahead(input dummy);
        integer c;
        begin
            cell_ahead=-1;
            for(c=1;c<63;c=c+1)
                if(dut.path_mask[c] && c!=dut.robot_cell) cell_ahead=c;
        end
    endfunction

    integer wall;
    initial begin
        out=$fopen("sim/lan_frames.txt","w");
        repeat(8) @(negedge clk);
        if(led[1:0]!=2'b00) $fatal(1,"LD1 LD0 should be 00 at the start");
        btn_up=1;
        repeat(12) @(negedge clk);
        btn_up=0;
        wait(dut.phase==4);
        if(!dut.result_found || dut.result_length!=14 || !dut.path_mask[32])
            $fatal(1,"Initial path must include cell 32 and be length 14");
        wait(dut.robot_cell==8);
        $display("Robot moved to 8; introducing obstacle at 32");
        @(negedge clk); sw[2]=0;
        wait(dut.phase==3);
        wait(dut.phase==4);
        if(dut.robot_cell!=8 || !dut.map[32] || dut.path_mask[32] || !dut.result_found)
            $fatal(1,"Replanning did not avoid new obstacle from robot cell 8");
        if(led[1:0]!=2'b01) $fatal(1,"LD1 LD0 should be 01 while running");

        wall=cell_ahead(0);
        $display("PC: bad-CRC and wrong-port commands for cell %0d (must be ignored)", wall);
        send_cmd(1,wall,5006,1);
        send_cmd(1,wall,5007,0);
        repeat(20) @(negedge clk);
        if(dut.map[wall]) $fatal(1,"FPGA accepted a bad command");
        wall=cell_ahead(0);
        $display("PC: add wall at cell %0d on the route", wall);
        send_cmd(1,wall,5006,0);
        wait(dut.phase==3);
        wait(dut.phase==4);
        if(!dut.map[wall] || dut.path_mask[wall] || !dut.result_found)
            $fatal(1,"Replanning did not avoid the clicked wall");
        $display("FPGA replanned from cell %0d around cell %0d", dut.robot_cell, wall);
        send_cmd(0,wall,5006,0);
        repeat(20) @(negedge clk);
        if(dut.map[wall] || dut.user_walls[wall]) $fatal(1,"Wall was not removed");
        $display("PC: removed wall at cell %0d", wall);
        send_cmd(1,dut.robot_cell,5006,0);
        repeat(20) @(negedge clk);
        if(dut.map[dut.robot_cell]) $fatal(1,"Wall placed on the robot");
        wait(dut.phase==5);
        repeat(2000) @(negedge clk);
        if(last_robot!=63) $fatal(1,"last frame did not show the robot at the goal");
        if(led[1:0]!=2'b11) $fatal(1,"LD1 LD0 should be 11 at the goal");

        // Blocked map (SW1 on): no route, then the PC removes a wall.
        sw=3'b101;
        repeat(20) @(negedge clk);
        btn_up=1; repeat(12) @(negedge clk); btn_up=0;
        wait(dut.phase==3);
        wait(dut.phase==5); @(negedge clk);
        if(dut.result_found || led[1:0]!=2'b10) $fatal(1,"Blocked map should give no route (10)");
        $display("Blocked map: no route. PC removes wall at cell 28");
        send_cmd(0,28,5006,0);
        wait(dut.phase==4); @(negedge clk);
        if(!dut.result_found || dut.map[28] || led[1:0]!=2'b01)
            $fatal(1,"Removing a wall did not give a new route");
        wait(dut.phase==5); @(negedge clk);
        if(dut.robot_cell!=63 || led[1:0]!=2'b11) $fatal(1,"Robot did not reach the goal");
        $display("Robot reached 63 through the opened cell");
        $display("PASS: %0d valid Ethernet frames, robot reached 63", frames);
        $fclose(out);
        $finish;
    end
    initial begin #20000000; $fatal(1,"LAN test timeout"); end
endmodule
