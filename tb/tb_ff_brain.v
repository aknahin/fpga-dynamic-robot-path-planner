`timescale 1ns/1ps
module tb_ff_brain;
    // Plays a vector file (ref/make_ff_vectors.py) straight into ff_brain,
    // decoding each UDP payload the way eth_ff_rx does, and checks every B6
    // reply byte for byte against ref/brain_ff.py. Bad-CRC (B) and wrong-port
    // (W) lines never reach the brain, so they are skipped here; tb_nexys3_ff
    // sends them as real Ethernet frames.
    //   +vectors=sim/ff_vectors.txt  +max_frames=N  +show=N  +cycles=sim/cycles.csv
    // With +cycles, the clocks each frame took (pose to reply) are written as frame,clocks.
    reg clk=0;
    always #5 clk=~clk;

    reg pkt_valid=0, pkt_add=0;
    reg [1:0] pkt_type=0, pkt_cam=0;
    reg [7:0] pkt_frame=0, pkt_cell=0;
    reg [255:0] pkt_cols=0;
    reg [15:0] pkt_x=0, pkt_y=0, pkt_h=0;
    wire reply_valid, planning;
    wire [575:0] reply;
    wire [1:0] status_code;
    ff_brain dut(.clk(clk),.pkt_valid(pkt_valid),.pkt_type(pkt_type),.pkt_frame(pkt_frame),
        .pkt_cam(pkt_cam),.pkt_cols(pkt_cols),.pkt_add(pkt_add),.pkt_cell(pkt_cell),
        .pkt_x(pkt_x),.pkt_y(pkt_y),.pkt_h(pkt_h),.reply_valid(reply_valid),
        .reply(reply),.status_code(status_code),.planning(planning));

    reg [7:0] payload [0:127];
    reg [575:0] want;
    reg [7:0] kind, byte_in, a, b2;
    integer fd, n, r, j, frames=0, bad=0, show=5, max_frames=0, waited, maxcycles=0;
    reg [8*256-1:0] path, cyc_path;
    integer cfd=0;

    task send(input [1:0] t);
        begin
            @(posedge clk);
            pkt_type<=t; pkt_valid<=1;
            @(posedge clk);
            pkt_valid<=0;
            repeat(40) @(posedge clk);   // the brain copies a camera strip in 32 clocks; real frames are 9 us apart
        end
    endtask

    task deliver;   // payload[0..n-1] -> brain inputs, as the receiver would
        begin
            case(payload[0])
                8'h5a: begin pkt_add=payload[1][0]; pkt_cell=payload[2]; send(0); end
                8'hc0: send(1);
                8'hc3: begin
                    pkt_frame=payload[1]; pkt_cam=payload[2][1:0];
                    for(j=0;j<32;j=j+1) begin
                        a=payload[3+j]; b2=payload[35+j];
                        pkt_cols[j*8 +: 8]=(a<b2) ? a : b2;
                    end
                    send(2);
                end
                8'hc2: begin
                    pkt_frame=payload[1];
                    pkt_x={payload[3],payload[2]}; pkt_y={payload[5],payload[4]};
                    pkt_h={payload[7],payload[6]};
                    send(3);
                end
            endcase
        end
    endtask

    initial begin
        if(!$value$plusargs("vectors=%s", path)) path="sim/ff_vectors.txt";
        if($value$plusargs("show=%d", show)) ;
        if($value$plusargs("max_frames=%d", max_frames)) ;
        if($value$plusargs("cycles=%s", cyc_path)) cfd=$fopen(cyc_path,"w");
        fd=$fopen(path,"r");
        if(fd==0) $fatal(1,"cannot open %0s (run ref/make_ff_vectors.py first)", path);
        repeat(10) @(posedge clk);
        forever begin
            r=$fscanf(fd," %c",kind);
            if(r!=1 || kind=="E" || (max_frames!=0 && frames==max_frames)) begin
                if(cfd!=0) $fclose(cfd);
                if(bad==0)
                    $display("PASS: %0d frames, every B6 reply matches ref/brain_ff.py (slowest frame %0d clocks)",
                             frames, maxcycles);
                else $display("FAIL: %0d of %0d replies differ", bad, frames);
                $finish;
            end
            r=$fscanf(fd," %d",n);
            for(j=0;j<n;j=j+1) begin r=$fscanf(fd," %h",byte_in); payload[j]=byte_in; end
            if(kind=="P") begin
                deliver;
                if(payload[0]==8'hc2) begin
                    r=$fscanf(fd," %c",kind);
                    if(kind!="R") $fatal(1,"vector file: expected R after a C2");
                    for(j=0;j<72;j=j+1) begin r=$fscanf(fd," %h",byte_in); want[j*8 +: 8]=byte_in; end
                    waited=0;
                    while(!reply_valid && waited<2000000) begin @(posedge clk); waited=waited+1; end
                    if(waited==2000000) $fatal(1,"frame %0d: no reply", frames);
                    if(waited>maxcycles) maxcycles=waited;
                    if(cfd!=0) $fdisplay(cfd,"%0d,%0d",frames,waited);
                    if(reply!==want) begin
                        bad=bad+1;
                        if(bad<=show) begin
                            $display("frame %0d differs (frame byte %0d):", frames, payload[1]);
                            for(j=0;j<72;j=j+1)
                                if(reply[j*8 +: 8]!==want[j*8 +: 8])
                                    $display("  byte %0d: got %h want %h", j, reply[j*8 +: 8], want[j*8 +: 8]);
                        end
                    end
                    frames=frames+1;
                    if(frames%50==0) $display("%0d frames checked, %0d differ", frames, bad);
                    @(posedge clk);
                end
            end
        end
    end
endmodule
