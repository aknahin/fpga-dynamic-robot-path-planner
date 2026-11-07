`timescale 1ns/1ps
module ff_brain(
    input wire clk,
    input wire pkt_valid, input wire [1:0] pkt_type, input wire [7:0] pkt_frame,
    input wire [1:0] pkt_cam, input wire [255:0] pkt_cols,
    input wire pkt_add, input wire [7:0] pkt_cell,
    input wire signed [15:0] pkt_x, pkt_y, input wire [15:0] pkt_h,
    output reg reply_valid=0, output wire [575:0] reply,
    output wire [1:0] status_code, output wire planning);
    // The FPGA brain of the warehouse robot: ref/brain_ff.py in hardware. It
    // must match that model bit for bit (tb_ff_brain and tb_nexys3_ff check it).
    //
    // Camera strips (C3) and wall edits (5A) are stored as they arrive. Each
    // pose (C2) runs one frame:
    //   1. perception  cast the 128 camera rays, mark cells seen occupied or
    //                  seen free, then update each cell's occupancy count and
    //                  mover timer (ref/perception.py)
    //   2. halo        distance of every cell to the nearest mover timer cell
    //   3. cost field  reverse sweeps from the goal over racks, seen
    //                  obstacles and the mover halo (ref/flowfield.py)
    //   4. flow        pick the next cell from the field when a new waypoint
    //                  is needed (ref/brain_ff.py)
    //   5. drive       waypoint controller, same as hil_brain.v
    // reply is the 72-byte B6 payload; reply_valid pulses when it is ready.
    //
    // v2: everything per cell lives in small RAMs (camera ranges, hit/miss marks,
    // cell memory, halo distance, clicked walls), not in 256-bit registers with
    // bit-addressed writes. v1 needed 159% of the LUTs of an XC6SLX16 (see
    // docs/HANDOFF.md). The behaviour is unchanged.
    localparam [1:0] T_EDIT=0, T_RESET=1, T_CAM=2, T_POSE=3;
    localparam [1:0] WAIT_MAP=0, RUNNING=1, NO_ROUTE=2, AT_GOAL=3; // = B6 codes
    localparam [9:0] INF=10'd1023;
    localparam [7:0] GOAL=8'd255;
    localparam signed [17:0] ARRIVE_MM=15;
    localparam signed [15:0] TURN_LIMIT=2731;
    localparam signed [8:0] DRIVE_SPEED=100, TURN_SPEED=40, MAX_SPEED=125;

    // ---- known map and tables ----
    wire [255:0] RACK, LANE;
    ff_rack_map racks(.mask(RACK));
    ff_lane_map lanes(.mask(LANE));
    wire [10:0] qv_s, qv_c;
    wire [8:0] qa_s, qa_c;
    ff_sin_quarter quarter_s(.i(qa_s), .v(qv_s));
    ff_sin_quarter quarter_c(.i(qa_c), .v(qv_c));

    // ---- stored state: all RAMs ----
    reg [7:0] camram [0:127];             // 4 cameras x 32 column ranges, address {cam, column}
    reg [7:0] cellmem [0:255];            // {mover timer[5:0], occupancy[1:0]}
    reg [1:0] hm [0:255];                 // this frame's rays: {seen free, seen occupied}
    reg [2:0] hdA [0:255], hdB [0:255];   // distance (cells, capped at 4) to a mover timer cell
    reg uw [0:255];                       // clicked walls
    reg [9:0] fa [0:255], fb [0:255], fc [0:255], fd [0:255]; // cost field, 4 read ports
    reg [255:0] blk_sh=0, mov_sh=0;       // reply masks, built by shifting during the memory pass
    integer ii;
    initial begin
        for(ii=0;ii<256;ii=ii+1) begin
            cellmem[ii]=8'd0; hm[ii]=2'd0; hdA[ii]=3'd4; hdB[ii]=3'd4; uw[ii]=1'b0;
            fa[ii]=INF; fb[ii]=INF; fc[ii]=INF; fd[ii]=INF;
        end
        for(ii=0;ii<128;ii=ii+1) camram[ii]=8'hff;
    end
    reg [4:0] nr_cnt=0;                    // frames spent in NO_ROUTE (wraps at 32 = warehouse.AGE_PERIOD)
    reg age_r=0;                           // this frame: fade unobserved cells everywhere (no route: deadlock breaker)
    reg [4:0] fr_cnt=0;                    // frames since reset, wraps at 32
    reg age_lane_r=0;                      // this frame: fade unobserved cells on cart lanes
    reg ln_c=0;                            // current cell is a cart-lane cell
    reg [1:0] status=WAIT_MAP;
    reg [7:0] robot_cell=0, prev=0, target=0, replans=0, frame=0;
    reg signed [15:0] x=0, y=0;
    reg [15:0] heading=0;
    reg signed [8:0] left=0, right=0;
    reg pose_pending=0, reset_pending=0;
    assign status_code=status;
    assign reply={mov_sh, blk_sh, replans, (status==RUNNING ? target : robot_cell),
                  robot_cell, 6'b0, status, right[7:0], left[7:0], frame, 8'hb6};

    // ---- camera range writer: copies the 32 bytes of a C3 packet into camram ----
    // (pkt_cols holds until the next packet, which is at least 9 us later.)
    reg cw_active=0;
    reg [1:0] cw_cam=0;
    reg [4:0] cw_i=0;

    // ---- control ----
    localparam S_IDLE=0, S_RST_MEM=1, S_CELL=2, S_TRIG=3,
               S_RAY_LOAD=4, S_RAY_D=5, S_RAY_LAT=6, S_RAY_V=7, S_RAY_M=8, S_RAY_D2=9,
               S_SMP_M=10, S_SMP_P=11, S_SMP_A=12, S_SMP_Q=13, S_SMP_C=14, S_SMP_MARK=15,
               S_MEM_RD=16, S_MEM_WR=17, S_FINIT=18, S_SW0=19, S_SW1=20, S_ADV=21,
               S_DEC=22, S_DEC_T=23, S_PICK=24, S_FS_INIT=25, S_FS_ADDR=26, S_FS_CMP=27,
               S_FS_AV0=28, S_FS_AV1=29, S_PICK_DONE=30, S_GEOM=31, S_AIM=32, S_DRIVE=33,
               S_ARR_DONE=34, S_REPLY=35, S_HMCLR=36, S_HF0=37, S_HF1=38, S_HADV=39, S_TRIG2=40, S_SW2=41, S_SW1B=42;
    reg [5:0] state=S_IDLE;
    assign planning=(state>=S_FINIT && state<=S_DEC) || (state>=S_HF0 && state<=S_HADV);
    reg [7:0] cidx=0;                     // cell counter for the loops

    // robot cell and heading index (S_CELL)
    reg [9:0] hidx=0;
    wire [8:0] hj_s=hidx[8:0];
    wire [9:0] hidx_c=hidx+10'd256;
    wire [8:0] hj_c=hidx_c[8:0];
    wire [9:0] fold_s=10'd512-{1'b0,hj_s}, fold_c=10'd512-{1'b0,hj_c};
    assign qa_s=(hj_s<=9'd256) ? hj_s : fold_s[8:0];
    assign qa_c=(hj_c<=9'd256) ? hj_c : fold_c[8:0];
    reg signed [11:0] sn=0, cs=0;
    reg [10:0] qvs_r=0, qvc_r=0;          // ROM outputs, registered before the sign is applied
    reg negs_r=0, negc_r=0;

    wire signed [17:0] ux_c=$signed({{2{x[15]}},x})+18'sd1200;
    wire signed [17:0] uy_c=18'sd1200-$signed({{2{y[15]}},y});
    wire [31:0] colq=(ux_c<0) ? 32'd0 : ($unsigned(ux_c)*32'd437)>>16;
    wire [31:0] rowq=(uy_c<0) ? 32'd0 : ($unsigned(uy_c)*32'd437)>>16;
    wire [3:0] cell_col=(colq>32'd15) ? 4'd15 : colq[3:0];
    wire [3:0] cell_row=(rowq>32'd15) ? 4'd15 : rowq[3:0];

    // ---- ray casting ----
    reg [6:0] ray=0;
    reg [7:0] b=0;
    reg hit=0;
    reg [10:0] d=0;
    reg signed [7:0] sc=0;
    reg signed [13:0] lat=0, vx=0, vy=0;
    wire signed [19:0] lat_full=$signed({1'b0,d})*sc;   // full width, shifted after
    reg signed [25:0] p1=0, p2=0, p3=0, p4=0;
    reg signed [15:0] dx=0, dy=0;
    reg [4:0] k=0;
    reg signed [21:0] mx=0, my=0;
    reg signed [17:0] px=0, py=0;
    reg in_arena=0;
    reg [11:0] ux=0, uy=0;
    reg [3:0] qx=0, qy=0;
    reg [7:0] rx=0, ry=0;
    reg [7:0] scell=0;
    wire [20:0] qxm=ux*9'd437, qym=uy*9'd437;

    // ---- cell memory ----
    wire sweeping=state==S_SW0 || state==S_SW1 || state==S_SW1B || state==S_SW2 || state==S_ADV;
    reg [7:0] c=0;                        // cell being relaxed / halo pass cell
    wire [7:0] cm_addr=sweeping ? c : cidx;
    wire [7:0] cm_rd=cellmem[cm_addr];
    reg [7:0] m=0;
    wire [1:0] occ=m[1:0];
    wire [5:0] ttl=m[7:2];
    reg h_c=0, m_c=0;
    wire age=age_r || (age_lane_r && ln_c);
    wire [5:0] ttl_dec=(ttl!=0) ? ttl-6'd1 : 6'd0;
    reg [1:0] occ_n;
    reg [5:0] ttl_n;
    always @* begin
        occ_n=occ; ttl_n=ttl_dec;
        if(h_c) occ_n=(occ==2'd3) ? 2'd3 : occ+2'd1;
        else if(m_c) begin
            occ_n=(occ==2'd0) ? 2'd0 : occ-2'd1;
            if(occ>=2'd2 && occ_n<2'd2) ttl_n=6'd63;
        end else if(age && occ!=2'd0) occ_n=occ-2'd1;
    end

    // ---- halo distance transform (exact Manhattan distance, capped at 4) ----
    reg fwd=1;
    wire [2:0] hn1=fwd ? hdA[c-8'd16] : hdA[c+8'd16];
    wire [2:0] hn2=fwd ? hdB[c-8'd1]  : hdB[c+8'd1];
    wire hn1_ok=fwd ? (c[7:4]!=4'd0) : (c[7:4]!=4'd15);
    wire hn2_ok=fwd ? (c[3:0]!=4'd0) : (c[3:0]!=4'd15);
    reg [2:0] hn1_r=0, hn2_r=0;
    wire [2:0] hcur=hdA[c];
    wire [3:0] hc1={1'b0,hn1_r}+4'd1, hc2={1'b0,hn2_r}+4'd1;
    wire [2:0] h_a=(hn1_ok && hc1<{1'b0,hcur}) ? hc1[2:0] : hcur;
    wire [2:0] h_new=(hn2_ok && hc2<{1'b0,h_a}) ? hc2[2:0] : h_a;

    // ---- cost field sweeps ----
    reg [5:0] pair=0;
    reg changed=0;
    reg [9:0] best=0;
    reg [3:0] wreg=0;
    reg skip_c=0;
    reg [7:0] fs_addr=0;
    wire [7:0] addr_a=(state==S_SW2) ? c : sweeping ? c-8'd16 : fs_addr;
    wire [7:0] addr_b=c+8'd16, addr_c=c-8'd1, addr_d=c+8'd1;
    wire [9:0] ra=fa[addr_a], rb=fb[addr_b], rc=fc[addr_c], rd=fd[addr_d];
    wire [9:0] up_v   =(c[7:4]!=4'd0)  ? ra : INF;
    wire [9:0] down_v =(c[7:4]!=4'd15) ? rb : INF;
    wire [9:0] left_v =(c[3:0]!=4'd0)  ? rc : INF;
    wire [9:0] right_v=(c[3:0]!=4'd15) ? rd : INF;
    reg [9:0] u_r=0, d_r=0, l_r=0, r_r=0;  // neighbour costs, captured before the minimum is taken
    wire [9:0] min_ud=(u_r<d_r) ? u_r : d_r;
    wire [9:0] min_lr=(l_r<r_r) ? l_r : r_r;
    wire [9:0] min4=(min_ud<min_lr) ? min_ud : min_lr;
    wire [2:0] hd_c=hdA[c];
    wire [3:0] halo=(hd_c<=3'd1) ? 4'd10 : (hd_c==3'd2) ? 4'd6 : (hd_c==3'd3) ? 4'd3 : 4'd0;
    // blocked for planning: seen, clicked or rack, but never the robot's own cell or the goal
    wire blocked_c=(cm_rd[1]|uw[c]|RACK[c]) && c!=robot_cell && c!=GOAL;
    wire [10:0] relax_sum={1'b0,wreg}+{1'b0,best};
    wire [9:0] relax_val=(relax_sum>11'd1023) ? INF : relax_sum[9:0];
    reg [9:0] relax_r=0;                   // registered so the RAM write path is only read, compare, write

    // ---- flow_step ----
    localparam RET_PICK=0, RET_ARR=1;
    reg [7:0] fs_sel=0, fs_avoid=0, fs_best=0, res=0;
    reg fs_avoid_v=0, fs_ret=0, nbv=0, res_v=0, best_v=0;
    reg [9:0] best_cost=0;
    reg [1:0] dirc=0;
    reg [7:0] nb;
    reg nb_ok;
    always @* begin
        nb=fs_sel; nb_ok=0;
        case(dirc)
            0: begin nb=fs_sel-8'd16; nb_ok=fs_sel[7:4]!=4'd0;  end
            1: begin nb=fs_sel+8'd16; nb_ok=fs_sel[7:4]!=4'd15; end
            2: begin nb=fs_sel-8'd1;  nb_ok=fs_sel[3:0]!=4'd0;  end
            3: begin nb=fs_sel+8'd1;  nb_ok=fs_sel[3:0]!=4'd15; end
        endcase
    end
    wire use_avoid=fs_avoid_v && ra<INF && ({1'b0,ra}+11'd8)<{1'b0,best_cost};

    // ---- waypoint geometry (as hil_brain.v, 16x16 cells) ----
    wire signed [17:0] cx=-18'sd1125+18'sd150*$signed({14'b0,target[3:0]});
    wire signed [17:0] cy=18'sd1125-18'sd150*$signed({14'b0,target[7:4]});
    reg signed [17:0] ex=0, ey=0;
    reg go_e=0, go_n=0, go_w=0;
    wire signed [17:0] along=go_e ? -ex : go_n ? -ey : go_w ? ex : ey;
    wire signed [17:0] latd =go_e ? ey : go_n ? -ex : go_w ? -ey : ex;
    wire [15:0] base=go_e ? 16'd0 : go_n ? 16'd16384 : go_w ? 16'd32768 : 16'd49152;
    wire signed [15:0] lat_clamp=latd>18'sd64 ? 16'sd4096 : latd<-18'sd64 ? -16'sd4096
                                 : {latd[9:0],6'b0};
    reg signed [17:0] along_r=0;
    reg signed [15:0] err=0;
    wire signed [8:0] steer=err>>>6;
    wire signed [9:0] l_raw=DRIVE_SPEED-steer, r_raw=DRIVE_SPEED+steer;
    function signed [8:0] clamp_speed(input signed [9:0] v);
        clamp_speed = v>MAX_SPEED ? MAX_SPEED : v<-MAX_SPEED ? -MAX_SPEED : v[8:0];
    endfunction

    wire [1:0] hm_rd=hm[scell];
    wire [1:0] hm_cell=hm[cidx];

    // ---- RAM write ports: exactly one write port per RAM (a Spartan-6 distributed RAM has one) ----
    // The FSM below decides *what* to write by its state; these blocks do the writing on the same clock edge.
    reg f_we;                              // cost field (4 identical copies, one per read port)
    reg [7:0] f_wa;
    reg [9:0] f_wd;
    always @* begin
        f_we=0; f_wa=cidx; f_wd=relax_r;
        if(state==S_FINIT) begin f_we=1; f_wa=cidx; f_wd=(cidx==GOAL) ? 10'd0 : INF; end
        else if(state==S_SW2 && !skip_c && relax_r<ra) begin f_we=1; f_wa=c; end
    end
    always @(posedge clk) if(f_we) begin
        fa[f_wa]<=f_wd; fb[f_wa]<=f_wd; fc[f_wa]<=f_wd; fd[f_wa]<=f_wd;
    end

    reg cm_we;                             // cell memory (always addressed by cidx)
    reg [7:0] cm_wd;
    always @* begin
        cm_we=0; cm_wd=8'd0;
        if(state==S_RST_MEM) cm_we=1;
        else if(state==S_MEM_WR) begin cm_we=1; cm_wd={ttl_n,occ_n}; end
    end
    always @(posedge clk) if(cm_we) cellmem[cidx]<=cm_wd;

    reg hm_we;                             // hit/miss marks
    reg [7:0] hm_wa;
    reg [1:0] hm_wd;
    always @* begin
        hm_we=0; hm_wa=cidx; hm_wd=2'd0;
        if(state==S_HMCLR) hm_we=1;
        else if(state==S_SMP_MARK && in_arena) begin
            if(hit && k==5'd16) begin
                if(rx>=8'd14 && rx<=8'd136 && ry>=8'd14 && ry<=8'd136) begin
                    hm_we=1; hm_wa=scell; hm_wd=hm_rd|2'b01;
                end
            end else if((!hit || k<=5'd13) && rx>=8'd20 && rx<=8'd130 && ry>=8'd20 && ry<=8'd130) begin
                hm_we=1; hm_wa=scell; hm_wd=hm_rd|2'b10;   // free evidence only through the cell's central part
            end
        end
    end
    always @(posedge clk) if(hm_we) hm[hm_wa]<=hm_wd;

    reg hd_we;                             // halo distance (2 identical copies)
    reg [7:0] hd_wa;
    reg [2:0] hd_wd;
    always @* begin
        hd_we=0; hd_wa=cidx; hd_wd=3'd0;
        if(state==S_MEM_WR) begin hd_we=1; hd_wa=cidx; hd_wd=(ttl_n!=6'd0) ? 3'd0 : 3'd4; end
        else if(state==S_HF1) begin hd_we=1; hd_wa=c; hd_wd=h_new; end
    end
    always @(posedge clk) if(hd_we) begin hdA[hd_wa]<=hd_wd; hdB[hd_wa]<=hd_wd; end

    always @(posedge clk) if(pkt_valid && pkt_type==T_EDIT && pkt_cell!=GOAL && pkt_cell!=robot_cell && !RACK[pkt_cell])
        uw[pkt_cell]<=pkt_add;

    always @(posedge clk) if(cw_active) camram[{cw_cam,cw_i}]<=pkt_cols[cw_i*8 +: 8];

    always @(posedge clk) begin
        reply_valid<=0;

        // Camera range writer: one byte per clock (the RAM write is in its own block above).
        if(cw_active) begin
            if(cw_i==5'd31) cw_active<=0; else cw_i<=cw_i+5'd1;
        end

        // Packets. Camera strips and edits apply at once; reset and pose wait
        // for the frame in progress (the PC only sends the next pose after
        // the reply, so they never actually queue up).
        if(pkt_valid) case(pkt_type)
            T_CAM: begin cw_cam<=pkt_cam; cw_i<=0; cw_active<=1; end
            T_EDIT: ;   // clicked walls: see the uw write block
            T_RESET: reset_pending<=1;
            T_POSE: begin
                pose_pending<=1; frame<=pkt_frame;
                x<=pkt_x; y<=pkt_y; heading<=pkt_h;
            end
        endcase

        case(state)
            S_IDLE: if(reset_pending) begin
                reset_pending<=0; status<=WAIT_MAP; nr_cnt<=0; fr_cnt<=0;
                robot_cell<=0; prev<=0; target<=0; replans<=0;
                cidx<=0; state<=S_RST_MEM;
            end else if(pose_pending) begin
                pose_pending<=0; state<=S_CELL;
            end
            S_RST_MEM: begin
                if(cidx==8'd255) state<=S_IDLE; else cidx<=cidx+8'd1;
            end

            // ---- perception ----
            S_CELL: begin
                nr_cnt<=(status==NO_ROUTE) ? nr_cnt+5'd1 : 5'd0;
                age_r<=(status==NO_ROUTE) && (nr_cnt+5'd1==5'd0);
                fr_cnt<=fr_cnt+5'd1;
                age_lane_r<=(fr_cnt+5'd1==5'd0);
                robot_cell<={cell_row,cell_col};
                hidx<=(heading+16'd32)>>6;
                state<=S_TRIG;
            end
            S_TRIG: begin
                qvs_r<=qv_s; qvc_r<=qv_c; negs_r<=hidx[9]; negc_r<=hidx_c[9];
                state<=S_TRIG2;
            end
            S_TRIG2: begin
                sn<=negs_r ? -$signed({1'b0,qvs_r}) : $signed({1'b0,qvs_r});
                cs<=negc_r ? -$signed({1'b0,qvc_r}) : $signed({1'b0,qvc_r});
                ray<=0; cidx<=0;
                state<=S_HMCLR;
            end
            S_HMCLR: begin
                if(cidx==8'd255) state<=S_RAY_LOAD; else cidx<=cidx+8'd1;
            end
            S_RAY_LOAD: begin b<=camram[ray]; state<=S_RAY_D; end
            S_RAY_D: begin
                hit<=b<8'd250;
                d<=(b<8'd250) ? {b,2'b00}+{b,1'b0}+11'd12 : 11'd1500; // b*6+12
                sc<=8'sd31-$signed({2'b0,ray[4:0],1'b0});
                state<=S_RAY_LAT;
            end
            S_RAY_LAT: begin
                lat<=lat_full>>>5;
                state<=S_RAY_V;
            end
            S_RAY_V: begin
                case(ray[6:5])
                    2'd0: begin vx<=$signed({3'b0,d}); vy<=lat; end
                    2'd1: begin vx<=-lat; vy<=$signed({3'b0,d}); end
                    2'd2: begin vx<=-$signed({3'b0,d}); vy<=-lat; end
                    2'd3: begin vx<=lat; vy<=-$signed({3'b0,d}); end
                endcase
                state<=S_RAY_M;
            end
            S_RAY_M: begin
                p1<=vx*cs; p2<=vy*sn; p3<=vx*sn; p4<=vy*cs;
                state<=S_RAY_D2;
            end
            S_RAY_D2: begin
                dx<=(p1-p2)>>>10; dy<=(p3+p4)>>>10;
                k<=0; state<=S_SMP_M;
            end
            S_SMP_M: begin
                mx<=dx*$signed({1'b0,k}); my<=dy*$signed({1'b0,k});
                state<=S_SMP_P;
            end
            S_SMP_P: begin
                px<=x+(mx>>>4); py<=y+(my>>>4);
                state<=S_SMP_A;
            end
            S_SMP_A: begin
                in_arena<=px>=-18'sd1200 && px<18'sd1200 && py>-18'sd1200 && py<=18'sd1200;
                ux<=px+18'sd1200; uy<=18'sd1200-py;
                state<=S_SMP_Q;
            end
            S_SMP_Q: begin
                qx<=qxm[19:16]; qy<=qym[19:16];
                state<=S_SMP_C;
            end
            S_SMP_C: begin
                rx<=ux-qx*8'd150; ry<=uy-qy*8'd150; scell<={qy,qx};
                state<=S_SMP_MARK;
            end
            S_SMP_MARK: begin
                if(k==5'd16) begin
                    if(ray==7'd127) begin cidx<=0; state<=S_MEM_RD; end
                    else begin ray<=ray+7'd1; state<=S_RAY_LOAD; end
                end else begin k<=k+5'd1; state<=S_SMP_M; end
            end

            // ---- occupancy / mover memory; also seeds the halo distances and the reply masks ----
            S_MEM_RD: begin
                m<=cm_rd;
                h_c<=hm_cell[0] & ~RACK[cidx]; m_c<=hm_cell[1] & ~RACK[cidx]; ln_c<=LANE[cidx];
                state<=S_MEM_WR;
            end
            S_MEM_WR: begin
                blk_sh<={(occ_n>=2'd2)|uw[cidx], blk_sh[255:1]};
                mov_sh<={ttl_n!=6'd0, mov_sh[255:1]};
                if(cidx==8'd255) begin
                    c<=0; fwd<=1; state<=S_HF0;
                end else begin cidx<=cidx+8'd1; state<=S_MEM_RD; end
            end

            // ---- halo: forward pass (up, left), then backward pass (down, right) ----
            S_HF0: begin hn1_r<=hn1; hn2_r<=hn2; state<=S_HF1; end
            S_HF1: begin state<=S_HADV; end
            S_HADV: begin
                state<=S_HF0;
                if(fwd) begin
                    if(c==8'd255) fwd<=0; else c<=c+8'd1;
                end else begin
                    if(c==8'd0) begin cidx<=0; fwd<=1; state<=S_FINIT; end
                    else c<=c-8'd1;
                end
            end

            // ---- cost field ----
            S_FINIT: begin
                if(cidx==8'd255) begin
                    c<=0; fwd<=1; pair<=0; changed<=0; state<=S_SW0;
                end else cidx<=cidx+8'd1;
            end
            S_SW0: begin
                u_r<=up_v; d_r<=down_v; l_r<=left_v; r_r<=right_v;
                state<=S_SW1;
            end
            S_SW1: begin
                best<=min4; wreg<=4'd1+halo; skip_c<=(c==GOAL) || blocked_c;
                state<=S_SW1B;
            end
            S_SW1B: begin relax_r<=relax_val; state<=S_SW2; end
            S_SW2: begin
                if(!skip_c && relax_r<ra) changed<=1;
                state<=S_ADV;
            end
            S_ADV: begin
                state<=S_SW0;
                if(fwd) begin
                    if(c==8'd255) begin fwd<=0; end else c<=c+8'd1;
                end else begin
                    if(c==8'd0) begin
                        if(!changed || pair==6'd63) state<=S_DEC;
                        else begin pair<=pair+6'd1; changed<=0; fwd<=1; end
                    end else c<=c-8'd1;
                end
            end

            // ---- decide what to do with the new field ----
            S_DEC: begin
                if(status==WAIT_MAP || status==NO_ROUTE) state<=S_PICK;
                else if(status==RUNNING) begin fs_addr<=target; state<=S_DEC_T; end
                else begin left<=0; right<=0; state<=S_REPLY; end
            end
            S_DEC_T: if(ra>=INF) state<=S_PICK; else state<=S_GEOM;
            S_PICK: begin
                replans<=replans+8'd1;
                if(robot_cell==GOAL) begin
                    status<=AT_GOAL; prev<=robot_cell; target<=robot_cell;
                    left<=0; right<=0; state<=S_REPLY;
                end else begin
                    fs_sel<=robot_cell; fs_avoid_v<=0; fs_avoid<=0; fs_ret<=RET_PICK;
                    state<=S_FS_INIT;
                end
            end

            // ---- flow_step(field, fs_sel, avoid) ----
            S_FS_INIT: begin best_cost<=INF; best_v<=0; dirc<=0; state<=S_FS_ADDR; end
            S_FS_ADDR: begin
                fs_addr<=nb;
                nbv<=nb_ok && !(fs_avoid_v && nb==fs_avoid);
                state<=S_FS_CMP;
            end
            S_FS_CMP: begin
                if(nbv && ra<best_cost) begin best_cost<=ra; fs_best<=fs_addr; best_v<=1; end
                if(dirc==2'd3) state<=S_FS_AV0;
                else begin dirc<=dirc+2'd1; state<=S_FS_ADDR; end
            end
            S_FS_AV0: begin fs_addr<=fs_avoid; state<=S_FS_AV1; end
            S_FS_AV1: begin
                if(use_avoid) begin res<=fs_avoid; res_v<=1; end
                else begin res<=fs_best; res_v<=best_v; end
                state<=(fs_ret==RET_PICK) ? S_PICK_DONE : S_ARR_DONE;
            end
            S_PICK_DONE: begin
                if(res_v) begin
                    status<=RUNNING; prev<=robot_cell; target<=res; state<=S_GEOM;
                end else begin
                    status<=NO_ROUTE; left<=0; right<=0; state<=S_REPLY;
                end
            end

            // ---- drive ----
            S_GEOM: begin
                ex<=$signed({{2{x[15]}},x})-cx; ey<=$signed({{2{y[15]}},y})-cy;
                go_e<=target==prev+8'd1; go_n<=target==prev-8'd16; go_w<=target==prev-8'd1;
                state<=S_AIM;
            end
            S_AIM: begin
                along_r<=along;
                err<=base-lat_clamp-heading; // wraps like brain.wrap16()
                state<=S_DRIVE;
            end
            S_DRIVE: if(along_r<=ARRIVE_MM) begin
                // Waypoint reached.
                if(target==GOAL) begin
                    status<=AT_GOAL; left<=0; right<=0; state<=S_REPLY;
                end else begin
                    fs_sel<=target; fs_avoid_v<=1; fs_avoid<=prev; fs_ret<=RET_ARR;
                    state<=S_FS_INIT;
                end
            end else begin
                if(err>TURN_LIMIT) begin left<=-TURN_SPEED; right<=TURN_SPEED; end
                else if(err<-TURN_LIMIT) begin left<=TURN_SPEED; right<=-TURN_SPEED; end
                else begin left<=clamp_speed(l_raw); right<=clamp_speed(r_raw); end
                state<=S_REPLY;
            end
            S_ARR_DONE: begin
                prev<=target;
                if(res_v) begin target<=res; state<=S_GEOM; end
                else begin status<=NO_ROUTE; left<=0; right<=0; state<=S_REPLY; end
            end
            S_REPLY: begin reply_valid<=1; state<=S_IDLE; end
            default: state<=S_IDLE;
        endcase
    end
endmodule
