`timescale 1ns/1ps
module hil_brain(
    input wire clk,
    input wire pkt_valid, input wire [1:0] pkt_type, input wire [7:0] pkt_frame,
    input wire [2:0] pkt_band, input wire [7:0] pkt_bits,
    input wire pkt_add, input wire [5:0] pkt_cell,
    input wire signed [15:0] pkt_x, pkt_y, input wire [15:0] pkt_h,
    output reg reply_valid=0, output wire [191:0] reply,
    output wire [1:0] status_code, output wire planning);
    // The FPGA brain of Phase 2. It is ref/brain.py in hardware and must
    // match it exactly; bridge/replay_frames.py and tb_nexys3_hil check that.
    // Camera bands (C1) and wall edits (5A) update the grid as they arrive.
    // Each pose (C2) runs one step: robot cell from the pose, replan with
    // bfs_engine if needed, then the wheel speeds toward the next waypoint.
    // reply is the 24-byte B5 payload; reply_valid pulses when it is ready.
    localparam [1:0] T_EDIT=0, T_RESET=1, T_BAND=2, T_POSE=3;
    localparam [1:0] WAIT_MAP=0, RUNNING=1, NO_ROUTE=2, AT_GOAL=3; // = B5 codes
    localparam signed [17:0] ARRIVE_MM=15;
    localparam signed [15:0] TURN_LIMIT=2731;
    localparam signed [8:0] DRIVE_SPEED=100, TURN_SPEED=40, MAX_SPEED=125;

    reg [63:0] camera_grid=0, user_walls=0, obstacles=0;
    reg [63:0] route_mask=0;      // cells path[index-1..end], as brain.route_mask()
    reg [1:0] status=WAIT_MAP;
    reg [5:0] robot_cell=0, target=0, prev=0;
    reg [5:0] path_seq [0:63];    // goal first, robot cell at path_seq[length]
    reg [7:0] length=0, trace_count=0, replans=0, frame=0;
    reg [6:0] index=0;            // path[index] = path_seq[length-index]
    reg signed [15:0] x=0, y=0;
    reg [15:0] heading=0;
    reg signed [8:0] left=0, right=0;
    reg pose_pending=0, reset_pending=0;
    assign status_code=status;
    assign reply={route_mask, obstacles, replans, 2'b00, (status==RUNNING ? target : robot_cell),
                  2'b00, robot_cell, 6'b0, status, right[7:0], left[7:0], frame, 8'hb5};

    // Cell under the pose: count the column/row boundaries passed.
    // Same as (x+600)//150 and (600-y)//150, clamped to 0..7.
    function [2:0] col_of(input signed [15:0] v);
        col_of = (v>=-450)+(v>=-300)+(v>=-150)+(v>=0)+(v>=150)+(v>=300)+(v>=450);
    endfunction
    function [2:0] row_of(input signed [15:0] v);
        row_of = (v<=450)+(v<=300)+(v<=150)+(v<=0)+(v<=-150)+(v<=-300)+(v<=-450);
    endfunction
    wire [5:0] pose_cell={row_of(y), col_of(x)};

    // Waypoint geometry, spread over three clock cycles to keep each one
    // short at 100 MHz: GEOM registers the offset from the waypoint centre
    // and the travel direction, AIM registers the distance still to go and
    // the heading error, DRIVE picks the wheel speeds.
    wire signed [17:0] cx=-18'sd525+18'sd150*$signed({15'b0,target[2:0]});
    wire signed [17:0] cy=18'sd525-18'sd150*$signed({15'b0,target[5:3]});
    reg signed [17:0] ex=0, ey=0;          // robot minus waypoint centre (GEOM)
    reg go_e=0, go_n=0, go_w=0;
    wire signed [17:0] along=go_e ? -ex : go_n ? -ey : go_w ? ex : ey;
    wire signed [17:0] lat=go_e ? ey : go_n ? -ex : go_w ? -ey : ex;
    wire [15:0] base=go_e ? 16'd0 : go_n ? 16'd16384 : go_w ? 16'd32768 : 16'd49152;
    // clamp(lat << 6, +-4096) is the same as clamping lat to +-64 first.
    wire signed [15:0] lat_clamp=lat>18'sd64 ? 16'sd4096 : lat<-18'sd64 ? -16'sd4096
                                 : {lat[9:0],6'b0};
    reg signed [17:0] along_r=0;           // (AIM)
    reg signed [15:0] err=0;
    wire signed [8:0] steer=err>>>6;
    wire signed [9:0] l_raw=DRIVE_SPEED-steer, r_raw=DRIVE_SPEED+steer;

    function signed [8:0] clamp_speed(input signed [9:0] v);
        clamp_speed = v>MAX_SPEED ? MAX_SPEED : v<-MAX_SPEED ? -MAX_SPEED : v[8:0];
    endfunction

    // BFS engine, unchanged from Phase 1.
    reg engine_reset=1, engine_start=0;
    wire done, found, event_valid;
    wire [7:0] path_length, explored_count;
    wire [1:0] event_type;
    wire [5:0] event_cell;
    bfs_engine engine(.clk(clk),.reset(engine_reset),.start(engine_start),
        .start_cell(robot_cell),.obstacles(obstacles),.done(done),.found(found),
        .path_length(path_length),.explored_count(explored_count),
        .event_valid(event_valid),.event_type(event_type),.event_cell(event_cell));

    localparam IDLE=0, CELL=1, GRID=2, PLAN_RESET=3, PLAN_START=4, PLAN_WAIT=5,
               FETCH=6, GEOM=7, AIM=8, DRIVE=9, REPLY=10;
    reg [3:0] phase=IDLE;
    assign planning=phase==PLAN_RESET || phase==PLAN_START || phase==PLAN_WAIT;

    wire [63:0] seen=(camera_grid|user_walls) & ~((64'd1<<robot_cell)|(64'd1<<63));
    wire [63:0] remaining=route_mask & ~(64'd1<<prev);

    always @(posedge clk) begin
        reply_valid<=0; engine_reset<=0; engine_start<=0;

        // Packets. Bands and edits apply at once; reset and pose wait for
        // the step in progress (the PC only sends the next pose after the
        // reply, so they never actually queue up).
        if(pkt_valid) case(pkt_type)
            T_BAND: camera_grid[pkt_band*8 +: 8]<=pkt_bits;
            T_EDIT: if(pkt_cell!=0 && pkt_cell!=63 && pkt_cell!=robot_cell)
                        user_walls[pkt_cell]<=pkt_add;
            T_RESET: reset_pending<=1;
            T_POSE: begin
                pose_pending<=1; frame<=pkt_frame;
                x<=pkt_x; y<=pkt_y; heading<=pkt_h;
            end
        endcase

        if(event_valid && event_type==1 && phase==PLAN_WAIT) begin
            path_seq[trace_count]<=event_cell;
            trace_count<=trace_count+1'b1;
            route_mask[event_cell]<=1'b1;
        end

        case(phase)
            IDLE: if(reset_pending) begin
                reset_pending<=0; status<=WAIT_MAP; obstacles<=0; route_mask<=0;
                index<=0; robot_cell<=0; replans<=0; camera_grid<=0;
            end else if(pose_pending) begin
                pose_pending<=0; phase<=CELL;
            end
            CELL: begin robot_cell<=pose_cell; phase<=GRID; end
            GRID: begin
                obstacles<=seen;
                if(status==WAIT_MAP ||
                   (status==RUNNING && |(seen & ~obstacles & remaining)) ||
                   (status==NO_ROUTE && seen!=obstacles))
                    phase<=PLAN_RESET;
                else if(status==RUNNING) phase<=FETCH;
                else begin left<=0; right<=0; phase<=REPLY; end
            end
            PLAN_RESET: begin engine_reset<=1; route_mask<=0; trace_count<=0; phase<=PLAN_START; end
            PLAN_START: begin engine_start<=1; phase<=PLAN_WAIT; end
            PLAN_WAIT: if(done) begin
                replans<=replans+1'b1;
                length<=path_length; index<=1;
                if(found && path_length!=0) begin status<=RUNNING; phase<=FETCH; end
                else begin
                    status<=found ? AT_GOAL : NO_ROUTE;
                    left<=0; right<=0; phase<=REPLY;
                end
            end
            FETCH: begin
                target<=path_seq[length-index];
                prev<=path_seq[length-index+1'b1];
                phase<=GEOM;
            end
            GEOM: begin
                ex<=x-cx; ey<=y-cy;
                go_e<=target==prev+6'd1; go_n<=target==prev-6'd8; go_w<=target==prev-6'd1;
                phase<=AIM;
            end
            AIM: begin
                along_r<=along;
                err<=base-lat_clamp-heading; // wraps like brain.wrap16()
                phase<=DRIVE;
            end
            DRIVE: if(along_r<=ARRIVE_MM) begin
                // Waypoint reached: drop the cell just left from the route.
                route_mask[prev]<=1'b0;
                index<=index+1'b1;
                if(index==length) begin
                    status<=AT_GOAL; left<=0; right<=0; phase<=REPLY;
                end else phase<=FETCH;
            end else begin
                if(err>TURN_LIMIT) begin left<=-TURN_SPEED; right<=TURN_SPEED; end
                else if(err<-TURN_LIMIT) begin left<=TURN_SPEED; right<=-TURN_SPEED; end
                else begin left<=clamp_speed(l_raw); right<=clamp_speed(r_raw); end
                phase<=REPLY;
            end
            REPLY: begin reply_valid<=1; phase<=IDLE; end
            default: phase<=IDLE;
        endcase
    end
endmodule
