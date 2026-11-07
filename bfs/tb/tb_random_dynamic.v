`timescale 1ns/1ps
// Simulation-only scenario: random walls appear while the robot moves and one
// block patrols row 4. Every route comes from the real bfs_engine; the robot
// and world controller below is a testbench model, not nexys3_top.
// Options: +seed=N +wall_rate=PERCENT +max_walls=N
module tb_random_dynamic;
    localparam GOAL=63, MAX_STEPS=300;
    localparam [63:0] COL0=64'h0101010101010101, COL7=64'h8080808080808080,
                      ROW4=64'h000000ff00000000;
    reg clk=0, reset=1, start=0;
    reg [63:0] walls=0, path_mask=0, explored=0;
    reg [5:0] robot=0, mover=34, cand, path_seq [0:63];
    reg mover_right=0;
    // Procedural code reads the expression directly: a continuous-assign wire
    // would still hold its old value until simulation time advances.
    `define OBSTACLES (walls | (64'd1 << mover))
    wire [63:0] obstacles = `OBSTACLES;
    wire done, found, event_valid;
    wire [1:0] event_type;
    wire [5:0] event_cell;
    wire [7:0] path_length, explored_count;
    integer fd, seed, wall_rate, max_walls, wall_count=0, step=0, move_index=0,
            trace_n=0, replans=0, waits=0, k;

    bfs_engine dut(.clk(clk),.reset(reset),.start(start),.start_cell(robot),
        .obstacles(obstacles),.done(done),.found(found),.path_length(path_length),
        .explored_count(explored_count),.event_valid(event_valid),
        .event_type(event_type),.event_cell(event_cell));
    always #5 clk=~clk;
    always @(negedge clk) if (event_valid) begin
        if (event_type==0) explored[event_cell]=1'b1;
        if (event_type==1) begin path_seq[trace_n]=event_cell; trace_n=trace_n+1; end
    end

    // Reference shortest distance by bitmask flood fill; 255 = unreachable.
    function [7:0] ref_dist(input [63:0] obs, input [5:0] from);
        reg [63:0] r; integer i;
        begin
            r = 64'd1 << from; ref_dist = 8'hff;
            for (i=0; i<64 && ref_dist==8'hff; i=i+1)
                if (r[GOAL]) ref_dist = i;
                else r = (r | r<<8 | r>>8 | ((r<<1) & ~COL0) | ((r>>1) & ~COL7)) & ~obs;
        end
    endfunction

    function adjacent(input [5:0] a, b);
        adjacent = (a[2:0]==b[2:0] && (a[5:3]==b[5:3]+1 || b[5:3]==a[5:3]+1)) ||
                   (a[5:3]==b[5:3] && (a[2:0]==b[2:0]+1 || b[2:0]==a[2:0]+1));
    endfunction

    task frame(input [8*24-1:0] note);
        $fdisplay(fd,"FRAME,%0d,%0d,%0d,%016h,%016h,%016h,%0s",
                  step,robot,mover,walls,path_mask,explored,note);
    endtask

    task plan;
        begin
            trace_n=0; explored=0; path_mask=0;
            reset=1; @(negedge clk); reset=0; start=1; @(negedge clk); start=0;
            wait(done===1'b1); @(negedge clk);
            if (found !== (ref_dist(`OBSTACLES,robot)!=8'hff) ||
                (found && path_length!==ref_dist(`OBSTACLES,robot)))
                $fatal(1,"step %0d: engine found=%b len=%0d, reference len=%0d",
                       step,found,path_length,ref_dist(`OBSTACLES,robot));
            if (found) begin
                if (trace_n!=path_length+1 || path_seq[0]!=GOAL || path_seq[path_length]!=robot)
                    $fatal(1,"step %0d: malformed traceback",step);
                for (k=0; k<path_length; k=k+1) begin
                    if (!adjacent(path_seq[k],path_seq[k+1]) || walls[path_seq[k]] || mover==path_seq[k])
                        $fatal(1,"step %0d: route is not a legal walk",step);
                    path_mask[path_seq[k]]=1'b1;
                end
                move_index=path_length;
            end
            frame(found ? "planned" : "no route - waiting");
        end
    endtask

    task move_mover;
        reg [5:0] next;
        begin
            next = mover_right ? mover+1 : mover-1;
            if ((mover_right ? mover[2:0]==7 : mover[2:0]==0) || next==robot) begin
                mover_right = !mover_right;
                next = mover_right ? mover+1 : mover-1;
            end
            if (next!=robot) mover = next;
        end
    endtask

    // Half the drops land on the remaining route to force replanning. A wall is
    // rejected if it would seal the goal off permanently; row 4 stays free.
    task maybe_drop_wall;
        begin
            if (wall_count<max_walls && $unsigned($random(seed))%100 < wall_rate) begin
                if (move_index>1 && $unsigned($random(seed))%2)
                    cand = path_seq[$unsigned($random(seed)) % (move_index-1)];
                else cand = $unsigned($random(seed)) % 64;
                if (cand!=robot && cand!=GOAL && cand!=mover && !walls[cand] && !ROW4[cand] &&
                    ref_dist(walls | (64'd1<<cand), robot)!=8'hff) begin
                    walls[cand]=1'b1; wall_count=wall_count+1;
                    $display("step %0d: wall appears at cell %0d%0s",step,cand,
                             path_mask[cand] ? " (on route)" : "");
                end
            end
        end
    endtask

    initial begin
        if (!$value$plusargs("seed=%d",seed)) seed=1;
        if (!$value$plusargs("wall_rate=%d",wall_rate)) wall_rate=35;
        if (!$value$plusargs("max_walls=%d",max_walls)) max_walls=14;
        fd=$fopen("sim/random_dynamic.csv","w");
        if (fd==0) $fatal(1,"Create sim directory first");
        $display("seed=%0d wall_rate=%0d%% max_walls=%0d",seed,wall_rate,max_walls);
        frame("start");
        plan;
        while (robot!=GOAL) begin
            step=step+1;
            if (step>MAX_STEPS) $fatal(1,"Robot did not reach goal in %0d steps",MAX_STEPS);
            explored=0;
            move_mover;
            maybe_drop_wall;
            if (!found || (path_mask & `OBSTACLES)) begin
                replans=replans+1;
                frame(found ? "route blocked" : "retry");
                plan;
            end
            if (found) begin
                robot=path_seq[move_index-1]; move_index=move_index-1;
                path_mask[robot]=1'b0;
                if (walls[robot] || mover==robot) $fatal(1,"step %0d: robot collided at %0d",step,robot);
                frame("move");
            end else begin
                waits=waits+1;
                frame("waiting");
            end
        end
        frame("goal reached");
        $display("PASS: goal reached in %0d steps, %0d replans, %0d waits, %0d walls",
                 step,replans,waits,wall_count);
        $fclose(fd); $finish;
    end
    initial begin #50_000_000; $fatal(1,"Random dynamic test timeout"); end
endmodule
