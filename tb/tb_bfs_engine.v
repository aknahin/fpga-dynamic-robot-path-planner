`timescale 1ns/1ps
module tb_bfs_engine;
    reg clk=0, reset=1, start=0;
    reg [63:0] obstacles=0;
    wire done, found, event_valid;
    wire [1:0] event_type;
    wire [5:0] event_cell;
    wire [7:0] path_length, explored_count;
    integer i, fd, scenario;
    bfs_engine dut(.clk(clk),.reset(reset),.start(start),.start_cell(6'd0),.obstacles(obstacles),
        .done(done),.found(found),.path_length(path_length),.explored_count(explored_count),
        .event_valid(event_valid),.event_type(event_type),.event_cell(event_cell));
    always #5 clk=~clk;
    always @(negedge clk) if (event_valid)
        $fdisplay(fd,"EVENT,%0d,%0d,%0d",scenario,event_type,event_cell);

    task run_case;
        input integer case_id;
        input expected_found;
        input [7:0] expected_length;
        begin
            scenario=case_id;
            $fdisplay(fd,"MAP,%0d,%016h",scenario,obstacles);
            @(negedge clk); reset=1;
            @(negedge clk); reset=0; start=1;
            @(negedge clk); start=0;
            wait(done===1'b1);
            #2;
            $display("case %0d: found=%b length=%0d explored=%0d",scenario,found,path_length,explored_count);
            $fdisplay(fd,"RESULT,%0d,%0d,%0d,%0d",scenario,found,path_length,explored_count);
            if(found!==expected_found || (expected_found && path_length!==expected_length))
                $fatal(1,"Incorrect BFS result");
        end
    endtask
    initial begin
        fd=$fopen("sim/events.csv","w");
        if(fd==0) $fatal(1,"Create sim directory first");
        $dumpfile("sim/bfs.vcd"); $dumpvars(0,tb_bfs_engine);
        run_case(0,1'b1,8'd14);
        for(i=0;i<7;i=i+1) obstacles[24+i]=1;
        for(i=1;i<8;i=i+1) obstacles[40+i]=1;
        run_case(1,1'b1,8'd28);
        obstacles[31]=1;
        run_case(2,1'b0,8'd0);
        obstacles[31]=0;
        run_case(3,1'b1,8'd28);
        $display("PASS: empty, detour, blocked, replanned");
        $fclose(fd); $finish;
    end
    initial begin #100000; $fatal(1,"Timeout"); end
endmodule
