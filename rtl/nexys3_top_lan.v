`timescale 1ns/1ps
module nexys3_top_lan #(parameter MOVE_TICKS=200000000, DEBOUNCE_TICKS=1000000,
                        HEARTBEAT_TICKS=50000000, PHY_RESET_TICKS=1000000)(
    input wire clk, btn_up, input wire [2:0] sw,
    output wire [6:0] led,
    input wire phy_tx_clk, output wire [3:0] phy_txd, output wire phy_tx_en,
    output wire phy_tx_er, output reg phy_rstn=0, output wire phy_mdc,
    input wire phy_rx_clk, input wire [3:0] phy_rxd, input wire phy_rx_dv);
    // Same planner as nexys3_top, but each update goes out over the RJ-45
    // port as a UDP broadcast (port 5005) instead of UART. The UDP payload
    // is the same 21-byte packet the UART version sends.
    // SW0/SW1: preset map; SW2: add/remove an obstacle at cell 32.
    // The PC can also add/remove a wall at any cell over UDP port 5006.
    // SW pins are pulled high when OFF, so ON corresponds to ~sw.
    reg [63:0] map=0, path_mask=0;
    reg [5:0] robot_cell=0;
    reg [5:0] path_seq [0:63]; // goal first, current robot last
    reg [6:0] trace_count=0, move_index=0;
    reg [27:0] move_timer=0;
    reg [2:0] sw_sync_a=3'b111, sw_sync_b=3'b111;
    reg [1:0] button_sync=0;
    reg stable_button=0, old_button=0, old_obstacle=0;
    reg [19:0] debounce=0;
    reg [2:0] phase=0;
    localparam WAIT_BUTTON=0, RESET_ENGINE=1, START_ENGINE=2,
               PLANNING=3, MOVING=4, STOPPED=5;
    reg engine_reset=1, engine_start=0;
    wire done, found, event_valid;
    wire [7:0] path_length, explored_count;
    wire [1:0] event_type;
    wire [5:0] event_cell;
    reg [7:0] result_length=0, result_explored=0;
    reg result_found=0;

    // Walls clicked on the PC. They stay for the next BTNU run, on top of
    // the SW0/SW1 map. Edits wait while BFS is running.
    reg [63:0] user_walls=0;
    reg edit_pending=0, edit_add=0;
    reg [5:0] edit_cell=0;
    wire cmd_valid, cmd_add;
    wire [5:0] cmd_cell;
    wire edit_ok=edit_cell!=0 && edit_cell!=63 && edit_cell!=robot_cell;

    // Packet: A5, robot, found, length, explored, 8 map bytes, 8 path bytes.
    // It is also resent every HEARTBEAT_TICKS so the PC can join at any time.
    reg packet_request=0;
    reg [25:0] heartbeat=0;
    reg [19:0] phy_reset_timer=0;
    wire eth_ready, sent_toggle;
    wire eth_send=packet_request && eth_ready;
    wire [167:0] payload={path_mask,map,result_explored,result_length,
                          7'b0,result_found,2'b00,robot_cell,8'ha5};

    // LD1 LD0: 00 at start, 01 running, 11 reached the goal, 10 no route.
    reg [1:0] status=2'b00;
    localparam AT_START=2'b00, RUNNING=2'b01, AT_GOAL=2'b11, NO_ROUTE=2'b10;
    assign led[1:0]=status;
    assign led[2]=phase==PLANNING;
    assign led[3]=sent_toggle; // flips once per Ethernet frame
    // Bring-up LEDs: LED4 on = PHY out of reset, LED5 blinks when the PHY's
    // TX_CLK is running, LED6 blinks when the 100 MHz clock is running.
    reg [24:0] tx_clk_count=0;
    reg [26:0] clk_count=0;
    always @(posedge phy_tx_clk) tx_clk_count<=tx_clk_count+1'b1;
    assign led[4]=phy_rstn;
    assign led[5]=tx_clk_count[24];
    assign led[6]=clk_count[26];
    assign phy_tx_er=1'b0;
    assign phy_mdc=1'b0; // PHY registers are left at their power-on settings

    bfs_engine engine(.clk(clk),.reset(engine_reset),.start(engine_start),
        .start_cell(robot_cell),.obstacles(map),.done(done),.found(found),
        .path_length(path_length),.explored_count(explored_count),
        .event_valid(event_valid),.event_type(event_type),.event_cell(event_cell));
    eth_udp_tx ethernet(.clk(clk),.send(eth_send),.payload(payload),.ready(eth_ready),
        .mii_tx_clk(phy_tx_clk),.mii_txd(phy_txd),.mii_tx_en(phy_tx_en),
        .sent_toggle(sent_toggle));
    eth_udp_rx receiver(.mii_rx_clk(phy_rx_clk),.mii_rxd(phy_rxd),.mii_rx_dv(phy_rx_dv),
        .clk(clk),.cmd_valid(cmd_valid),.cmd_add(cmd_add),.cmd_cell(cmd_cell));

    // Edit while waiting or stopped: no replan, except that removing a wall
    // after a "no route" result tries again from the robot's cell.
    task apply_idle_edit;
        begin
            edit_pending<=0;
            if(edit_ok) begin
                map[edit_cell]<=edit_add; user_walls[edit_cell]<=edit_add;
                packet_request<=1;
                if(!edit_add && status==NO_ROUTE) begin
                    engine_reset<=1; phase<=RESET_ENGINE; status<=RUNNING;
                end
            end
        end
    endtask

    always @(posedge clk) begin
        clk_count<=clk_count+1'b1;
        // Hold the PHY in reset for ~10 ms after configuration.
        if(!phy_rstn) begin
            if(phy_reset_timer==PHY_RESET_TICKS-1) phy_rstn<=1;
            else phy_reset_timer<=phy_reset_timer+1'b1;
        end

        // Placed before the state machine so a new request in the same
        // cycle is not lost.
        if(eth_send) packet_request<=0;
        if(heartbeat==HEARTBEAT_TICKS-1) begin heartbeat<=0; packet_request<=1; end
        else heartbeat<=heartbeat+1'b1;

        sw_sync_a<=sw; sw_sync_b<=sw_sync_a;
        button_sync<={button_sync[0],btn_up};
        if(button_sync[1]==stable_button) debounce<=0;
        else if(debounce==DEBOUNCE_TICKS-1) begin
            stable_button<=button_sync[1]; debounce<=0;
        end else debounce<=debounce+1'b1;
        old_button<=stable_button;
        old_obstacle<=~sw_sync_b[2];
        engine_reset<=0; engine_start<=0;
        if(cmd_valid) begin
            edit_pending<=1; edit_add<=cmd_add; edit_cell<=cmd_cell;
        end

        if(event_valid && event_type==1 && phase==PLANNING) begin
            path_seq[trace_count]<=event_cell;
            trace_count<=trace_count+1'b1;
            path_mask[event_cell]<=1'b1;
        end

        case(phase)
            WAIT_BUTTON: if(stable_button && !old_button) begin
                robot_cell<=0; path_mask<=0; move_timer<=0;
                case(~sw_sync_b[1:0])
                    2'd0: map<=user_walls;
                    2'd1: map<=64'h0000fe007f000000 | user_walls;
                    2'd2: map<=64'h0000fe00ff000000 | user_walls;
                    2'd3: map<=user_walls;
                endcase
                // Obstacle switch can be turned on after movement starts.
                engine_reset<=1; phase<=RESET_ENGINE; status<=RUNNING;
            end else if(edit_pending) apply_idle_edit;
            RESET_ENGINE: begin
                engine_reset<=1; path_mask<=0; trace_count<=0;
                phase<=START_ENGINE;
            end
            START_ENGINE: begin engine_start<=1; phase<=PLANNING; end
            PLANNING: if(done) begin
                result_found<=found; result_length<=path_length;
                result_explored<=explored_count;
                move_index<=path_length[6:0]; move_timer<=0;
                packet_request<=1;
                if(found && robot_cell!=63) phase<=MOVING;
                else begin
                    phase<=STOPPED; status<=found ? AT_GOAL : NO_ROUTE;
                end
            end
            MOVING: begin
                if((~sw_sync_b[2]) != old_obstacle) begin
                    map[32]<=~sw_sync_b[2];
                    // Only replan when the new obstacle intersects
                    // the remaining path (the current robot cell is excluded).
                    if((~sw_sync_b[2]) && path_mask[32] && robot_cell!=32) begin
                        engine_reset<=1; phase<=RESET_ENGINE; move_timer<=0;
                    end else packet_request<=1;
                end else if(edit_pending) begin
                    edit_pending<=0;
                    if(edit_ok) begin
                        map[edit_cell]<=edit_add; user_walls[edit_cell]<=edit_add;
                        // Same rule as SW2: replan only if the new wall is
                        // on the remaining route.
                        if(edit_add && path_mask[edit_cell]) begin
                            engine_reset<=1; phase<=RESET_ENGINE; move_timer<=0;
                        end else packet_request<=1;
                    end
                end else if(move_timer==MOVE_TICKS-1) begin
                    move_timer<=0;
                    if(move_index!=0) begin
                        robot_cell<=path_seq[move_index-1'b1];
                        path_mask[robot_cell]<=0;
                        move_index<=move_index-1'b1;
                        packet_request<=1;
                        if(move_index==1) begin phase<=STOPPED; status<=AT_GOAL; end
                    end
                end else move_timer<=move_timer+1'b1;
            end
            STOPPED: if(edit_pending) apply_idle_edit;
                else if(!stable_button) phase<=WAIT_BUTTON;
        endcase
    end
endmodule
