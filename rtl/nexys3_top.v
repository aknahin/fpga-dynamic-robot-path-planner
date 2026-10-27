`timescale 1ns/1ps
module nexys3_top #(parameter MOVE_TICKS=200000000, DEBOUNCE_TICKS=1000000,
                    UART_DIV=868)(
    input wire clk, btn_up, input wire [2:0] sw,
    output wire [2:0] led, output wire uart_tx_pin);
    // SW0/SW1: preset map; SW2: add/remove an obstacle at cell 32.
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

    // UART packet: A5, robot, found, length, explored, 8 map bytes,
    // 8 path bytes. Each packet comes from actual FPGA registers.
    reg [4:0] packet_index=0;
    reg [1:0] tx_phase=0;
    reg packet_request=0, tx_send=0;
    reg [7:0] tx_data=0;
    reg uart_reset=1;
    wire tx_busy;
    reg [63:0] packet_map=0, packet_path=0;
    reg [5:0] packet_robot=0;
    reg packet_found=0;
    reg [7:0] packet_length=0, packet_explored=0;

    assign led[0]=phase==MOVING;
    assign led[1]=result_found;
    assign led[2]=phase==PLANNING;
    bfs_engine engine(.clk(clk),.reset(engine_reset),.start(engine_start),
        .start_cell(robot_cell),.obstacles(map),.done(done),.found(found),
        .path_length(path_length),.explored_count(explored_count),
        .event_valid(event_valid),.event_type(event_type),.event_cell(event_cell));
    uart_tx #(.DIV(UART_DIV)) serial(.clk(clk),.reset(uart_reset),
        .send(tx_send),.data(tx_data),.tx(uart_tx_pin),.busy(tx_busy));

    always @(posedge clk) begin
        sw_sync_a<=sw; sw_sync_b<=sw_sync_a;
        button_sync<={button_sync[0],btn_up};
        if(button_sync[1]==stable_button) debounce<=0;
        else if(debounce==DEBOUNCE_TICKS-1) begin
            stable_button<=button_sync[1]; debounce<=0;
        end else debounce<=debounce+1'b1;
        old_button<=stable_button;
        old_obstacle<=~sw_sync_b[2];
        engine_reset<=0; engine_start<=0; uart_reset<=0;

        if(event_valid && event_type==1 && phase==PLANNING) begin
            path_seq[trace_count]<=event_cell;
            trace_count<=trace_count+1'b1;
            path_mask[event_cell]<=1'b1;
        end

        case(phase)
            WAIT_BUTTON: if(stable_button && !old_button) begin
                robot_cell<=0; path_mask<=0; move_timer<=0;
                case(~sw_sync_b[1:0])
                    2'd0: map<=64'd0;
                    2'd1: map<=64'h0000fe007f000000;
                    2'd2: map<=64'h0000fe00ff000000;
                    2'd3: map<=64'd0;
                endcase
                // Obstacle switch can be turned on after movement starts.
                engine_reset<=1; phase<=RESET_ENGINE;
            end
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
                else phase<=STOPPED;
            end
            MOVING: begin
                if((~sw_sync_b[2]) != old_obstacle) begin
                    map[32]<=~sw_sync_b[2];
                    // Only replan when the new obstacle intersects
                    // the remaining path (the current robot cell is excluded).
                    if((~sw_sync_b[2]) && path_mask[32] && robot_cell!=32) begin
                        engine_reset<=1; phase<=RESET_ENGINE; move_timer<=0;
                    end else packet_request<=1;
                end else if(move_timer==MOVE_TICKS-1) begin
                    move_timer<=0;
                    if(move_index!=0) begin
                        robot_cell<=path_seq[move_index-1'b1];
                        path_mask[robot_cell]<=0;
                        move_index<=move_index-1'b1;
                        packet_request<=1;
                        if(move_index==1) phase<=STOPPED;
                    end
                end else move_timer<=move_timer+1'b1;
            end
            STOPPED: if(!stable_button) phase<=WAIT_BUTTON;
        endcase

        tx_send<=0;
        case(tx_phase)
            0: if(packet_request && !tx_busy) begin
                packet_request<=0;
                packet_map<=map; packet_path<=path_mask;
                packet_robot<=robot_cell; packet_found<=result_found;
                packet_length<=result_length; packet_explored<=result_explored;
                packet_index<=0; tx_phase<=1;
            end
            1: if(!tx_busy) begin
                case(packet_index)
                    0: tx_data<=8'hA5;
                    1: tx_data<={2'b00,packet_robot};
                    2: tx_data<={7'b0,packet_found};
                    3: tx_data<=packet_length;
                    4: tx_data<=packet_explored;
                    default: if(packet_index<13)
                        tx_data<=packet_map[(packet_index-5)*8 +: 8];
                    else tx_data<=packet_path[(packet_index-13)*8 +: 8];
                endcase
                tx_phase<=2;
            end
            2: begin tx_send<=1; tx_phase<=3; end
            3: if(tx_busy) tx_phase<=4;
            4: if(!tx_busy) begin
                if(packet_index==20) tx_phase<=0;
                else begin packet_index<=packet_index+1'b1; tx_phase<=1; end
            end
        endcase
    end
endmodule
