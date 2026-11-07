`timescale 1ns/1ps
module bfs_engine(input wire clk, reset, start, input wire [5:0] start_cell,
                  input wire [63:0] obstacles,
                  output reg done, found, output reg [7:0] path_length, explored_count,
                  output reg event_valid, output reg [1:0] event_type,
                  output reg [5:0] event_cell);
    localparam IDLE=0, CLEAR=1, INIT=2, POP=3, TEST_GOAL=4,
               CHECK=5, ENQUEUE=6, NEXT_DIR=7, FOUND=8, TRACE=9,
               DONE_STATE=10, NO_PATH=11;
    reg [3:0] state;
    reg visited [0:63];
    reg [5:0] queue [0:63], parent [0:63];
    reg [6:0] head, tail;
    reg [5:0] current_cell, clear_index, trace_cell;
    reg [1:0] direction;
    wire valid_neighbor;
    wire [5:0] neighbor_cell;
    neighbor_check nc(.cell_id(current_cell), .direction(direction),
                      .valid(valid_neighbor), .neighbor(neighbor_cell));
    // event_type: 0=explored, 1=path (backward), 2=finished, 3=no path.
    always @(posedge clk) begin
        if (reset) begin
            state<=IDLE; head<=0; tail<=0; clear_index<=0;
            current_cell<=0; trace_cell<=0; direction<=0;
            done<=0; found<=0; path_length<=0; explored_count<=0;
            event_valid<=0; event_type<=0; event_cell<=0;
        end else begin
            event_valid <= 1'b0;
            case (state)
                IDLE: if (start) begin
                    clear_index<=0; done<=0; found<=0;
                    path_length<=0; explored_count<=0; state<=CLEAR;
                end
                CLEAR: begin
                    visited[clear_index]<=0;
                    if (clear_index==63) state<=INIT;
                    else clear_index<=clear_index+1'b1;
                end
                INIT: begin
                    queue[0]<=start_cell; visited[start_cell]<=1; head<=0; tail<=1; state<=POP;
                end
                POP: if (head==tail) state<=NO_PATH;
                     else begin
                         current_cell<=queue[head]; head<=head+1'b1;
                         direction<=0; explored_count<=explored_count+1'b1;
                         state<=TEST_GOAL;
                         event_valid<=1; event_type<=0; event_cell<=queue[head];
                     end
                TEST_GOAL: if (current_cell==63) state<=FOUND;
                           else state<=CHECK;
                CHECK: if (valid_neighbor && !obstacles[neighbor_cell] && !visited[neighbor_cell])
                           state<=ENQUEUE;
                       else state<=NEXT_DIR;
                ENQUEUE: begin
                    visited[neighbor_cell]<=1; parent[neighbor_cell]<=current_cell;
                    queue[tail]<=neighbor_cell; tail<=tail+1'b1; state<=NEXT_DIR;
                end
                NEXT_DIR: if (direction==3) state<=POP;
                          else begin direction<=direction+1'b1; state<=CHECK; end
                FOUND: begin trace_cell<=63; path_length<=0; state<=TRACE; end
                TRACE: begin
                    event_valid<=1; event_type<=1; event_cell<=trace_cell;
                    if (trace_cell==start_cell) state<=DONE_STATE;
                    else begin trace_cell<=parent[trace_cell]; path_length<=path_length+1'b1; end
                end
                DONE_STATE: if (!done) begin
                    done<=1; found<=1; event_valid<=1; event_type<=2; event_cell<=63;
                end
                NO_PATH: if (!done) begin
                    done<=1; found<=0; event_valid<=1; event_type<=3; event_cell<=0;
                end
                default: state<=IDLE;
            endcase
        end
    end
endmodule
