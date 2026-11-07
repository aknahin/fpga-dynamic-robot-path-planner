`timescale 1ns/1ps
module neighbor_check(input wire [5:0] cell_id, input wire [1:0] direction,
                      output reg valid, output reg [5:0] neighbor);
    always @* begin
        valid = 1'b0;
        neighbor = cell_id;
        case (direction)
            2'd0: if (cell_id[5:3] != 0) begin valid=1; neighbor=cell_id-6'd8; end
            2'd1: if (cell_id[5:3] != 7) begin valid=1; neighbor=cell_id+6'd8; end
            2'd2: if (cell_id[2:0] != 0) begin valid=1; neighbor=cell_id-6'd1; end
            2'd3: if (cell_id[2:0] != 7) begin valid=1; neighbor=cell_id+6'd1; end
        endcase
    end
endmodule
