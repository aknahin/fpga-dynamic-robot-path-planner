`timescale 1ns/1ps
module tb_nexys3_dynamic;
    reg clk=0, btn_up=0;
    reg [2:0] sw=3'b111;
    wire [2:0] led;
    wire uart_tx_pin;
    integer timeout;
    nexys3_top #(.MOVE_TICKS(200),.DEBOUNCE_TICKS(4),.UART_DIV(4)) dut (
        .clk(clk),.btn_up(btn_up),.sw(sw),.led(led),.uart_tx_pin(uart_tx_pin));
    always #5 clk=~clk;
    initial begin
        $dumpfile("sim/dynamic.vcd");
        $dumpvars(0,tb_nexys3_dynamic);
        repeat(8) @(negedge clk);
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
        $display("PASS: FPGA replanned from cell 8 around new obstacle at 32");
        $finish;
    end
    initial begin #200000; $fatal(1,"Dynamic test timeout"); end
endmodule
