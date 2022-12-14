`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 08/31/2022 08:52:46 PM
// Design Name: sysclk_divider
// Module Name: sysclk_divider
// Project Name: See top module
// Target Devices: 
// Tool Versions: 
// Description: Clock divider
// 
// Dependencies: None
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module sysclk_divider(
   input wire clk8388,  // 8.388MHz output from mmcm
   input wire rst_n,
   output reg clk32768  // 32768KHz clock for always-on domain
);
	parameter NUM_DIV = 9'd256;
    reg [8:0] cnt;
    
    always @(posedge clk8388 or negedge rst_n) begin
        if(!rst_n) begin
            cnt <= 9'd0;
            clk32768 <= 1'b0;
        end
        else if(cnt < NUM_DIV / 2 - 1) begin
            cnt <= cnt + 1'b1;
            clk32768 <= clk32768;
        end
        else begin
            cnt <= 9'd0;
            clk32768 <= ~clk32768;
        end
    end

endmodule
