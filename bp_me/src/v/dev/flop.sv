/* Turns continuous input into pulses
*
* Inputs:
*  - clk:       System clock
*  - in :			 input desired to turn into a pulse
*
* Outputs:
*  - out:       resulting signal
*
*/
module flop(clk, in, out);
  input logic clk, in;
  output logic out;

  logic [1:0] ps;
  logic [1:0] ns;
  always_comb
    case(ps)
      2'b00: if (in == 1) ns = 2'b01;
        else ns = 2'b00;
      2'b01: if (in == 1) ns = 2'b10;
        else ns = 2'b00;
      2'b10: if(in == 1) ns = 2'b10;
        else ns = 2'b00;
      default: ns = 2'b00;
    endcase

  always_ff @(posedge clk)
    if (ns == 2'b01) begin
      out <= 1;
      ps <= ns;
    end
    else begin
      out <= 0;
      ps <= ns;
    end
endmodule