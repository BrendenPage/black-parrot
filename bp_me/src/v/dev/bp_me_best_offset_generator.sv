module bp_me_best_offset_generator
  #(parameter daddr_width_p = 64
    ,parameter lg_offsets_p = 6)
  (input                          clk_i
  , input                         reset_i
  , input [daddr_width_p-1:0]     daddr_i
  , input                         v_i // Valid data is an address that missed in the cache
  , output                        yumi_o //for the fifo
  , output                        ready_and_o
  , output [lg_offsets_p-1:0]    offset_o
  , output                        v_o
  );

  assign v_o = '0;
endmodule