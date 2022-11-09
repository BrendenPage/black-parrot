module bp_me_best_offset_generator
  #(parameter daddr_width_p = 64)
  (input                          clk_i
  , input                         reset_i
  , input [daddr_width_p-1:0]     daddr_i
  , input                         miss_v_i
  , output                        ready_and_o
  , output                        prefetching_active_o
  , output [daddr_width_p-1:0]    prefetch_addr_o
  , output v_o
  );

  assign prefetching_active_o = '0;
  assign v_o = '1;
endmodule