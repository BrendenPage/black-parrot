 /* testbench.sv
  * 
  * Testbench for bp_be_prefetch_generator.sv
  *
  */



module testbench
 import bp_common_pkg::*;
 import bp_be_pkg::*;
 #(parameter bp_params_e bp_params_p = e_bp_default_cfg
   `declare_bp_proc_params(bp_params_p)

   // Tracing parameters
   , parameter stride_width_p  = 8
   , parameter loop_range_width_p = 8


   , parameter trace_file_p = "/mnt/users/ssd0/homes/bpage1/bsg/black-parrot-sim/black-parrot/bp_be/test/tb/rpt/random_access_rpt.tr"

   // Derived parameters
   , localparam trace_rom_addr_width_lp    = 20
   , localparam dispatch_pkt_width_lp      = `bp_be_dispatch_pkt_width(vaddr_width_p)
   , localparam decode_width_lp = $bits(bp_be_decode_s)
   , localparam trace_replay_data_width_lp = vaddr_width_p*4 + stride_width_p + 3
   )
  (output bit reset_i);


  bit clk_i;

  bsg_nonsynth_clock_gen
   #(.cycle_time_p(`BP_SIM_CLK_PERIOD))
   clock_gen
    (.o(clk_i));

  bsg_nonsynth_reset_gen
   #(.num_clocks_p(1)
     ,.reset_cycles_lo_p(0)
     ,.reset_cycles_hi_p(20)
     )
   reset_gen
    (.clk_i(clk_i)
     ,.async_reset_o(reset_i)
     );

  logic trace_v_lo;
  logic dut_ready_lo, dut_v_lo, stride_v_lo;

  logic [trace_replay_data_width_lp-1:0] trace_data_li, trace_data_lo;
  logic init_done_lo, init_done_r;
  logic start_discovery_lo, confirm_discovery_lo;

  logic [vaddr_width_p-1:0] pc_li;
  logic [vaddr_width_p-1:0] eff_addr_li;

  logic [trace_replay_data_width_lp-1:0] dut_data_lo;

  logic [trace_rom_addr_width_lp-1:0] trace_rom_addr_lo;
  logic [trace_replay_data_width_lp+3:0] trace_rom_data_li;

  logic [15:0] counter;
  always_ff @(posedge clk_i) begin
    if(reset_i)
      counter <= '0;
    else
      counter <= counter + 1'b1;
  end
  always_comb begin
    if(counter == 16'd65535) begin
      $display("FAIL: Timeout");
      $finish();
    end
  end

  logic  test_done_lo;
  always_ff @(negedge clk_i) begin
    if (&test_done_lo) begin
      $display("PASS");
      $finish();
    end
  end


  bsg_trace_replay
    #(.payload_width_p(trace_replay_data_width_lp)
      ,.rom_addr_width_p(trace_rom_addr_width_lp)
      ,.debug_p(2)
      )
    trace_replay
    (.clk_i(clk_i)
      ,.reset_i(reset_i)
      ,.en_i(1'b1)

      ,.v_i(stride_v_lo & (start_discovery_lo | confirm_discovery_lo))
      ,.data_i(dut_data_lo)
      ,.ready_o()

      ,.v_o(trace_v_lo)
      ,.data_o(trace_data_lo)
      ,.yumi_i(init_done_lo & trace_v_lo)

      ,.rom_addr_o(trace_rom_addr_lo)
      ,.rom_data_i(trace_rom_data_li)

      ,.done_o(test_done_lo)
      ,.error_o()
      );

  bsg_nonsynth_test_rom
    #(.data_width_p(trace_replay_data_width_lp+4)
      ,.addr_width_p(trace_rom_addr_width_lp)
      ,.filename_p(trace_file_p)
      )
    ROM
    (.addr_i(trace_rom_addr_lo)
      ,.data_o(trace_rom_data_li)
      );


  // recover inputs from trace data [w_v_i, pc_i, eff_addr_i]
  assign eff_addr_li     = trace_data_lo[vaddr_width_p-1:0];
  assign pc_li           = trace_data_lo[vaddr_width_p*2-1:vaddr_width_p];
  assign w_v_li          = trace_data_lo[vaddr_width_p*2];


  logic [vaddr_width_p-1:0]  eff_addr_lo, pc_lo;
  logic [stride_width_p-1:0] stride_lo;

  // [eff_addr, stride, pc, start_discovery, confirm_discovery]
  assign dut_data_lo = {eff_addr_lo, stride_lo, pc_lo, start_discovery_lo, confirm_discovery_lo, {(trace_replay_data_width_lp - (vaddr_width_p*2 + stride_width_p + 2)){1'b0}}};


  bp_be_rpt
    DUT
    (.clk_i(clk_i)
    ,.reset_i(reset_i)
    ,.init_done_o(init_done_lo)
    
    ,.w_v_i(w_v_li)
    ,.pc_i(pc_li)
    ,.eff_addr_i(eff_addr_li)

    ,.eff_addr_o(eff_addr_lo)
    ,.stride_o(stride_lo)
    ,.stride_v_o(stride_v_lo)
    ,.pc_o(pc_lo)
    ,.start_discovery_o(start_discovery_lo)
    ,.confirm_discovery_o(confirm_discovery_lo)
    );


  `ifndef VERILATOR
      initial
        begin
          $assertoff();
          @(posedge clk_i);
          @(negedge reset_i);
          $asserton();
        end
  `endif
endmodule

