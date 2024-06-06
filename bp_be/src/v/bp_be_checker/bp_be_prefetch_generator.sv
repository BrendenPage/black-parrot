/*
 * bp_be_prefetch_generator.sv
 *
 * 
 */

`include "bp_common_defines.svh"
`include "bp_be_defines.svh"

module bp_be_prefetch_generator
 import bp_common_pkg::*;
 import bp_be_pkg::*;
 #(parameter bp_params_e bp_params_p = e_bp_default_cfg
   `declare_bp_proc_params(bp_params_p)

   , parameter loop_range_p = 8 // width of output amount
   , parameter stride_width_p = 8
   , localparam dcache_block_width_p
   , localparam default_loop_size_lp = 128
   )
   (input                                            clk_i
   , input                                           reset_i

   , input  rv64_instr_fmatype_s                     instr_i
   , input  logic [vaddr_width_p-1:0]                pc_i
   , input  logic [loop_range_p-1:0]                 loop_counter_i
   , input  logic [dpath_width_gp-1:0]               eff_addr_i
   , input  logic [stride_width_p-1:0]               stride_i

  // Striding load interface
   , input  logic                                    v_i
   , output logic                                    ready_and_o
   , input  logic                                    yumi_i
   , output logic                                    v_o

   );

  // immediate offset for branch instruction
  logic [dword_width_gp-1:0] imm_n, imm_r;

  // Store the register values for the first and second time we see each branch
  logic [dpath_width_gp-1:0] rs1_r, rs2_r, rs1_r2, rs2_r2;

  // Keep striding pc to filter out any branches that don't have our desired target
  logic [vaddr_width_p-1:0] striding_pc_r, branch_pc_r, branch_pc_n;

  // If we confirm the discovery mode, we don't want to be interrupted until we finish prediction.
  logic confirm_discovery_r, confirm_discovery_n;

  // Change the order of operands for LT and LTU to only implement GE/GEU
  logic swap_ops, swap_ops_r, swap_ops_n;

  // output value
  logic [output_range_p-1:0] remaining_iteratons_n;

  // final branch op register holds branch op for the final scouted branch instruction
  bp_be_int_fu_op_e branch_op_n, branch_op_r, f_branch_op_r;

  // Registers to store the denominator and distance values for output calculation
  logic [dpath_width_gp-1:0] denom_r, rdist_r;

  logic [vaddr_width_p-1:0] taken_tgt;

  logic [2:0] state_n, state_r;

  bsg_counter_set_down
    #(.width_p(loop_range_p))
    remaining_prefetches_ctr
      (.clk_i(clk_i)
      ,.reset_i(reset_i)
      ,.set_i()
      ,.val_i(loop_counter_i)
      ,.down_i(ready_and_o & v_i)
      ,.count_r_o()
      );

  always_ff @(posedge clk_i) begin
    if (reset_i) begin
      state_r <= '0;
    end else begin

    end
  end


  // FSM
  always_comb begin
    state_n = 3'b000;
    case(state_r)
      3'b000:
    endcase
  end


endmodule
