/*
 * bp_fe_loop_profiler.sv
 *
 * 
 */

`include "bp_common_defines.svh"
`include "bp_be_defines.svh"

module bp_be_loop_inference
 import bp_common_pkg::*;
 import bp_fe_pkg::*;
 #(parameter bp_params_e bp_params_p = e_bp_default_cfg
   `declare_bp_proc_params(bp_params_p)

   , localparam output_range_lp = 8 // width of output amount
   , localparam default_loop_size_lp = 128
   )
   (input                                            clk_i
   , input                                           reset_i

  // Branch interface
   , input  rv64_instr_fmatype_s                     instr_i
   , input  logic [dpath_width_gp-1:0]               rs1_i
   , input  logic [dpath_width_gp-1:0]               rs2_i

   // Second cycle input
   , input  logic [vaddr_width_p-1:0]                npc_i

  // Striding load interface
   , input  logic                                    start_discovery_i
   , input  logic [vaddr_width_p-1:0]                striding_pc

  // output interface
   , output logic [output_range_lp-1:0]              remaining_iteratons_o
   , input  logic                                    yumi_i
   , output logic                                    v_o

   );

  logic is_backwards_branch;
  logic calculate_target_offset;


  // immediate offset for branch instruction
  logic [dword_width_gp-1:0] imm_n, imm_r;

  // Store the register values for the first and second time we see each branch
  logic [dpath_width_gp-1:0] rs1_r, rs2_r, rs1_r2, rs2_r2;

  // Keep striding pc to filter out any branches that don't have our desired target
  logic [vaddr_width_p-1:0] striding_pc_r, branch_pc_r, branch_pc_n;

  logic check_tgt_vs_stride;

  // Change the order of operands for LT and LTU to only implement GE/GEU
  logic swap_ops, swap_ops_r, swap_ops_n;

  // final branch op register holds branch op for the final scouted branch instruction
  bp_be_int_fu_op_e branch_op_n, branch_op_r, f_branch_op_r;

  logic [vaddr_width_p-1:0] taken_tgt;

  logic [2:0] state_n, state_r;

  // bp_be_pipe_int ~90

  // Set the immediate from the predecode packet to latch for next cycle
  // target computation and comparison
  assign imm_n = `rv64_signext_b_imm(instr_i);
  wire  [vaddr_width_p-1:0] taken_raw = npc_i + imm_r;
  assign taken_tgt = {taken_raw[vaddr_width_p-1:1], 1'b0};


  always_ff @(posedge clk_i) begin
    if (reset) begin
      // Set all to zero
      state_r <= 3'b000;
      striding_pc_r <= '0;
      check_tgt_vs_stride <= 1'b0;
      branch_op_r <= '0;
      branch_pc_r <= '0;
      swap_ops_r <= '0;
      {rs1_r, rs1_r2, rs2_r, rs2_r2} <= '0;
    end else if (start_discovery_i) begin
        // Discovering new striding load, set all to zero, latch striding load pc
        state_r <= 3'b000;
        striding_pc_r <= striding_pc;
        check_tgt_vs_stride <= '0;
        branch_op_r <= '0;
        branch_pc_r <= '0;
        swap_ops_r <= '0;
        {rs1_r, rs1_r2, rs2_r, rs2_r2} <= '0;
      // If there is a branch and its target is signed negative relative to PC
    end else begin
      state_r <= state_n;
      swap_ops_r <= swap_ops_n;
      branch_pc_r <= branch_pc_n;

      if (state_n == 3'b001) begin
        branch_op_r <= branch_op_n;
        check_tgt_vs_stride <= 1'b1;
        imm_r <= imm_n;
      end

      if (state_n == 3'b010) begin
        if (swap_ops_n) begin
          rs1_r <= rs2_i;
          rs2_r <= rs1_i;
        end else begin
          rs1_r <= rs1_i;
          rs2_r <= rs2_i;
        end
      end
      
      if (state_n == 3'b011) begin
        if (swap_ops_n) begin
          rs1_r2 <= rs2_i;
          rs2_r2 <= rs1_i;
        end else begin
          rs1_r2 <= rs1_i;
          rs2_r2 <= rs2_i;
        end
      end
    end


  end


  // register difference calculations

  logic [dpath_width_gp-1:0] denom_r, rdist_r;


  // We are only doing BGE/BGEU calculations for simplicity
  // so we can always do r1 distance - r2 distance
  wire [dpath_width_gp-1:0] r1d = rs1_r2 - rs1_r;
  wire [dpath_width_gp-1:0] r2d = rs2_r2 - rs2_r;
  // If both registers or no registers change then theres no accurate estimate we can make
  wire unable_to_determine = ((r1d != 0) && (r2d != 0)) || ((r1d == 0) && (r2d == 0))

  wire [dpath_width_gp-1:0] rdist = rs1_r2 - rs2_r2;
  wire [dpath_width_gp-1:0] temp  = (| r1d) ? ($clog2(r1d) - 1) : ($clog2(r2d) -1)
  wire [dpath_width_gp-1:0] denom_n =  1 << temp;
  
  assign remaining_iteratons_o = unable_to_determine ? default_loop_size_lp : rdist_r >> denom_r;


  always_comb begin
    branch_op_n = '0;
    swap_ops = '0;
    unique casez (instr_i.opcode)
      `RV64_BRANCH_OP:
          begin
            unique casez (instr_i)
              `RV64_BEQ  : branch_op_n = e_int_op_eq;
              `RV64_BNE  : branch_op_n = e_int_op_ne;
              `RV64_BLT  : begin
                branch_op_n = e_int_op_sge;
                swap_ops = 1'b1;
              end
              `RV64_BGE  : branch_op_n = e_int_op_sge;
              `RV64_BLTU : begin
                branch_op_n = e_int_op_sgeu;
                swap_ops = 1'b1;
              end
              `RV64_BGEU : branch_op_n = e_int_op_sgeu;
            endcase
          end
    endcase
  end

  // FSM
  always_comb begin
    state_n = 3'b000;
    branch_pc_n = '0;
    swap_ops_n = swap_ops_r;
    case(state_r)
      3'b000:
        // look for a branch instruction, we have just entered discovery mode
        if (|branch_op_n & imm_n[dword_width_gp-1]) begin
          state_n = 3'b001;
          swap_ops_n = swap_ops;
      end

      3'b001: begin
        // Calculate the target address and compare against the striding pc, if less
        // then we have found our branch, go to next state to wait to see this branch again
        // else restart search
        if (taken_tgt > striding_pc_r) begin
          state_n = 3'b000;
        end else begin
          // branch is to before the striding load
          state_n = 3'b010;
          branch_pc_n = npc_i;
        end
      end

      3'b010:
        // We now are just waiting until we see the same branch again to compare register state
        if (npc_i == branch_pc_r) begin
          state_n = 3'b011;
        end else begin
          state_n = 3'b010;
          branch_pc_n = branch_pc_r;
        end

      3'b011: 
        // Get difference of registers and set up for division
          state_n = 3'b100;
      3'b100: 
        // Get difference of registers and set up for division
          state_n = 3'b101;
      3'b101:
          if (yumi_i) begin
            state_n = 3'b000;
          end else begin
            state_n = 3'b101;
          end
    endcase
  end


endmodule