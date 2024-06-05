/*
 * bp_be_stride_detector.sv
 *
 * 
 */

`include "bp_common_defines.svh"
`include "bp_be_defines.svh"

module bp_be_stride_detector
 import bp_common_pkg::*;
 import bp_fe_pkg::*;
 #(parameter bp_params_e bp_params_p = e_bp_default_cfg
   `declare_bp_proc_params(bp_params_p)

   , parameter stride_width_p = 8
   , parameter rpt_sets_p = 32
   , parameter effective_addr_width_p = vaddr_width_p

   , localparam mem_ops_to_track_lp = 32
   , localparam sat_count_width_lp = 2
   )
   (input                                            clk_i
   , input                                           reset_i

  // Instruction interface
   , input  rv64_instr_fmatype_s                     instr_i
   , input  logic [dpath_width_gp-1:0]               rs1_i
   , input  logic                                    instr_v_i // dont need??

   // Second cycle input
   , input  logic [vaddr_width_p-1:0]                npc_i

  // Output interface
   , output  logic                                   start_discovery_o
   , output  logic                                   confirm_discovery_o
   , output  logic [vaddr_width_p-1:0]               striding_pc_o
   );

  // immediate offset for branch instruction
  logic [dword_width_gp-1:0] imm;

  // Is the instruction we are looking at a load instruction
  logic load_instr_v_n;

  // Set the immediate from the predecode packet to latch for next cycle
  // target computation and comparison
  wire  [effective_addr_width_p-1:0] effective_addr_n = rs1_i + imm;

  logic [vaddr_width-p-1:0] prev_prefetch_addr_1, prev_prefetch_addr_2;

  logic [vaddr_width_p-1:0] striding_pc_lo;

  logic [stride_width_p-1:0] stride_lo;

  logic start_discovery_lo, confirm_discovery_lo, v_lo;

  always_ff @(posedge clk_i) begin
    if (reset_i) begin
      prev_prefetch_addr_1 <= '0;
      prev_prefetch_addr_2 <= '0;
      start_discovery_o <= '0;
      striding_pc_o <= '0;
    end else begin
      if (confirm_discovery_lo || start_discovery_lo) begin
        // We haven't just tried to prefetch on this address
        prev_prefetch_addr_2 <= prev_prefetch_addr_1;
        prev_prefetch_addr_1 <= striding_pc_lo;
        confirm_discovery_o <= confirm_discovery_lo;
        start_discovery_o <= start_discovery_lo;

        striding_pc_o <= striding_pc_lo;
        stride_o <= stride_lo;
      end
    end
  end

  // Need to latch inputs to rpt as the pc comes in next cycle
  logic [effective_addr_width_p-1:0] eff_addr_r;
  logic load_instr_v_r;
  always_ff @(posedge clk_i) begin
    eff_addr_r <= effective_addr_n;
    load_instr_v_r <= load_instr_v_n;
  end

  bp_be_rpt #(.rpt_sets_p(rpt_sets_p)
              ,.stride_width_p(stride_width_p)
              ,.effective_addr_width_p(effective_addr_width_p))
    rpt
    (.clk_i(clk_i)
      ,.reset_i(reset_i)

      ,.init_done_o() // ignore
      ,.w_v_i(load_instr_v_r)
      ,.pc_in(npc_i)
      ,.eff_addr_i(eff_addr_r)

      ,.stride_o(stride_lo)
      ,.stride_v_o(v_lo)
      ,.pc_o(striding_pc_lo)
      ,.start_discovery_o(start_discovery_lo)
      ,.confirm_discovery_o(confirm_discovery_lo)

      );

  always_comb begin
    unique casez (instr_i.opcode)
      `RV64_LOAD_OP  : begin
        load_instr_v_n = 1'b1;
        imm = `rv64_signext_i_imm(instr_i);
      end
      `RV64_STORE_OP : begin
        load_instr_v_n = 1'b1;
        imm = `rv64_signext_s_imm(instr_i);
      end
      default: begin
        imm = 0;
        load_instr_v_n = 1'b0;
      end
    endcase
  end
endmodule