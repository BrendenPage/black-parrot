/*
 * bp_be_stride_detector.sv
 *
 * 
 */

`include "bp_common_defines.svh"
`include "bp_be_defines.svh"

module bp_be_stride_detector
 import bp_common_pkg::*;
 import bp_be_pkg::*;
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
   , input  logic [rv64_instr_width_gp-1:0]          instr_i
   , input  logic                                    instr_v_i
   , input  logic [effective_addr_width_p-1:0]       eff_addr_i

   // Second cycle input
   , input  logic [vaddr_width_p-1:0]                pc_i

  // Output interface
   , output  logic                                   start_discovery_o
   , output  logic                                   confirm_discovery_o
   , output  logic [vaddr_width_p-1:0]               striding_pc_o
   , output  logic [effective_addr_width_p-1:0]      eff_addr_o

   , output  logic [stride_width_p-1:0]              stride_o
   );

  `bp_cast_i(rv64_instr_fmatype_s, instr);

  // immediate offset for branch instruction
  logic [dword_width_gp-1:0] imm;

  // Is the instruction we are looking at a load instruction
  logic load_instr_v;

  logic [vaddr_width_p-1:0] striding_pc_lo;

  logic [stride_width_p-1:0] stride_lo;

  logic start_discovery_lo, confirm_discovery_lo, v_lo;

  // Need to latch inputs to rpt as the pc comes in next cycle
  logic [effective_addr_width_p-1:0] eff_addr_lo;

  always_ff @(posedge clk_i) begin
    if (reset_i) begin
      start_discovery_o    <= '0;
      confirm_discovery_o  <= '0;
      striding_pc_o        <= '0;
    end else begin
      if (confirm_discovery_lo || start_discovery_lo) begin
        confirm_discovery_o <= confirm_discovery_lo & |stride_lo;
        start_discovery_o <= start_discovery_lo & |stride_lo;

        striding_pc_o <= striding_pc_lo;
        eff_addr_o <= eff_addr_lo;
        stride_o <= stride_lo;
      end else begin
        confirm_discovery_o <= '0;
        start_discovery_o   <= '0;
        striding_pc_o       <= '0;
        eff_addr_o          <= '0;
        stride_o            <= '0;
      end
    end
  end

  bp_be_rpt #(.rpt_sets_p(rpt_sets_p)
              ,.stride_width_p(stride_width_p)
              ,.effective_addr_width_p(effective_addr_width_p))
    rpt
    (.clk_i(clk_i)
      ,.reset_i(reset_i)

      ,.init_done_o() // ignore
      ,.w_v_i(instr_v_i & load_instr_v)
      ,.pc_i(pc_i)
      ,.eff_addr_i(eff_addr_i)
      ,.eff_addr_o(eff_addr_lo)

      ,.stride_o(stride_lo)
      ,.stride_v_o(v_lo)
      ,.pc_o(striding_pc_lo)
      ,.start_discovery_o(start_discovery_lo)
      ,.confirm_discovery_o(confirm_discovery_lo));

  always_comb begin
    unique casez (instr_cast_i.opcode)
      `RV64_LOAD_OP  : begin
        load_instr_v = 1'b1;
        imm = `rv64_signext_i_imm(instr_cast_i);
      end
      `RV64_STORE_OP : begin
        load_instr_v = 1'b1;
        imm = `rv64_signext_s_imm(instr_cast_i);
      end
      default: begin
        imm = 0;
        load_instr_v = 1'b0;
      end
    endcase
  end
endmodule
