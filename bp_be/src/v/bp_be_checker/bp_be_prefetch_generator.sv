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
   , parameter effective_addr_width_p = vaddr_width_p
   , localparam block_width_p = dcache_block_width_p
   , localparam dispatch_pkt_width_lp = `bp_be_dispatch_pkt_width(vaddr_width_p)
   )
   (input                                            clk_i
   , input                                           reset_i

   , input  logic [vaddr_width_p-1:0]                pc_i
   , input  logic [loop_range_p-1:0]                 loop_counter_i
   , input  logic [effective_addr_width_p-1:0]       eff_addr_i
   , input  logic [stride_width_p-1:0]               stride_i

  // Striding load interface
   , input  logic                                    v_i
   , output logic                                    ready_and_o

  // Dispatch pkt interface
   , input  logic                                    yumi_i
   , output logic                                    v_o
   , output logic [dispatch_pkt_width_lp-1:0]        dispatch_pkt_o

   );

  `declare_bp_be_internal_if_structs(vaddr_width_p, paddr_width_p, asid_width_p, branch_metadata_fwd_width_p);

  `bp_cast_o(bp_be_dispatch_pkt_s, dispatch_pkt);
  // `bp_cast_i(rv64_instr_ftype_s, instr);

  // Store the register values for the first and second time we see each branch
  logic [dpath_width_gp-1:0] eff_addr_r, eff_addr_n;
  logic [stride_width_p-1:0] stride_r;
  logic [vaddr_width_p-1:0]  pc_r;
  logic [loop_range_p-1:0]   loop_counter_r;

  logic [2:0] state_n, state_r;

  logic [vaddr_width_p-`BSG_SAFE_CLOG2(block_width_p)-1:0] prev_block_n, prev_block_r;

  bsg_counter_set_down
    #(.width_p(loop_range_p))
    remaining_prefetches_ctr
      (.clk_i(clk_i)
      ,.reset_i(reset_i)
      ,.set_i(state_r == 3'b000 & state_n == 3'b001)
      ,.val_i(loop_counter_i)
      ,.down_i(((ready_and_o & v_i) || (state_r == 3'b001 && prev_block_n == prev_block_r)) && loop_counter_r != '0)
      ,.count_r_o(loop_counter_r)
      );

  always_ff @(posedge clk_i) begin
    if (reset_i) begin
      state_r <= '0;
    end else begin
        case (state_r)
          3'b000: begin
            stride_r <= stride_i;
            prev_block_r <= eff_addr_i[vaddr_width_p-1:`BSG_SAFE_CLOG2(block_width_p)];
            eff_addr_r <= eff_addr_i;
            pc_r <= pc_i;
          end
          3'b001: begin
            prev_block_r <= prev_block_n;
            eff_addr_r <= eff_addr_n;
          end
        endcase
        state_r <= state_n;
    end
  end


  assign eff_addr_n = eff_addr_r + stride_r;
  assign prev_block_n = eff_addr_n[vaddr_width_p-1:`BSG_SAFE_CLOG2(block_width_p)];

  // FSM
  always_comb begin
    state_n = 3'b000;
    case(state_r)
      // wait
      3'b000: begin
        state_n = v_i && loop_counter_i != '0 & |stride_i ? 3'b001 : 3'b000;
      end
      // latched prefetch info, iterate stride and loop count until next block
      3'b001: begin
        state_n = loop_counter_r == 1 && prev_block_r == prev_block_n ? 3'b000 : prev_block_r == prev_block_n ? 3'b001 : 3'b010;
      end
      // Send prefetch
      3'b010: begin
        state_n = yumi_i ? loop_counter_r == 3'b000 ? 3'b000 : 3'b001 : 3'b010;
      end
    endcase
  end


  rv64_instr_stype_s instr;
  always_comb
    begin // prefetch.r specification CMO ext
      instr.imm11to5  = '0;
      instr.rs2       = 5'b00001;
      instr.rs1       = '0;
      instr.funct3    = 3'b110;
      instr.imm4to0   = '0;
      instr.opcode    = `RV64_OP_IMM_OP;
    end


  bp_be_decode_s decode;
  always_comb
    begin
      decode = '0;
      // Pulled from decode of load instruction
      decode.pipe_mem_early_v = 1'b1;
      decode.irf_w_v          = 1'b0;
      decode.spec_w_v         = 1'b1;
      decode.score_v          = 1'b0;
      decode.dcache_r_v       = 1'b1;
      decode.mem_v            = 1'b1;
      decode.fu_op  = e_dcache_op_lb;
      decode.prefetch         = 1'b1; // flag to prevent faults in MMU
      decode.irs1_tag         = e_int_word;
      decode.irs1_unsigned    = 1'b1;
      decode.ird_tag          = e_int_word;
    end

  always_comb
    begin
      // Form dispatch packet
      dispatch_pkt_cast_o = '0;
      dispatch_pkt_cast_o.pc         = '1;
      dispatch_pkt_cast_o.v          = 1'b1;
      dispatch_pkt_cast_o.nspec_v    = 1'b1;
      dispatch_pkt_cast_o.pc         = pc_r;
      dispatch_pkt_cast_o.instr      = instr;
      dispatch_pkt_cast_o.rs1        = eff_addr_r;
      dispatch_pkt_cast_o.decode     = decode;
    end

  assign ready_and_o = state_r == 3'b000;
  assign v_o         = state_r == 3'b010;

endmodule
