/*
 * bp_fe_loop_profiler.sv
 *
 * 
 */

`include "bp_common_defines.svh"
`include "bp_be_defines.svh"

module bp_be_loop_inference
 import bp_common_pkg::*;
 import bp_be_pkg::*;
 #(parameter bp_params_e bp_params_p = e_bp_default_cfg
   `declare_bp_proc_params(bp_params_p)

   , parameter output_range_p = 8 // width of output amount
   , parameter effective_addr_width_p = vaddr_width_p
   , parameter stride_width_p = 8
   , parameter discovery_misses_p = 4'd8
   , parameter register_width_p = dpath_width_gp
   , localparam default_loop_size_lp = 128
   , localparam wb_pkt_width_lp = `bp_be_wb_pkt_width(vaddr_width_p)
   )
   (input                                            clk_i
   , input                                           reset_i

  // Branch interface
   , input  logic [effective_addr_width_p-1:0]       eff_addr_i
   , input  logic [stride_width_p-1:0]               stride_i

   // dispatch instruction input for register snooping
   , input  logic [rv64_instr_width_gp-1:0]          preissue_instr_i
   , input  logic [register_width_p-1:0]             rs1_i
   , input  logic [register_width_p-1:0]             rs2_i
   , input  logic [vaddr_width_p-1:0]                preissue_npc_i

  // writeback packet for register snooping
   , input [wb_pkt_width_lp-1:0]                     iwb_pkt_i

   , input logic [rv64_instr_width_gp-1:0]           instr_i
   , input logic                                     instr_v_i
   , input logic [vaddr_width_p-1:0]                 pc_i
   , input logic [vaddr_width_p-1:0]                 vaddr_i
   , input logic [vaddr_width_p-1:0]                 npc_i


  // Striding load interface
   , input  logic                                    start_discovery_i
   , input  logic                                    confirm_discovery_i
   , input  logic [vaddr_width_p-1:0]                striding_pc_i
  // output interface
   , output logic [output_range_p-1:0]               remaining_iteratons_o
   , output logic [vaddr_width_p-1:0]                pc_o
   , output logic [effective_addr_width_p-1:0]       eff_addr_o
   , output logic [stride_width_p-1:0]               stride_o
   , input  logic                                    yumi_i
   , output logic                                    v_o

   );

  `declare_bp_be_internal_if_structs(vaddr_width_p, paddr_width_p, asid_width_p, branch_metadata_fwd_width_p);

  `bp_cast_i(rv64_instr_btype_s, instr);
  `bp_cast_i(rv64_instr_fmatype_s, preissue_instr);
  `bp_cast_i(bp_be_wb_pkt_s, iwb_pkt);


  // passthrough registers for prefetch generator down the line
  logic [effective_addr_width_p-1:0] eff_addr_r;
  logic [stride_width_p-1:0] stride_r;

  // Keep striding pc to filter out any branches that don't have our desired target
  logic [vaddr_width_p-1:0] striding_pc_r, branch_pc_r, branch_pc_n;

  // If we confirm the discovery mode, we don't want to be interrupted until we finish prediction.
  logic confirm_discovery_r, confirm_discovery_n;

  // Change the order of operands for LT and LTU to only implement GE/GEU
  logic swap_ops, swap_ops_r, swap_ops_n;

  // output value
  logic [output_range_p-1:0] remaining_iteratons_n;

  // final branch op register holds if branch op for the final scouted branch instruction
  logic branch_op_v;

  // Registers to store the denominator and distance values for output calculation
  logic [register_width_p-1:0] denom_r, rdist_r;

  logic [vaddr_width_p-1:0] taken_tgt;

  logic [2:0] state_n, state_r;

  logic [`BSG_SAFE_CLOG2(discovery_misses_p + 1)-1:0] skips_remaining;
  bsg_counter_set_down
    #(.width_p(`BSG_SAFE_CLOG2(discovery_misses_p + 1)))
    discovery_cooldown
      (.clk_i(clk_i)
      ,.reset_i(reset_i)
      ,.set_i(state_r == 3'b001 && state_n == 3'b010)
      ,.val_i(discovery_misses_p)
      ,.down_i(start_discovery_i & state_r == 3'b010 & striding_pc_i != striding_pc_r)
      ,.count_r_o(skips_remaining)
      );

  // register difference calculations

  // Snooped registers for bounds determination
  logic [register_width_p-1:0] rs1_n, rs2_n, rs1_r, rs2_r, rs1_n2, rs2_n2, rs1_r2, rs2_r2;
  logic [reg_addr_width_gp-1:0] rs1_addr_n, rs2_addr_n, rs1_addr_r, rs2_addr_r;


  // We are only doing BGE/BGEU calculations for simplicity
  // so we can always do r1 distance - r2 distance
  wire [register_width_p-1:0] r1d = rs1_r2 - rs1_r;
  wire [register_width_p-1:0] r2d = rs2_r2 - rs2_r;
  // If both registers or no registers change then theres no accurate estimate we can make
  logic unable_to_determine_n, unable_to_determine_r;
  assign unable_to_determine_n = ((r1d != 0) && (r2d != 0)) || ((r1d == 0) && (r2d == 0));

  // width / stride

  wire [register_width_p-1:0] signed_rdist = rs1_r2 - rs2_r2;
  wire [register_width_p-1:0] rdist_n = signed_rdist[register_width_p-1] ? (~signed_rdist) + 1 : signed_rdist;
  wire [register_width_p-1:0] stride  = | r1d ? r1d : r2d;
  wire [register_width_p-1:0] magnitude = stride[register_width_p-1] ? (~stride) + 1 : stride;
  wire [register_width_p-1:0] denom_n  = $clog2(magnitude);


  assign remaining_iteratons_n = unable_to_determine_r ? default_loop_size_lp : rdist_r >> denom_r;
  assign v_o = state_r == 3'b111;

  always_ff @(posedge clk_i) begin
    if (reset_i) begin
      // Set all to zero
      state_r <= 3'b000;
      striding_pc_r <= '0;
      branch_pc_r <= '0;
      swap_ops_r <= '0;
      confirm_discovery_r <= '0;
      remaining_iteratons_o <= '0;
      eff_addr_r <= '0;
      stride_r <= '0;
      unable_to_determine_r <= '0;
      denom_r <= '0;
      rdist_r <= '0;
    end else
      confirm_discovery_r <= confirm_discovery_n;
      if (start_discovery_i & (!confirm_discovery_r | state_r == 3'b001)) begin
        // Discovering new striding load, set all to zero, latch striding load pc
        state_r <= 3'b001;
        striding_pc_r <= striding_pc_i;
        eff_addr_r <= eff_addr_i;
        stride_r <= stride_i;
        branch_pc_r <= '0;
        swap_ops_r <= '0;
      end else begin

        if (pc_i == striding_pc_r) begin
          eff_addr_r <= vaddr_i;
        end

        if (state_r == 3'b001) begin
          swap_ops_r <= swap_ops_n;
          branch_pc_r <= branch_pc_n;
        end

        if (state_r == 3'b100) begin
          denom_r <= denom_n;
          rdist_r <= rdist_n;
        end
        if (state_r == 3'b101) begin
          remaining_iteratons_o <= remaining_iteratons_n > default_loop_size_lp ? default_loop_size_lp : remaining_iteratons_n;
        end
      state_r <= state_n;
      unable_to_determine_r <= unable_to_determine_n;
      end

  end

  always_comb begin
    unique casez (instr_cast_i.opcode)
      `RV64_BRANCH_OP:
          begin
            unique casez (instr_cast_i)
              `RV64_BEQ  : branch_op_v = instr_v_i;
              `RV64_BNE  : branch_op_v = instr_v_i;
              `RV64_BLT  : begin
                branch_op_v = instr_v_i;
                swap_ops = 1'b1;
              end
              `RV64_BGE  : branch_op_v = instr_v_i;
              `RV64_BLTU : begin
                branch_op_v = instr_v_i;
                swap_ops = 1'b1;
              end
              `RV64_BGEU : branch_op_v = instr_v_i;
              default: begin
                branch_op_v = 1'b0;
                swap_ops = '0;
              end
            endcase
          end
      default: begin
        branch_op_v = 1'b0;
        swap_ops = '0;
      end

    endcase
  end

  assign confirm_discovery_n = state_n == 3'b000 ? 1'b0 : confirm_discovery_i ? 1'b1 : confirm_discovery_r;


  // REGISTER SNOOPING FSM

  logic [1:0] rstate_n, rstate_r;
  logic snoop_regs_r, snoop_regs_n, init_br_r, init_br_n;

  rv64_instr_fmatype_s preissue_instr_r;


  assign rs1_n = iwb_pkt_cast_i.ird_w_v & (iwb_pkt_cast_i.rd_addr == rs1_addr_r) ? iwb_pkt_cast_i.rd_data : rstate_r == 2'b01 ? rs1_r : rs1_r2;
  assign rs2_n = iwb_pkt_cast_i.ird_w_v & (iwb_pkt_cast_i.rd_addr == rs2_addr_r) ? iwb_pkt_cast_i.rd_data : rstate_r == 2'b01 ? rs2_r : rs2_r2;

  always_ff @(posedge clk_i) begin
    if (reset_i) begin
      rstate_r                        <= '0;
      snoop_regs_r                    <= '0;
      preissue_instr_r                <= '0;
      init_br_r                       <= '0;
      {rs1_r, rs2_r, rs1_r2, rs2_r2}  <= '0;
      {rs1_addr_r, rs2_addr_r}        <= '0;
    end else begin
      if (rstate_r == 2'b00) begin
        // capture initial values
        if (swap_ops_r) begin
          rs1_r <= rs2_i;
          rs2_r <= rs1_i;
          rs1_r2<= rs2_i;
          rs2_r2<= rs1_i;
          rs1_addr_r <= preissue_instr_r.rs2_addr;
          rs2_addr_r <= preissue_instr_r.rs1_addr;
        end else begin
          rs1_r <= rs1_i;
          rs2_r <= rs2_i;
          rs1_r2<= rs1_i;
          rs2_r2<= rs2_i;
          rs1_addr_r <= preissue_instr_r.rs1_addr;
          rs2_addr_r <= preissue_instr_r.rs2_addr;
        end
        init_br_r <= 1'b0;
      end

      if (rstate_r == 2'b01) begin
        init_br_r <= init_br_n;
        rs1_r <= {{$bits(bp_be_int_tag_e){rs1_n[register_width_p-$bits(bp_be_int_tag_e)-1]}}, rs1_n[register_width_p-$bits(bp_be_int_tag_e)-1:0]};
        rs2_r <= {{$bits(bp_be_int_tag_e){rs2_n[register_width_p-$bits(bp_be_int_tag_e)-1]}}, rs2_n[register_width_p-$bits(bp_be_int_tag_e)-1:0]};
      end

      if (rstate_r == 2'b10) begin
        rs1_r2 <= {{$bits(bp_be_int_tag_e){rs1_n[register_width_p-$bits(bp_be_int_tag_e)-1]}}, rs1_n[register_width_p-$bits(bp_be_int_tag_e)-1:0]};
        rs2_r2 <= {{$bits(bp_be_int_tag_e){rs2_n[register_width_p-$bits(bp_be_int_tag_e)-1]}}, rs2_n[register_width_p-$bits(bp_be_int_tag_e)-1:0]};
      end

      rstate_r <= rstate_n;
      snoop_regs_r <= snoop_regs_n;
      preissue_instr_r <= preissue_instr_cast_i;
    end
  end

  always_comb  begin
    rstate_n = 2'b00;
    init_br_n = 1'b0;
    case(rstate_r)
      2'b00:
      // wait to see branch instruction to get initial values and addresses
        rstate_n = preissue_npc_i == branch_pc_r && snoop_regs_r ? 2'b01 : 2'b00;
      2'b01: begin
        init_br_n = init_br_r ? 1'b1 : pc_i == branch_pc_r;
        rstate_n = state_n == 3'b000 ? 2'b00 : pc_i == branch_pc_r & init_br_r ? 2'b10 : 2'b01;
      end
      // snoop writeback commit packets into registers until we see the branch instruction committed, update the initial values with any register values found.
      // set "valid" on instruction committed is the second time we see the branch.
      2'b10:
        rstate_n = state_n == 3'b000 ? 2'b00 : pc_i == branch_pc_r ? 2'b00 : 2'b10;
      // wait for second branch instruction to be seen
    endcase
  end

  // LOOP PROFILER NEXT STATE LOGIC
  always_comb begin
    state_n = 3'b000;
    branch_pc_n = '0;
    swap_ops_n = swap_ops_r;
    snoop_regs_n = 1'b0;
    case(state_r)
      3'b000:
        // Waiting to enter discovery mode
        state_n = start_discovery_i ? 3'b001 : 3'b000;
      3'b001:
        // look for a branch instruction, we have just entered discovery mode
        if (branch_op_v && (npc_i < striding_pc_r) && (pc_i > striding_pc_r)) begin
          state_n = 3'b010;
          swap_ops_n = swap_ops;
          branch_pc_n = pc_i;
          snoop_regs_n = 1'b1;
        end else state_n = 3'b001;
      3'b010: begin
        // find the registers associated with the branch instruction
        snoop_regs_n = 1'b1;
        state_n = skips_remaining == 0 ? 3'b000 : (rstate_r == 2'b10 && rstate_n == 2'b00) ? 3'b100 : 3'b010;
      end

      3'b100:
        // Get difference of registers and set up for division
          state_n = 3'b101;
      3'b101:
        // Divide
          state_n = confirm_discovery_n | confirm_discovery_r ? 3'b111 : 3'b110;
      3'b110:
        // Wait to confirm discovery
          state_n = confirm_discovery_n | confirm_discovery_r ? 3'b111 : 3'b110;
      3'b111:
          if (yumi_i) begin
            state_n = 3'b000;
          end else begin
            state_n = 3'b111;
          end
    endcase
  end

  assign pc_o = striding_pc_r;
  assign eff_addr_o = eff_addr_r;
  assign stride_o = stride_r;
endmodule