/*
 * bp_be_rpt.v
 *
 * Reference Prediction Table (RPT) records the stride history of loads, i.e.
 * previously calculated stride and previous effective address + saturating ctr.
 * Each entry consists of 2 bit saturation counter and stride info. If the counter value is
 * saturated, the RPT predicts striding; otherwise, we have not found a striding load.
 *
 * element: {instruction tag, previous effective address, previous stride, saturating ctr}
 * row: {element_1, element_0, lru}
 * associativity: 2, last bit in row is LRU bit
 *
 * First cycle: latch read, second cycle: compute dependencies, write, and set output values
 */
`include "bp_common_defines.svh"
`include "bp_be_defines.svh"

module bp_be_rpt
 import bp_common_pkg::*;
 import bp_be_pkg::*;
 #(parameter bp_params_e bp_params_p = e_bp_default_cfg
   `declare_bp_proc_params(bp_params_p)

   , parameter rpt_sets_p = 32
   , parameter stride_width_p = 8
   , parameter effective_addr_width_p = vaddr_width_p
   , localparam rpt_ctr_width_lp = 2
   , localparam rpt_tag_width_lp = vaddr_width_p - `BSG_SAFE_CLOG2(rpt_sets_p)
   , localparam rpt_entry_width_lp = (rpt_tag_width_lp + rpt_ctr_width_lp + stride_width_p + effective_addr_width_p)
   , localparam rpt_row_width_lp = rpt_entry_width_lp*2+1 // + 1 lru bit
   )
  (input                                  clk_i
   , input                                reset_i

   , output logic                         init_done_o

   , input                                w_v_i
   , input [vaddr_width_p-1:0]            pc_i
   , input [effective_addr_width_p-1:0]   eff_addr_i

   , output logic [stride_width_p-1:0]    stride_o
   , output logic                         stride_v_o
   , output logic [vaddr_width_p-1:0]     pc_o
   , output logic                         start_discovery_o
   , output logic                         confirm_discovery_o
   );

  // Initialization state machine, write all elements as zero
  enum logic [1:0] {e_reset, e_clear, e_run} state_n, state_r;
  wire is_reset = (state_r == e_reset);
  wire is_clear = (state_r == e_clear);
  wire is_run   = (state_r == e_run); // ready to accept input

  assign init_done_o = is_run;

  localparam idx_width_lp = `BSG_SAFE_CLOG2(rpt_sets_p);
  localparam rpt_init_lp = 'b0;


  logic [`BSG_WIDTH(rpt_sets_p)-1:0] init_cnt;
  bsg_counter_clear_up
   #(.max_val_p(rpt_sets_p), .init_val_p(0))
   init_counter
    (.clk_i(clk_i)
     ,.reset_i(reset_i)

     ,.clear_i(1'b0)
     ,.up_i(is_clear)
     ,.count_o(init_cnt)
     );
  wire finished_init = (init_cnt == rpt_sets_p-1'b1);

  always_comb
    case (state_r)
      e_clear: state_n = finished_init ? e_run : e_clear;
      e_run  : state_n = e_run;
      // e_reset
      default: state_n = e_clear;
    endcase

  // synopsys sync_set_reset "reset_i"
  always_ff @(posedge clk_i) begin
    if (reset_i)
      state_r <= e_reset;
    else
      state_r <= state_n;
  end


  bsg_dff
   #(.width_p(1))
   write_valid_reg
    (.clk_i(clk_i)
     ,.data_i(w_v_i)
     ,.data_o(w_v_li)
     );

  logic [effective_addr_width_p-1:0] eff_addr_r;
  bsg_dff
    #(.width_p(effective_addr_width_p))
    eff_addr_reg
    (.clk_i(clk_i)
     ,.data_i(eff_addr_i)
     ,.data_o(eff_addr_r)
     );

  logic [vaddr_width_p-1:0] pc_r;
  bsg_dff
    #(.width_p(vaddr_width_p))
    pc_reg
    (.clk_i(clk_i)
     ,.data_i(pc_i)
     ,.data_o(pc_r)
     );

  wire  [idx_width_lp-1:0] idx_li = is_clear ? init_cnt : pc_i[idx_width_lp-1:0];
  logic [idx_width_lp-1:0] idx_r;
  bsg_dff
    #(.width_p(idx_width_lp))
    idx_reg
    (.clk_i(clk_i)
     ,.data_i(idx_li)
     ,.data_o(idx_r)
     );


  wire                           mem_v_li = is_clear | w_v_li;
  logic [rpt_row_width_lp-1:0]   w_data_li;
  logic [rpt_entry_width_lp-1:0] w_data_1, w_data_2;

  logic [rpt_row_width_lp-1:0]   r_data_lo;
  logic [1:0]                    tag_match;
  logic [1:0]                    stride_match;
  logic [stride_width_p-1:0]     stride_li;
  logic [effective_addr_width_p-1: 0] effective_addr_lo;
  logic [rpt_ctr_width_lp-1:0]        ctr_1_n, ctr_2_n;

  wire lru = r_data_lo[0];
  // effective address retrieval
  wire [effective_addr_width_p-1:0] effective_addr_1 = r_data_lo[rpt_entry_width_lp-rpt_tag_width_lp : stride_width_p + rpt_ctr_width_lp + 1];
  wire [effective_addr_width_p-1:0] effective_addr_2 = r_data_lo[rpt_row_width_lp-rpt_tag_width_lp - 1 : rpt_entry_width_lp + stride_width_p + rpt_ctr_width_lp + 1];
  // tag retrieval
  wire [rpt_tag_width_lp-1:0] tag_1  = r_data_lo[rpt_entry_width_lp:rpt_entry_width_lp-rpt_tag_width_lp + 1];
  wire [rpt_tag_width_lp-1:0] tag_2  = r_data_lo[rpt_row_width_lp-1:rpt_row_width_lp-rpt_tag_width_lp];
  wire [rpt_tag_width_lp-1:0] tag_li = pc_r[vaddr_width_p-1 : vaddr_width_p - rpt_tag_width_lp];

  assign tag_match = {tag_2 == tag_li, tag_1 == tag_li};

  // stride retrieval
  wire [stride_width_p-1:0] stride_1 = r_data_lo[stride_width_p+rpt_ctr_width_lp:rpt_ctr_width_lp+1];
  wire [stride_width_p-1:0] stride_2 = r_data_lo[rpt_entry_width_lp + stride_width_p+rpt_ctr_width_lp : rpt_entry_width_lp + rpt_ctr_width_lp + 1];

  assign effective_addr_lo = |tag_match ? tag_match[0] ? effective_addr_1 : effective_addr_2 : '0;
  wire [effective_addr_width_p-1:0] addr_diff = eff_addr_i - effective_addr_lo; // todo why eff addr i
  assign stride_li = addr_diff[stride_width_p-1:0];


  assign stride_match = {stride_li == stride_2, stride_li == stride_1};

  // FSM retreival
  wire [rpt_ctr_width_lp-1:0] ctr_1 = r_data_lo[rpt_ctr_width_lp:1];
  wire [rpt_ctr_width_lp-1:0] ctr_2 = r_data_lo[rpt_entry_width_lp + rpt_ctr_width_lp:rpt_entry_width_lp + 1];

  // FSM update
  assign ctr_1_n = stride_match[0] ? &ctr_1 ? ctr_1 : {ctr_1[0] ^ ctr_1[1], ~ctr_1[0]} : '0;
  assign ctr_2_n = stride_match[1] ? &ctr_2 ? ctr_2 : {ctr_2[0] ^ ctr_2[1], ~ctr_2[0]} : '0;

  wire lru_n = tag_match[0] ? 1'b1 : tag_match[1] ? 1'b0 : ~lru;

  // Compile data
  assign w_data_1 = (&(~tag_match) & ~lru) | tag_match[0] ? {tag_li, eff_addr_i, stride_li, ctr_1_n} : r_data_lo[rpt_entry_width_lp:1]; // todo why eff addr i
  assign w_data_2 = (&(~tag_match) & lru) | tag_match[1] ? {tag_li, eff_addr_i, stride_li, ctr_2_n} : r_data_lo[rpt_row_width_lp-1:rpt_entry_width_lp]; // todo why eff addr i
  assign w_data_li = is_clear ? '0 : {w_data_2, w_data_1, lru_n};


  bsg_mem_1r1w_sync
   #(.width_p(rpt_row_width_lp), .els_p(rpt_sets_p), .read_write_same_addr_p(1))
   rpt_mem
    (.clk_i(clk_i)
     ,.reset_i(reset_i)

     ,.w_v_i(mem_v_li)
     ,.w_addr_i(idx_r)
     ,.w_data_i(w_data_li)

     ,.r_v_i(w_v_i)
     ,.r_addr_i(idx_li)
     ,.r_data_o(r_data_lo)
     );

  // Vector for each entry in RPT
  logic [rpt_sets_p*2-1:0] stride_vector_n, stride_vector_r, start_index;
  // output logic
  logic [stride_width_p-1:0] stride_r;
  wire [rpt_sets_p*2-1:0] new_idx = (1 << ((idx_r << 1) ^ lru));
  assign stride_vector_n = stride_vector_r | new_idx;
  assign stride_o = stride_r;
  always_ff @(posedge clk_i) begin
    confirm_discovery_o <= 1'b0;
    start_discovery_o <= 1'b0;
    stride_v_o <= 1'b0;
    if (mem_v_li & ((&ctr_1_n & tag_match[0]) | (&ctr_2_n & tag_match[1]))) begin
      stride_r <= tag_match[0] ? stride_1 : stride_2;
      stride_v_o <= 1'b1;
      pc_o <= pc_r;
      if (!stride_vector_r) begin
        // Start discovery, init stride vec
        start_discovery_o <= 1'b1;
        stride_vector_r <= stride_vector_n;
      end else begin
        // Check if we matched an index we've seen before
        if (new_idx & stride_vector_r) begin
          if (new_idx & ~start_index) begin
            // We have matched a secondary stride twice
            // restart discovery on new load, reinit vec
            start_discovery_o <= 1'b1;
            stride_vector_r <= new_idx;
            start_index <= new_idx;
          end else begin
            // End discovery
            confirm_discovery_o <= 1'b1;
            start_index <= '0;
            stride_vector_r <= '0;
          end
        end else
          stride_vector_r <= stride_vector_n;
      end
    end else begin
      stride_r <= '0;
      stride_v_o <= 1'b0;
      pc_o <= '0;
      start_discovery_o <= 1'b0;
    end
  end

endmodule
