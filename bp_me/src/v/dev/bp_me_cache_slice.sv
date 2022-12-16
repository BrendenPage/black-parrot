/**
 *
 * Name:
 *   bp_me_cache_slice.sv
 *
 * Description:
 *
 */

`include "bp_common_defines.svh"
`include "bp_me_defines.svh"
`include "bsg_cache.vh"
`include "bsg_noc_links.vh"

module bp_me_cache_slice
 import bp_common_pkg::*;
 import bp_me_pkg::*;
 import bsg_noc_pkg::*;
 import bsg_cache_pkg::*;
 #(parameter bp_params_e bp_params_p = e_bp_default_cfg
   `declare_bp_proc_params(bp_params_p)

   `declare_bp_bedrock_mem_if_widths(paddr_width_p, did_width_p, lce_id_width_p, lce_assoc_p)

   , localparam dma_pkt_width_lp = `bsg_cache_dma_pkt_width(daddr_width_p)
   )
  (input                                                 clk_i
   , input                                               reset_i

   , input  [mem_header_width_lp-1:0]                    mem_cmd_header_i
   , input  [l2_data_width_p-1:0]                        mem_cmd_data_i
   , input                                               mem_cmd_v_i
   , output logic                                        mem_cmd_ready_and_o
   , input                                               mem_cmd_last_i

   , output logic [mem_header_width_lp-1:0]              mem_resp_header_o
   , output logic [l2_data_width_p-1:0]                  mem_resp_data_o
   , output logic                                        mem_resp_v_o
   , input                                               mem_resp_ready_and_i
   , output logic                                        mem_resp_last_o

   // DRAM interface
   , output logic [l2_banks_p-1:0][dma_pkt_width_lp-1:0] dma_pkt_o
   , output logic [l2_banks_p-1:0]                       dma_pkt_v_o
   , input [l2_banks_p-1:0]                              dma_pkt_ready_and_i

   , input [l2_banks_p-1:0][l2_fill_width_p-1:0]         dma_data_i
   , input [l2_banks_p-1:0]                              dma_data_v_i
   , output logic [l2_banks_p-1:0]                       dma_data_ready_and_o

   , output logic [l2_banks_p-1:0][l2_fill_width_p-1:0]  dma_data_o
   , output logic [l2_banks_p-1:0]                       dma_data_v_o
   , input [l2_banks_p-1:0]                              dma_data_ready_and_i
   );

  `declare_bp_cfg_bus_s(vaddr_width_p, hio_width_p, core_id_width_p, cce_id_width_p, lce_id_width_p);
  `declare_bp_bedrock_mem_if(paddr_width_p, did_width_p, lce_id_width_p, lce_assoc_p);
  `declare_bsg_cache_dma_pkt_s(daddr_width_p);

  `declare_bsg_cache_pkt_s(daddr_width_p, l2_data_width_p);

  localparam dma_queue_length_p = 16;
  localparam cache_pkt_width_lp = `bsg_cache_pkt_width(daddr_width_p, l2_data_width_p);
  localparam lg_data_mask_width_lp=`BSG_SAFE_CLOG2(l2_data_width_p>>3);
  localparam lg_block_size_in_words_lp=`BSG_SAFE_CLOG2(l2_block_size_in_words_p);
  localparam block_offset_width_lp=(l2_block_size_in_words_p > 1) ? lg_data_mask_width_lp+lg_block_size_in_words_lp : lg_data_mask_width_lp;
  `declare_bp_prefetch_pkt_s(daddr_width_p - block_offset_width_lp);
  localparam lg_sets_lp = `BSG_SAFE_CLOG2(l2_en_p ? l2_sets_p : 2);
  localparam tag_width_lp = (daddr_width_p-lg_sets_lp-block_offset_width_lp);
  localparam dma_cache_pkt_width_lp = `bsg_cache_dma_pkt_width(daddr_width_p);
  localparam MAX = `bp_cache_slice_fills_per_block(l2_block_size_in_words_p, l2_data_width_p, l2_fill_width_p);
  localparam lg_offsets_p = 6;
localparam prefetch_buffer_depth_p = 8;

  bsg_cache_pkt_s [l2_banks_p-1:0] cache_pkt_li;
  logic [l2_banks_p-1:0] cache_pkt_v_li, cache_pkt_ready_and_lo;
  logic [l2_banks_p-1:0][l2_data_width_p-1:0] cache_data_lo;
  logic [l2_banks_p-1:0] cache_data_v_lo, cache_data_yumi_li, dma_pkt_v_lo, miss_rr_v_li, prefetch_rr_v_li;
  logic [l2_banks_p-1:0][daddr_width_p-block_offset_width_lp-1:0] dma_miss_addr, prefetch_addr_to_gen;

  logic [1:0][`BSG_SAFE_CLOG2(2147483647)-1:0] miss_ctr, cache_ctr;
  logic [`BSG_SAFE_CLOG2(2147483647)-1:0] miss_ctr_n, miss_ctr_r, cache_ctr_n, cache_ctr_r;

  bsg_adder_ripple_carry
    #(.width_p(`BSG_SAFE_CLOG2(2147483647)))
    adr
    (.a_i(miss_ctr[0])
    ,.b_i(miss_ctr[1])
    ,.s_o(miss_ctr_n)
    ,.c_o());
  bsg_adder_ripple_carry
    #(.width_p(`BSG_SAFE_CLOG2(2147483647)))
    adr_c
    (.a_i(cache_ctr[0])
    ,.b_i(cache_ctr[1])
    ,.s_o(cache_ctr_n)
    ,.c_o());
  always_ff @(posedge clk_i) begin
    if (reset_i) begin
      miss_ctr_r <= '0;
      cache_ctr_r <= '0;
    end
    else begin
      miss_ctr_r <= miss_ctr_n;
      cache_ctr_r <= cache_ctr_n;
    end
  end
  // synopsis translate_off
  always_ff @(negedge clk_i) begin
    if (miss_ctr_r != miss_ctr_n) $display("miss_ctr: %d", miss_ctr_n);
    if (cache_ctr_r != cache_ctr_n) $display("cache_ctr: %d", cache_ctr_n);
  end

  // synopsis translate_on

  bp_me_cce_to_cache
   #(.bp_params_p(bp_params_p))
   cce_to_cache
    (.clk_i(clk_i)
     ,.reset_i(reset_i)

     ,.mem_cmd_header_i(mem_cmd_header_i)
     ,.mem_cmd_data_i(mem_cmd_data_i)
     ,.mem_cmd_v_i(mem_cmd_v_i)
     ,.mem_cmd_ready_and_o(mem_cmd_ready_and_o)
     ,.mem_cmd_last_i(mem_cmd_last_i)

     ,.mem_resp_header_o(mem_resp_header_o)
     ,.mem_resp_data_o(mem_resp_data_o)
     ,.mem_resp_v_o(mem_resp_v_o)
     ,.mem_resp_ready_and_i(mem_resp_ready_and_i)
     ,.mem_resp_last_o(mem_resp_last_o)

     ,.cache_pkt_o(cache_pkt_li)
     ,.cache_pkt_v_o(cache_pkt_v_li)
     ,.cache_pkt_ready_and_i(cache_pkt_ready_and_lo)

     ,.cache_data_i(cache_data_lo)
     ,.cache_data_v_i(cache_data_v_lo)
     ,.cache_data_yumi_o(cache_data_yumi_li)
     );

  logic [lg_offsets_p-1:0] offset;
  logic miss_ready_and_lo, miss_v_li, prefetch_v_li;
  logic [daddr_width_p - block_offset_width_lp-1:0] miss_addr_li, prefetch_addr_li;
  logic [l2_banks_p-1:0] miss_rr_yumi_lo, prefetch_rr_yumi_lo;
  // Generates stride distance for prefetch
  bp_me_best_offset_generator
   #(.addr_width_p(daddr_width_p - block_offset_width_lp)
    ,.lg_offsets_p(lg_offsets_p))
   prefetch_offset_generator
    (.clk_i(clk_i)
     ,.reset_i(reset_i)
     ,.miss_addr_i(miss_addr_li)
     ,.miss_v_i(miss_v_li)
     ,.miss_ready_and_o(miss_ready_and_lo)
     ,.prefetch_addr_i(prefetch_addr_li)
     ,.prefetch_v_i(prefetch_v_li)
     ,.offset_o(offset)
     );
  
  bsg_round_robin_n_to_1
   #(.width_p(daddr_width_p - block_offset_width_lp)
    ,.num_in_p(l2_banks_p)
    ,.strict_p(0)
    )
    miss_addr_bank_to_generator
     (.clk_i(clk_i)
     ,.reset_i(reset_i)
     // From banks
     ,.data_i(dma_miss_addr)
     ,.v_i(miss_rr_v_li)
     ,.yumi_o(miss_rr_yumi_lo)
     // To gen
     ,.v_o(miss_v_li)
     ,.data_o(miss_addr_li)
     ,.tag_o()
     ,.yumi_i(miss_ready_and_lo & miss_v_li)
     );

  bsg_round_robin_n_to_1
   #(.width_p(daddr_width_p - block_offset_width_lp)
    ,.num_in_p(l2_banks_p)
    ,.strict_p(0)
    )
    prefetch_addr_bank_to_generator
     (.clk_i(clk_i)
     ,.reset_i(reset_i)
     // From banks
     ,.data_i(prefetch_addr_to_gen)
     ,.v_i(prefetch_rr_v_li)
     ,.yumi_o(prefetch_rr_yumi_lo)
     // To gen
     ,.v_o(prefetch_v_li)
     ,.data_o(prefetch_addr_li)
     ,.tag_o()
     ,.yumi_i(|prefetch_rr_v_li)
     );

  logic [daddr_width_p-1:0] stride;
assign stride = '0;
  // assign stride = {{(daddr_width_p-7){1'b0}},7'b1000000};
// assign stride = {{(daddr_width_p-lg_offsets_p-block_offset_width_lp){1'b0}},{offset},{block_offset_width_lp{1'b0}}};

  bsg_cache_dma_pkt_s [l2_banks_p-1:0] prefetch_pre_stride, prefetch_pkt, prefetch_pkt_r, dma_pkt_lo;

  for (genvar i = 0; i < l2_banks_p; i++)
    begin : bank
    // change widths
      logic [prefetch_buffer_depth_p-1:0] tag_r_match_lo;
      logic [prefetch_buffer_depth_p-1:0] repl_way_lo;
      logic [prefetch_buffer_depth_p-1:0] tag_empty_lo;
      logic [prefetch_buffer_depth_p-1:0][l2_fill_width_p-1:0] mux_one_hot_data_li;
      logic [prefetch_buffer_depth_p-1:0] mux_one_hot_v_li;
      wire  [l2_fill_width_p-1:0]         mux_one_hot_data_lo;
      wire                                mux_one_hot_v_lo;
      logic prefetch_request_v_lo;
      bsg_cache_pkt_s cache_pkt_li_interm;
      bsg_cache_dma_pkt_s cache_pkt_li_interm_dma;
      bsg_cache_dma_pkt_s cache_pkt_li_r;
      bsg_cache_dma_pkt_s dma_pkt_o_spy;
      assign dma_pkt_o_spy = dma_pkt_o[i];

      logic prefetch_request_yumi;
      logic prefetch_request_exists_v;
      logic prefetch_request_unique_n;
      logic prefetch_request_unique_r;
      logic prefetch_request_unique_v;
      logic cache_buffer_hit_v, cache_buffer_hit_n, cache_buffer_hit_r;
      logic [`BSG_SAFE_CLOG2(MAX):0] buffer_hit_count;
      logic dma_fill_finished;
      logic [`BSG_SAFE_CLOG2(MAX):0] fill_counter;
      logic dma_request_buffer_ready_and_lo;
      logic [prefetch_buffer_depth_p - 1: 0] read_in_progress;
      logic [prefetch_buffer_depth_p - 1: 0] safe_write_lo;
      logic [`BSG_SAFE_CLOG2(dma_queue_length_p):0] dma_queue_count_lo;
      bp_prefetch_pkt_s queued_dma_pkt, dma_op_in_progress_n, dma_op_in_progress, dma_op_ip_no_pre;
      logic cache_dma_pkt_miss_v;
      logic dma_data_ready_and_lo;
      logic next_prefetch_request_ready_lo;
      logic page_bound_v;
      logic [prefetch_buffer_depth_p-1:0] read_start;
      logic bypass_dma_op_v;

      bsg_counter_clear_up
        #(.init_val_p(0)
        ,.max_val_p(2147483647))
        cache_total_ctr
        (.clk_i(clk_i)
        ,.reset_i(reset_i)
        ,.clear_i('0)
        ,.up_i(cache_data_yumi_li[i])
        ,.count_o(cache_ctr[i])
        );

      bsg_counter_clear_up
        #(.init_val_p(0)
        ,.max_val_p(2147483647))
        cache_miss_ctr
        (.clk_i(clk_i)
        ,.reset_i(reset_i)
        ,.clear_i('0)
        ,.up_i(dma_pkt_v_o[i] & dma_pkt_v_lo[i] & ~cache_buffer_hit_v & dma_pkt_ready_and_i[i])
        ,.count_o(miss_ctr[i])
        );

      wire op_is_write = dma_pkt_v_lo[i] & dma_pkt_lo[i].write_not_read;

      assign cache_dma_pkt_miss_v = ~cache_buffer_hit_v & dma_pkt_v_lo[i];

      logic cache_buffer_hit_start_v;
      assign cache_buffer_hit_start_v = |tag_r_match_lo & dma_pkt_v_lo[i] & ~dma_pkt_lo[i].write_not_read;
      logic cache_buffer_hit_count_v;

      logic miss_to_gen_v_li;

      bsg_edge_detect
        #(.falling_not_rising_p(0))
        miss_flop
        (.clk_i(clk_i)
        ,.reset_i(reset_i)
        ,.sig_i(dma_pkt_v_lo[i] & ~dma_pkt_lo[i].write_not_read)
        ,.detect_o(miss_to_gen_v_li)
        );

      bsg_one_fifo
        #(.width_p(daddr_width_p-block_offset_width_lp))
        miss_to_gen_buff
        (.clk_i(clk_i)
        ,.reset_i(reset_i)
        ,.ready_o()
        ,.data_i(dma_pkt_lo[i].addr[daddr_width_p-1:block_offset_width_lp])
        ,.v_i(miss_to_gen_v_li)
        ,.v_o(miss_rr_v_li[i])
        ,.data_o(dma_miss_addr[i])
        ,.yumi_i(miss_rr_yumi_lo[i])
        );
      
      bsg_one_fifo
        #(.width_p(daddr_width_p-block_offset_width_lp))
        prefetch_to_gen_buff
        (.clk_i(clk_i)
        ,.reset_i(reset_i)
        ,.ready_o()
        ,.data_i(dma_op_in_progress.addr)
        ,.v_i(dma_op_in_progress_v & dma_op_in_progress.prefetch & dma_data_v_i[i] & fill_counter == 0)
        ,.v_o(prefetch_rr_v_li[i])
        ,.data_o(prefetch_addr_to_gen[i])
        ,.yumi_i(prefetch_rr_yumi_lo[i])
        );

      //TODO: Fix this
      bsg_dff_reset_set_clear
        #(.width_p(1)
         ,.clear_over_set_p(1))
        cache_buffer_hit_tracker
        (.clk_i(clk_i)
        ,.reset_i(reset_i)
        ,.set_i(cache_buffer_hit_start_v)
        ,.clear_i(buffer_hit_count == MAX)
        ,.data_o(cache_buffer_hit_count_v)
        );

      assign cache_buffer_hit_v = cache_buffer_hit_start_v | cache_buffer_hit_count_v;

      assign prefetch_request_exists_v = |tag_r_match_lo & ~dma_pkt_v_lo[i] & prefetch_request_v_lo & ~page_bound_v;

      assign next_prefetch_request_yumi = ~(dma_pkt_v_lo[i] & ~cache_buffer_hit_v & ~bypass_dma_op_v) & dma_pkt_ready_and_i[i] & dma_request_buffer_ready_and_lo;

      assign prefetch_request_yumi = prefetch_request_exists_v | (prefetch_request_unique_n & next_prefetch_request_ready_lo) | page_bound_v;

      assign prefetch_request_unique_n = (~|tag_r_match_lo & ~dma_pkt_v_lo[i] & prefetch_request_v_lo & ~page_bound_v);

      // Keeps track of the data that is being funneled to the cache from buffers
      bsg_counter_clear_up
       #(.max_val_p(MAX)
        ,.init_val_p('0))
       buff_hit_counter
        (.clk_i(clk_i)
        ,.reset_i(reset_i)
        ,.clear_i(buffer_hit_count == MAX)
        ,.up_i(cache_buffer_hit_v & dma_data_ready_and_lo)
        ,.count_o(buffer_hit_count));

      bsg_fifo_1r1w_small
        #(.width_p(dma_cache_pkt_width_lp)
         ,.els_p(1)
        )
       next_prefetch_request
        (.clk_i(clk_i)
        ,.reset_i(reset_i)
        ,.v_i(prefetch_request_unique_n)
        ,.ready_o(next_prefetch_request_ready_lo)
        ,.data_i(prefetch_pkt[i])
        ,.v_o(prefetch_request_unique_v)
        ,.data_o(prefetch_pkt_r[i])
        ,.yumi_i(next_prefetch_request_yumi)
        );

      // Keeps track of the data being streamed in from DMA
      bsg_counter_clear_up
       #(.max_val_p(MAX)
        ,.init_val_p('0))
       fill_tracker
        (.clk_i(clk_i)
        ,.reset_i(reset_i)
        ,.clear_i(dma_fill_finished)
        ,.up_i(dma_data_v_i[i] & dma_data_ready_and_o[i])
        ,.count_o(fill_counter));

      assign dma_fill_finished = fill_counter == MAX;

      bsg_counter_up_down
       #(.max_val_p(dma_queue_length_p)
        ,.init_val_p('0)
        ,.max_step_p(1)
        )
       queue_size_tracker
        (.clk_i(clk_i)
        ,.reset_i(reset_i)
        ,.up_i((dma_pkt_v_o[i] & ~dma_pkt_o_spy.write_not_read & dma_pkt_ready_and_i[i]))
        ,.down_i(dma_fill_finished)
        ,.count_o(dma_queue_count_lo));

      assign queued_dma_pkt.prefetch = ~cache_dma_pkt_miss_v;
      assign queued_dma_pkt.addr = dma_pkt_o_spy.addr[daddr_width_p-1:block_offset_width_lp];

      // synopsis translate_off
      always_ff @(negedge clk_i) begin
        // if (dma_pkt_v_lo[i]) begin
        //   $display("\nAdjusted addr: %b", queued_dma_pkt.addr);
        //   $display("Original addr: %b\n", dma_pkt_o_spy.addr);
        // end
        // if (dma_op_in_progress_v) $display("dma_op_in_progress.addr: %b", dma_op_in_progress.addr);
      end
      // synopsis translate_on

      bsg_fifo_1r1w_small
       #(.width_p($bits(bp_prefetch_pkt_s))
         ,.els_p(dma_queue_length_p)
        )
       dma_request_tracker
        (.clk_i(clk_i)
        ,.reset_i(reset_i)
        ,.v_i(dma_pkt_v_o[i] & ~dma_pkt_o_spy.write_not_read & dma_pkt_ready_and_i[i])
        ,.ready_o(dma_request_buffer_ready_and_lo)
        ,.data_i(queued_dma_pkt)
        ,.v_o(dma_op_in_progress_v)
        ,.data_o(dma_op_in_progress_n)
        ,.yumi_i(dma_fill_finished)
        );

      assign dma_op_ip_no_pre.prefetch = 1'b0;
      assign dma_op_ip_no_pre.addr = dma_op_in_progress.addr;
      logic bypass_dma_op_start, bypass_dma_op_v;
      assign bypass_dma_op_start = fill_counter == 0 & dma_pkt_v_lo[i] & ~dma_pkt_lo[i].write_not_read & dma_pkt_lo[i].addr[daddr_width_p-1:block_offset_width_lp] == dma_op_in_progress_n.addr & dma_op_in_progress_v & ~cache_buffer_hit_v;

      bsg_dff_reset_set_clear
        #(.width_p(1)
         ,.clear_over_set_p(1))
        dma_op_in_progress_bypass
        (.clk_i(clk_i)
        ,.reset_i(reset_i)
        ,.set_i(bypass_dma_op_start)
        ,.clear_i(dma_fill_finished)
        ,.data_o(bypass_dma_op_n)
        );

      assign bypass_dma_op_v = bypass_dma_op_start | bypass_dma_op_n;

      assign dma_op_in_progress = bypass_dma_op_v ? dma_op_ip_no_pre : dma_op_in_progress_n;


      // Cam tag array that keeps track of which fifo specific cache blocks are
      // held in and which fifo to place new prefetched blocks into.
      bsg_cam_1r1w_tag_array
       #(.width_p(daddr_width_p - block_offset_width_lp) // Shrink this when we know how big the relevant portion of the address is
         ,.els_p(prefetch_buffer_depth_p)
         ,.multiple_entries_p(1)
        )
       prefetched_data_tracker
        (.clk_i(clk_i)
         ,.reset_i(reset_i)

         ,.w_v_i((|read_start & dma_data_ready_and_lo) ? (read_start & dma_data_ready_and_lo) : safe_write_lo)
         ,.w_set_not_clear_i(dma_op_in_progress.prefetch & dma_op_in_progress_v & dma_data_v_i[i] & ~op_is_write & fill_counter == 0 & ~(|read_start & dma_data_ready_and_lo))
         ,.w_tag_i(op_is_write ? dma_pkt_lo[i].addr[daddr_width_p-1:block_offset_width_lp] : dma_op_in_progress.addr)
         ,.w_empty_o(tag_empty_lo)

         ,.r_v_i((dma_pkt_v_lo[i] & ~dma_pkt_lo[i].write_not_read) | prefetch_request_v_lo)
         ,.r_tag_i((dma_pkt_v_lo[i] & ~dma_pkt_lo[i].write_not_read) ? dma_pkt_lo[i].addr[daddr_width_p-1:block_offset_width_lp] : prefetch_pkt[i].addr[daddr_width_p-1:block_offset_width_lp])
         ,.r_match_o(tag_r_match_lo) // One hot scheme
         );

      logic cam_replace_w_v_li_n, cam_replace_w_v_li_r, cam_replace_w_v_li;
      assign cam_replace_w_v_li_n = dma_op_in_progress.prefetch & dma_op_in_progress_v & dma_data_v_i[i] & fill_counter == 0;

      bsg_dff_reset
       #(.width_p(1))
       cam_replace_w_v_li_counterpart
        (.clk_i(clk_i)
         ,.reset_i(reset_i)
         ,.data_i(cam_replace_w_v_li)
         ,.data_o(cam_replace_w_v_li_r)
        );

      assign cam_replace_w_v_li = cam_replace_w_v_li_n & ~cam_replace_w_v_li_r;

      // The replacement scheme for the CAM
      bsg_cam_1r1w_replacement
      #(.els_p(prefetch_buffer_depth_p))
       prefetcher_cam_replacement
        (.clk_i(clk_i)
         ,.reset_i(reset_i)
          // Only want to update LRU when its an actual read, but honestly this may do nothing
         ,.read_v_i(tag_r_match_lo & {prefetch_buffer_depth_p{dma_pkt_v_lo[i]}})

         ,.alloc_v_i(cam_replace_w_v_li)
         ,.alloc_empty_i(tag_empty_lo)
         ,.alloc_v_o(repl_way_lo)
         );

      assign safe_write_lo = repl_way_lo & ~read_in_progress;

      for (genvar j = 0; j < prefetch_buffer_depth_p; j++) 
      begin : prefetch_buffer

        logic unused_ready;
        logic [`BSG_SAFE_CLOG2(MAX):0] yumi_count_lo;
        logic write_in_progress;
        logic read_yumi;
        logic write_yumi;
        logic empty_write;
        wire stash_v_lo;
        logic read_in_progress_r;
        logic write_in_progress_r;
        logic write_start;
        logic write_yumi_r, write_yumi_start;
        logic empty_write_start, empty_write_r;
        logic yumi_count_up;

        assign yumi_count_up = (write_yumi | (read_in_progress[j] & dma_data_ready_and_lo & stash_v_lo));

        // Keep track of cache line clear progress for reads and writes
        bsg_counter_clear_up
          #(.max_val_p(MAX)
           ,.init_val_p('0)
           ,.disable_overflow_warning_p(1))
          yumi_counter
           (.clk_i(clk_i)
           ,.reset_i(reset_i)
           ,.clear_i(yumi_count_lo == (MAX - 1) && yumi_count_up)
           ,.up_i(yumi_count_up && yumi_count_lo != (MAX - 1))
           ,.count_o(yumi_count_lo));

        bsg_dff_reset_set_clear
          #(.width_p(1)
           ,.clear_over_set_p(1)
           )
          read_setter
          (.clk_i(clk_i)
          ,.reset_i(reset_i)
          ,.set_i(read_start[j])
          ,.clear_i(yumi_count_lo == MAX - 1 && yumi_count_up)
          ,.data_o(read_in_progress_r)
          );

        assign read_start[j] = dma_pkt_v_lo[i] & ~dma_pkt_lo[i].write_not_read & tag_r_match_lo[j] & yumi_count_lo == 0;

        assign read_in_progress[j] = read_in_progress_r | read_start[j];

        bsg_dff_reset_set_clear
          #(.width_p(1)
           ,.clear_over_set_p(1)
           )
          write_setter
          (.clk_i(clk_i)
          ,.reset_i(reset_i)
          ,.set_i(write_start)
          ,.clear_i(dma_fill_finished)
          ,.data_o(write_in_progress_r)
          );

        assign write_in_progress = write_in_progress_r | write_start;

        assign write_start = safe_write_lo[j] & dma_op_in_progress.prefetch & dma_op_in_progress_v & dma_data_v_i[i] & fill_counter == 0;

        bsg_dff_reset_set_clear
          #(.width_p(1)
           ,.clear_over_set_p(1)
           )
          write_yumi_setter
          (.clk_i(clk_i)
          ,.reset_i(reset_i)
          ,.set_i(write_yumi_start)
          ,.clear_i(yumi_count_lo == (MAX - 1) && yumi_count_up)
          ,.data_o(write_yumi_r)
          );

        bsg_dff_reset_set_clear
          #(.width_p(1)
           ,.clear_over_set_p(1)
           )
          empty_write_setter
          (.clk_i(clk_i)
          ,.reset_i(reset_i)
          ,.set_i(empty_write_start)
          ,.clear_i(dma_fill_finished)
          ,.data_o(empty_write_r)
          );

        assign read_yumi = read_in_progress[j] & dma_data_ready_and_lo & stash_v_lo;

        assign write_yumi = write_yumi_r | write_yumi_start;

        assign empty_write_start = write_start & ~stash_v_lo;

        assign empty_write = empty_write_r | empty_write_start;

        assign write_yumi_start = write_start & ~empty_write;

        assign mux_one_hot_v_li[j] = ~write_yumi & stash_v_lo;

        bsg_fifo_1r1w_small
         #(.width_p(l2_fill_width_p)
          ,.els_p(MAX + 1)
          )
         prefetched_block_buffer
          (.clk_i(clk_i)
           ,.reset_i(reset_i)
           ,.v_i(dma_data_v_i[i] & write_in_progress)
           ,.ready_o(unused_ready)
           ,.data_i(dma_data_i[i])
           ,.v_o(stash_v_lo)
           ,.data_o(mux_one_hot_data_li[j])
           ,.yumi_i(write_yumi | read_yumi)
           );

          // buffer assertion block
          // synopsis translate_off
          always_ff @(negedge clk_i) begin

          end
          // synopsis translate_on
      end
      bsg_mux_one_hot
       #(.width_p(l2_fill_width_p)
         ,.els_p(prefetch_buffer_depth_p)
        )
       prefetch_data_mux
        (.data_i(mux_one_hot_data_li)
         ,.sel_one_hot_i(read_in_progress)
         ,.data_o(mux_one_hot_data_lo)
        );
      
      bsg_mux_one_hot
       #(.width_p(1)
         ,.els_p(prefetch_buffer_depth_p)
        )
       prefetch_valid_mux
        (.data_i(mux_one_hot_v_li)
         ,.sel_one_hot_i(read_in_progress)
         ,.data_o(mux_one_hot_v_lo)
        );

      // Sends prefetch requests to DMA when unit is not busy, drops oldest
      // if overflown.
      bsg_cache_dma_pkt_s cache_pkt_li_dma;
      assign cache_pkt_li_dma.addr = cache_pkt_li[i].addr;

      // Only prefetching for load operations
      bsg_cache_decode_s decoded_pkt;
      bsg_cache_decode
       cache_decoder
        (.opcode_i(cache_pkt_li[i].opcode)
         ,.decode_o(decoded_pkt)
        );

      logic cache_pkt_v_li_interm, cache_pkt_v_li_r, cache_pkt_v_li_interm_cont;
      bsg_dff_reset
       #(.width_p(1))
       first_valid_dff
        (.clk_i(clk_i)
         ,.reset_i(reset_i)
         ,.data_i(cache_pkt_v_li[i] & decoded_pkt.ld_op)
         ,.data_o(cache_pkt_v_li_interm_cont)
        );

      bsg_edge_detect
        #(.falling_not_rising_p(0))
        cache_pkt_valid_twitch
        (.clk_i(clk_i)
        ,.reset_i(reset_i)
        ,.sig_i(cache_pkt_v_li_interm_cont)
        ,.detect_o(cache_pkt_v_li_interm)
        );

      bsg_dff_reset
        #(.width_p(1))
        second_valid_dff
        (.clk_i(clk_i)
        ,.reset_i(reset_i)
        ,.data_i(cache_pkt_v_li_interm)
        ,.data_o(cache_pkt_v_li_r)
        );

      bsg_dff_reset
       #(.width_p(cache_pkt_width_lp))
       first_pkt_stall
        (.clk_i(clk_i)
        ,.reset_i(reset_i)
        ,.data_i(cache_pkt_li[i])
        ,.data_o(cache_pkt_li_interm)
        );

      // Translate cache packet to dma packet
      logic [lg_sets_lp-1:0] addr_index;
      logic [tag_width_lp-1:0] addr_tag;
      assign addr_index = cache_pkt_li_interm.addr[block_offset_width_lp+:lg_sets_lp];
      assign addr_tag = cache_pkt_li_interm.addr[block_offset_width_lp+lg_sets_lp+:tag_width_lp];
      assign cache_pkt_li_interm_dma.addr = {addr_tag,
                                             addr_index,
                                             {(block_offset_width_lp){1'b0}}
                                            };
      assign cache_pkt_li_interm_dma.write_not_read = 1'b0;

      bsg_dff_reset
        #(.width_p(dma_cache_pkt_width_lp))
        second_pkt_stall
         (.clk_i(clk_i)
          ,.reset_i(reset_i)
          ,.data_i(cache_pkt_li_interm_dma)
          ,.data_o(cache_pkt_li_r)
         );

      bsg_fifo_1r1w_small
       #(.width_p(dma_pkt_width_lp)
         ,.els_p(4)
        )
       prefetch_request_generator
        (.clk_i(clk_i)
         ,.reset_i(reset_i)
         ,.v_i(cache_pkt_v_li_r)
         ,.ready_o()
         ,.data_i(cache_pkt_li_r)
         ,.v_o(prefetch_request_v_lo)
         ,.data_o(prefetch_pre_stride[i])
         ,.yumi_i(prefetch_request_yumi & prefetch_request_v_lo)
         );

      assign prefetch_pkt[i].write_not_read = 1'b0;
      bsg_adder_ripple_carry
       #(.width_p(daddr_width_p))
       stride_adder
        (.a_i(prefetch_pre_stride[i].addr)
         ,.b_i(stride)
         ,.s_o(prefetch_pkt[i].addr)
         ,.c_o()
        );

      // High when we need to ignore data as it may cause a page fault or when prefetching is disabled
      assign page_bound_v = (({prefetch_pkt[i].addr[daddr_width_p-1:12]} !=
                            {prefetch_pre_stride[i].addr[daddr_width_p-1:12]} & prefetch_request_v_lo)
                            | stride == '0);

      bsg_cache
       #(.addr_width_p(daddr_width_p)
         ,.data_width_p(l2_data_width_p)
         ,.dma_data_width_p(l2_fill_width_p)
         ,.block_size_in_words_p(l2_block_size_in_words_p)
         ,.sets_p(l2_en_p ? l2_sets_p : 2)
         ,.ways_p(l2_en_p ? l2_assoc_p : 2)
         ,.amo_support_p(((l2_amo_support_p[e_amo_swap]) << e_cache_amo_swap)
                         | ((l2_amo_support_p[e_amo_fetch_logic]) << e_cache_amo_xor)
                         | ((l2_amo_support_p[e_amo_fetch_logic]) << e_cache_amo_and)
                         | ((l2_amo_support_p[e_amo_fetch_logic]) << e_cache_amo_or)
                         | ((l2_amo_support_p[e_amo_fetch_arithmetic]) << e_cache_amo_add)
                         | ((l2_amo_support_p[e_amo_fetch_arithmetic]) << e_cache_amo_min)
                         | ((l2_amo_support_p[e_amo_fetch_arithmetic]) << e_cache_amo_max)
                         | ((l2_amo_support_p[e_amo_fetch_arithmetic]) << e_cache_amo_minu)
                         | ((l2_amo_support_p[e_amo_fetch_arithmetic]) << e_cache_amo_maxu)
                         )
        )
       cache
        (.clk_i(clk_i)
         ,.reset_i(reset_i)

         ,.cache_pkt_i(cache_pkt_li[i])
         ,.v_i(cache_pkt_v_li[i])
         ,.ready_o(cache_pkt_ready_and_lo[i])

         ,.data_o(cache_data_lo[i])
         ,.v_o(cache_data_v_lo[i])
         ,.yumi_i(cache_data_yumi_li[i])

         ,.dma_pkt_o(dma_pkt_lo[i])
         ,.dma_pkt_v_o(dma_pkt_v_lo[i])
         ,.dma_pkt_yumi_i(((dma_pkt_ready_and_i[i] & dma_pkt_v_lo[i]) & dma_request_buffer_ready_and_lo) | cache_buffer_hit_v | bypass_dma_op_v)

         ,.dma_data_i(cache_buffer_hit_v ? mux_one_hot_data_lo : dma_data_i[i])
         ,.dma_data_v_i((~dma_op_in_progress.prefetch & dma_op_in_progress_v & dma_data_v_i[i]) | (cache_buffer_hit_v & mux_one_hot_v_lo))
         ,.dma_data_ready_o(dma_data_ready_and_lo)

         ,.dma_data_o(dma_data_o[i])
         ,.dma_data_v_o(dma_data_v_o[i])
         ,.dma_data_yumi_i(dma_data_ready_and_i[i] & dma_data_v_o[i])

         ,.v_we_o()
         );

      assign dma_pkt_o[i] = op_is_write ? dma_pkt_lo[i] : (cache_dma_pkt_miss_v ? dma_pkt_lo[i] 
                                                  : prefetch_pkt_r[i]);

      assign dma_data_ready_and_o[i] = (dma_op_in_progress.prefetch) ? |{safe_write_lo} : dma_data_ready_and_lo;

      // dma packet is valid if either the cache output missed in our CAM or if we have a valid prefetch queued,
      // OR if the cache is trying to output a write packet.
      assign dma_pkt_v_o[i] = (((cache_dma_pkt_miss_v & ~bypass_dma_op_v) | prefetch_request_unique_v) & dma_request_buffer_ready_and_lo)
                              | (dma_pkt_v_lo[i] & dma_pkt_lo[i].write_not_read);

    // synopsis translate_off
    always_ff @(negedge clk_i) begin
      // ("dma_data_ready_and_o[i], %b", dma_data_ready_and_o[i]);
      // if (dma_pkt_v_lo[i]) $display("REQUESTING FROM DMA, MISS ADDR: %x", dma_pkt_lo[i].addr);
      // if(cache_buffer_hit_start_v) $display("We prefetched usefully!");
      // $display("dma_pkt_v_o[i]: %b", dma_pkt_v_o[i]);
      assert (~(dma_queue_count_lo == 0 ~^ dma_op_in_progress_v) || reset_i) else
        $error("DMA operation miscount! Tracked number: %d, In progress: %b", dma_queue_count_lo, dma_op_in_progress_v);
    end
    // synopsis translate_on

    end

endmodule

module bp_me_best_offset_generator
  #(parameter addr_width_p = 64
    ,parameter lg_offsets_p = 6
    ,parameter history_len_p = 8
    ,parameter max_score_p = 10
    ,parameter min_score_p = 1
    ,parameter max_rounds_p = 20
    ,parameter num_outstanding_misses_p = 2
    ,parameter init_offset_p = 1
  )
  (input                          clk_i
  , input                         reset_i
  , input  [addr_width_p-1:0]     miss_addr_i
  , input                         miss_v_i // Valid data is an address that missed in the cache
  , input  [addr_width_p-1:0]     prefetch_addr_i
  , input                         prefetch_v_i
  , output                        miss_ready_and_o // Ready to recieve miss data
  , output [lg_offsets_p-1:0]     offset_o
  );

  logic [lg_offsets_p-1:0] test_offset;
  logic [lg_offsets_p-1:0] best_offset;
  logic [2**lg_offsets_p-1:0][`BSG_SAFE_CLOG2(max_score_p)-1:0] offset_scores;
  logic [addr_width_p-1:0] miss_addr_lo, test_addr;
  logic processing_miss_v_lo, fifo_yumi_li;
  logic [`BSG_SAFE_CLOG2(max_rounds_p)-1:0] round_counter;
  logic [lg_offsets_p-1:0] current_best_offset;
  logic [`BSG_SAFE_CLOG2(max_score_p)-1:0] current_best_offset_score;
  logic [history_len_p-1:0] tag_empty_lo, tag_r_match_lo, repl_way_lo;

  bsg_fifo_1r1w_small
    #(.width_p(addr_width_p)
    ,.els_p(num_outstanding_misses_p)
    )
    prefetched_block_buffer
    (.clk_i(clk_i)
    ,.reset_i(reset_i)
    ,.v_i(miss_v_i)
    ,.ready_o(miss_ready_and_o)
    ,.data_i(miss_addr_i)
    ,.v_o(processing_miss_v_lo)
    ,.data_o(miss_addr_lo)
    ,.yumi_i(fifo_yumi_li)
    );

  bsg_cam_1r1w_tag_array
    #(.width_p(addr_width_p)
     ,.els_p(history_len_p)
     ,.multiple_entries_p(1)
     )
    prefetched_data_tracker
    (.clk_i(clk_i)
    ,.reset_i(reset_i)

    ,.w_v_i(repl_way_lo)
    ,.w_set_not_clear_i(prefetch_v_i)
    ,.w_tag_i(prefetch_addr_i)
    ,.w_empty_o(tag_empty_lo)

    ,.r_v_i(processing_miss_v_lo)
    ,.r_tag_i(test_addr)
    ,.r_match_o(tag_r_match_lo) // One hot scheme
    );

  bsg_cam_1r1w_replacement
  #(.els_p(history_len_p))
   prefetcher_cam_replacement
    (.clk_i(clk_i)
    ,.reset_i(reset_i)
    // Only want to update LRU when its an actual read, but honestly this may do nothing
    ,.read_v_i(tag_r_match_lo)

    ,.alloc_v_i(prefetch_v_i)
    ,.alloc_empty_i(tag_empty_lo)
    ,.alloc_v_o(repl_way_lo)
    );

  bsg_adder_ripple_carry
    #(.width_p(addr_width_p))
    stride_adder
    (.a_i(miss_addr_lo)
    ,.b_i({{(addr_width_p-lg_offsets_p){'0}},{test_offset}})
    ,.s_o(test_addr)
    ,.c_o()
    );

  always_ff @(posedge clk_i) begin
    if (reset_i) begin
      test_offset <= 1;
      best_offset <= 1;
      offset_scores <= '0;
      fifo_yumi_li <= '0;
      current_best_offset <= '0;
      current_best_offset_score <= '0;
      round_counter <= '0;
    end else if (test_offset == {(lg_offsets_p){1'b1}} & processing_miss_v_lo) begin
      // We are on the last check of this round
      test_offset = 1;
      if (round_counter == max_rounds_p - 1) begin
        best_offset <= current_best_offset_score > min_score_p ? current_best_offset : init_offset_p;
        $display("current stride: %d", current_best_offset_score > min_score_p ? current_best_offset : init_offset_p);
        round_counter <= '0;
        current_best_offset <= '0;
        current_best_offset_score <= '0;
      end else begin
        round_counter <= round_counter + 1;
      end
    end else if (processing_miss_v_lo) begin
      // We are in the middle of processing a miss, increment to next test value
      test_offset <= test_offset + 1;
      if (|tag_r_match_lo) begin
        // We've found a hit in the previously prefetched data for this test address!
        offset_scores[test_offset] <= offset_scores[test_offset] + 1;
        if (offset_scores[test_offset] == max_score_p - 1) begin
          // Weve hit the maximum value, reset and start the next round
          offset_scores <= '0;
          fifo_yumi_li <= 1'b1;
          current_best_offset <= '0;
          test_offset <= 1;
          best_offset <= test_offset;
          round_counter <= '0;
          current_best_offset_score <= '0;
          $display("current stride: %d", test_offset);
        end else if (current_best_offset_score < (offset_scores[test_offset] + 1)) begin
          // Update intermediate values
          current_best_offset_score <= offset_scores[test_offset] + 1;
          current_best_offset <= test_offset;
        end
      end
    end else begin
      // We wait idly while there are no misses to process
      test_offset <= 1;
    end
    if (test_offset == {(lg_offsets_p){1'b1}}) fifo_yumi_li <= 1'b1;
    else fifo_yumi_li <= 1'b0;
  end

  assign offset_o = best_offset;
endmodule