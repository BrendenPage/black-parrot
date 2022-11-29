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

  localparam dma_queue_length_p = 4;
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
  localparam prefetch_buffer_depth_p = 32;

  bsg_cache_pkt_s [l2_banks_p-1:0] cache_pkt_li;
  logic [l2_banks_p-1:0] cache_pkt_v_li, cache_pkt_ready_and_lo;
  logic [l2_banks_p-1:0][l2_data_width_p-1:0] cache_data_lo;
  logic [l2_banks_p-1:0] cache_data_v_lo, cache_data_yumi_li, dma_pkt_v_lo;;

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

  logic [daddr_width_p-1:0] best_offset_v_lo;
  logic [lg_offsets_p-1:0] offset;
  // Generates stride distance for prefetch
  // bp_me_best_offset_generator
  //  #(.daddr_width_p(daddr_width_p)
  //    ,.lg_offsets_p(lg_offsets_p))
  //  prefetch_offset_generator
  //   (.clk_i(clk_i)
  //    ,.reset_i(reset_i)
  //    ,.daddr_i()
  //    ,.v_i()
  //    ,.yumi_o()
  //    ,.ready_and_o()
  //    ,.offset_o(offset)
  //    ,.v_o(best_offset_v_lo)
  //    );

  // // Stores miss addresses waiting to be processed by best offset generator
  // bsg_fifo_1r1w_large
  //  #(.width_p(daddr_width_p)
  //   ,.els_p(20)
  //   )
  //  prefetch_dma_buffer
  //   (.clk_i(clk_i)
  //    ,.reset_i(reset_i)
  //    ,.data_i()
  //    ,.v_i(best_offset_v_o)
  //    ,.ready_o()
  //    ,.v_o()
  //    ,.data_o()
  //    ,.yumi_i()
  //    );
  logic [daddr_width_p-1:0] stride;
  assign stride = '0;

  bsg_cache_dma_pkt_s [l2_banks_p-1:0] prefetch_pre_stride, prefetch_pkt, prefetch_pkt_r, dma_pkt_lo;

  for (genvar i = 0; i < l2_banks_p; i++)
    begin : bank
    // change widths
      logic [l2_banks_p-1:0] prefetch_v;
      logic [prefetch_buffer_depth_p-1:0] tag_r_match_lo;
      logic [prefetch_buffer_depth_p-1:0] repl_way_lo;
      logic [prefetch_buffer_depth_p-1:0] tag_empty_lo;
      logic [prefetch_buffer_depth_p-1:0][l2_fill_width_p-1:0] mux_one_hot_data_li;
      logic [prefetch_buffer_depth_p-1:0] mux_one_hot_v_li;
      wire  [l2_fill_width_p-1:0]         mux_one_hot_data_lo;
      wire                                mux_one_hot_v_lo;
      logic prefetch_request_v_lo;
      wire prefetch_req_gen_ready_lo;
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
      logic cache_buffer_hit_v;
      logic buffer_hit_count;
      logic dma_fill_finished;
      logic [`BSG_SAFE_CLOG2(MAX):0] fill_counter;
      logic dma_request_buffer_ready_and_lo;
      logic [prefetch_buffer_depth_p - 1: 0] read_in_progress;
      logic [prefetch_buffer_depth_p - 1: 0] safe_write_lo;
      logic [`BSG_SAFE_CLOG2(dma_queue_length_p):0] dma_queue_count_lo;
      bp_prefetch_pkt_s queued_dma_pkt, dma_op_in_progress;
      logic cache_dma_pkt_miss_v;
      logic dma_data_ready_and_lo;
      logic page_bound_v;

      wire op_is_write = dma_pkt_v_lo[i] & dma_pkt_lo[i].write_not_read;

      assign cache_dma_pkt_miss_v = ~cache_buffer_hit_v & dma_pkt_v_lo[i];

      assign cache_buffer_hit_v = |{tag_r_match_lo} & dma_pkt_v_lo[i] & ~dma_pkt_v_lo[i] & ~dma_pkt_lo[i].write_not_read
                                  | (cache_buffer_hit_v & buffer_hit_count != MAX);

      assign prefetch_request_exists_v = |{tag_r_match_lo} & ~dma_pkt_v_lo[i] & prefetch_request_v_lo & ~page_bound_v;

      assign prefetch_request_yumi = prefetch_request_exists_v | (prefetch_request_unique_v & ~(dma_pkt_v_lo[i] & ~cache_buffer_hit_v)
                                     & dma_pkt_ready_and_i[i] & dma_request_buffer_ready_and_lo) | ~page_bound_v;

      assign prefetch_request_unique_n = prefetch_request_unique_v | 
                                         (~|{tag_r_match_lo} & ~dma_pkt_v_lo[i] & prefetch_request_v_lo & ~page_bound_v);

      assign prefetch_request_unique_v = prefetch_request_unique_r & (prefetch_pkt_r[i].addr == prefetch_pkt[i].addr);

      bsg_dff_reset_set_clear
        #(.width_p(1)
         ,.clear_over_set_p(1))
        prefetch_unique_dff
        (.clk_i(clk_i)
        ,.reset_i(reset_i)
        ,.set_i(prefetch_request_unique_v
                | (~|{tag_r_match_lo} & ~dma_pkt_v_lo[i] & prefetch_request_v_lo & ~page_bound_v))
        ,.clear_i(prefetch_pkt_r[i].addr != prefetch_pkt[i].addr)
        ,.data_o(prefetch_request_unique_r)
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

      bsg_counter_clear_up
       #(.max_val_p(dma_queue_length_p)
        ,.init_val_p('0))
       queue_size_tracker
        (.clk_i(clk_i)
        ,.reset_i(reset_i)
        ,.clear_i(dma_fill_finished)
        ,.up_i(dma_pkt_v_o[i] & ~dma_pkt_o_spy.write_not_read)
        ,.count_o(dma_queue_count_lo));

      assign queued_dma_pkt.prefetch = cache_dma_pkt_miss_v;
      assign queued_dma_pkt.addr = dma_pkt_o_spy.addr[daddr_width_p-1:block_offset_width_lp];

      bsg_fifo_1r1w_small
       #(.width_p($bits(bp_prefetch_pkt_s))
         ,.els_p(4)
        )
       dma_request_tracker
        (.clk_i(clk_i)
         ,.reset_i(reset_i)
         ,.v_i(dma_pkt_v_o[i] & ~dma_pkt_o_spy.write_not_read)
         ,.ready_o(dma_request_buffer_ready_and_lo)
         ,.data_i(queued_dma_pkt)
         ,.v_o(dma_op_in_progress_v)
         ,.data_o(dma_op_in_progress)
         ,.yumi_i(dma_fill_finished)
         );
      

      // Cam tag array that keeps track of which fifo specific cache blocks are
      // held in and which fifo to place new prefetched blocks into.
      bsg_cam_1r1w_tag_array
       #(.width_p(daddr_width_p) // Shrink this when we know how big the relevant portion of the address is
         ,.els_p(prefetch_buffer_depth_p)
        )
       prefetched_data_tracker
        (.clk_i(clk_i)
         ,.reset_i(reset_i)

         ,.w_v_i(safe_write_lo)
         ,.w_set_not_clear_i(dma_op_in_progress.prefetch & dma_op_in_progress_v & dma_data_v_i[i] & ~op_is_write)
         ,.w_tag_i(op_is_write ? dma_pkt_lo[i].addr : dma_op_in_progress.addr)
         ,.w_empty_o(tag_empty_lo)

         ,.r_v_i((dma_pkt_v_lo[i] & ~dma_pkt_lo[i].write_not_read) | prefetch_request_v_lo)
         ,.r_tag_i((dma_pkt_v_lo[i] & ~dma_pkt_lo[i].write_not_read) ? dma_pkt_lo[i].addr : prefetch_pkt[i].addr)
         ,.r_match_o(tag_r_match_lo) // One hot scheme
         );

      // The replacement scheme for the CAM
      bsg_cam_1r1w_replacement
      #(.els_p(prefetch_buffer_depth_p))
       prefetcher_cam_replacement
        (.clk_i(clk_i)
         ,.reset_i(reset_i)

         ,.read_v_i(tag_r_match_lo)

         ,.alloc_v_i(dma_op_in_progress.prefetch & dma_op_in_progress_v & dma_data_v_i[i])
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

        // Keep track of cache line clear progress for reads and writes
        bsg_counter_clear_up
          #(.max_val_p(MAX)
           ,.init_val_p('0))
          yumi_counter
           (.clk_i(clk_i)
           ,.reset_i(reset_i)
           ,.clear_i(yumi_count_lo == MAX)
           ,.up_i(write_yumi | (read_in_progress[j] & dma_data_ready_and_lo))
           ,.count_o(yumi_count_lo));

        assign read_in_progress[j] = (dma_pkt_v_lo[i] & ~dma_pkt_lo[i].write_not_read & tag_r_match_lo[j]) | (yumi_count_lo != MAX & read_in_progress[j]);
        
        assign read_yumi = read_in_progress[j] & dma_data_ready_and_lo;

        assign write_yumi = dma_op_in_progress.prefetch & dma_op_in_progress_v & dma_data_v_i[i] & safe_write_lo[j] & ~empty_write | (write_yumi & yumi_count_lo != MAX);

        assign write_in_progress = safe_write_lo[j] & dma_op_in_progress.prefetch & dma_op_in_progress_v & dma_data_v_i[i] | (~dma_fill_finished & write_in_progress);

        assign empty_write = dma_op_in_progress.prefetch & dma_op_in_progress_v & dma_data_v_i[i] & safe_write_lo[j] & stash_v_lo | (empty_write & yumi_count_lo != MAX);

        assign mux_one_hot_v_li[j] = ~write_yumi & stash_v_lo;

        bsg_fifo_1r1w_small
         #(.width_p(l2_fill_width_p)
           ,.els_p( + 1) // make this nicer, needs to be # fill width per block + 1
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
      end
      bsg_mux_one_hot
       #(.width_p(l2_fill_width_p)
         ,.els_p(prefetch_buffer_depth_p)
        )
       prefetch_data_mux
        (.data_i(mux_one_hot_data_li)
         ,.sel_one_hot_i(tag_r_match_lo)
         ,.data_o(mux_one_hot_data_lo)
        );
      
      bsg_mux_one_hot
       #(.width_p(1)
         ,.els_p(prefetch_buffer_depth_p)
        )
       prefetch_valid_mux
        (.data_i(mux_one_hot_v_li)
         ,.sel_one_hot_i(tag_r_match_lo)
         ,.data_o(mux_one_hot_v_lo)
        );

      // Stores misses reported by the bsg_cache to be sent to best offset gen
      // bsg_fifo_1r1w_small
      //  #(.width_p(daddr_width_p)
      //    ,.els_p(4)
      //   )
      //  incoming_address_buffer
      //   (.clk_i(clk_i)
      //    ,.reset_i(reset_i)
      //    ,.v_i(dma_pkt_o[i].addr)
      //    ,.ready_o()
      //    ,.data_i(dma_pkt_v_o[i])
      //    ,.v_o()
      //    ,.data_o()
      //    ,.yumi_i()
      //    );
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

      logic cache_pkt_v_li_interm, cache_pkt_v_li_r;
      bsg_dff_reset
       #(.width_p(1))
       first_valid_dff
        (.clk_i(clk_i)
         ,.reset_i(reset_i)
         ,.data_i(cache_pkt_v_li[i] & decoded_pkt.ld_op)
         ,.data_o(cache_pkt_v_li_interm)
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
      wire addr_index = cache_pkt_li_interm.addr[block_offset_width_lp+:lg_sets_lp];
      wire addr_tag = cache_pkt_li_interm.addr[block_offset_width_lp+lg_sets_lp+:tag_width_lp];
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
         ,.yumi_i(prefetch_request_yumi)
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

      bsg_dff_reset
        #(.width_p(dma_cache_pkt_width_lp))
        prefetch_pkt_flop
         (.clk_i(clk_i)
          ,.reset_i(reset_i)
          ,.data_i(prefetch_pkt[i])
          ,.data_o(prefetch_pkt_r[i])
         );

      assign page_bound_v = ({prefetch_pkt[i].addr[daddr_width_p-1:12]} ==
                            {prefetch_pre_stride[i].addr[daddr_width_p-1:12]})
                            | stride == '0;

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
         ,.dma_pkt_yumi_i(((dma_pkt_ready_and_i[i] & dma_pkt_v_lo[i]) & dma_request_buffer_ready_and_lo) | cache_buffer_hit_v)

         ,.dma_data_i(cache_buffer_hit_v ? mux_one_hot_data_lo : dma_data_i[i])
         ,.dma_data_v_i((~dma_op_in_progress.prefetch & dma_op_in_progress_v & dma_data_v_i[i]) | (cache_buffer_hit_v & mux_one_hot_v_lo))
         ,.dma_data_ready_o(dma_data_ready_and_lo)

         ,.dma_data_o(dma_data_o[i])
         ,.dma_data_v_o(dma_data_v_o[i])
         ,.dma_data_yumi_i(dma_data_ready_and_i[i] & dma_data_v_o[i])

         ,.v_we_o()
         );
        assign dma_pkt_o[i] = cache_dma_pkt_miss_v ? dma_pkt_lo[i] 
                                                   : prefetch_pkt_r[i];

        assign dma_data_ready_and_o[i] = dma_op_in_progress.prefetch ? |{safe_write_lo} : dma_data_ready_and_lo;

        assign dma_pkt_v_o[i] = (cache_dma_pkt_miss_v | prefetch_request_unique_v) & dma_request_buffer_ready_and_lo;
    end
  // synopsis translate_off
    // always_ff @(negedge clk_i) begin
    //   assert(&{prefetch_buffer.unused_ready} || reset_i)
    //   else $error("Prefetch buffer overflown, should be cleared each access");

    //   assert(~|{banks.mux_one_hot_v_li & banks.prefetch_buffer.write_in_progress} || reset_i)
    //   else $error("Cache recieved line flush data");
    // end
  // synopsis translate_on

endmodule

