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

  `declare_bsg_cache_pkt_s(daddr_width_p, l2_data_width_p);
  bsg_cache_pkt_s [l2_banks_p-1:0] cache_pkt_li;
  parameter lg_offsets_p = 6;
  parameter prefetch_buffer_depth_p = 32;
  logic [l2_banks_p-1:0] cache_pkt_v_li, cache_pkt_ready_and_lo;
  logic [l2_banks_p-1:0][l2_data_width_p-1:0] cache_data_lo;
  logic [l2_banks_p-1:0] cache_data_v_lo, cache_data_yumi_li;

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
  bp_me_best_offset_generator
   #(.daddr_width_p(daddr_width_p)
     ,.lg_offsets_p(lg_offsets_p))
   prefetch_offset_generator
    (.clk_i(clk_i)
     ,.reset_i(reset_i)
     ,.daddr_i()
     ,.v_i()
     ,.yumi_o()
     ,.ready_and_o()
     ,.offset_o(offset)
     ,.v_o(best_offset_v_lo)
     );

  // Stores miss addresses waiting to be processed by best offset generator
  bsg_fifo_1r1w_large
   #(.width_p(daddr_width_p)
    ,.els_p(20)
    )
   prefetch_dma_buffer
    (.clk_i(clk_i)
     ,.reset_i(reset_i)
     ,.data_i()
     ,.v_i(best_offset_v_o)
     ,.ready_o()
     ,.v_o()
     ,.data_o()
     ,.yumi_i()
     );

  for (genvar i = 0; i < l2_banks_p; i++)
    begin : bank
    // change widths
      logic [safe_els_lp-1:0][prefetch_buffer_depth_p-1:0] tag_r_match_lo;
      logic [safe_els_lp-1:0][prefetch_buffer_depth_p-1:0] repl_way_lo;
      logic [safe_els_lp-1:0][prefetch_buffer_depth_p-1:0] tag_empty_lo;
      // Cam tag array that keeps track of which fifo specific cache blocks are
      // held in and which fifo to place new prefetched blocks into.
      bsg_cam_1r1w_tag_array
       #(.width_p(daddr_width_p) // Shrink this when we know how big the relevant portion of the address is
         ,.els_p(prefetch_buffer_depth_p)
        )
       prefetched_data_tracker
        (.clk_i(clk_i)
         ,.reset_i(reset_i)

         ,.w_v_i(repl_way_lo[i])
         ,.w_set_not_clear_i(prefetch_v[i] & dma_data_v_i[i])
         ,.w_tag_i(prefetch_addr[i])
         ,.w_empty_o(tag_empty_lo[i])

         ,.r_v_i(r_v_i)
         ,.r_tag_i(r_tag_i)
         ,.r_match_o(tag_r_match_lo[i]) // One hot scheme
         );

      // The replacement scheme for the CAM
      bsg_cam_1r1w_replacement
      #(.els_p(prefetch_buffer_depth_p))
       prefetcher_cam_replacement
        (.clk_i(clk_i)
         ,.reset_i(reset_i)

         ,.read_v_i(tag_r_match_lo[i])

         ,.alloc_v_i(prefetch_v[i] & dma_data_v_i[i])
         ,.alloc_empty_i(tag_empty_lo[i])
         ,.alloc_v_o(repl_way_lo[i])
         );

      logic [][] mux_one_hot_data;
      for (genvar j = 0; j < prefetch_buffer_depth_p; j++) 
      begin : prefetch_buffer
        logic [][prefetch_buffer_depth_p-1:0] block_buffer_v_r;
        wire block_write_valid = repl_way_lo[i][j] & prefetch_v[i] & dma_data_v_i[i];
        wire unused_ready;
        bsg_fifo_1r1w_small
         #(.width_p(l2_fill_width_p)
           ,.els_p((l2_block_size_in_words_p * l2_data_width_p)*(l2_data_width_p/l2_fill_width_p) + 1) // make this nicer, needs to be # fill width per block + 1
          )
         prefetched_block_buffer
          (.clk_i(clk_i)
           ,.reset_i(reset_i)
           ,.v_i(block_write_valid)
           ,.ready_o(unused_ready)
           ,.data_i(dma_pkt_v_o[i])
           ,.v_o(block_buffer_v_n[i][j])
           ,.data_o(mux_one_hot_data[i][j])
           ,.yumi_i((block_write_valid & block_buffer_v_r[i][j]) |
                    (tag_r_match_lo[i][j] & dma_pkt_v_lo[i]))
           )

        bsg_dff_reset_en
         #(.width_p(1))
         block_write_valid_dff
          (.clk_i(clk_i)
           ,.reset_i(reset_i)
           ,.en_i(~block_write_valid)
           ,.data_i(block_buffer_v_n[i][j])
           ,.data_o(block_buffer_v_r[i][j])
          )
      end
      wire [][] mux_one_hot_data_lo;
      bsg_mux_one_hot
       #(.width_p(l2_fill_width_p)
         ,.els_p(prefetch_buffer_depth_p)
        )
       prefetch_data_mux
        (.data_i(mux_one_hot_data[i])
         ,.sel_one_hot_i(tag_r_match_lo[i])
         ,.data_o(mux_one_hot_data_lo[i])
        )

      // Stores misses reported by the bsg_cache to be sent to best offset gen
      bsg_fifo_1r1w_small
       #(.width_p(daddr_width_p)
         ,.els_p(4)
        )
       incoming_address_buffer
        (.clk_i(clk_i)
         ,.reset_i(reset_i)
         ,.v_i(dma_pkt_o[i].addr)
         ,.ready_o()
         ,.data_i(dma_pkt_v_o[i])
         ,.v_o()
         ,.data_o()
         ,.yumi_i()
         )

      // Sends prefetch requests to DMA when unit is not busy, drops oldest
      // if overflown.
      bsg_fifo_1r1w_small
       #(.width_p(daddr_width_p)
         ,.els_p(8)
        )
       prefetch_request_generator
        (.clk_i(clk_i)
         ,.reset_i(reset_i)
         ,.v_i()
         ,.ready_o()
         ,.data_i(cache_pkt_li[i].addr)
         ,.v_o(prefetch_request_v[i])
         ,.data_o()
         ,.yumi_i()
         )

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

         ,.dma_pkt_o(dma_pkt_o[i])
         ,.dma_pkt_v_o(dma_pkt_v_lo[i])
         ,.dma_pkt_yumi_i(dma_pkt_ready_and_i[i] & dma_pkt_v_o[i])

         ,.dma_data_i(dma_data_i[i])
         ,.dma_data_v_i(dma_data_v_i[i])
         ,.dma_data_ready_o(dma_data_ready_and_o[i]) // The cache's internal fifo is ready to accept data

         ,.dma_data_o(dma_data_o[i])
         ,.dma_data_v_o(dma_data_v_o[i])
         ,.dma_data_yumi_i(dma_data_ready_and_i[i] & dma_data_v_o[i])

         ,.v_we_o()
         );
        assign dma_pkt_o = prefetch_v[i] ? prefetch_pkt[i]
                                           : dma_pkt_lo[i];

        assign dma_pkt_v_o[i] = (~(|{tag_r_match_lo[i]}) & dma_pkt_v_lo[i]) | 
                                (prefetch_request_v[i] & ~page_bound_v[i]);
    end
  // synopsis translate_off
    always_ff (@negedge clk_i) begin
      assert(&{prefetch_buffer.unused_ready} || reset_i)
      else $error("Prefetch buffer overflown, should be cleared each access");
    end
  // synopsis translate_on

endmodule

