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

  bsg_fifo_1r1w_large
   #(.width_p(daddr_width_p)
    ,.els_p(20)
    )
   prefetch_dma_buffer
    (.clk_i(clk_i)
     ,.reset_i(reset_i)
     ,.data_i(prefetch_addr)
     ,.v_i(best_offset_v_o)
     ,.ready_o()
     ,.v_o()
     ,.data_o()
     ,.yumi_i()
    );

  for (genvar i = 0; i < l2_banks_p; i++)
    begin : bank
      // Buffer that holds prefetched values and checks to see if a request for a held
      // value is send to the main cache bank. If the request is sent it waits for bsg_cache
      // to miss and then handles the DMA access as if it is returning from DMA to give the cache
      // the memory requested.
      bsg_cam_1r1w
       #(.els_p(prefetch_buffer_depth_p)
         ,.tag_width_p(daddr_width_p - $log2(prefetch_buffer_depth_p))
         ,.data_width_p(l2_data_width_p)
        )
       prefetch_buffer
        (.clk_i(clk_i)
         ,.reset_i(reset_i)
         ,.w_v_i()
         ,.w_nuke_i()
         ,.w_tag_i()
         ,.w_data_i()
         ,.r_v_i()
         ,.r_tag_i()
         ,.r_data_o()
         ,.r_v_o()
        );

      // Stores the raw addresses sent to the bsg_cache
      // If more than 4 accumulate, just drop incoming.
      bsg_fifo_1r1w_small
       #(.width_p(daddr_width_p)
         ,.els_p(4)
        )
       incoming_address_buffer
        (.clk_i(clk_i)
         ,.reset_i(reset_i)
         ,.v_i(cache_pkt_v_li[i])
         ,.ready_o()
         ,.data_i(cache_pkt_li[i].addr)
         ,.v_o()
         ,.data_o()
         ,.yumi_i()
        )

      // Keeps track of the addresses this unit is currently prefetching
      bsg_fifo_1r1w_small
       #(.width_p(daddr_width_p)
         ,.els_p(4)
        )
       incoming_address_buffer
        (.clk_i(clk_i)
         ,.reset_i(reset_i)
         ,.v_i()
         ,.ready_o()
         ,.data_i(cache_pkt_li[i].addr)
         ,.v_o()
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
         ,.dma_pkt_v_o(dma_pkt_v_o[i])
         ,.dma_pkt_yumi_i(dma_pkt_ready_and_i[i] & dma_pkt_v_o[i])

         ,.dma_data_i(dma_data_i[i])
         ,.dma_data_v_i(dma_data_v_i[i])
         ,.dma_data_ready_o(dma_data_ready_and_o[i])

         ,.dma_data_o(dma_data_o[i])
         ,.dma_data_v_o(dma_data_v_o[i])
         ,.dma_data_yumi_i(dma_data_ready_and_i[i] & dma_data_v_o[i])

         ,.v_we_o()
         );
    end

endmodule

