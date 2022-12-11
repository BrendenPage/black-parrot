`ifndef BP_ME_CACHE_SLICE_DEFINES_SVH
`define BP_ME_CACHE_SLICE_DEFINES_SVH

  // Miss Status Handling Register Struct
  // This struct tracks the information required to process an LCE request
  `define declare_bp_prefetch_pkt_s(addr_width_no_offset_mp) \
    typedef struct packed                                                   \
    {                                                                       \
      logic                                         prefetch;               \
      logic [addr_width_no_offset_mp-1:0]             addr;                 \
    } bp_prefetch_pkt_s

  `define bp_cache_slice_fills_per_block(l2_block_size_in_words_mp, l2_data_width_mp, l2_fill_width_mp)          \
    (l2_block_size_in_words_mp * l2_data_width_mp)/l2_fill_width_mp

`endif

