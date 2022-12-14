module bp_me_best_offset_generator
  #(parameter addr_width_p = 64
    ,parameter lg_offsets_p = 6
    ,parameter history_len_p = 8
    ,parameter max_score_p = 10
    ,parameter min_score_p = 1
    ,parameter max_rounds_p = 20
    ,parameter num_outstanding_misses_p = 2
  )
  (input                          clk_i
  , input                         reset_i
  , input  [addr_width_p-1:0]     miss_addr_i
  , input                         miss_v_i // Valid data is an address that missed in the cache
  , input                         prefetch_addr_i
  , input                         prefetch_v_i
  , output                        miss_ready_and_o // Ready to recieve miss data
  , output [lg_offsets_p-1:0]     offset_o
  );

  logic [lg_offsets_p-1:0] test_offset;
  logic [lg_offsets_p-1:0] best_offset;
  logic [`BSG_SAFE_CLOG2(max_score_p)-1:0][2**lg_offsets_p-1:0] offset_scores;
  logic [addr_width_p-1:0] miss_addr_lo, test_addr;
  logic processing_miss_v_lo, fifo_yumi_li;
  logic [`BSG_SAFE_CLOG2(max_rounds_p)-1:0] round_counter;
  logic [lg_offsets_p-1:0] current_best_offset;
  logic [`BSG_SAFE_CLOG2(max_score_p)-1:0] current_best_offset_score;
  logic [num_outstanding_misses_p-1:0] tag_empty_lo, tag_r_match_lo, repl_way_lo;

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
    ,.yumi_i(test_offset == '1 & processing_miss_v_lo)
    );

  bsg_cam_1r1w_tag_array
    #(.width_p(addr_width_p)
     ,.els_p(history_len_p)
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
    ,.b_i({'0,{test_offset}})
    ,.s_o(test_addr)
    ,.c_o()
    );

  always_ff @(posedge clk_i) begin
    if (reset_i) begin
      test_offset <= '0;
      best_offset <= 1;
      offset_scores <= '0;
      fifo_yumi_li <= '0;
      current_best_offset <= '0;
    end else if (test_offset == '1 & processing_miss_v_lo) begin
      // We are on the last check of this round
      test_offset = 1;
      if (round_counter == max_rounds_p - 1) begin
        best_offset <= current_best_offset_score > min_score_p ? current_best_offset : '0;
        round_counter <= '0;
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
        end else if (current_best_offset_score < offset_scores[test_offset] + 1) begin
          // Update intermediate values
          current_best_offset_score <= offset_scores[test_offset] + 1;
          current_best_offset <= test_offset;
        end
      end
    end else begin
      // We wait idly while there are no misses to process
      test_offset <= 1;
    end
  end

  assign offset_o <= best_offset;
endmodule