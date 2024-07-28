#!/bin/usr/python

import sys
from trace_gen_prefetch_gen import TraceGen

def main():
# loop_range_width_p, stride_width_p, vaddr_width_p, block_width_p, instr_width_p, decode_width_p
  tracer = TraceGen(8, 8, 39, 512, 32, 56)
  filepath = sys.argv[1]

  # Store/Load double word test
  filename = filepath + "basic_prefetch_gen.tr"
  file = open(filename, "w")

  pc = 3456
  eff_addr = 511
  stride = 1
  loop_counter = 2
  file.write(tracer.print_header())
  file.write(tracer.print_comment("prefetch on one loop iteration at loop boundary"))
  
  file.write(tracer.send_pref_info(pc, eff_addr, loop_counter, stride))
  file.write(tracer.recv_dispatch_pkts(pc, eff_addr, loop_counter,stride))
  
  file.write(tracer.print_comment("Prefetch on one loop iteration test done\n"))
  file.write(tracer.test_done())

  file.close()

if __name__ == "__main__":
  main()
