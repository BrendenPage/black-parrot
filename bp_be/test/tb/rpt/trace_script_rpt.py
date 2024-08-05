#!/bin/usr/python

from trace_gen_rpt import TraceGen
from rpt import SVElement
import random

def main():
# rpt_sets_p, stride_width_p, vaddr_width_p, rpt_ctr_width_p
  tracer = TraceGen(32, 8, 39, 2)

  # Store/Load double word test
  filename = "./basic_rpt.tr"
  file = open(filename, "w")

  pc = SVElement("000000000000000000000001000000000000000")
  eff_addr = SVElement("111111110")
  file.write(tracer.print_header())
  file.write(tracer.print_comment("typical for loop iteration"))
  for _ in range(100):
    pc_prev = pc
    file.write(tracer.put(pc, eff_addr))
    pc += 8 #8
    file.write(tracer.put(pc, eff_addr + 32))
    pc += 8 #16
    file.write(tracer.put(pc, eff_addr + 64))
    pc += 16 #32
    file.write(tracer.put(pc, eff_addr))
    eff_addr += 32
    pc = pc_prev
  
  file.write(tracer.print_comment("test done\n"))
  file.write(tracer.test_done())

  file.close()

  tracer.reset()
  filename = "./random_access_rpt.tr"
  file = open(filename, "w")

  pc = SVElement("000000000000000000000001000000000000000")
  eff_addr = SVElement("111111110")
  file.write(tracer.print_header())
  file.write(tracer.print_comment("random access"))
  eff_addr_lower = 5000
  eff_addr_upper = 5500
  pc_l = 26000
  pc_h = 800000
  for _ in range(10000):
    eff_addr = SVElement(bin(int(random.uniform(eff_addr_lower, eff_addr_upper)))[2:])
    pc = SVElement(bin(int(random.uniform(pc_l, pc_h)))[2:])
    file.write(tracer.put(eff_addr, pc))

  file.write(tracer.print_comment("test done\n"))
  file.write(tracer.test_done())

  file.close()

if __name__ == "__main__":
  main()
