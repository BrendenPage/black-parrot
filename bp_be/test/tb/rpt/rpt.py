#
#   rpt.py
#   simulates the reference prediciton table to produce accurate test traces
#

import math


def log2(x):
  return math.ceil(math.log2(x))

def one():
  return SVElement("1")

def zero():
  return SVElement("0")

# Class to allow array access to strings equivalent to array access in SV
class SVElement:
  def __init__(self, s):
    if isinstance(s, SVElement):
      s = s.s
    if isinstance(s, bool):
      s = "1" if s else "0"
    if isinstance(s, int):
      s = str(s)
    self.s = s.replace('_', '').replace(' ', '')

  # Written and modified from ChatGPT, only tested for s[x, y] x>y, neither None and single element accessing s[x], x>0
  def __getitem__(self, index):
    # If the index is 0, return the rightmost character
    if isinstance(index, int):
      return SVElement(self.s[index - 1])

    # If the index is a slice, handle it
    if isinstance(index, slice):
      start = index.start
      stop = index.stop
      step = index.step
      # Adjust the start and stop indices for 1-based indexing and reverse slicing
      if start is not None and stop is not None:
        if start > stop:
          start = len(self.s) - start - 1
          stop = len(self.s) - stop
        else:
          start = start - 1 if start > 0 else None
          stop = stop - 1 if stop > 0 else None
      else:
        if start is not None:
          start = start - 1 if start > 0 else None
        if stop is not None:
          stop = stop - 1 if stop > 0 else None

      # Use the slice to get the substring from self.s
      return SVElement(self.s[slice(start, stop, step)])

  def __setitem__(self, index, value):
    # Convert the string to a list to allow mutation
    s_list = list(self.s)

    if isinstance(value, str) or isinstance(value, int):
      value = SVElement(value)
    
    # Handle integer indexing
    if isinstance(index, int):
      s_list[index - 1] = value.s
    
    # Handle slicing
    elif isinstance(index, slice):
      start = index.start
      stop = index.stop

      # Adjust start and stop indices for 1-based indexing and reverse slicing
      if start is not None and stop is not None and start > stop:
        start = len(self.s) - start - 1
        stop = len(self.s) - stop
        for i in range(stop-start):
          s_list[start + i] = value.s[i]

    # Convert the list back to a string
    self.s = ''.join(s_list)

  def __len__(self):
    return len(self.s)

  def __str__(self):
    return self.s
  
  def __add__(self, other):
      if isinstance(other, SVElement):
        return SVElement(self.s + other.s)
      elif isinstance(other, str):
        return SVElement(self.s + other)
      elif isinstance(other, int):
        self_int = int(self.s, 2)
        prev_len = len(self.s)
        res = SVElement(bin(self_int + other)[2:])
        if len(res) > prev_len:
          return res
        return SVElement("0"*(prev_len-len(res)))+res
      else:
        return NotImplemented

  # self - other
  def __sub__(self, other):
    num = 0
    if isinstance(other, SVElement):
      for i in range(len(other.s)):
        if (other[i] != 0 and other[i] != 1):
          raise("Non-binary subtraction of SVElement not allowed")
      num = int(other.s, 2)
    elif isinstance(other, int):
      num = other
    elif isinstance(other, str):
      num = int(other,2)
    else:
      raise("Not implemented")
    self_int = int(self.s, 2)
    res = SVElement(bin(self_int - num)[2:])
    prev_len = len(self.s)
    if len(res) > prev_len:
      return res
    return SVElement("0"*(prev_len-len(res)))+res

  def __repr__(self):
    return f'SVElement({self.s})'

  def __eq__(self, other):
    if isinstance(other, SVElement):
      return self.s == other.s
    if isinstance(other, str):
      return self.s == other
    if isinstance(other, int):
      if (other == 1 or other == 0):
        return self.s == str(other)
    return False
  
  def __int__(self):
    return int(self.s)
  
  def __bool__(self):
    return self.bitwise_or()

  def bitwise_and(self):
    for i in range(len(self.s)):
      if not int(self.s[i]):
        return False
    return True
  
  def bitwise_or(self):
    for i in range(len(self.s)):
      if int(self.s[i]):
        return True
    return False
  
  def bitwise_or_other(self, other):
    assert(len(other) == len(self.s))
    other_str = other
    if isinstance(other, SVElement):
      other_str = other.s
    elif not isinstance(other, str):
      raise("bitwise operations not available for non string, non SVElement operations")
    retval = ""
    l = len(self.s)
    for i in range(l):
      retval += str(int(int(other_str[i]) or int(self.s[i])))
    return SVElement(retval)

  def bitwise_and_other(self, other):
    assert(len(other) == len(self.s))
    other_str = other
    if isinstance(other, SVElement):
      other_str = other.s
    elif not isinstance(other, str):
      raise("bitwise operations not available for non string, non SVElement operations")
    retval = ""
    l = len(self.s)
    for i in range(l):
      retval += str(int(int(other_str[i]) and int(self.s[i])))
    return SVElement(retval)


  def bitwise_not(self):
    retval = SVElement("")
    for i in range(len(self.s)):
      retval += str(int(not(int(self.s[i]))))
    return retval

# pass only SVElements as arguments to methods not strings
class Rpt:

  # constructor
  def __init__(self, rpt_sets_p, stride_width_p, vaddr_width_p, rpt_ctr_width_p):
    self.sets         = rpt_sets_p
    self.stride_width = stride_width_p
    self.vaddr_width  = vaddr_width_p
    self.ctr_width    = rpt_ctr_width_p

    self.tag_width    = self.vaddr_width - log2(self.sets)
    self.entry_width  = self.tag_width + self.ctr_width + self.stride_width + self.vaddr_width
    self.row_width    = self.entry_width*2 + 1
    self.idx_width    = self.vaddr_width - self.tag_width # which set are we accessing

    self.cache = [(SVElement("0"*self.row_width)) for _ in range(self.sets)]

    # Metadata
    self.start_index= SVElement("0"*self.sets*2)
    self.vector     = SVElement("0"*self.sets*2)
    self.stride_r   = SVElement("0"*self.stride_width)
    self.eff_addr_o = SVElement("0"*self.vaddr_width)
    self.stride_v_o = zero()
    self.pc_o       = SVElement("0"*self.vaddr_width)
    self.confirm_discovery_o = zero()
    self.start_discovery_o = zero()

  def get_addr_set(self, address):
    return address[self.idx_width-1:0]
  
  def get_addr_tag(self, address):
    return address[self.vaddr_width-1:self.vaddr_width-self.tag_width]

  def reset(self):
    self.cache = [(SVElement("0"*self.row_width)) for _ in range(self.sets)]
    self.start_index= SVElement("0"*self.sets*2)
    self.vector     = SVElement("0"*self.sets*2)
    self.stride_r   = SVElement("0"*self.stride_width)
    self.eff_addr_o = SVElement("0"*self.vaddr_width)
    self.stride_v_o = zero()
    self.pc_o       = SVElement("0"*self.vaddr_width)

  # Recover the fsm counter from the entry of the index in the given row
  def get_row_ctr(self, row, idx):
    if (idx == 0): # [1, 0]
      return row[self.ctr_width:1]
    else:
      return row[self.ctr_width+self.entry_width:self.entry_width+1]


  # Recover the tag from the entry of the index in the given row
  def get_row_tag(self, row, idx):
    if (idx==0):
      return row[self.entry_width:self.entry_width-self.tag_width + 1]
    else:
      return row[self.row_width-1:self.row_width - self.tag_width]


  # Recover the stride from the entry of the index in the given row
  def get_row_stride(self, row, idx):
    if (idx==0):
      return row[self.stride_width + self.ctr_width : self.ctr_width + 1]
    else:
      return row[self.stride_width + self.ctr_width + self.entry_width : self.ctr_width + self.entry_width + 1]


  # Recover the effective address from the entry of the index in the given row
  def get_row_eff_addr(self, row, idx):
    if (idx==0):
      return row[self.entry_width - self.tag_width : self.stride_width + self.ctr_width + 1]
    else:
      return row[self.row_width - self.tag_width - 1 : self.entry_width + self.stride_width + self.ctr_width + 1]


  def replace_element(self, row, element):
    if len(row) != self.row_width:
      raise("Incorrect row width")
    if (not int(row[0])): #[1,0] replace 0 because 0 LRU
      row[self.entry_width:1] = element
      row[0] = "1"
    else: # replace 1
      row[self.row_width-1:self.entry_width+1] = element
      row[0] = "0"
    return row
  
  def update_ctr(self, ctr, stride_match):
    ctr_n = SVElement("00")
    if (stride_match):
      if (int(ctr[0]) and int(ctr[1])):
        ctr_n = ctr
      else:
        ctr_n[1] = SVElement(str(int(bool(ctr[0]) ^ bool(ctr[1]))))
        ctr_n[0] = SVElement(ctr[0].bitwise_not())
    return ctr_n

  def put(self, pc, eff_addr):
    assert(isinstance(pc, SVElement) and isinstance(eff_addr, SVElement))
    assert(len(pc)<=self.vaddr_width)
    assert(len(eff_addr)<=self.vaddr_width)
    # Normalize widths
    pc        = SVElement("0"*(self.vaddr_width - len(pc))) + pc
    eff_addr  = SVElement("0"*(self.vaddr_width - len(eff_addr))) + eff_addr
    #CACHE READING AND UPDATING LOGIC
    idx_i = self.get_addr_set(pc)
    idx_i_int = int(str(idx_i),2)
    tag_i = self.get_addr_tag(pc)
    row   = self.cache[idx_i_int]

    tag_1 = self.get_row_tag(row, 0)
    tag_2 = self.get_row_tag(row, 1)

    stride_1 = self.get_row_stride(row,0)
    stride_2 = self.get_row_stride(row,1)

    eff_addr_1 = self.get_row_eff_addr(row,0)
    eff_addr_2 = self.get_row_eff_addr(row,1)

    ctr_1 = self.get_row_ctr(row,0)
    ctr_2 = self.get_row_ctr(row,1)

    t1 = SVElement("1" if tag_1 == tag_i else "0")
    t2 = SVElement("1" if tag_2 == tag_i else "0")
    tag_match = t2 + t1

    if (tag_match[0]):
      eff_addr_lo = eff_addr_1
    elif (tag_match[1]):
      eff_addr_lo = eff_addr_2
    else:
      eff_addr_lo = SVElement("0"*self.vaddr_width)

    addr_diff = eff_addr - eff_addr_lo
    stride_li = addr_diff[self.stride_width-1:0]


    sm1 = SVElement("1" if stride_1 == stride_li else "0")
    sm2 = SVElement("1" if stride_2 == stride_li else "0")

    stride_match = sm2 + sm1

    ctr_1_n = self.update_ctr(ctr_1, stride_match[0])
    ctr_2_n = self.update_ctr(ctr_2, stride_match[1])


    if tag_match[0]:
      lru_n = one()
    elif tag_match[1]:
      lru_n = zero()
    else:
      lru_n = SVElement(str(int(not row[0])))

    if ((not tag_match[0]) and not tag_match[0] and not row[0]) or tag_match[0]:
      w_data_1 = tag_i + eff_addr + stride_li + ctr_1_n
    else:
      w_data_1 = row[self.entry_width:1]
    
    if ((not tag_match[0]) and not tag_match[0] and row[0]) or tag_match[1]:
      w_data_2 = tag_i + eff_addr + stride_li + ctr_2_n
    else:
      w_data_2 = row[self.row_width -1:self.entry_width+1]

    assert(len(tag_i) == self.tag_width)
    assert(len(eff_addr) == self.vaddr_width)
    assert(len(stride_li) == self.stride_width)
    assert(len(ctr_1_n) == self.ctr_width and len(ctr_2_n) == self.ctr_width)
    assert(len(w_data_2) == self.entry_width)
    assert(len(w_data_1) == self.entry_width)

    new_row = w_data_2 + w_data_1 + lru_n

    for i in range(len(new_row)):
      if not(str(new_row[i]) == "0" or str(new_row[i]) == "1"):
        print(new_row)
        exit("non-binary value spotted in row")

    self.cache[idx_i_int] = w_data_2 + w_data_1 + lru_n

    # METADATA UPDATING AND OUTPUT LOGIC
    lru_int = int(row[0])

    new_idx_int = 1 << ((idx_i_int << 1) ^ lru_int)
    idx_res = SVElement(bin(new_idx_int)[2:])
    displaced_len = self.sets*2 - len(idx_res)
    new_idx = SVElement("0"*displaced_len) + idx_res

    stride_vector_n = self.vector.bitwise_or_other(new_idx)
    retval = False #return flag to indicate change in discovery mode

    self.start_discovery_o = zero()
    self.confirm_discovery_o = zero()
    if (ctr_1_n.bitwise_and() and bool(tag_match[0]) & stride_1.bitwise_or()) or (ctr_2_n.bitwise_and() and bool(tag_match[1]) & stride_2.bitwise_or()):
      self.stride_r = stride_1 if tag_match[0] else stride_2
      self.stride_v_o = one()
      self.eff_addr_o = eff_addr
      self.pc_o = pc

      
      if (self.vector.bitwise_not().bitwise_and()):
        self.start_discovery_o = one()
        # print("Started discovery 1")
        retval = True
        self.vector = stride_vector_n
        self.start_index = new_idx
      else:
        if (self.vector.bitwise_and_other(new_idx).bitwise_or()):
          if(new_idx.bitwise_and_other(self.start_index.bitwise_not()).bitwise_or()):
            self.start_discovery_o = one()
            # print("Started discovery 2")
            retval = True
            # print(new_idx.bitwise_and_other(self.start_index.bitwise_not()))
            # print(self.start_index)
            self.vector = new_idx
            self.start_index = new_idx
          else:
            self.confirm_discovery_o = one()
            # print("Confirmed discovery")
            retval = True
            # print("idx: " + str(math.log2(int(str(new_idx),2))))
            self.start_index = zero()
            self.vector = SVElement("0"*self.sets*2)
        else:
          self.vector = stride_vector_n
    else:
      self.pc_o = SVElement("0"*self.vaddr_width)
      self.stride_v_o = zero()
      self.eff_addr_o = SVElement("0"*self.vaddr_width)
      self.stride_r = SVElement("0"*self.stride_width)
    
    return retval

  def discovery_mode(self):
    return self.start_discovery_o + self.confirm_discovery_o

  def snapshot(self):
    return str(self.eff_addr_o) + "_" + str(self.stride_r) + "_" + str(self.pc_o)
  
  def snapshot_verbose(self):
    return "Eff_addr: " + str(self.eff_addr_o) + ", stride: " + str(self.stride_r) + ", stride_v_o: " + str(self.stride_v_o) + ", pc: " + str(self.pc_o)

  def __str__(self):
    res = ""
    for i in range(self.sets):
      res += "idx: " + str(i).zfill(2) + ": " + self.get_row_info(self.cache[i]) + "\n"
    return res

  def get_row_info(self, row):
    tag_1      = self.get_row_tag(row, 0)
    tag_2      = self.get_row_tag(row, 1)

    eff_addr_1 = self.get_row_eff_addr(row, 0)
    eff_addr_2 = self.get_row_eff_addr(row, 1)

    stride_1   = self.get_row_stride(row,0)
    stride_2   = self.get_row_stride(row,1)

    ctr_1      = self.get_row_ctr(row, 0)
    ctr_2      = self.get_row_ctr(row, 1)

    retval = "tag_1: " + str(tag_1) + ", eff_addr_1: " + str(eff_addr_1) + ", stride_1: " + str(stride_1) + ", ctr_1: " + str(ctr_1) + " "
    retval +="tag_2: " + str(tag_2) + ", eff_addr_2: " + str(eff_addr_2) + ", stride_2: " + str(stride_2) + ", ctr_2: " + str(ctr_2)
    return retval + ", lru: " + str(row[0])

def main():
  t1 = SVElement("0100111001")
  t2 = SVElement("000")
  t3 = SVElement("111")
  t4 = SVElement("0101")
  t5 = SVElement("1010")
  t6 = SVElement("0100")
  t7 = SVElement("0000000000100001")
  t8 = SVElement("0000000000000100")
  t9 = SVElement("0000000001100101")
  ttrue = SVElement(True)
  tfalse = SVElement(False)
  tone = SVElement(1)
  tzero = SVElement(0)
  assert(ttrue == "1")
  assert(tfalse == "0")
  assert(tone == "1")
  assert(tzero == "0")
  assert(t1[0] == 1)
  assert(t1[1:0] == "01")
  assert(t1[3:1] == "100")
  assert(t1 + t2 == "0100111001000")
  assert(t1.bitwise_not() == "1011000110")
  assert(not t1.bitwise_and())
  assert(t1.bitwise_or())
  assert(not t2.bitwise_or())
  assert(t3.bitwise_and())
  t1[3:1] = t2
  assert(t1 == "0100110001")
  assert(t2.bitwise_and_other(t3) == "000")
  assert(t2.bitwise_or_other(t3) == "111")
  assert(t4.bitwise_and_other(t5) == "0000")
  assert(t4.bitwise_or_other(t5) == "1111")
  assert(t4.bitwise_and_other(t6) == "0100")
  assert(t4 + 2 == "0111")
  assert(t2 + 11 == "1011")
  assert(t7.bitwise_or_other(t8) == "0000000000100101")
  assert(t8.bitwise_or_other(t7) == t7.bitwise_or_other(t8))
  assert(t7.bitwise_and_other(t9) == t9.bitwise_and_other(t7))
  assert(t7.bitwise_and_other(t9) == "0000000000100001")
  #RPT Tests
  #rpt_sets_p, stride_width_p, vaddr_width_p, rpt_ctr_width_p
  rpt = Rpt(32, 8, 16, 2)
  addr = SVElement("0110100110100111")
  assert(rpt.get_addr_set(addr) == SVElement("00111"))
  assert(rpt.get_addr_tag(addr) == "01101001101")
  assert(rpt.get_addr_tag(addr) + rpt.get_addr_set(addr) == addr)
  el_0 = SVElement("01010111010 1011100101101101 10000001 10")
  el_1 = SVElement("10000010101 1111000011001111 01111110 11")
  el_2 = SVElement("00011100011 1111111111111111 11111111 11")
  row  = el_1 + el_0 + "0"
  #RPT Row tests
  assert(rpt.get_row_tag(row,0) == "01010111010")
  assert(rpt.get_row_tag(row,1) == "10000010101")
  assert(rpt.replace_element(row, el_2) == "100000101011111000011001111011111101100011100011111111111111111111111111111")
  assert(rpt.get_row_ctr(row, 0) == "11")
  assert(rpt.get_row_ctr(row, 1) == "11")
  assert(rpt.replace_element(row, el_2) == "000111000111111111111111111111111111100011100011111111111111111111111111110")

  ctr1 = SVElement("00")
  ctr2 = SVElement("10")
  stride_matched = one()
  stride_not_matched = zero()
  ctr1 = rpt.update_ctr(ctr1, stride_matched)
  assert(ctr1 == "01")
  ctr1 = rpt.update_ctr(ctr1, stride_matched)
  assert(ctr1 == "10")
  ctr1 = rpt.update_ctr(ctr1, stride_matched)
  assert(ctr1 == "11")
  ctr1 = rpt.update_ctr(ctr1, stride_matched)
  assert(ctr1 == "11")
  ctr1 = rpt.update_ctr(ctr1, stride_not_matched)
  assert(ctr1 == "00")
  ctr2 = rpt.update_ctr(ctr2, stride_not_matched)
  assert(ctr2 == "00")

  # Test new index generation (passed as is)
  # for i in range(int(rpt.sets/2)):
  #   for j in range(2):
  #     idx_r_int = i
  #     lru_int = j
  #     new_idx = 1 << ((idx_r_int << 1) ^ lru_int)
  #     idx_res = SVElement(bin(new_idx)[2:])
  #     displaced_len = rpt.sets - len(idx_res)
  #     fin_index = SVElement("0"*displaced_len) + idx_res
  #     print(fin_index)

  rpt = Rpt(8, 8, 16, 2)
  pc       = SVElement("0011100111111000")
  pc_2     = SVElement("0000000000001000")
  eff_addr = SVElement("0000000000000000")
  for _ in range(4):
    rpt.put(pc, eff_addr)
    print(rpt)
    eff_addr += 4
  rpt.put(pc_2, eff_addr + 43)
  print(rpt)
  rpt.put(pc, eff_addr)
  print(rpt)
  print(rpt.snapshot())
  print("\n\n\n")
  rpt = Rpt(8,8,16,2)
  for _ in range(4):
    pc_prev = pc_2
    for _ in range(16):
      rpt.put(pc_2, eff_addr)
      print(rpt)
      pc_2 += 1
    eff_addr += 16
    pc_2 = pc_prev
  rpt.put(pc_2, eff_addr)
  print(rpt)
  
  rpt = Rpt(8, 8, 16, 2)
