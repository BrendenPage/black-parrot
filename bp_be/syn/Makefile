## Set common environment variables
TOP ?= $(shell git rev-parse --show-toplevel)

include $(TOP)/Makefile.common

export SYN_PATH     := $(BP_BE_DIR)/syn
export TB_PATH      := $(BP_BE_DIR)/test/tb

export LOG_PATH     := $(BP_BE_DIR)/syn/logs
export RESULTS_PATH := $(BP_BE_DIR)/syn/results
export REPORT_PATH  := $(BP_BE_DIR)/syn/reports
export TOUCH_PATH   := $(BP_BE_DIR)/syn/touchfiles

export TB    ?= rpt
export CFG   ?= e_bp_test_multicore_half_cfg
export DRAM  ?= dramsim3
export SUITE ?= none
export PROG  ?= random_access_rpt
export TAG   ?= none

include $(TB_PATH)/$(TB)/Makefile.cfgs
include $(TB_PATH)/$(TB)/Makefile.params
include $(TB_PATH)/$(TB)/Makefile.testlist
include $(BP_COMMON_DIR)/syn/Makefile.common
include $(BP_COMMON_DIR)/syn/Makefile.dc
include $(BP_COMMON_DIR)/syn/Makefile.verilator
include $(BP_COMMON_DIR)/syn/Makefile.vcs
include $(BP_COMMON_DIR)/syn/Makefile.xcelium

