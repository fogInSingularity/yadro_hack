PROJECT_ROOT:=$(abspath .)
export PROJECT_ROOT

TASK_NAME?=08_cpu
TB_NAME?=i2c_test_bench
DEFINES_RAW?=
RAND_SEED?=25

# =========================
# ----- FILES & DIRS ------
# =========================

TASK_DIR:=$(PROJECT_ROOT)/$(TASK_NAME)
export TASK_DIR

SRC_DIR:=$(TASK_DIR)/src
TB_DIR:=$(TASK_DIR)/tb

# may not exist
INC_DIR:=$(TASK_DIR)/inc

TB_FILE:=$(TB_DIR)/$(TB_NAME).v
DUMP_FILE:=$(TASK_DIR)/dump.vcd
SIM_FILE:=$(TASK_DIR)/sim
LOCAL_MK:=$(TASK_DIR)/local.mk

QUARTUS_FILES_DIR:=$(TASK_DIR)/quartus_files
FPGA_TOP_FILE:=$(QUARTUS_FILES_DIR)/fpga_top.v

# ============================
# -- ICARUS COMPILE OPTIONS --
# ============================

INCDIR_BASIC_OPT:= -y $(SRC_DIR)

ifneq ($(wildcard $(INC_DIR)),)
INCDIR_BASIC_OPT+= -I$(INC_DIR)
endif

# ============================
# ---- ICARUS RUN OPTIONS ----
# ============================

COMPILE_OPTIONS:=-g2005-sv $(INCDIR_BASIC_OPT)
RUN_OPTIONS:= +RAND_SEED=$(RAND_SEED)

# ===========================
# ------ LOCAL MAKEFILE -----
# ===========================

ifneq ($(wildcard $(LOCAL_MK)),)
include $(LOCAL_MK)
endif

DEFINES:=$(addprefix -D,$(DEFINES_RAW))

# ===========================
# -------- TARGETS ---------- 
# ===========================

# VERILATOR

VERILATOR_LINT_OPTS:= --lint-only --Wall \
					  --Wno-procassinit --Wno-unusedsignal --Wno-unusedparam\
					  --timing --sv --timescale 1ns/1ps $(INCDIR_BASIC_OPT) --top $(TB_NAME)\
					  -D__ICARUS__

.PHONY: verilator_lint
verilator_lint:
	echo $(TASK_DIR)
	cd $(TASK_DIR) && verilator $(VERILATOR_LINT_OPTS) $(DEFINES) $(TB_FILE)

# ICARUS

.PHONY: compile
compile:
	cd $(TASK_DIR) && iverilog $(COMPILE_OPTIONS) $(DEFINES) \
	$(TB_FILE) -o $(SIM_FILE)

.PHONY: run
run:
	cd $(TASK_DIR) && $(SIM_FILE) $(RUN_OPTIONS)

# QUARTUS

.PHONY: quartus_gui
quartus_gui:
	cd $(QUARTUS_FILES_DIR) && quartus fpga & 

.PHONY: quartus_syn
quartus_syn:
	cd $(QUARTUS_FILES_DIR) && quartus_sh --flow compile fpga

.PHONY: quartus_drc
quartus_drc:
	cd $(QUARTUS_FILES_DIR) && quartus_drc fpga

.PHONY: quartus_pgm
quartus_pgm:
	cd $(QUARTUS_FILES_DIR) && quartus_pgm -c "USB-Blaster" -m JTAG -o "p;output/fpga.sof"

# WAVEFORMS

.PHONY: show_waves
show_waves:
	gtkwave $(DUMP_FILE) &

# OTHER

.PHONY: create_task_template
create_task_template:
	mkdir $(TASK_DIR) $(SRC_DIR) $(INC_DIR) \
	&& mkdir $(TB_DIR) && touch $(TB_FILE) \
	&& mkdir $(QUARTUS_FILES_DIR) && touch $(FPGA_TOP_FILE)

# CLEAN

.PHONY: clean_dump
clean_dump:
	rm -f $(wildcard $(PROJECT_ROOT)/**/dump.vcd)

.PHONY: clean_sim
clean_sim:
	rm -f $(wildcard $(PROJECT_ROOT)/**/sim)

.PHONY: clean_quartus
clean_quartus:
	cd $(QUARTUS_FILES_DIR) && rm -rf db incremental_db output simulation fpga.qws greybox_tmp *.bak 

.PHONY: clean_all
clean_all: clean_dump clean_sim clean_quartus
