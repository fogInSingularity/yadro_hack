PROG_NAME ?= prog
IMEM_WORDS_CNT ?= 4096

RISCV_PREFIX := riscv64-unknown-elf-

AS      := $(RISCV_PREFIX)as
CC      := $(RISCV_PREFIX)gcc
OBJCOPY := $(RISCV_PREFIX)objcopy
OBJDUMP := $(RISCV_PREFIX)objdump
READELF := $(RISCV_PREFIX)readelf

PROGS_DIR      := $(TASK_DIR)/c_dispprog
OUT_DIR        := $(TASK_DIR)/out_prog
ASM_COMMON_DIR := $(TASK_DIR)/asm_common
SCRIPTS_DIR    := $(TASK_DIR)/scripts
SCRIPT_NOP_PAD := $(SCRIPTS_DIR)/nop_pad.py
SCRIPT_TXT2MIF := $(SCRIPTS_DIR)/txt2mif.py

C_SRCS := $(wildcard $(PROGS_DIR)/*.c)
C_OBJS := $(patsubst $(PROGS_DIR)/%.c,$(OUT_DIR)/%.o,$(C_SRCS))
C_DEPS := $(C_OBJS:.o=.d)

COMMON_RV_FLAGS := -march=rv32i -mabi=ilp32
BARE_FLAGS      := -ffreestanding -nostdlib -nostartfiles
NO_GOT_FLAGS    := -fno-pic -fno-pie -msmall-data-limit=0

CFLAGS  := $(COMMON_RV_FLAGS) $(BARE_FLAGS) $(NO_GOT_FLAGS) -Oz -flto
LDFLAGS := $(COMMON_RV_FLAGS) $(BARE_FLAGS) $(NO_GOT_FLAGS) -Oz -flto
LDSCRIPT := $(abspath $(ASM_COMMON_DIR)/riscv.ld)

# ---------------------------------

$(OUT_DIR)/%.txt: $(OUT_DIR)/%.bin
	hexdump -v -e '"%08x\n"' $< > $@
	python3 $(SCRIPT_NOP_PAD) $@ $(IMEM_WORDS_CNT)

$(OUT_DIR)/%.mif: $(OUT_DIR)/%.txt
	python3 $(SCRIPT_TXT2MIF) $< $@

$(OUT_DIR)/%.bin: $(OUT_DIR)/%.out
	$(OBJCOPY) -j .text -O binary $< $@

$(OUT_DIR)/loader.o: $(ASM_COMMON_DIR)/loader.s
	@mkdir -p $(OUT_DIR)
	$(AS) $(COMMON_RV_FLAGS) -c $< -o $@

$(OUT_DIR)/%.o: $(PROGS_DIR)/%.c
	@mkdir -p $(OUT_DIR)
	$(CC) $(CFLAGS) -MMD -MP -c $< -o $@

$(OUT_DIR)/$(PROG_NAME).out: $(OUT_DIR)/loader.o $(C_OBJS)
	@mkdir -p $(OUT_DIR)
	cd $(OUT_DIR) && \
	$(CC) $(LDFLAGS) \
		-Wl,-Map=$(PROG_NAME).map \
		-T $(LDSCRIPT) \
		loader.o $(notdir $(C_OBJS)) -o $(PROG_NAME).out

.PHONY: build_prog
build_prog: clean_out $(OUT_DIR)/$(PROG_NAME).txt $(OUT_DIR)/$(PROG_NAME).mif
	@echo "Result hex file: " $(OUT_DIR)/$(PROG_NAME).txt $(OUT_DIR)/$(PROG_NAME).mif
	@echo "Result disasm file: " $(OUT_DIR)/$(PROG_NAME).dis

.PHONY: clean_out
clean_out:
	rm -rf $(OUT_DIR)

-include $(C_DEPS)
