#[asm/c]
PROG_TYPE?=c

PROG_NAME?=fibc

IMEM_WORDS_CNT?=256
# ---------------------------------

ifneq ($(PROG_TYPE),asm)
ifneq ($(PROG_TYPE),c)
$(error "WRONG PROG_TYPE! MUST BE ONE OF: asm, c.")
endif
endif

RISCV_PREFIX:=riscv64-unknown-elf-

AS=$(RISCV_PREFIX)as
LD=$(RISCV_PREFIX)ld
CC=$(RISCV_PREFIX)gcc
OBJCOPY=$(RISCV_PREFIX)objcopy
OBJDUMP=$(RISCV_PREFIX)objdump
READELF=$(RISCV_PREFIX)readelf

PROGS_DIR:=$(TASK_DIR)/$(PROG_TYPE)_progs
OUT_DIR:=$(TASK_DIR)/out_prog
ASM_COMMON_DIR:=$(TASK_DIR)/asm_common
SCRIPTS_DIR:=$(TASK_DIR)/scripts
SCRIPT_NOP_PAD:=$(SCRIPTS_DIR)/nop_pad.py
SRCIPT_TXT2MIF:=$(SCRIPTS_DIR)/txt2mif.py

# ---------------------------------

$(OUT_DIR)/%.txt: $(OUT_DIR)/%.bin
	hexdump -v -e '"%08x\n"' $^ > $@
	python3 $(SCRIPT_NOP_PAD) $@ $(IMEM_WORDS_CNT)

$(OUT_DIR)/%.mif: $(OUT_DIR)/%.txt
	python3 $(SRCIPT_TXT2MIF) $^ $@

$(OUT_DIR)/%.bin: $(OUT_DIR)/%.out
	$(OBJCOPY) -j .text -O binary $^ $@

$(OUT_DIR)/loader.o: $(ASM_COMMON_DIR)/loader.s
	@mkdir -p $(OUT_DIR)
	$(AS) -march=rv32i -mabi=ilp32 -c $(ASM_COMMON_DIR)/loader.s -o $(OUT_DIR)/loader.o

ifeq ($(PROG_TYPE),asm)
$(OUT_DIR)/%.out: $(PROGS_DIR)/%.s
	@mkdir -p $(OUT_DIR) 
	$(AS) -march=rv32i -mabi=ilp32 -c $^ -o $(OUT_DIR)/$*.o
	$(LD) -melf32lriscv -Map=$(OUT_DIR)/$*.map \
	-T $(ASM_COMMON_DIR)/riscv.ld $(OUT_DIR)/$*.o -o $@
	$(OBJDUMP) -d --visualize-jumps -M no-aliases $@ | tee $(OUT_DIR)/$*.dis
else ifeq ($(PROG_TYPE),c)
$(OUT_DIR)/%.out: $(PROGS_DIR)/%.c $(OUT_DIR)/loader.o
	@mkdir -p $(OUT_DIR) 
	$(CC) -nostdlib -march=rv32i -mabi=ilp32 -fPIC -c $< -o $(OUT_DIR)/$*.o
	$(LD) -melf32lriscv -Map=$(OUT_DIR)/$*.map \
	-T $(ASM_COMMON_DIR)/riscv.ld $(OUT_DIR)/loader.o $(OUT_DIR)/$*.o -o $@
	$(OBJDUMP) -d --visualize-jumps -M no-aliases $@ | tee $(OUT_DIR)/$*.dis
endif

.PHONY: build_prog
build_prog: $(OUT_DIR)/$(PROG_NAME).txt $(OUT_DIR)/$(PROG_NAME).mif
	@echo "Result hex file: " $^
	@echo "Result disasm file: " $(OUT_DIR)/$(PROG_NAME).dis

.PHONY: clean_out
clean_out:
	rm -rf $(OUT_DIR)
