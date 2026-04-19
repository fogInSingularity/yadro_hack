riscv64-elf-gcc -march=rv32i -mabi=ilp32 -ffreestanding -nostdlib -nostartfiles rover.c main.c rover_high.c vl53l1x_simple.c i2c.c loader.s -o rover -Oz -flto
