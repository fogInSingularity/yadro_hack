.global read
.global write
.global exit
.global _start
.section .text

exit:
li a7, 93
ecall

_start:
lw a0, 0(sp)
     addi a1, sp, 4 # 8 for 64-bit mode
call main
j exit

