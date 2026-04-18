.text
.globl _start
.globl _finish

_start:
    li      t0, 3
    addi    t0, t0, 5
    addi    t0, t0, 7
    addi    t0, t0, 10
    addi    t0, t0, 10
    addi    t0, t0, 7
    sb      t0, 0x21(zero)

_finish:
    beqz  zero, _finish


