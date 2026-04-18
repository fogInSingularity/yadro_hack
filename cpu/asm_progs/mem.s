.text
.globl _start
.globl _finish

_start:
    li      t0, 0x13
    li      t5, 0x1000
    sb      t0, 0x1(t5)
    lb      t1, 0x1(t5)
    sb      t1, 0x21(zero)
    addi    t1, t1, 3
    sb      t1, 0x0(t5)
    lb      t2, 0x0(t5)
    sb      t2, 0x20(zero)

_finish:
    beqz  zero, _finish


