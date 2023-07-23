.equ IO_BASE_ADDR, 0x400000
.section .text
.globl start

start:
    li gp,IO_BASE_ADDR
    li sp,0x1800
    call main
    ebreak
