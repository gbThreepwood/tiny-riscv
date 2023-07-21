.equ IO_BASE_ADDR, 0x400000
.equ IO_LEDS_ADDR, 4
.section .text
.globl main

main:
    la a0, hello_msg
    call putstring
    ret

hello_msg:
    .asciz "Hei, verden!\r\n"
