
.equ IO_BASE_ADDR, 0x400000
.equ IO_LEDS_ADDR, 4

.section .text
.globl hello

hello:
    la a0, my_message
    call putstring
    ret

my_message:
    .asciz "PQRS er ein fest\r\n"
