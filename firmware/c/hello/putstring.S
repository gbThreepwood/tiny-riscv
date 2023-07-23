# Sends a NULL terminated string to the UART

.equ IO_LEDS, 4
.equ IO_UART_DATA, 8
.equ IO_UART_CTRL, 16

.section .text
.globl putstring

putstring:
    addi sp,sp,-4
    sw ra,0(sp) # Store return address from ra on the stack
    mv t2,a0
.L2:
    lbu a0,0(t2)
    beqz a0,.L3
    call uart_putchar
    addi t2,t2,1
    j .L2
.L3:
    lw ra,0(sp) # restore return adress to ra from the stack
    addi sp,sp,4
    ret
