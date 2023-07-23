# Simple blinker

.equ IO_BASE, 0x400000  
.equ IO_LEDS, 4

.section .text

.globl start

start:
    li   gp,IO_BASE
	li   sp,0x1800

	li s0,16
	li a0,0

#.L0:
#	sw		a0, IO_LEDS(gp)
#	call	wait
#	addi	a0,a0,1
#	bne		a0,s0,.L0

.L0:
	li   t0, 2
	sw   t0, IO_LEDS(gp)
	call wait
	li   t0, 4
	sw   t0, IO_LEDS(gp)
	call wait
	j .L0

wait:
    li t0,1
	slli t0, t0, 21
.L1:
    addi t0,t0,-1
	bnez t0, .L1
	ret
