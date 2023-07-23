
.section .text
.globl delay

delay:
    addi sp,sp,-4
    sw ra,0(sp) # Store return address from ra on the stack
    li t0,1
	slli t0, t0, 21
.L1:
    addi t0,t0,-1
	bnez t0, .L1
    lw ra,0(sp) # restore return adress to ra from the stack
    addi sp,sp,4
	ret
