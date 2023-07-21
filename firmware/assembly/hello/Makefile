
OBJ = hello_world.o putstring.o start.o

#all: hello_world.mem

#hello_world: 

#objects: $(OBJ)

hello_world.mem: hello_world.elf
	riscv64-elf-objcopy -O verilog $< $@ --verilog-data-width=4

%.elf: $(OBJ)
	riscv64-elf-ld $^ -o $@ -T blockram.ld -m elf32lriscv -nostdlib --no-relax

%.o: %.S
	riscv64-elf-as -march=rv32i -mabi=ilp32 -mno-relax -o $@ $^

.PHONY: all
