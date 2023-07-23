
LIBDIR=../libs
INCDIR=../libs

CC=riscv64-elf-gcc
CFLAGS=-fno-pic -march=rv32i -mabi=ilp32 -fno-stack-protector -w -Wl,--no-relax

#SRC=hello_world.c

#TODO: This should somehow not be an absolute path
#LIBGCC=/usr/lib/gcc/riscv64-elf/12.2.0/rv32i/ilp32/libgcc.a

LIBS=putchar.o delay.o printf.o memcpy.o errno.o

APP=hello_world

ODIR=build

HEADERS = printf.h
DEPS = $(patsubst %,$(INCDIR)/%,$(HEADERS))

#OBJCFILES = printf.c.o hello_world.c.o
OBJCFILES = hello_world.c.o
OBJSFILES = uart_putchar.S.o putstring.S.o start.S.o hello.S.o
OBJC = $(patsubst %,$(ODIR)/%,$(OBJCFILES))
OBJS = $(patsubst %,$(ODIR)/%,$(OBJSFILES))

all: $(ODIR)/$(APP).mem

$(ODIR)/%.S.o: %.S
	riscv64-elf-as -march=rv32i -mabi=ilp32 -mno-relax -o $@ $^

$(ODIR)/%.c.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

$(ODIR)/$(APP).mem: $(ODIR)/$(APP).elf
	riscv64-elf-objcopy -O verilog $< $@ --verilog-data-width=4

$(ODIR)/$(APP).elf: $(OBJC) $(OBJS) $(LIBGCC)
	riscv64-elf-ld $^ -o $@ -T blockram.ld -m elf32lriscv -nostdlib --no-relax

#putchar.o: putchar.S
#	riscv64-elf-as -march=rv32i -mabi=ilp32 -mno-relax -o $@ $^

#%.o: $(SRC)
#	$(CC) -fno-pic -march=rv32i -mabi=ilp32 -fno-stack-protector -w -Wl,--no-relax -c $< -o $@

.PHONY: all clean

clean:
	rm -f $(ODIR)/*.o
