# tiny-riscv

Verilog implementation of a basic RISC-V CPU.

> :warning: **Warning: This project is a work in progress**

The CPU in this repository is mainly intended to serve as a learning tool for people interested in the internal operations of a CPU, or for people who would like to design their own CPU.

The ISA manual is available here: https://riscv.org/wp-content/uploads/2019/06/riscv-spec.pdf

## Features

* Support the RV32I Base Instruction Set
* VGA output
* Small size

## Hardware

The CPU is initially designed to run on the Nandland Go Board, although most of the Verilog source is general in nature and easily ported to different hardware. The Lattice iCE40HX1K FPGA on the Go Board has a limited number of logic cells, and thus addition of additional hardware features such as UART, SPI, I2C, VGA, etc. will require a more capable FPGA.

TODO: Find a nice board with a more capable FPGA and add support for it.

## Installation of the required tools

I **strongly** recommend that you use a GNU/Linux system, although I am sure it is possible to get this working in Windows or MacOS.

Initially my intention was to use APIO as the build system (PlatformIO for FPGAs) but unfortunately at the moment it is not terribly flexible and quickly got annoying to work with for larger projects. Instead I have installed some software tools using my package manager, and manually downloaded others.

You will need to (at least) install the following tools:

* Yosys (https://github.com/YosysHQ/yosys)
* nextpnr (https://github.com/YosysHQ/nextpnr)
* icestorm (https://github.com/YosysHQ/icestorm)

For simulation you should also install:

* gtkwave (https://gtkwave.sourceforge.net/)
* iverilog (https://github.com/steveicarus/iverilog)
* verilator (https://www.veripool.org/verilator/)

The simplest way to get the required software tools is to download pre-built binaries from here: https://github.com/YosysHQ/oss-cad-suite-build/releases



### APIO

~~The project uses APIO (https://apiodoc.readthedocs.io/en/stable/index.html) in order to simplify installation of the required tools for building and testing. Manual installation of each required tool is also possible, but unless packages are available in your favorite Linux distro the installation can be a pain to complete.~~

I also **strongly** recommend that you use a virtual python environment for the installation of the PIP package (this is a general recommendation any time you install something using PIP).

`python -m venv fpga-python`

Activate the virtual environment:

`source ./fpga-python/bin/activate`

(optionally) upgrade pip:

`pip install --upgrade pip`

Install APIO:

`pip install apio`

Install the required tools (yosys, nextpnr, etc) using APIO:

`apio install --all`

## Building the Hardware

`apio build`

`apio upload`

## Software to run on the CPU
 One of the nice things about implementing a standardized instruction set (such as RISC-V), as opposed to inventing your own instruction set is that assemblers and compilers already exists for several programming languages.

### Assembly programming

To explain assembly programming a ledblink program is used as example.

To assemble the program you should invoke:

`riscv64-elf-as -march=rv32i -mabi=ilp32 -mno-relax ledblink.S -o ledblink.o`

The linker ensures correct placement in memory by means of a simle linker script ledblink.elf

`riscv64-elf-ld ledblink.o -o ledblink.elf -T blockram.ld -m elf32lriscv -nostdlib`

The elf-objcopy utility support output in "verilog" format, i.e. a listing of hexadecimal values encoded as ASCII.

`riscv64-elf-objcopy -O verilog --verilog-data-width=4 ledblink.elf ledblink.mem`

For "fun" you can also disassemble the ELF using:

`riscv64-elf-objdump -d ledblink.elf`

Intel HEX format:

`riscv64-elf-objcopy -O ihex ledblink.elf ledblink.hex`

### C/C++

### Rust


## UART

`picocom -b 115200 /dev/ttyUSB1 --imap lfcrlf,crcrlf --omap delbs,crlf --send-cmd "ascii-xfr -s -l 30 -n"`

