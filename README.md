# tiny-riscv

Verilog implementation of a basic RISC-V CPU

## Features



## Hardware

The CPU is initially designed to run on the Nandland Go Board, although most of the Verilog source is general in nature and easily ported to different hardware. The Lattice iCE40HX1K FPGA on the Go Board has a limited number of logic cells, and thus addition of additional hardware features such as UART, SPI, I2C, VGA, etc. will require a more capable FPGA.


## Building the Hardware

The project uses APIO (https://apiodoc.readthedocs.io/en/stable/index.html) in order to simplify installation of the required tools for building and testing.

Under the hood it (amoung others) rely on the following tools:

* Yosys (https://github.com/YosysHQ/yosys)
* nextpnr (https://github.com/YosysHQ/nextpnr)
* icestorm (https://github.com/YosysHQ/icestorm)

## Software to run on the CPU

### Assembly programming

### C/C++

### Rust


