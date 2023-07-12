# tiny-riscv

Verilog implementation of a basic RISC-V CPU.

> :warning: **Warning: This project is a work in progress**

The CPU in this repository is mainly intended to serve as a learning tool for people interested in the internal operations of a CPU, or for people who would like to design their own CPU.

## Features



## Hardware

The CPU is initially designed to run on the Nandland Go Board, although most of the Verilog source is general in nature and easily ported to different hardware. The Lattice iCE40HX1K FPGA on the Go Board has a limited number of logic cells, and thus addition of additional hardware features such as UART, SPI, I2C, VGA, etc. will require a more capable FPGA.

TODO: Find a nice board with a more capable FPGA and add support for it.

## Installation of the required tools

I **strongly** recommend that you use a GNU/Linux system, although I am sure it is possible to get this working in Windows or MacOS.

The project uses APIO (https://apiodoc.readthedocs.io/en/stable/index.html) in order to simplify installation of the required tools for building and testing. Manual installation of each required tool is also possible, but unless packages are available in your favorite Linux distro the installation can be a pain to complete.

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

Under the hood it (among others) rely on the following tools:

* Yosys (https://github.com/YosysHQ/yosys)
* nextpnr (https://github.com/YosysHQ/nextpnr)
* icestorm (https://github.com/YosysHQ/icestorm)

## Software to run on the CPU
 One of the nice things about implementing a standardized instruction set (such as RISC-V CPU), as opposed to inventing your own instruction set is that assemblers and compilers already exists for several programming languages.

### Assembly programming

### C/C++

### Rust



