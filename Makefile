# Makefile for the tiny RISCV processor

PROJECT_NAME = tiny_riscv
SIM = iverilog
SIM_RUNTIME = vvp
SYNTH = yosys
PNR = nextpnr-ice40
CPU_FREQ = 25
PNR_ARGS = --force --timing-allow-fail --pcf-allow-unconstrained --opt-timing --freq $(CPU_FREQ)
PACK = icepack
PROG = iceprog
TIME = icetime
WAVE_VIEWER = gtkwave

OUTDIR = build
SOURCEDIR = rtl
BENCHDIR = simulation
PIN_DEF = pcf/go-board.pcf
DEVICE = hx1k
PACKAGE = vq100

#VERILOG_SOURCES =\
#	tiny_riscv_processor.v \
#	tiny_riscv_memory.v

VERILOG_TESTBENCH_SOURCES = simulation/tiny_riscv_tb.v

VERILOG_SOURCES := $(wildcard $(SOURCEDIR)/*.v)
VERILOG_SOURCES += $(wildcard $(SOURCEDIR)/cpu/*.v)
VERILOG_SOURCES += $(wildcard $(SOURCEDIR)/modules/*.v)
#VERILOG_SOURCES += $(wildcard $(SOURCEDIR)/hw_specific/*.v)
$(info $$VERILOG_SOURCES is [${VERILOG_SOURCES}])

all: $(OUTDIR)/$(PROJECT_NAME).bin $(OUTDIR)/$(PROJECT_NAME).rpt

sim: $(OUTDIR)/$(PROJECT_NAME)_tb.vcd

wave: $(OUTDIR)/$(PROJECT_NAME)_tb.vcd
	$(WAVE_VIEWER) $(OUTDIR)/$(PROJECT_NAME)_tb.vcd gtkwave_state_instr.gtkw &

synth: $(OUTDIR)/$(PROJECT_NAME).json

# Synth
$(OUTDIR)/$(PROJECT_NAME).json: $(VERILOG_SOURCES)
	$(SYNTH) -p 'synth_ice40 -relut -json $@ -top $(PROJECT_NAME)_top' $^

# Build simulation
$(OUTDIR)/$(PROJECT_NAME)_tb: $(VERILOG_SOURCES) $(VERILOG_TESTBENCH_SOURCES)
	$(SIM) -o $@ $^ -DTESTBENCH

# Run simulation
$(OUTDIR)/$(PROJECT_NAME)_tb.vcd: $(OUTDIR)/$(PROJECT_NAME)_tb
	$(SIM_RUNTIME) -N $< +vcd=$@

# Generate timing estimates
$(OUTDIR)/$(PROJECT_NAME).rpt: $(OUTDIR)/$(PROJECT_NAME).asc
	$(TIME) -d $(DEVICE) -mtr $@ $<

# Place and route
$(OUTDIR)/$(PROJECT_NAME).asc: $(OUTDIR)/$(PROJECT_NAME).json
	$(PNR) $(PNR_ARGS) --$(DEVICE) --package $(PACKAGE) --pcf $(PIN_DEF) --json $< --asc $@

# Pack
$(OUTDIR)/$(PROJECT_NAME).bin: $(OUTDIR)/$(PROJECT_NAME).asc
	$(PACK) $< $@

# Program the FPGA
prog: $(OUTDIR)/$(PROJECT_NAME).bin
	$(PROG) $<

verilator:
	verilator -DTESTBENCH -DBOARD_FREQ=12 -Wno-fatal --top-module tiny_riscv_top -cc -exe verilator/main.cpp $(VERILOG_SOURCES)
	cd obj_dir; make -f Vtiny_riscv_top.mk
	
#rtl/tiny_riscv_top.v rtl/cpu/tiny_riscv_processor.v rtl/cpu/tiny_riscv_memory.v rtl/modules/uart.v

# Clean all the build artifacts
clean:
	rm $(PROJECT_NAME).json $(PROJECT_NAME).asc $(PROJECT_NAME).bin

.PHONY: all sim synth wave prog clean verilator
