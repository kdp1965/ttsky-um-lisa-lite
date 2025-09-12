# Makefile
# See https://docs.cocotb.org/en/stable/quickstart.html for more info

# defaults
SIM ?= icarus
TOPLEVEL_LANG ?= verilog
SRC_DIR = $(PWD)/../src
PROJECT_SOURCES =  tt_um_lisa_lite.v
PROJECT_SOURCES += ram.v
PROJECT_SOURCES += lisa_core.v
PROJECT_SOURCES += lisa_dbg.v
PROJECT_SOURCES += lisa_div.v
PROJECT_SOURCES += lisa_periph.v
PROJECT_SOURCES += lisa_io_mux.v
PROJECT_SOURCES += lisa_qqspi.v
PROJECT_SOURCES += lisa_qspi_controller.v
PROJECT_SOURCES += lisa_rx8n.v
PROJECT_SOURCES += lisa_tx8n.v
PROJECT_SOURCES += debug_ctrl.v
PROJECT_SOURCES += debug_autobaud.v
PROJECT_SOURCES += debug_brg.v
PROJECT_SOURCES += debug_regs.v
PROJECT_SOURCES += data_cache8.v
PROJECT_SOURCES += inst_cache.v
PROJECT_SOURCES += lisa_uart.v
PROJECT_SOURCES += fops.v
PROJECT_SOURCES += lampFPU_div_top.sv
PROJECT_SOURCES += lampFPU_div.sv
PROJECT_SOURCES += lampFPU_fractDiv.sv
PROJECT_SOURCES += fops_ftoi.v
PROJECT_SOURCES += lampFPU_utils.v

ifneq ($(GATES),yes)

# RTL simulation:
SIM_BUILD				= sim_build/rtl
VERILOG_SOURCES += $(addprefix $(SRC_DIR)/,$(PROJECT_SOURCES))
VERILOG_SOURCES += psram.v spiflash.v

else

# Gate level simulation:
SIM_BUILD				= sim_build/gl
COMPILE_ARGS    += -DGL_TEST
COMPILE_ARGS    += -DFUNCTIONAL
COMPILE_ARGS    += -DUSE_POWER_PINS
COMPILE_ARGS    += -DSIM
COMPILE_ARGS    += -DUNIT_DELAY=\#1
VERILOG_SOURCES += $(PDK_ROOT)/sky130A/libs.ref/sky130_fd_sc_hd/verilog/primitives.v
VERILOG_SOURCES += $(PDK_ROOT)/sky130A/libs.ref/sky130_fd_sc_hd/verilog/sky130_fd_sc_hd.v

# this gets copied in by the GDS action workflow
VERILOG_SOURCES += $(PWD)/gate_level_netlist.v

endif

# Allow sharing configuration between design and testbench via `include`:
COMPILE_ARGS 		+= -I$(SRC_DIR)

# Include the testbench sources:
VERILOG_SOURCES += $(PWD)/tb.v
TOPLEVEL = tb

# MODULE is the basename of the Python test file
MODULE = test

# include cocotb's make rules to take care of the simulator setup
include $(shell cocotb-config --makefiles)/Makefile.sim
