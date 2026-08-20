CXX = g++
CXX_STD = c++17
OPT_FLAGS ?= -O3
CXX_FLAGS = -std=$(CXX_STD) $(OPT_FLAGS) -Wall -Wextra -I./include -I./third_party
SRC = ./src/main.cpp \
	  $(wildcard ./src/circuit/*.cpp) \
	  $(wildcard ./src/config/*.cpp) \
	  $(wildcard ./src/io/*.cpp) \
	  $(wildcard ./src/netlist/*.cpp)
HEADERS = $(shell find ./include -type f 2>/dev/null)
TARGET = spice
BUILD_DIR ?= build
OBJECTS = $(patsubst ./src/%.cpp,$(BUILD_DIR)/%.o,$(SRC))
DEPENDENCIES = $(OBJECTS:.o=.d)
UNIT_TEST_SOURCE ?= tests/unit/transient_analysis_test.cpp
UNIT_TEST_BUILD_DIR ?= tests/.build
UNIT_TEST_TARGET ?= $(UNIT_TEST_BUILD_DIR)/transient_analysis_test
TESTCASE_ROOT ?= tests/cases
TEST_ROOT ?= tests
OP_TESTCASE_DIR ?= $(TESTCASE_ROOT)/op
TRAN_TESTCASE_DIR ?= $(TESTCASE_ROOT)/tran
PRIVATE_TESTCASE_DIR ?= tests/private
ACTUAL_DIR ?= tests/output
OP_ACTUAL_DIR ?= $(ACTUAL_DIR)/op
TRAN_ACTUAL_DIR ?= $(ACTUAL_DIR)/tran
PTA_OUTPUT_ROOT ?= $(ACTUAL_DIR)/pta
PTA_MODE ?= disabled
PTA_OUTPUT_DIR ?= $(PTA_OUTPUT_ROOT)/$(PTA_MODE)
STANDARD_ROOT ?= tests/references
OP_STANDARD_DIR ?= $(STANDARD_ROOT)/op
TRAN_STANDARD_DIR ?= $(STANDARD_ROOT)/tran
TEST_SCRIPT_DIR ?= tests/scripts
CASE_RUNNER ?= $(TEST_SCRIPT_DIR)/run_cases.py
PYTHON ?= python3
OP_ABS_TOL ?= 5e-4
OP_REL_TOL ?= 1e-4
TRAN_ABS_TOL ?= 1e-4
TRAN_REL_TOL ?= 1e-3
TIME_ABS_TOL ?= 1e-15
OP_COMPARE_FLAGS ?=
TRAN_COMPARE_FLAGS ?=
PTA_COMPARE_FLAGS ?= $(OP_COMPARE_FLAGS)
PRIVATE_TIMEOUT ?= 120

UNAME_S := $(shell uname -s)

# Override: make EIGEN_INCLUDE=/path/to/eigen3
EIGEN_INCLUDE ?=

# pkg-config (typical on Linux; also works if eigen3.pc is installed elsewhere)
EIGEN_PKG_CFLAGS := $(shell pkg-config --cflags eigen3 2>/dev/null)

# Homebrew (macOS / Linuxbrew)
BREW_EIGEN_PREFIX := $(shell brew --prefix eigen 2>/dev/null)

# Search order: user path > pkg-config dirs > brew > common install locations
EIGEN_CANDIDATES :=
ifneq ($(strip $(EIGEN_INCLUDE)),)
EIGEN_CANDIDATES += $(EIGEN_INCLUDE)
endif
ifneq ($(strip $(EIGEN_PKG_CFLAGS)),)
EIGEN_CANDIDATES += $(patsubst -I%,%,$(filter -I%,$(EIGEN_PKG_CFLAGS)))
endif
ifneq ($(strip $(BREW_EIGEN_PREFIX)),)
EIGEN_CANDIDATES += $(BREW_EIGEN_PREFIX)/include/eigen3
endif
EIGEN_CANDIDATES += \
	/opt/homebrew/include/eigen3 \
	/usr/local/include/eigen3 \
	/usr/include/eigen3

EIGEN_DIR := $(firstword $(foreach d,$(EIGEN_CANDIDATES),$(if $(wildcard $(d)/Eigen/Core),$(d),)))

ifneq ($(EIGEN_DIR),)
EIGEN_FLAGS := -I$(EIGEN_DIR)
else ifneq ($(strip $(EIGEN_PKG_CFLAGS)),)
EIGEN_FLAGS := $(EIGEN_PKG_CFLAGS)
endif

.PHONY: all clean test test-unit test-config test-io test-cases test-op test-tran test-netlists test-private test-pta-hard-op compare compare-op \
	compare-tran generate-standards check-eigen check-deps pta pta-run pta-accuracy \
	pta-force-standard pta-force-disabled pta-fallback-standard

all: $(TARGET)

check-eigen:
	@if [ -z "$(EIGEN_FLAGS)" ]; then \
		echo "Error: Eigen3 headers not found."; \
		echo ""; \
		case "$(UNAME_S)" in \
			Darwin) \
				echo "  macOS:  brew install eigen"; \
				echo "  Or:     make EIGEN_INCLUDE=/path/to/eigen3" ;; \
			Linux) \
				echo "  Debian/Ubuntu:  sudo apt install libeigen3-dev"; \
				echo "  Fedora/RHEL:    sudo dnf install eigen3-devel"; \
				echo "  Arch:           sudo pacman -S eigen"; \
				echo "  Or:             make EIGEN_INCLUDE=/path/to/eigen3" ;; \
			MINGW*|MSYS*|CYGWIN*) \
				echo "  MSYS2:  pacman -S mingw-w64-x86_64-eigen"; \
				echo "  Or:     make EIGEN_INCLUDE=/path/to/eigen3" ;; \
			*) \
				echo "  Set:    make EIGEN_INCLUDE=/path/to/eigen3" ;; \
		esac; \
		exit 1; \
	fi
	@echo "Eigen OK ($(UNAME_S)): $(if $(EIGEN_DIR),$(EIGEN_DIR),$(strip $(EIGEN_PKG_CFLAGS)))"
	@echo '#include <Eigen/Core>' | \
		$(CXX) -std=$(CXX_STD) $(EIGEN_FLAGS) -x c++ - -c -o /dev/null || \
		(echo "Error: Eigen headers found but compile test failed."; exit 1)

check-deps: check-eigen

$(TARGET): $(OBJECTS) | check-eigen
	$(CXX) $(CXX_FLAGS) $(EIGEN_FLAGS) -o $(TARGET) $(OBJECTS)

$(BUILD_DIR)/%.o: ./src/%.cpp
	@mkdir -p "$(@D)"
	$(CXX) $(CXX_FLAGS) $(EIGEN_FLAGS) -MMD -MP -c -o "$@" "$<"

-include $(DEPENDENCIES)

$(UNIT_TEST_TARGET): $(UNIT_TEST_SOURCE) ./src/circuit/circuit.cpp \
	./src/circuit/nodeMap.cpp $(HEADERS) | check-eigen
	@mkdir -p "$(@D)"
	$(CXX) $(CXX_FLAGS) $(EIGEN_FLAGS) -o "$@" "$<" \
		./src/circuit/circuit.cpp ./src/circuit/nodeMap.cpp

test-unit: $(UNIT_TEST_TARGET)
	@"$(UNIT_TEST_TARGET)"

CONFIG_TEST_SOURCE ?= tests/unit/config_test.cpp
CONFIG_TEST_TARGET ?= $(UNIT_TEST_BUILD_DIR)/config_test
CONFIG_CLI_TEST_SCRIPT ?= $(TEST_SCRIPT_DIR)/test_config.py

$(CONFIG_TEST_TARGET): $(CONFIG_TEST_SOURCE) \
	./src/config/config.cpp ./src/config/overrides.cpp $(HEADERS)
	@mkdir -p "$(@D)"
	$(CXX) $(CXX_FLAGS) -o "$@" "$<" \
		./src/config/config.cpp ./src/config/overrides.cpp

test-config: $(CONFIG_TEST_TARGET) $(TARGET)
	@"$(CONFIG_TEST_TARGET)"
	@$(PYTHON) "$(CONFIG_CLI_TEST_SCRIPT)" ./$(TARGET)

test: $(TARGET) $(UNIT_TEST_TARGET) $(CONFIG_TEST_TARGET)
	@status=0; \
	"$(UNIT_TEST_TARGET)" || status=1; \
	$(MAKE) --no-print-directory test-config || status=1; \
	$(MAKE) --no-print-directory test-io || status=1; \
	$(MAKE) --no-print-directory test-op || status=1; \
	$(MAKE) --no-print-directory test-tran || status=1; \
	$(MAKE) --no-print-directory test-netlists || status=1; \
	exit $$status

test-io: $(TARGET)
	@$(PYTHON) $(TEST_SCRIPT_DIR)/test_io.py ./$(TARGET)

test-cases:
	@$(PYTHON) $(TEST_SCRIPT_DIR)/check_case_complexity.py

test-op: $(TARGET)
	@rm -rf "$(OP_ACTUAL_DIR)"; \
	mkdir -p "$(OP_ACTUAL_DIR)"; \
	status=0; \
	$(PYTHON) "$(CASE_RUNNER)" \
		--analysis op \
		--simulator "./$(TARGET)" \
		--case-dir "$(OP_TESTCASE_DIR)" \
		--output-dir "$(OP_ACTUAL_DIR)" || status=1; \
	$(PYTHON) $(TEST_SCRIPT_DIR)/validate_raw.py \
		--analysis op \
		--listing-dir "$(OP_ACTUAL_DIR)" \
		"$(OP_ACTUAL_DIR)"/*.raw || status=1; \
	$(PYTHON) $(TEST_SCRIPT_DIR)/compare_spice.py \
		--analysis op \
		--standard "$(OP_STANDARD_DIR)" \
		--actual "$(OP_ACTUAL_DIR)" \
		--atol "$(OP_ABS_TOL)" \
		--rtol "$(OP_REL_TOL)" \
		--time-atol "$(TIME_ABS_TOL)" \
		$(OP_COMPARE_FLAGS) || status=1; \
	exit $$status

test-tran: $(TARGET)
	@rm -rf "$(TRAN_ACTUAL_DIR)"; \
	mkdir -p "$(TRAN_ACTUAL_DIR)"; \
	status=0; \
	$(PYTHON) "$(CASE_RUNNER)" \
		--analysis tran \
		--simulator "./$(TARGET)" \
		--case-dir "$(TRAN_TESTCASE_DIR)" \
		--output-dir "$(TRAN_ACTUAL_DIR)" || status=1; \
	$(PYTHON) $(TEST_SCRIPT_DIR)/validate_raw.py \
		--analysis tran \
		--listing-dir "$(TRAN_ACTUAL_DIR)" \
		"$(TRAN_ACTUAL_DIR)"/*.raw || status=1; \
	$(PYTHON) $(TEST_SCRIPT_DIR)/compare_spice.py \
		--analysis tran \
		--standard "$(TRAN_STANDARD_DIR)" \
		--actual "$(TRAN_ACTUAL_DIR)" \
		--atol "$(TRAN_ABS_TOL)" \
		--rtol "$(TRAN_REL_TOL)" \
		--time-atol "$(TIME_ABS_TOL)" \
		$(TRAN_COMPARE_FLAGS) || status=1; \
	exit $$status

test-private: $(TARGET)
	@$(PYTHON) $(TEST_SCRIPT_DIR)/test_private_netlists.py \
		./$(TARGET) \
		$(PRIVATE_TESTCASE_DIR) \
		--timeout $(PRIVATE_TIMEOUT)

test-netlists: $(TARGET)
	@$(PYTHON) $(TEST_SCRIPT_DIR)/test_private_netlists.py \
		./$(TARGET) \
		$(TEST_ROOT) \
		--recursive \
		--timeout $(PRIVATE_TIMEOUT)

# This is a solver-differentiation benchmark, not a permanent release gate:
# improving ordinary Newton so it converges should prompt updating the fixture.
test-pta-hard-op: $(TARGET)
	@$(PYTHON) $(TEST_SCRIPT_DIR)/test_pta_hard_op.py \
		./$(TARGET) \
		$(TESTCASE_ROOT)/pta/nr_fail_cross_coupled_cmos_latch.cir \
		$(STANDARD_ROOT)/pta/nr_fail_cross_coupled_cmos_latch.out

# PTA tests intentionally use only OP decks.  Every mode is compared against
# the existing ngspice OP references; pta-run prints the end-to-end suite time.
# PTA is implemented as an experimental solver path.  Until its adaptive
# capacitor control and scaled derivative convergence test are complete,
# Force failures remain useful diagnostics rather than release regressions.
pta-run: $(TARGET)
	@case "$(PTA_MODE)" in \
		disabled|force|fallback) ;; \
		*) echo "Invalid PTA_MODE <$(PTA_MODE)>; expected disabled, force, or fallback"; exit 2 ;; \
	esac; \
	rm -rf "$(PTA_OUTPUT_DIR)"; \
	mkdir -p "$(PTA_OUTPUT_DIR)"; \
	start=$$($(PYTHON) -c 'import time; print(time.perf_counter())'); \
	status=0; count=0; \
	for netlist in "$(OP_TESTCASE_DIR)"/*.cir; do \
		if [ ! -f "$$netlist" ]; then \
			echo "No OP cases found in $(OP_TESTCASE_DIR)"; status=1; break; \
		fi; \
		name=$${netlist##*/}; name=$${name%.cir}; \
		"./$(TARGET)" --pta "$(PTA_MODE)" -b \
			-o "$(PTA_OUTPUT_DIR)/$$name.out" "$$netlist" \
			>/dev/null 2>"$(PTA_OUTPUT_DIR)/$$name.err" || status=1; \
		count=$$((count + 1)); \
	done; \
	finish=$$($(PYTHON) -c 'import time; print(time.perf_counter())'); \
	$(PYTHON) -c 'import sys; print("TIME PTA {:<8} suite ({} cases) {:.3f} ms".format(sys.argv[3], sys.argv[4], (float(sys.argv[2]) - float(sys.argv[1])) * 1000.0))' \
		"$$start" "$$finish" "$(PTA_MODE)" "$$count"; \
	exit $$status

pta-accuracy:
	@$(PYTHON) $(TEST_SCRIPT_DIR)/compare_spice.py \
		--analysis op \
		--standard "$(OP_STANDARD_DIR)" \
		--actual "$(PTA_OUTPUT_DIR)" \
		--atol "$(OP_ABS_TOL)" \
		--rtol "$(OP_REL_TOL)" \
		--time-atol "$(TIME_ABS_TOL)" \
		$(PTA_COMPARE_FLAGS)

# Force versus standard: accuracy.  The comparator is still run when Force
# fails so missing listings and stderr diagnostics are reported explicitly.
pta-force-standard:
	@status=0; \
	$(MAKE) --no-print-directory pta-run PTA_MODE=force || status=1; \
	$(MAKE) --no-print-directory pta-accuracy PTA_MODE=force || status=1; \
	exit $$status

# Force versus Disabled: each mode reports its suite time and is independently
# checked against the same ngspice OP reference set for accuracy.
pta-force-disabled:
	@status=0; \
	$(MAKE) --no-print-directory pta-run PTA_MODE=disabled || status=1; \
	$(MAKE) --no-print-directory pta-accuracy PTA_MODE=disabled || status=1; \
	$(MAKE) --no-print-directory pta-run PTA_MODE=force || status=1; \
	$(MAKE) --no-print-directory pta-accuracy PTA_MODE=force || status=1; \
	exit $$status

# Fallback versus standard: accuracy.
pta-fallback-standard:
	@status=0; \
	$(MAKE) --no-print-directory pta-run PTA_MODE=fallback || status=1; \
	$(MAKE) --no-print-directory pta-accuracy PTA_MODE=fallback || status=1; \
	exit $$status

pta:
	@status=0; \
	$(MAKE) --no-print-directory pta-force-standard || status=1; \
	$(MAKE) --no-print-directory pta-force-disabled || status=1; \
	$(MAKE) --no-print-directory pta-fallback-standard || status=1; \
	exit $$status

compare: compare-op compare-tran

generate-standards: test-cases
	@$(PYTHON) $(TEST_SCRIPT_DIR)/generate_ngspice_standards.py

compare-op:
	@$(PYTHON) $(TEST_SCRIPT_DIR)/compare_spice.py \
		--analysis op \
		--standard "$(OP_STANDARD_DIR)" \
		--actual "$(OP_ACTUAL_DIR)" \
		--atol "$(OP_ABS_TOL)" \
		--rtol "$(OP_REL_TOL)" \
		--time-atol "$(TIME_ABS_TOL)" \
		$(OP_COMPARE_FLAGS)

compare-tran:
	@$(PYTHON) $(TEST_SCRIPT_DIR)/compare_spice.py \
		--analysis tran \
		--standard "$(TRAN_STANDARD_DIR)" \
		--actual "$(TRAN_ACTUAL_DIR)" \
		--atol "$(TRAN_ABS_TOL)" \
		--rtol "$(TRAN_REL_TOL)" \
		--time-atol "$(TIME_ABS_TOL)" \
		$(TRAN_COMPARE_FLAGS)

clean:
	rm -f $(TARGET)
	rm -rf "$(BUILD_DIR)"
	rm -rf "$(ACTUAL_DIR)"
	rm -rf "$(UNIT_TEST_BUILD_DIR)"
