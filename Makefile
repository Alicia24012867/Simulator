CXX = g++
CXX_STD = c++17
CXX_FLAGS = -std=$(CXX_STD) -Wall -Wextra -I./include
SRC = ./src/main.cpp \
	  $(wildcard ./src/math/*.cpp) \
	  $(wildcard ./src/devices/*.cpp) \
	  $(wildcard ./src/core/*.cpp) \
	  $(wildcard ./src/io/*.cpp)
HEADERS = $(shell find ./include ./utils -type f 2>/dev/null)
TARGET = spice
TESTCASE_ROOT ?= testcase
OP_TESTCASE_DIR ?= $(TESTCASE_ROOT)/op
TRAN_TESTCASE_DIR ?= $(TESTCASE_ROOT)/tran
ACTUAL_DIR ?= actual
OP_ACTUAL_DIR ?= $(ACTUAL_DIR)/op
TRAN_ACTUAL_DIR ?= $(ACTUAL_DIR)/tran
STANDARD_ROOT ?= standard
OP_STANDARD_DIR ?= $(STANDARD_ROOT)/op
TRAN_STANDARD_DIR ?= $(STANDARD_ROOT)/tran
PYTHON ?= python3
OP_ABS_TOL ?= 1e-3
OP_REL_TOL ?= 2e-3
TRAN_ABS_TOL ?= 1e-7
TRAN_REL_TOL ?= 1e-4
TIME_ABS_TOL ?= 1e-15
OP_COMPARE_FLAGS ?=
TRAN_COMPARE_FLAGS ?=

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

.PHONY: all clean test test-io test-op test-tran compare compare-op compare-tran \
	check-eigen check-deps

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

$(TARGET): check-eigen $(SRC) $(HEADERS)
	$(CXX) $(CXX_FLAGS) $(EIGEN_FLAGS) -o $(TARGET) $(SRC)

test: test-io test-op test-tran

test-io: $(TARGET)
	@$(PYTHON) scripts/test_io.py ./$(TARGET)

test-op: $(TARGET)
	@rm -rf "$(OP_ACTUAL_DIR)"; \
	mkdir -p "$(OP_ACTUAL_DIR)"; \
	status=0; \
	for f in "$(OP_TESTCASE_DIR)"/*.cir; do \
		base=$${f##*/}; \
		base=$${base%.cir}; \
		out="$(OP_ACTUAL_DIR)/$$base.out"; \
		raw="$(OP_ACTUAL_DIR)/$$base.raw"; \
		err="$(OP_ACTUAL_DIR)/$$base.err"; \
		rm -f "$$out" "$$raw" "$$err"; \
		echo "Running OP $$base"; \
		./$(TARGET) -b -o "$$out" -r "$$raw" "$$f" 2> "$$err" || status=1; \
	done; \
	$(PYTHON) scripts/validate_raw.py \
		--analysis op \
		--listing-dir "$(OP_ACTUAL_DIR)" \
		"$(OP_ACTUAL_DIR)"/*.raw || status=1; \
	$(PYTHON) scripts/compare_spice.py \
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
	for f in "$(TRAN_TESTCASE_DIR)"/*.cir; do \
		base=$${f##*/}; \
		base=$${base%.cir}; \
		out="$(TRAN_ACTUAL_DIR)/$$base.out"; \
		raw="$(TRAN_ACTUAL_DIR)/$$base.raw"; \
		err="$(TRAN_ACTUAL_DIR)/$$base.err"; \
		rm -f "$$out" "$$raw" "$$err"; \
		echo "Running TRAN $$base"; \
		./$(TARGET) -b -o "$$out" -r "$$raw" "$$f" 2> "$$err" || status=1; \
	done; \
	$(PYTHON) scripts/validate_raw.py \
		--analysis tran \
		--listing-dir "$(TRAN_ACTUAL_DIR)" \
		"$(TRAN_ACTUAL_DIR)"/*.raw || status=1; \
	$(PYTHON) scripts/compare_spice.py \
		--analysis tran \
		--standard "$(TRAN_STANDARD_DIR)" \
		--actual "$(TRAN_ACTUAL_DIR)" \
		--atol "$(TRAN_ABS_TOL)" \
		--rtol "$(TRAN_REL_TOL)" \
		--time-atol "$(TIME_ABS_TOL)" \
		$(TRAN_COMPARE_FLAGS) || status=1; \
	exit $$status

compare: compare-op compare-tran

compare-op:
	@$(PYTHON) scripts/compare_spice.py \
		--analysis op \
		--standard "$(OP_STANDARD_DIR)" \
		--actual "$(OP_ACTUAL_DIR)" \
		--atol "$(OP_ABS_TOL)" \
		--rtol "$(OP_REL_TOL)" \
		--time-atol "$(TIME_ABS_TOL)" \
		$(OP_COMPARE_FLAGS)

compare-tran:
	@$(PYTHON) scripts/compare_spice.py \
		--analysis tran \
		--standard "$(TRAN_STANDARD_DIR)" \
		--actual "$(TRAN_ACTUAL_DIR)" \
		--atol "$(TRAN_ABS_TOL)" \
		--rtol "$(TRAN_REL_TOL)" \
		--time-atol "$(TIME_ABS_TOL)" \
		$(TRAN_COMPARE_FLAGS)

clean: 
	rm -f $(TARGET)
	rm -rf "$(ACTUAL_DIR)"
