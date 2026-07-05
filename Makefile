CXX = g++
CXX_STD = c++17
CXX_FLAGS = -std=$(CXX_STD) -Wall -Wextra -I./include
SRC = ./src/main.cpp \
	  $(wildcard ./src/math/*.cpp) \
	  $(wildcard ./src/devices/*.cpp) \
	  $(wildcard ./src/core/*.cpp)
TARGET = spice
TESTCASE_DIR ?= testcase
ACTUAL_DIR ?= actual
STANDARD_DIR ?= standard
PYTHON ?= python3
OP_ABS_TOL ?= 1e-3
OP_REL_TOL ?= 2e-3
OP_COMPARE_FLAGS ?=

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

.PHONY: clean test compare check-eigen check-deps

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

$(TARGET):  check-eigen $(SRC)
	$(CXX) $(CXX_FLAGS) $(EIGEN_FLAGS) -o $(TARGET) $(SRC)

test: $(TARGET)
	@mkdir -p $(ACTUAL_DIR)
	@for f in $(TESTCASE_DIR)/*.cir; do \
		base=$$(basename "$$f" .cir); \
		out_dir="$(ACTUAL_DIR)/$$base"; \
		mkdir -p "$$out_dir"; \
		echo "Running $$base"; \
		./$(TARGET) "$$f" > "$$out_dir/$$base.out" 2> "$$out_dir/$$base.err"; \
	done
	@$(PYTHON) scripts/compare_op.py \
		--standard "$(STANDARD_DIR)" \
		--actual "$(ACTUAL_DIR)" \
		--atol "$(OP_ABS_TOL)" \
		--rtol "$(OP_REL_TOL)" \
		$(OP_COMPARE_FLAGS)

compare:
	@$(PYTHON) scripts/compare_op.py \
		--standard "$(STANDARD_DIR)" \
		--actual "$(ACTUAL_DIR)" \
		--atol "$(OP_ABS_TOL)" \
		--rtol "$(OP_REL_TOL)" \
		$(OP_COMPARE_FLAGS)

clean: 
	rm -f $(TARGET)
	rm -rf $(ACTUAL_DIR)/
