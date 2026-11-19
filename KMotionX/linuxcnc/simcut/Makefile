# kmotion_simcut: KMotionCNC's material removal engine (SimCut.cpp) as a Python module, for the Stock
# in a LinuxCNC preview (kmotion_stock.py).  See README.md.
#
#   make           kmotion_simcut.<python tag>.so here, from simcut_module.cc and the engine in
#                  ../../../PC VC Examples/KMotionCNC (copied to build/ first: SimCut.cpp includes
#                  "stdafx.h", which must find this folder's stand-in and not KMotionCNC's)
#   make check     imports the module and cuts a slot
#   make check-feed stock-demo.ngc through LinuxCNC's interpreter and kmotion_stock into the engine
#   make check-ui   the gmoccapy buttons and the Create tool dialog with stand-ins (needs a display), and the
#                  executed path drawn in an offscreen GL context (Mesa's EGL, software rendering)
#   make clean
KMOTIONX_DIR ?= $(abspath $(CURDIR)/../../..)
ENGINE_DIR := $(KMOTIONX_DIR)/PC VC Examples/KMotionCNC
EMPTY :=
SPACE := $(EMPTY) $(EMPTY)
ENGINE_DEP := $(subst $(SPACE),\$(SPACE),$(ENGINE_DIR))

CXX ?= g++
OPT ?= -O2
CXXFLAGS ?= $(OPT) -std=gnu++17 -Wall -Wno-unknown-pragmas -fPIC -fvisibility=hidden -pthread
PY_INC := $(shell python3 -m pybind11 --includes)
EXT := $(shell python3-config --extension-suffix)
MODULE := kmotion_simcut$(EXT)

all: $(MODULE)

build/SimCut.cpp: $(ENGINE_DEP)/SimCut.cpp
	mkdir -p build
	cp "$(ENGINE_DIR)/SimCut.cpp" "$(ENGINE_DIR)/SimCut.h" "$(ENGINE_DIR)/SimCutPort.h" build/
build/SimCut.h build/SimCutPort.h: build/SimCut.cpp
	@test -f $@ || cp "$(ENGINE_DIR)/$(notdir $@)" build/
build/SimCut.o: build/SimCut.cpp build/SimCut.h build/SimCutPort.h stdafx.h $(ENGINE_DEP)/SimCut.h $(ENGINE_DEP)/SimCutPort.h
	cp "$(ENGINE_DIR)/SimCut.h" "$(ENGINE_DIR)/SimCutPort.h" build/
	$(CXX) $(CXXFLAGS) -D_FILE_OFFSET_BITS=64 -I$(CURDIR) -c -o $@ build/SimCut.cpp
build/simcut_module.o: simcut_module.cc build/SimCut.h
	$(CXX) $(CXXFLAGS) $(PY_INC) -Ibuild -c -o $@ simcut_module.cc
$(MODULE): build/simcut_module.o build/SimCut.o
	$(CXX) -shared -pthread -o $@ $^

check: $(MODULE)
	python3 check.py

LINUXCNC_DIR ?= $(HOME)/linuxcnc-dev
check-feed: $(MODULE)
	LINUXCNC_DIR=$(LINUXCNC_DIR) python3 check_feed.py
check-ui: $(MODULE)
	LINUXCNC_DIR=$(LINUXCNC_DIR) python3 check_ui.py

clean:
	rm -rf build $(MODULE) __pycache__

.PHONY: all check check-feed check-ui clean
