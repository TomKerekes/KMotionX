# kmotion-motion: LinuxCNC's motion module as a userspace process (see ../README.md).
# Builds against a LinuxCNC source tree (run-in-place build): headers from its src/ and
# include/, libraries from its lib/. Point LINUXCNC_DIR at it.
LINUXCNC_DIR ?= $(HOME)/linuxcnc-dev
CXX ?= g++
CC ?= gcc
OPT ?= -g -O2

INC = -I$(LINUXCNC_DIR)/src -I$(LINUXCNC_DIR)/include -I$(LINUXCNC_DIR)/src/emc \
      -I$(LINUXCNC_DIR)/src/emc/motion -I$(LINUXCNC_DIR)/src/rtapi
DEFS = -DULAPI
LDFLAGS += -L$(LINUXCNC_DIR)/lib -Wl,-rpath,$(LINUXCNC_DIR)/lib
LIBS = -llinuxcnchal -lnml -lm -lpthread

# motion's error ring buffer helpers come from LinuxCNC's tree; its libraries don't export them
LCNC_C_SRCS = $(LINUXCNC_DIR)/src/emc/motion/emcmotutil.c \
              $(LINUXCNC_DIR)/src/emc/motion/dbuf.c \
              $(LINUXCNC_DIR)/src/emc/motion/stashf.c

OBJS = build/kmotion-motion.o build/kmotion-backend.o $(patsubst %.c,build/%.o,$(notdir $(LCNC_C_SRCS)))

# the KMotion side: KMotionX's headers and libraries (this file lives inside the KMotionX tree)
KMOTIONX_DIR ?= $(abspath $(CURDIR)/../../..)
KM_INC = -I$(KMOTIONX_DIR)/KMotionDLL -I$(KMOTIONX_DIR)/GCodeInterpreter -I$(KMOTIONX_DIR)/KMotionX/include \
         -I$(KMOTIONX_DIR)/DSP_KFLOP -I/usr/include/libftdi1
KM_DEFS = -D_KMOTIONX -DLIB_FTDI
KM_LIBS = -L$(KMOTIONX_DIR)/bin -Wl,-rpath,$(KMOTIONX_DIR)/bin -lGCodeInterpreter -lKMotion -lKMotionX

all: kmotion-motion

kmotion-motion: $(OBJS)
	$(CXX) -o $@ $(OBJS) $(LDFLAGS) $(LIBS) $(KM_LIBS)

build/kmotion-motion.o: kmotion-motion.cc kmotion-backend.h | build
	$(CXX) $(OPT) $(DEFS) $(INC) -Wall -Wextra -c -o $@ $<

build/kmotion-backend.o: kmotion-backend.cc kmotion-backend.h | build
	$(CXX) $(OPT) -std=c++20 $(KM_DEFS) $(KM_INC) -Wall -Wno-unknown-pragmas -Wno-overloaded-virtual -c -o $@ $<

build/%.o: $(LINUXCNC_DIR)/src/emc/motion/%.c | build
	$(CC) $(OPT) $(DEFS) $(INC) -std=gnu11 -c -o $@ $<

build:
	mkdir -p build

clean:
	rm -rf build kmotion-motion

.PHONY: all clean

# board-less check of the KMotion backend: the test program's moves through the planner
test: build/backend-test
	./build/backend-test 2>/dev/null | tail -14

build/backend-test: backend-test.cc build/kmotion-backend.o kmotion-backend.h | build
	$(CXX) $(OPT) -std=c++20 -I. -o $@ backend-test.cc build/kmotion-backend.o $(KM_LIBS) -lpthread

.PHONY: test
