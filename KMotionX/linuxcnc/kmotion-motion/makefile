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

OBJS = build/kmotion-motion.o $(patsubst %.c,build/%.o,$(notdir $(LCNC_C_SRCS)))

all: kmotion-motion

kmotion-motion: $(OBJS)
	$(CXX) -o $@ $(OBJS) $(LDFLAGS) $(LIBS)

build/kmotion-motion.o: kmotion-motion.cc | build
	$(CXX) $(OPT) $(DEFS) $(INC) -Wall -Wextra -c -o $@ $<

build/%.o: $(LINUXCNC_DIR)/src/emc/motion/%.c | build
	$(CC) $(OPT) $(DEFS) $(INC) -std=gnu11 -c -o $@ $<

build:
	mkdir -p build

clean:
	rm -rf build kmotion-motion

.PHONY: all clean
