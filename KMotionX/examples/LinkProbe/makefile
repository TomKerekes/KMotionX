# LinkProbe: timing of the PC-to-board link (USB KFLOP, Ethernet Kogna), through KMotionServer and
# raw. Built into bin/ by the top-level make; run none of them while LinuxCNC or the KMotion tool
# runs (they share the board through the server). From the KMotionX checkout:
#   bin/linkprobe    GetStatus and Version round trips, alone and against a thread polling status
#                    the way kmotion-motion's poller does (how long a command waits)
#   bin/linkprobe2   one status read taken apart: the server pipe, the USB write, each reply line
#   bin/linkprobe3   how often another client holds the board's token; a burst of 5 GetStatus
#   bin/rawusb       the KFLOP through libftdi alone, no server: <latency_ms> <eventchar 0|1> <burst>
#                    (stop KMotionServer first: close the tool and LinuxCNC, pkill -x KMotionServer)
#   bin/linktest     the kmotion-motion backend in board mode, machine off: a board call's wait
#                    behind the poller (needs kmotion-motion built: its kmotion-backend.o)
# Measured on Carl's laptop VM (VirtualBox OHCI, 2026-10-29): a status read 55 ms, a Version 12 ms,
# a command behind the old poller up to 1.5 s - see vm-build-notes section 50.
include $(BUILD_ROOT)/config.mak
BIN=$(BUILD_ROOT)/bin
CXX?=g++
FLAGS=-O2 -std=c++20 -D_KMOTIONX -DLIB_FTDI -I$(BUILD_ROOT)/KMotionDLL -I$(BUILD_ROOT)/GCodeInterpreter \
      -I$(BUILD_ROOT)/KMotionX/include -I$(BUILD_ROOT)/DSP_KFLOP -I/usr/include/libftdi1 \
      -Wno-unknown-pragmas -Wno-overloaded-virtual
LIBS=-L$(BIN) -Wl,-rpath,$(BIN) -lGCodeInterpreter -lKMotion -lKMotionX -lpthread
BACKEND=$(BUILD_ROOT)/KMotionX/linuxcnc/kmotion-motion/build/kmotion-backend.o

all: $(BIN)/linkprobe $(BIN)/linkprobe2 $(BIN)/linkprobe3 $(BIN)/rawusb linktest

$(BIN)/linkprobe: probe.cc
	$(CXX) $(FLAGS) -o $@ $< $(LIBS)
$(BIN)/linkprobe2: probe2.cc
	$(CXX) $(FLAGS) -o $@ $< $(LIBS)
$(BIN)/linkprobe3: probe3.cc
	$(CXX) $(FLAGS) -o $@ $< $(LIBS)
$(BIN)/rawusb: rawusb.cc
	$(CXX) -O2 -std=c++20 -I/usr/include/libftdi1 -o $@ $< -lftdi1
# the backend's object exists once kmotion-motion has been built (install-vm.sh builds it after this tree)
linktest: linktest.cc
	@if [ -f $(BACKEND) ]; then $(CXX) $(FLAGS) -I$(BUILD_ROOT)/KMotionX/linuxcnc/kmotion-motion -o $(BIN)/linktest $< $(BACKEND) $(LIBS); \
	else echo "linktest: kmotion-motion is not built yet (make -C KMotionX/linuxcnc/kmotion-motion), skipped"; fi

clean:
	rm -f $(BIN)/linkprobe $(BIN)/linkprobe2 $(BIN)/linkprobe3 $(BIN)/rawusb $(BIN)/linktest

.PHONY: all clean linktest
