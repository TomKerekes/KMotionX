include $(BUILD_ROOT)/config.mak

W_FLAGS+=-Wno-unknown-pragmas

SOURCES=KMotionServerX.cpp
IDIR+=$(BUILD_ROOT)/KMotionDLL

LD_LIBS+=KMotionX
LD_LIBS+=KMotion 

EXECUTABLE=KMotionServer
BIN_TYPE=$(EXESUF)

include $(BUILD_ROOT)/include.mk





# the token hand-over rules (BoardGate.h), driven with a fake clock: no board, no server
.PHONY: check
check: gatetest.cpp BoardGate.h
	mkdir -p build && $(CXX) -std=c++11 -pthread -Wall -o build/gatetest gatetest.cpp && ./build/gatetest
