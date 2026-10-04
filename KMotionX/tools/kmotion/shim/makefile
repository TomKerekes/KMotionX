# libkmx_c.so: the C interface the KMotion tool loads with ctypes. Built the way the
# LinuxCNC module is built against this tree's libraries.
KMOTIONX_DIR ?= $(abspath $(CURDIR)/../../../..)
CXX ?= g++
CXXFLAGS = -g -O2 -std=c++20 -fPIC -Wall -Wno-unknown-pragmas -Wno-overloaded-virtual \
           -D_KMOTIONX -DLIB_FTDI \
           -I$(KMOTIONX_DIR)/KMotionDLL -I$(KMOTIONX_DIR)/KMotionX/include -I$(KMOTIONX_DIR)/DSP_KFLOP -I/usr/include/libftdi1
LDFLAGS = -shared -L$(KMOTIONX_DIR)/bin -Wl,-rpath,$(KMOTIONX_DIR)/bin
LIBS = -lKMotion -lKMotionX -lpthread
TARGET = $(KMOTIONX_DIR)/bin/libkmx_c.so

all: $(TARGET)

$(TARGET): kmx_c.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< $(LDFLAGS) $(LIBS)

clean:
	rm -f $(TARGET)

.PHONY: all clean
