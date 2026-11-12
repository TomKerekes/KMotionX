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

# the headers kmx_c.cpp read, written by the compiler: the library holds CKMotionDLL's layout, so
# a change in KMotionDLL.h rebuilds it (the makefile too: a tree built before this has no kmx_c.d)
DEPFILE = kmx_c.d

all: $(TARGET)

$(TARGET): kmx_c.cpp makefile
	$(CXX) $(CXXFLAGS) -MMD -MP -MF $(DEPFILE) -MT $@ -o $@ $< $(LDFLAGS) $(LIBS)

-include $(DEPFILE)

clean:
	rm -f $(TARGET) $(DEPFILE)

.PHONY: all clean
