ODIR=build
BINDIR=$(BUILD_ROOT)/bin
LD_LIB_PATH+=$(BINDIR)
#LD_LIB_PATH+=../bin
# add KMotionX include first
IDIR_ALL=$(BUILD_ROOT)/KMotionX/include $(BUILD_ROOT)/DSP_KFLOP $(IDIR)
IFLAGS+=$(addprefix -I,$(IDIR_ALL))

CFLAGS+=-g -c -fmessage-length=0 -fPIC -MMD -MP
CFLAGS+=-std=c++20
#exceptions on warnings default
CFLAGS+=-Wall
# style only, in sources shared with the Windows tree, which stay as they are there
CFLAGS+=-Wno-misleading-indentation -Wno-format-truncation -Wno-sign-compare
#wflag set by others might exclude specific warnings
CFLAGS+=$(W_FLAGS)
CFLAGS+=$(IFLAGS)
CFLAGS+=$(DEFS)

ifeq ($(BIN_TYPE),$(LIBEXT))
ifeq ($(OSNAME),Linux)
LDFLAGS+=-shared
else ifeq ($(OSNAME),Darwin)
LDFLAGS+=-dynamiclib
LDFLAGS+=-install_name '$(kmxdir)/$(EXECUTABLE)'
endif
endif
LDFLAGS+=$(addprefix -l,$(LD_LIBS))
LDFLAGS+=$(addprefix -L,$(LD_LIB_PATH))
LDFLAGS+=-Wl,-rpath,$(abspath $(kmxdir))   # abspath, not realpath: realpath is empty while the install dir does not exist yet (a clean machine before make install) - the binaries then had no runpath and KMotionServer could not find libKMotionX.so
#LDFLAGS+=-Wl,-rpath $(realpath $(BINDIR))
#LDFLAGS+=-Wl,-rpath,$$ORIGIN
#LDFLAGS+=-Wl,-z,origin




# Make sure the output directory exists.
dummy := $(shell test -d $(BINDIR) || mkdir -p $(BINDIR))

 
OBJECTS=$(SOURCES:%.cpp=$(ODIR)/%.o)

# the file in bin/ is the target: with its bare name as the target, a file make never found,
# every make linked every library and program again
all: $(BINDIR)/$(EXECUTABLE)

# the dependency files -MMD writes: without them a header change rebuilt nothing, and kmxWeb's
# objects from before CoordMotion.h grew read the library pointer 72 bytes off (it crashed at start).
# After all: the first rule read is what a plain make builds, and above it that was the first
# object of the first .d file - a tree that had been built once compiled one file and linked nothing
-include $(OBJECTS:.o=.d)

$(ODIR):
	mkdir $@
	
# the objects' directory first, whatever the order the jobs of a make -j start in
$(OBJECTS): | $(ODIR)

$(BINDIR)/$(EXECUTABLE): $(OBJECTS) 
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@

#GNU style	
$(ODIR)/%.o: %.cpp
	$(CC) $(CFLAGS) $< -o $@

$(ODIR)/%.o: %.CPP
	$(CC) $(CFLAGS) $< -o $@
	

clean:
	@echo "Cleaning... $(shell pwd)"
	rm -rf $(ODIR)
	rm -f $(BINDIR)/$(EXECUTABLE)

.PHONY: clean