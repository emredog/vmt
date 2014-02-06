# set the binaries that have to be built
TARGETS := extractFeatures

# set the build configuration set 
BUILD := release
#BUILD := debug

# set bin and build dirs
BUILDDIR := .build_$(BUILD)
BINDIR := $(BUILD)

# libraries 
#LDLIBS = $(addprefix -l, $(LIBS) $(LIBS_$(notdir $*))) 
LDLIBS := `pkg-config --libs opencv` -lboost_regex -lboost_system -lboost_filesystem -lboost_program_options -lavformat -lavutil -lavcodec -lswscale
#LIBS_hog2D := boost_python-mt

# set some flags and compiler/linker specific commands
CXXFLAGS = -pipe -D __STDC_CONSTANT_MACROS -D STD=std -Wall $(CXXFLAGS_$(BUILD)) -I. -I/usr/include/ffmpeg
CXXFLAGS_debug := -ggdb
CXXFLAGS_release := -O3 -DNDEBUG -ggdb
LDFLAGS = -pipe -Wall $(LDFLAGS_$(BUILD))
LDFLAGS_debug := -ggdb -g3
LDFLAGS_release := -O3 -g3

include make/generic.mk
