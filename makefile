# Compiler specification
CXX = g++
# Specifying compiler options
# Parallel calculation enabled in the lower row
CXXFLAGS = -Wall -Wextra -Wuninitialized -std=c++11
#CXXFLAGS = -Wunused -Wuninitialized -std=c++11 -fopenmp

RCXXFLAGS  = $(CXXFLAGS) -O3
DCXXFLAGS  = $(CXXFLAGS) -O0 -g

# Specifying include directories
INCDIR = -I./include -I./include/%
# Specifying a link to a library
LIBS = -lm
LDFLAGS =
# Specifying the extension of the source to be compiled
EXTENSION = cpp
# Target name to generate
RTARGET = release.exe
DTARGET = debug.exe
# Output root directory for generate and intermediate binary files
RTARGETDIR = ./bin/release
DTARGETDIR = ./bin/debug
ROBJECTDIR = ./obj/release
DOBJECTDIR = ./obj/debug
# Root directory of source files
SRCROOT = ./SourceFile

# List all files using the foreach command based on the source directory
SRCS = $(foreach dir, $(SRCROOT), $(wildcard $(dir)/*.$(EXTENSION)))
# Specify object file names in the same structure as the source directory
ROBJLIST = $(patsubst $(SRCROOT)/%.o, $(ROBJECTDIR)/%.o, $(patsubst %.$(EXTENSION), %.o, $(SRCS)))
DOBJLIST = $(patsubst $(SRCROOT)/%.o, $(DOBJECTDIR)/%.o, $(patsubst %.$(EXTENSION), %.o, $(SRCS)))

.PHONY: all build clean alldebug debugbuild debugclean 

all: clean build

build: $(RTARGET)

clean:
	rm -rf $(ROBJLIST) $(RTARGETDIR)/$(RTARGET)

alldebug: debugclean debugbuild

debugbuild: $(DTARGET)

debugclean:
	rm -rf $(DOBJLIST) $(DTARGETDIR)/$(DTARGET)

$(RTARGET): $(ROBJLIST)
	@echo "$^"
	@if [ ! -e $(RTARGETDIR) ]; then mkdir -p $(RTARGETDIR); fi
	$(CXX) $(RCXXFLAGS) -o $(RTARGETDIR)/$@ $^ $(LDFLAGS)

$(DTARGET): $(DOBJLIST)
	@echo "$^"
	@if [ ! -e $(DTARGETDIR) ]; then mkdir -p $(DTARGETDIR); fi
	$(CXX) $(DCXXFLAGS) -o $(DTARGETDIR)/$@ $^ $(LDFLAGS)

$(ROBJECTDIR)/%.o: $(SRCROOT)/%.$(EXTENSION)
	@if [ ! -e `dirname $@` ]; then mkdir -p `dirname $@`; fi
	$(CXX) $(RCXXFLAGS) $(LIBS) $(INCDIR) -o $@ -c $<

$(DOBJECTDIR)/%.o: $(SRCROOT)/%.$(EXTENSION)
	@if [ ! -e `dirname $@` ]; then mkdir -p `dirname $@`; fi
	$(CXX) $(DCXXFLAGS) $(LIBS) $(INCDIR) -o $@ -c $<