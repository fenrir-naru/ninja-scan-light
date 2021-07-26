# Copyright (c) 2013, M.Naruoka (fenrir)
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without modification, 
# are permitted provided that the following conditions are met:
# 
# - Redistributions of source code must retain the above copyright notice, 
#   this list of conditions and the following disclaimer.
# - Redistributions in binary form must reproduce the above copyright notice, 
#   this list of conditions and the following disclaimer in the documentation 
#   and/or other materials provided with the distribution.
# - Neither the name of the naruoka.org nor the names of its contributors 
#   may be used to endorse or promote products derived from this software 
#   without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS 
# BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, 
# OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
# STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

PACKAGES = $(basename $(shell ls test_*.cpp))

BIN_PATH = /usr/bin:/usr/local/bin
CXX ?= g++
CPPFLAGS ?=
CFLAGS ?= $(CPPFLAGS) -Wall -Wno-parentheses # -Wno-sign-compare
LFLAGS =
INCLUDES = -I..
LIBS = -lm #-L
BUILD_DIR ?= build_GCC

SRCS_COMMON = $(filter-out $(addsuffix .cpp,$(PACKAGES)),$(shell ls *.cpp))
OBJS_COMMON = $(patsubst %.cpp,$(BUILD_DIR)/%.o,$(SRCS_COMMON))
SRCS_DEPEND = $(shell find $(PACKAGES) -name "*.cpp" 2>/dev/null)
OBJS_DEPEND = $(patsubst %.cpp,$(BUILD_DIR)/%.o,$(SRCS_DEPEND))
SRCS = $(addsuffix .cpp,$(PACKAGES)) $(SRCS_COMMON) $(SRCS_DEPEND)

BUILD_DIRS = $(sort $(BUILD_DIR) $(dir $(OBJS_COMMON) $(OBJS_DEPEND)))

all : $(BUILD_DIRS) packages

# Header file dependencies @see http://lists.gnu.org/archive/html/automake/2002-01/msg00155.html
# and package specific objects
$(BUILD_DIR)/depend.inc: $(SRCS) makefile
	mkdir -p $(dir $@); \
	for i in $(SRCS); do \
		$(CXX) -E -MM -MP $(INCLUDES) $(CPPFLAGS) $$i >> tempfile; \
		if ! [ $$? = 0 ]; then \
			rm -f tempfile; \
			exit 1; \
		fi; \
	done; \
	cat tempfile | sed -e 's/^+.*//g' -e "s/[^\.]\+\.o: \([^\/]\+\/\)\?[^\.]\+\.cpp/\$$(BUILD_DIR)\/\1&/g" > $@; \
	for i in $(PACKAGES); do \
		echo "\$$(BUILD_DIR)/$$i.out : \$$(addprefix \$$(BUILD_DIR)/,\$$(filter $$i%,$(SRCS_DEPEND:.cpp=.o)))" >> $@; \
	done; \
	rm -f tempfile

-include $(BUILD_DIR)/depend.inc

$(BUILD_DIR)/%.h.gch : ../%.h
	mkdir -p $(@D)
	$(CXX) $(CFLAGS) $(INCLUDES) -o $@ $<

$(BUILD_DIR)/%.o :
	$(MAKE) $(addsuffix .gch,$(addprefix $(BUILD_DIR)/,$(patsubst ../%.h,%.h,$(filter ../%.h, $^))))
	$(CXX) -c $(CFLAGS) -I$(BUILD_DIR) $(INCLUDES) -o $@ $<

$(BUILD_DIR)/%.out : $(BUILD_DIR)/%.o $(OBJS_COMMON)
	$(CXX) $(LFLAGS) $(INCLUDES) -o $@ $^ $(LIBS)

packages : $(patsubst %,$(BUILD_DIR)/%.out,$(PACKAGES))
	for f in $^; do ./$$f; done

$(BUILD_DIRS) :
	mkdir -p $@

clean :
	rm -rf $(BUILD_DIR)/*

run : all

.PHONY : clean all packages
