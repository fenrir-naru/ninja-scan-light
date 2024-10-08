# Copyright (c) 2015, M.Naruoka (fenrir)
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

PACKAGE = NinjaScanLight2.hex

BIN_PATH = /usr/bin:/usr/local/bin
CXX = sdcc
AS = sdas8051
CPPFLAGS = -V --use-stdout -D__SDCC__ -D__F380_VER__ -DNINJA_VER=200
override CFLAGS += -V --debug --stack-auto --nooverlay --model-small --use-stdout -D__SDCC__ -D__F380_VER__ -DNINJA_VER=200 --opt-code-speed #-mmcs51
LFLAGS = -V --debug --use-stdout --stack-auto --model-small --iram-size 0x0100 --xram-loc 0x0800 --xram-size 0x0800 --code-size 0xf800  #-mmcs51
ASFLAGS = -plosgff
MKFILE_DIR := $(patsubst %/,%,$(dir $(lastword $(MAKEFILE_LIST))))
SRC_DIR = $(MKFILE_DIR)/..
BUILD_DIR = build_by_sdcc
INCLUDES = -I$(SRC_DIR)
LIBS = #-L

SRCS_C = \
	$(filter-out %mpu6000.c %mag3110.c %ads122.c %all_sensors_elvr.c, $(shell ls $(SRC_DIR)/*.c || echo ''))
SRCS_ASM = \
	$(shell ls $(SRC_DIR)/*.asm || echo '')

ifeq ($(PITOT),ADS122)
  SRCS_C := $(SRCS_C) $(shell ls $(SRC_DIR)/ads122.c || echo '')
  override CPPFLAGS += -DI2C1_ADS122
  override CFLAGS += -DI2C1_ADS122
else ifeq ($(PITOT),ELVR)
  SRCS_C := $(SRCS_C) $(shell ls $(SRC_DIR)/all_sensors_elvr.c || echo '')
  override CPPFLAGS += -DI2C1_ELVR
  override CFLAGS += -DI2C1_ELVR
endif

OBJS = $(patsubst $(SRC_DIR)/%.c,$(BUILD_DIR)/%.rel, $(SRCS_C)) $(patsubst $(SRC_DIR)/%.asm,$(BUILD_DIR)/%.rel, $(SRCS_ASM))

all : $(BUILD_DIR) depend $(PACKAGE)

# ヘッダーファイルの依存関係
depend: $(SRCS_C)
	export PATH=$(BIN_PATH):$$PATH; \
	for i in $^; do \
		$(CXX) -E -MMM $(INCLUDES) $(CPPFLAGS) $$i >> tempfile; \
		if ! [ $$? = 0 ]; then \
			rm -f tempfile; \
			exit 1; \
		fi; \
	done; \
	cat tempfile | sed -e 's/^+.*//g' -e 's/[^\.]*\.rel/$(BUILD_DIR)\/&/g' > $(BUILD_DIR)/depend.inc; \
	rm -f tempfile

-include $(BUILD_DIR)/depend.inc

$(BUILD_DIR)/%.rel : $(SRC_DIR)/%.c
	export PATH=$(BIN_PATH):$$PATH; \
	$(CXX) -c $(CFLAGS) $(INCLUDES) -o $@ $<

$(BUILD_DIR)/%.rel : $(SRC_DIR)/%.asm
	export PATH=$(BIN_PATH):$$PATH; \
	cp $< $(BUILD_DIR)/; \
	cd $(BUILD_DIR); \
	$(AS) $(ASFLAGS) $<

$(PACKAGE) : $(patsubst %,$(BUILD_DIR)/%,$(PACKAGE))

$(BUILD_DIR)/$(PACKAGE) : $(OBJS)
	export PATH=$(BIN_PATH):$$PATH; \
	$(CXX) $(LFLAGS) $(INCLUDES) -o $@ $(LIBS) $^; \
	if [ -f $(basename $@) ]; then mv $(basename $@) $(basename $@).omf; fi

$(BUILD_DIR) :
	mkdir $@

clean :
	rm -f $(BUILD_DIR)/*

run : all

.PHONY : clean all depend

