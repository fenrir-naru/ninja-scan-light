PACKAGE = NAV4_C8051.hex

BIN_PATH = /usr/bin:/usr/local/bin
CXX = sdcc
AS = asx8051
CPPFLAGS = -V --use-stdout -D__SDCC__ -D__F380_VER__
CFLAGS = -V --debug --stack-auto --nooverlay --model-small --use-stdout -D__SDCC__ -D__F380_VER__ --opt-code-speed #-mmcs51
LFLAGS = -V --debug --use-stdout --stack-auto --model-small --iram-size 0x0100 --xram-loc 0x0800 --xram-size 0x0800 --code-size 0xfc00  #-mmcs51
ASFLAGS = -plosgff
INCLUDES = #-I
LIBS = #-L
BUILD_DIR = build_by_sdcc

SRCS_C = \
	$(shell ls *.c)
SRCS_ASM = \
	$(shell ls *.asm)
	
OBJS = $(SRCS_C:.c=.rel) $(SRCS_ASM:.asm=.rel)

all : $(BUILD_DIR) depend $(PACKAGE)

# ヘッダーファイルの依存関係
depend: $(SRCS_C)
	export PATH=$(BIN_PATH):$$PATH; \
	rm -f $(BUILD_DIR)/depend.inc; \
	for i in $^; do \
		$(CXX) -E -MMM $(INCLUDES) $(CPPFLAGS) $$i >> tempfile; \
		if ! [ $$? = 0 ]; then \
			rm -f tempfile; \
			exit 1; \
		fi; \
	done; \
	cat tempfile | sed -e 's/^+.*//g' -e 's/[^\.]*\.rel/$(BUILD_DIR)\/&/g' >> $(BUILD_DIR)/depend.inc; \
	rm -f tempfile

-include $(BUILD_DIR)/depend.inc

$(BUILD_DIR)/%.rel : %.c
	export PATH=$(BIN_PATH):$$PATH; \
	$(CXX) -c $(CFLAGS) $(INCLUDES) -o $@ $<

$(BUILD_DIR)/%.rel : %.asm
	export PATH=$(BIN_PATH):$$PATH; \
	cp $< $(BUILD_DIR)/; \
	cd $(BUILD_DIR); \
	$(AS) $(ASFLAGS) $<

$(PACKAGE) : $(patsubst %,$(BUILD_DIR)/%,$(PACKAGE))

$(BUILD_DIR)/$(PACKAGE) : $(patsubst %,$(BUILD_DIR)/%,$(OBJS))
	export PATH=$(BIN_PATH):$$PATH; \
	$(CXX) $(LFLAGS) $(INCLUDES) -o $@ $(LIBS) $^

$(BUILD_DIR) :
	mkdir $@

clean :
	rm -f $(BUILD_DIR)/*

run : all

.PHONY : clean all depend

