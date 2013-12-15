# MSP430 Makefile
# #####################################
#
# Part of the uCtools project
# uctools.github.com
#
#######################################
# user configuration:
#######################################
# TARGET: name of the output file
TARGET = c-i2c-test
# MCU: part number to build for
MCU = msp430f5528
# SOURCES: list of input source sources
SOURCES = i2c.c
# INCLUDES: list of includes, by default, use Includes directory
INCLUDES = -IInclude
# OUTDIR: directory to use for output
OUTDIR = Debug
# define flags
CFLAGS = -mmcu=$(MCU) -g -Os -Wall -Wunused $(INCLUDES)
ASFLAGS = -mmcu=$(MCU) -x assembler-with-cpp -Wa,-gstabs
LDFLAGS = -mmcu=$(MCU) -Wl,-Map=$(OUTDIR)/$(TARGET).map
#######################################
# end of user configuration
#######################################
#
#######################################
# binaries
#######################################
CC			= msp430-gcc
LD			= msp430-ld
AR			= msp430-ar
AS			= msp430-gcc
NM			= msp430-nm
OBJCOPY		= msp430-objcopy
OBJDUMP		= msp430-objdump
SIZE		= msp430-size
RM			= rm -f
MKDIR		= mkdir -p
#######################################

# file that includes all dependencies
DEPEND = $(SOURCES:.c=.d)

# list of object files, placed in the build directory regardless of source path
OBJECTS = $(addprefix $(OUTDIR)/,$(notdir $(SOURCES:.c=.o)))

# default: build hex file
all: $(OUTDIR)/$(TARGET).hex $(OUTDIR)/$(TARGET).lst

# intel hex file
$(OUTDIR)/%.hex: $(OUTDIR)/%.elf
		$(OBJCOPY) -O ihex $< $@

# elf file
$(OUTDIR)/$(TARGET).elf: $(OBJECTS)
		$(CC) $(OBJECTS) $(LDFLAGS) $(LIBS) -o $@

$(OUTDIR)/%.o: %.c | $(OUTDIR)
		$(CC) -c $(CFLAGS) -o $@ $<

# assembly listing
#%.lst: %.c
#		$(CC) -c $(ASFLAGS) -Wa,-anlhd $< > $@
$(OUTDIR)/%.lst: $(OUTDIR)/%.elf
		$(OBJDUMP) -DS $< > $@
		$(SIZE) $<

# create the output directory
$(OUTDIR):
		$(MKDIR) $(OUTDIR)

# remove build artifacts and executables
clean:
		-$(RM) $(OUTDIR)/*

.PHONY: all clean