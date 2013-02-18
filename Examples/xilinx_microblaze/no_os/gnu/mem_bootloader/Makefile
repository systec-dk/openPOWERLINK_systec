################################################################################
# Project		: Bootloader (Xilinx)
# Module		: build system
# Autor			: mairt
# Date			: 23.07.2012
# File			: Makefile
# contents		: Controls the build of the bootoader
################################################################################

VERSION = 0.2

##################################
# variables inside makefiles.settings get overriden
include makefile.settings

##################################
# override variable for different app
# [pdi = dual processor, directIO = single processor]
APP_MODE= pdi

##################################
# directories
BUILD_DIR= build
SRC_DIR= src
OBJDIR= build

##################################
#general compile defines
CC=mb-gcc
AR=mb-ar
AS=mb-as
CP=cp
RM=rm
EXECUTABLE=mem_bootloader
LSSCRIPT=$(SRC_DIR)/lscript_$(BOARD_NAME)_$(BUS_INTERFACE)_$(APP_MODE)_$(PROCESSOR_NAME).ld
MACROS=
OPT_LEVEL=-Os
CFLAGS=-g3 -Wall -fmessage-length=0 $(PROC_FEATURES) $(MACROS)
CPPFLAGS=-MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)"
LDFLAGS=-Wl,-T -Wl,$(LSSCRIPT) $(PROC_FEATURES) -nostartfiles -Wl,-Map,$(BUILD_DIR)/$(EXECUTABLE).map

##################################
# Include directory

INCLUDES=-I$(SRC_DIR) \
-I$(BSP_PATH)/$(PROCESSOR_NAME)/include

##################################
# Source files
SRC_C_FILES=$(SRC_DIR)/main.c \
$(SRC_DIR)/spi.c \
$(SRC_DIR)/util.c


SRC_S_FILES= ${SRC_DIR}/crt3.s \
$(SRC_DIR)/crtinit.s \
$(SRC_DIR)/init.s

###################################
# set endian according to bus interface
ifeq ($(BUS_INTERFACE),axi)
	PROC_FEATURES+= -mlittle-endian
	ASFLAGS= -mlittle-endian
else
	PROC_FEATURES+= -mbig-endian
	ASFLAGS= -mbig-endian
endif

###################################
# libraries
LIBSPATH=-L$(BSP_PATH)/$(PROCESSOR_NAME)/lib

###################################
# objects
OBJ= $(addprefix $(OBJDIR)/,$(notdir $(patsubst %.s, %.o, $(SRC_S_FILES))))
OBJ+= $(addprefix $(OBJDIR)/,$(notdir $(patsubst %.c, %.o, $(SRC_C_FILES))))


OBJD = $(addprefix $(OBJDIR)/,$(notdir $(patsubst %.c, %.d, $(SRC_C_FILES))))

OUTPUT_OPTION = -o $@

CFLAGS += $(INCLUDES)


.PHONY:	header clean builddir

header:
	@echo ""
	@echo "================================================================================"
	@echo " Bootloader - Build System (Xilinx)"
	@echo "================================================================================"
	@echo
	@echo " Copyright (c) 2012 B&R"
	@echo " Version $(VERSION)"
	@echo "================================================================================"
	@echo ""
	@echo "Write 'make all' to build the bootloader"
	@echo ""
	@echo "Write 'make clean' to remove all generated files"
	@echo ""
	@echo "Change 'makefile.setting' to configure the build system"

###################################
# bootloader rules

all : builddir $(BUILD_DIR)/$(EXECUTABLE).elf $(BUILD_DIR)/$(EXECUTABLE).size $(BUILD_DIR)/$(EXECUTABLE).elfcheck

builddir:
	$(shell mkdir $(OBJDIR))

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c
	@echo Building file: $<
	@echo Invoking: MicroBlaze gcc compiler
	$(CC) -c $(OPT_LEVEL) $(CFLAGS) $(CPPFLAGS) $< $(OUTPUT_OPTION)
	@echo Finished building: $<
	@echo ' '

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.s
	@echo Building file: $<
	@echo Invoking: MicroBlaze assembler
	$(AS) $(ASFLAGS) $< $(OUTPUT_OPTION)
	@echo Finished building: $<
	@echo ' '

$(BUILD_DIR)/$(EXECUTABLE).elf : $(OBJ) $(LSSCRIPT)
	@echo Building target: $@
	@echo Invoking: MicroBlaze gcc linker
	$(CC) $(LDFLAGS) $(LIBSPATH) $(OBJ) -o $(BUILD_DIR)/$(EXECUTABLE).elf
	@echo Finished building target: $@
	@echo ' '

$(BUILD_DIR)/$(EXECUTABLE).size: $(BUILD_DIR)/$(EXECUTABLE).elf
	@echo Invoking: MicroBlaze Print Size
	mb-size $(BUILD_DIR)/$(EXECUTABLE).elf  |tee "$(BUILD_DIR)/$(EXECUTABLE).size"
	@echo Finished building: $@
	@echo ' '

$(BUILD_DIR)/$(EXECUTABLE).elfcheck: $(BUILD_DIR)/$(EXECUTABLE).elf
	@echo Invoking: Xilinx ELF Check
	elfcheck $(BUILD_DIR)/$(EXECUTABLE).elf -hw $(HW_PLATFORM_PATH)/system.xml \
	-pe $(PROCESSOR_NAME)  |tee "$(BUILD_DIR)/$(EXECUTABLE).elfcheck"
	@echo Finished building: $@
	@echo ' '

clean :
	-@touch $(BUILD_DIR)/$(EXECUTABLE)
	-$(RM) -rf $(OBJ) $(OBJD) $(BUILD_DIR)/$(EXECUTABLE).*

-include $(OBJD)
