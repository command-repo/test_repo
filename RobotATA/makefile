
# ************************* START USER DEFINED SECTION ************************
TARGET = RobotATA

LDSCRIPT = STM32F10X_MD_VL.ld

DEF = STM32F10X_HD
DEF += HSE_VALUE=8000000

SRCDIR = src
SRCDIR += src/common
SRCDIR += src/drv
SRCDIR += src/hal
SRCDIR += src/hal/CMSIS/core-support
SRCDIR += src/hal/CMSIS/device-support
SRCDIR += src/third_party

LIBDIR = 
LIB = 

LINKED_OBJ = 

FLASHER = jlink
#FLASHER = openocd
#FLASHER = ST-LINK_CLI

JLINK_PARAM = -device STM32F100RB
JLINK_PARAM += -if SWD
JLINK_PARAM += -speed auto

JLINK_PARAM_DEBUG = $(JLINK_PARAM)
JLINK_PARAM_DEBUG += 

OPENOCD_PARAM = -c "source [find interface/stlink-v1.cfg]"
OPENOCD_PARAM += -c "transport select hla_swd"
OPENOCD_PARAM += -c "source [find target/stm32f1x.cfg]"

OPENOCD_PARAM_DEBUG = $(OPENOCD_PARAM)
OPENOCD_PARAM_DEBUG += -c "gdb_port 3333"
OPENOCD_PARAM_DEBUG += -c "debug_level 2"
OPENOCD_PARAM_DEBUG += -c "set WORKAREASIZE 0x2000"
OPENOCD_PARAM_DEBUG += -c "reset_config srst_only"

OS = Windows

#OUTPUT = silent

# ************************** END USER DEFINED SECTION *************************
CC = arm-none-eabi-gcc
CPP = arm-none-eabi-g++
AS = arm-none-eabi-gcc
LD = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
OBJDUMP = arm-none-eabi-objdump
SIZE = arm-none-eabi-size --format=sysv

INCDIR = $(addprefix -I,$(SRCDIR))
OBJDIR = out/obj
BINDIR = out/bin
LSTDIR = out/lst

ELF = $(BINDIR)/$(TARGET).elf
HEX = $(BINDIR)/$(TARGET).hex
BIN = $(BINDIR)/$(TARGET).bin
MAP = $(LSTDIR)/$(TARGET).map
LSS = $(LSTDIR)/$(TARGET).lss

CSRC = $(wildcard $(addsuffix /*.c,$(SRCDIR)))
CPPSRC = $(wildcard $(addsuffix /*.cpp,$(SRCDIR)))
ASRC = $(wildcard $(addsuffix /*.s,$(SRCDIR)))
OBJS = $(addprefix $(OBJDIR)/,$(notdir $(CSRC:.c=.o) $(CPPSRC:.cpp=.o) $(ASRC:.s=.o)))

DEF := $(addprefix -D,$(DEF))
LIB := $(addprefix -l,$(LIB))

CFLAGS = $(DEF)
CFLAGS += $(INCDIR)
CFLAGS += -mcpu=cortex-m3
CFLAGS += -mthumb
CFLAGS += -Wa,-adhlns=$(addprefix $(LSTDIR)/, $(notdir $(addsuffix .lst,$(basename $<))))
CFLAGS += -MD
CFLAGS += -O0
CFLAGS += -std=gnu99
CFLAGS += -g -gdwarf-2
#CFLAGS += -pipe
CFLAGS += -ffunction-sections -fdata-sections
#CFLAGS += -Wall -Wextra -Wundef -Wcast-align -Winline
#CFLAGS += -mfpu=fpv4-sp-d16
#CFLAGS += -mfloat-abi=hard

CPPFLAGS = $(CFLAGS)
CPPFLAGS += -fno-exceptions
CPPFLAGS += -fno-rtti
CPPFLAGS += -funsigned-bitfields
CPPFLAGS += -fshort-enums

AFLAGS = $(CFLAGS)
AFLAGS += -x assembler-with-cpp

LDFLAGS = -T$(LDSCRIPT)
LDFLAGS += -L$(LIBDIR)
LDFLAGS += -mcpu=cortex-m3
LDFLAGS += -mthumb
LDFLAGS += -Wl,-Map="$(MAP)",--cref
LDFLAGS += -Wl,--gc-sections
#LDFLAGS += -Wl,--start-group
#LDFLAGS += -Wl,-lnosys
LDFLAGS += -nostartfiles
LDFLAGS += -Wl,--start-group $(LIB) -Wl,--end-group

vpath %.c $(SRCDIR)
vpath %.cpp $(SRCDIR)
vpath %.s $(SRCDIR)

ifeq ($(OUTPUT),silent)
.SILENT :
endif

.PHONY: all erase flash reset debug clean dirs

all:
	@echo - building $(TARGET)...
	$(MAKE) clean
	$(MAKE) dirs
	$(MAKE) $(ELF)
	$(MAKE) $(HEX)
	$(MAKE) $(BIN)
	$(MAKE) $(LSS)
	$(SIZE) $(ELF)
	@echo "Errors: none"

clean:
	@echo - cleaning $(OBJDIR), $(LSTDIR), $(BINDIR)...
ifeq ($(OS),Windows)
	($(OBJDIR):&(rd /s /q "$(OBJDIR)" 2> NUL))&
	($(LSTDIR):&(rd /s /q "$(LSTDIR)" 2> NUL))&
	($(BINDIR):&(rd /s /q "$(BINDIR)" 2> NUL))&
endif
ifeq ($(OS),Linux)
	-@rm -rf $(OBJDIR)
	-@rm -rf $(LSTDIR)
	-@rm -rf $(BINDIR)
endif

dirs:
	@echo - making dirs $(OBJDIR), $(LSTDIR), $(BINDIR)...
ifeq ($(OS),Windows)
	($(OBJDIR):&(mkdir "$(OBJDIR)" 2> NUL))&
	($(LSTDIR):&(mkdir "$(LSTDIR)" 2> NUL))&
	($(BINDIR):&(mkdir "$(BINDIR)" 2> NUL))&
endif
ifeq ($(OS),Linux)
	-@mkdir -p $(OBJDIR)
	-@mkdir -p $(LSTDIR)
	-@mkdir -p $(BINDIR)
endif

erase:
	@echo - erasing memory with $(FLASHER)...
ifeq ($(FLASHER),jlink)
	@echo erase>script.jlink
	@echo q>>script.jlink
	$(FLASHER) $(JLINK_PARAM) -CommanderScript script.jlink
endif
ifeq ($(FLASHER),ST-LINK_CLI)
	$(FLASHER) -c swd -me
endif

flash:
	@echo - programming with $(FLASHER)...
ifeq ($(FLASHER),openocd)
	$(FLASHER) -c program $(ELF) -c verify_image $(ELF) -c reset run -c exit
endif
ifeq ($(FLASHER),ST-LINK_CLI)
	$(FLASHER) -c swd -me -p $(HEX) -v -rst -run
endif
ifeq ($(FLASHER),jlink)
	@echo r>script.jlink
	@echo loadbin $(BIN), 0 >>script.jlink
	@echo r>>script.jlink
	@echo q>>script.jlink
	$(FLASHER) $(JLINK_PARAM) -CommanderScript script.jlink
endif

reset:
	@echo - resetting device...
ifeq ($(FLASHER),openocd)
	$(FLASHER) -c reset run -c shutdown -c exit
endif
ifeq ($(FLASHER),ST-LINK_CLI)
	$(FLASHER) -Rst
endif
ifeq ($(FLASHER),jlink)
	@echo r>script.jlink
	@echo q>>script.jlink
	$(FLASHER) $(JLINK_PARAM) -CommanderScript script.jlink
endif

debug:
	$(MAKE) all
	$(MAKE) flash
ifeq ($(FLASHER),openocd)
	@echo - openocd server is running...
	$(FLASHER) $(OPENOCD_PARAM_DEBUG)
endif
ifeq ($(FLASHER),jlink)
	jlinkGDBserverCL $(JLINK_PARAM_DEBUG)
endif

$(ELF): $(OBJS) $(LINKED_OBJ)
	@echo - linking...
	$(LD) $(LDFLAGS) $^ -o $@

$(HEX): $(ELF)
	@echo - making hex from $<...
	$(OBJCOPY) -O ihex $< $@

$(BIN): $(ELF)
	@echo - making bin from $<...
	$(OBJCOPY) -O binary $< $@

$(LSS): $(ELF)
	@echo - disassembling $<...
	$(OBJDUMP) -dC $< >> $@

$(OBJDIR)/%.o: %.c
	@echo - compiling $<...
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJDIR)/%.o: %.cpp
	@echo - compiling $<...
	$(CPP) $(CPPFLAGS) -c $< -o $@

$(OBJDIR)/%.o: %.s
	@echo - compiling $<...
	$(AS) $(AFLAGS) -c $< -o $@
