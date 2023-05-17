
######################################
# target
######################################
TARGET = dyno_stm32


######################################
# building variables
######################################
# debug build?
DEBUG := 1

# optimization
ifeq ($(DEBUG), 1)
# Build path
BUILD_DIR = build/debug
OPT = -Og
else
# Build path
BUILD_DIR = build/release
OPT = -O2
endif

######################################
# source
######################################
# C sources
C_SOURCES = $(shell find -type f -name "*.c" | sort)

# ASM sources
ASM_SOURCES = startup_stm32f411xe.s


#######################################
# binaries
#######################################
PREFIX := arm-none-eabi-
# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable.
ifdef GCC_PATH
CC = $(GCC_PATH)/$(PREFIX)gcc
AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
SZ = $(GCC_PATH)/$(PREFIX)size
else
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S
 
#######################################
# CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m4

# fpu
FPU = -mfpu=fpv4-sp-d16

# float-abi
FLOAT-ABI = -mfloat-abi=hard

# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS = 

# C defines
C_DEFS =  -DUSE_HAL_DRIVER -DSTM32F411xE

# AS includes
AS_INCLUDES = 

# C includes
C_INCLUDES = $(shell find . -type f -iname "*.h" -printf "-I%h\n" | uniq | sort)
C_INCLUDES += -I/usr/arm-none-eabi/include


# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -Wno-unused-function -fdata-sections -ffunction-sections -fdiagnostics-color=always
CFLAGS += $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -Wno-unused-function -fdata-sections -ffunction-sections -fdiagnostics-color=always

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif


# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"


#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = STM32F411CEUx_FLASH.ld

# libraries
LIBS = -lc # -lnosys
LIBDIR = 
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections -Wl,--no-warn-rwx-segment -fdiagnostics-color=always
