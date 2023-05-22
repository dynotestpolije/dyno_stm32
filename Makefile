#
###################################################################################################
## @file           : Makefile
## @brief          : GNU Make Build configuration
###################################################################################################
##
## Copyright (c) 2023 Rizal Achmad Pahlevi <echo 'cml6YWwuYWhtYWRwQGdtYWlsLmNvbQo=' | base64 -d>
## All rights reserved.
##
## This software is licensed under terms that can be found in the LICENSE file
## in the root directory of this software component.
## If no LICENSE file comes with this software, it is provided AS-IS.
##
###################################################################################################
#
include tools/config.mk


# default action: build all
all: check options build

build: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

debug:
	@make -j build DEBUG=1 

release: 
	@make -j build DEBUG=0

debug_clean:
	@make clean DEBUG=1

release_clean:
	@make clean DEBUG=0

check: 
	@tools/checkexec.sh $(CC) "compiler and assembler"
	@tools/checkexec.sh $(CP) "compiler and assembler"
	@tools/checkexec.sh $(SZ) "compiler and assembler"


#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@	
	
$(BUILD_DIR):
	@mkdir -p $@		

#######################################
# clean up
#######################################
clean:
	@rm -fR $(BUILD_DIR) ./compile_commands.json

#######################################
# options
#######################################
options:
	@echo all dev release clean:
	@echo "TARGET    = $(TARGET)"
	@echo "MCU       = $(MCU)"
	@echo "DEBUG     = $(DEBUG)"
	@echo ""
	@echo "FLAGS: "
	@echo "- ASFLAGS = $(ASFLAGS)"
	@echo "- LDFLAGS = $(LDFLAGS)"
	@echo "- CFLAGS  = $(CFLAGS)"
	@echo ""
	@echo "tools: "
	@echo "- CC      = $(CC)"
	@echo "- AS      = $(AS)"
	@echo "- CP      = $(CP)"
	@echo "- SZ      = $(SZ)"


.PHONY: check options build  $(BUILD_DIR)

#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)

# *** EOF ***
