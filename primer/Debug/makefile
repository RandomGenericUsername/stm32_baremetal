PROJECT_NAME := primer
LINKER_PATH := /home/inumaki/primerBackup/Startup/stm32_ls.ld

-include ../makefile.init
RM := rm -rf

-include sources.mk
-include Src/subdir.mk
-include Src/Drivers/subdir.mk
-include Src/Classes/subdir.mk
-include Startup/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
endif

BUILD_ARTIFACT_NAME := $(PROJECT_NAME)
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

EXECUTABLES += \
$(PROJECT_NAME).elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
$(PROJECT_NAME).list \

OBJCOPY_BIN += \
$(PROJECT_NAME).bin \

# All Target
all: main-build

# Main-build Target
main-build: $(PROJECT_NAME).elf

#-o "cplus.elf" @"objects.list" $(USER_OBJS) $(LIBS) 
# Tool invocations
$(PROJECT_NAME).elf: $(OBJS) $(USER_OBJS) $(LINKER_PATH) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-g++ -mcpu=cortex-m4 -T "$(LINKER_PATH)"  --specs=nosys.specs -Wl,-Map="$(PROJECT_NAME).map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -lstdc++ -lsupc++ -Wl,--end-group @"objects.list" $(USER_OBJS) $(LIBS) -o $@
	@echo 'Finished building target: $@'
	@echo ' '


# Other Targets
clean:
	-$(RM) $(OBJDUMP_LIST)$(C_UPPER_DEPS)$(S_DEPS)$(C_DEPS)$(OBJCOPY_BIN)$(CC_DEPS)$(SIZE_OUTPUT)$(C++_DEPS)$(EXECUTABLES)$(OBJS)$(CXX_DEPS)$(S_UPPER_DEPS)$(CPP_DEPS) $(PROJECT_NAME).elf $(PROJECT_NAME).map Src/*.d Src/*.su Src/Drivers/*.d Src/Drivers/*.su Src/Drivers/*.o Src/Classes/*.d Src/Classes/*.su Src/Classes/*.o Startup/*.su Startup/*.d
	-@echo ' '

flash: all
	/opt/xpack-openocd-0.11.0-4/bin/./openocd -f interface/stlink.cfg -f board/st_nucleo_f4.cfg -c "program $(PROJECT_NAME).elf verify reset exit"
	

.PHONY: all clean

-include ../makefile.targets




# deps	:= $(patsubst %.o,%.d,$(objs))
# -include $(deps)
# DEPFLAGS = -MMD -MF $(@:.o=.d)

# %.o: %.cpp
# 	$(CC) $(CFLAGS) $< $(DEPFLAGS)

# final.elf: 	$(objs)
# 	$(CC) $(LINKERFLAG) $^ -o $@


# dump-h:
# 	arm-none-eabi-objdump -h final.elf > log.txt

# clean:
# 	rm -rf $(objs) $(deps) *.map

# flash:all
# 	openocd -f interface/stlink-v2.cfg -f board/st_nucleo_f4.cfg -c "program final.elf verify reset exit"
# 	rm -rf *.o *.dat *.map *.txt
	
# gdb:
# 	arm-none-eabi-gdb 
