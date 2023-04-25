################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F7xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
gnss_ins.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
gnss_ins.list \

OBJCOPY_BIN += \
gnss_ins.bin \


# All Target
all: gnss_ins.elf secondary-outputs

# Tool invocations
gnss_ins.elf: $(OBJS) $(USER_OBJS) D:\KTUnas\ MA\Magistras\gnss_ins\STM32F732VETX_FLASH.ld
	arm-none-eabi-gcc -o "gnss_ins.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m7 -T"D:\KTUnas MA\Magistras\gnss_ins\STM32F732VETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="gnss_ins.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -u _printf_float -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

gnss_ins.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "gnss_ins.list"
	@echo 'Finished building: $@'
	@echo ' '

gnss_ins.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "gnss_ins.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
