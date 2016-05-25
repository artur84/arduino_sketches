################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -f

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir.mk
-include libraries/ros_lib/tests/time_test/subdir.mk
-include libraries/ros_lib/tests/float64_test/subdir.mk
-include libraries/ros_lib/tests/array_test/subdir.mk
-include libraries/ros_lib/subdir.mk
-include core/core/subdir.mk
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
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(INO_DEPS)),)
-include $(INO_DEPS)
endif
ifneq ($(strip $(PDE_DEPS)),)
-include $(PDE_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
AR += \
arduino.ar \

ELF += \
MyBluetooth.elf \

EEP += \
MyBluetooth.eep \

FLASH_IMAGE += \
MyBluetooth.hex \


# All Target
all: MyBluetooth

# Tool invocations
MyBluetooth: $(FLASH_IMAGE)
	@echo 'Building target: $@'
	@echo 'Printing size:'
	"/home/arturo/ActiveSoftware/eclipse_mars/arduinoPlugin/tools/arduino/avr-gcc/4.8.1-arduino5/bin/avr-size" -A "/home/arturo/arduino_sketches/MyBluetooth/Release/MyBluetooth.elf"
	@echo 'Finished building target: $@'
	@echo ' '

arduino.ar: $(AR_OBJ)
	@echo 'Starting archiver'
	"/home/arturo/ActiveSoftware/eclipse_mars/arduinoPlugin/tools/arduino/avr-gcc/4.8.1-arduino5/bin/avr-ar" rcs  "/home/arturo/arduino_sketches/MyBluetooth/Release/arduino.ar"    arduino.ar $(AR_OBJ)
	@echo 'Finished building: $@'
	@echo ' '

MyBluetooth.elf: $(LINK_OBJ) $(AR)
	@echo 'Starting combiner'
	"/home/arturo/ActiveSoftware/eclipse_mars/arduinoPlugin/tools/arduino/avr-gcc/4.8.1-arduino5/bin/avr-gcc" -Os -Wl,--gc-sections -mmcu=atmega328p  -o "/home/arturo/arduino_sketches/MyBluetooth/Release/MyBluetooth.elf"   $(LINK_OBJ) $(AR)    "-L/home/arturo/arduino_sketches/MyBluetooth/Release" -lm
	@echo 'Finished building: $@'
	@echo ' '

MyBluetooth.eep: $(ELF)
	@echo 'Create eeprom image'
	"/home/arturo/ActiveSoftware/eclipse_mars/arduinoPlugin/tools/arduino/avr-gcc/4.8.1-arduino5/bin/avr-objcopy" -O ihex -j .eeprom --set-section-flags=.eeprom=alloc,load --no-change-warnings --change-section-lma .eeprom=0  "/home/arturo/arduino_sketches/MyBluetooth/Release/MyBluetooth.elf" "/home/arturo/arduino_sketches/MyBluetooth/Release/MyBluetooth.eep"
	@echo 'Finished building: $@'
	@echo ' '

MyBluetooth.hex: $(EEP)
	@echo 'Create Flash image (ihex format)'
	"/home/arturo/ActiveSoftware/eclipse_mars/arduinoPlugin/tools/arduino/avr-gcc/4.8.1-arduino5/bin/avr-objcopy" -O ihex -R .eeprom  "/home/arturo/arduino_sketches/MyBluetooth/Release/MyBluetooth.elf" "/home/arturo/arduino_sketches/MyBluetooth/Release/MyBluetooth.hex"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(C_UPPER_DEPS)$(PDE_DEPS)$(C_DEPS)$(AR)$(EEP)$(FLASH_IMAGE)$(CC_DEPS)$(AR_OBJ)$(C++_DEPS)$(LINK_OBJ)$(CXX_DEPS)$(ASM_DEPS)$(INO_DEPS)$(SIZEDUMMY)$(S_UPPER_DEPS)$(ELF)$(CPP_DEPS) MyBluetooth
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
