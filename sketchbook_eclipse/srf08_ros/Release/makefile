################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir.mk
-include arduino/core/subdir.mk
-include Libraries/ros_lib/subdir.mk
-include Libraries/Wire/utility/subdir.mk
-include Libraries/Wire/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
AR += \
/home/arturo/arduino_sketches/sketchbook_eclipse/srf08_ros/Release/arduino.ar \

ELF += \
srf08_ros.elf \

EEP += \
srf08_ros.eep \

FLASH_IMAGE += \
srf08_ros.hex \


# All Target
all: srf08_ros

# Tool invocations
srf08_ros: $(FLASH_IMAGE)
	@echo 'Building target: $@'
	@echo 'Printing size:'
	"/home/arturo/arduino-1.6.0/hardware/tools/avr/bin/avr-size" -A "/home/arturo/arduino_sketches/sketchbook_eclipse/srf08_ros/Release/srf08_ros.elf"
	@echo 'Finished building target: $@'
	@echo ' '

/home/arturo/arduino_sketches/sketchbook_eclipse/srf08_ros/Release/arduino.ar: $(AR_OBJ)
	@echo 'Starting archiver'
	"/home/arturo/arduino-1.6.0/hardware/tools/avr/bin/avr-ar" rcs  "/home/arturo/arduino_sketches/sketchbook_eclipse/srf08_ros/Release/arduino.ar"   $(AR_OBJ)
	@echo 'Finished building: $@'
	@echo ' '

srf08_ros.elf: $(LINK_OBJ) $(AR)
	@echo 'Starting combiner'
	"/home/arturo/arduino-1.6.0/hardware/tools/avr/bin/avr-gcc" -w -Os -Wl,--gc-sections -mmcu=atmega328p  -o "/home/arturo/arduino_sketches/sketchbook_eclipse/srf08_ros/Release/srf08_ros.elf"   $(LINK_OBJ) $(AR)  "/home/arturo/arduino_sketches/sketchbook_eclipse/srf08_ros/Release/arduino.ar" "-L/home/arturo/arduino_sketches/sketchbook_eclipse/srf08_ros/Release" -lm
	@echo 'Finished building: $@'
	@echo ' '

srf08_ros.eep: $(ELF)
	@echo 'Create eeprom image'
	"/home/arturo/arduino-1.6.0/hardware/tools/avr/bin/avr-objcopy" -O ihex -j .eeprom --set-section-flags=.eeprom=alloc,load --no-change-warnings --change-section-lma .eeprom=0  "/home/arturo/arduino_sketches/sketchbook_eclipse/srf08_ros/Release/srf08_ros.elf" "/home/arturo/arduino_sketches/sketchbook_eclipse/srf08_ros/Release/srf08_ros.eep"
	@echo 'Finished building: $@'
	@echo ' '

srf08_ros.hex: $(EEP)
	@echo 'Create Flash image (ihex format)'
	"/home/arturo/arduino-1.6.0/hardware/tools/avr/bin/avr-objcopy" -O ihex -R .eeprom  "/home/arturo/arduino_sketches/sketchbook_eclipse/srf08_ros/Release/srf08_ros.elf" "/home/arturo/arduino_sketches/sketchbook_eclipse/srf08_ros/Release/srf08_ros.hex"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(ELF)$(C++_DEPS)$(AR)$(C_DEPS)$(CC_DEPS)$(FLASH_IMAGE)$(AR_OBJ)$(CPP_DEPS)$(LINK_OBJ)$(EEP)$(CXX_DEPS)$(C_UPPER_DEPS)$(SIZEDUMMY) srf08_ros
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
