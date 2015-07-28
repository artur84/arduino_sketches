################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
/home/arturo/arduino-1.6.0/hardware/arduino/avr/libraries/Wire/Wire.cpp 

CPP_DEPS += \
./Libraries/Wire/Wire.cpp.d 

LINK_OBJ += \
./Libraries/Wire/Wire.cpp.o 


# Each subdirectory must supply rules for building sources it contributes
Libraries/Wire/Wire.cpp.o: /home/arturo/arduino-1.6.0/hardware/arduino/avr/libraries/Wire/Wire.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"/home/arturo/arduino-1.6.0/hardware/tools/avr/bin/avr-g++" -c -g -Os -w -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=160 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR     -I"/home/arturo/arduino-1.6.0/hardware/arduino/avr/cores/arduino" -I"/home/arturo/arduino-1.6.0/hardware/arduino/avr/variants/standard" -I"/home/arturo/arduino_sketches/sketchbook_eclipse/srf08_ros/utility" -I"/home/arturo/arduino_sketches/sketchbook_normal/libraries/ros_lib" -I"/home/arturo/arduino-1.6.0/hardware/arduino/avr/libraries/Wire" -I"/home/arturo/arduino-1.6.0/hardware/arduino/avr/libraries/Wire/utility" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -D__IN_ECLIPSE__=1 -x c++ "$<"  -o  "$@"
	@echo 'Finished building: $<'
	@echo ' '


