################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/home/arturo/arduino-1.6.0/hardware/arduino/avr/libraries/Wire/utility/twi.c 

C_DEPS += \
./Libraries/Wire/utility/twi.c.d 

LINK_OBJ += \
./Libraries/Wire/utility/twi.c.o 


# Each subdirectory must supply rules for building sources it contributes
Libraries/Wire/utility/twi.c.o: /home/arturo/arduino-1.6.0/hardware/arduino/avr/libraries/Wire/utility/twi.c
	@echo 'Building file: $<'
	@echo 'Starting C compile'
	"/home/arturo/arduino-1.6.0/hardware/tools/avr/bin/avr-gcc" -c -g -Os -w -ffunction-sections -fdata-sections -MMD -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=160 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR     -I"/home/arturo/arduino-1.6.0/hardware/arduino/avr/cores/arduino" -I"/home/arturo/arduino-1.6.0/hardware/arduino/avr/variants/standard" -I"/home/arturo/arduino_sketches/sketchbook_eclipse/srf08_ros/utility" -I"/home/arturo/arduino_sketches/sketchbook_normal/libraries/ros_lib" -I"/home/arturo/arduino-1.6.0/hardware/arduino/avr/libraries/Wire" -I"/home/arturo/arduino-1.6.0/hardware/arduino/avr/libraries/Wire/utility" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -D__IN_ECLIPSE__=1 "$<"  -o  "$@"
	@echo 'Finished building: $<'
	@echo ' '


