################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../PMSM_FOC/MCUInit/adc.c \
../PMSM_FOC/MCUInit/ccu4.c \
../PMSM_FOC/MCUInit/ccu8.c \
../PMSM_FOC/MCUInit/clock.c \
../PMSM_FOC/MCUInit/gpio.c \
../PMSM_FOC/MCUInit/math_init.c \
../PMSM_FOC/MCUInit/mcuinit.c \
../PMSM_FOC/MCUInit/uart.c \
../PMSM_FOC/MCUInit/wdt.c 

OBJS += \
./PMSM_FOC/MCUInit/adc.o \
./PMSM_FOC/MCUInit/ccu4.o \
./PMSM_FOC/MCUInit/ccu8.o \
./PMSM_FOC/MCUInit/clock.o \
./PMSM_FOC/MCUInit/gpio.o \
./PMSM_FOC/MCUInit/math_init.o \
./PMSM_FOC/MCUInit/mcuinit.o \
./PMSM_FOC/MCUInit/uart.o \
./PMSM_FOC/MCUInit/wdt.o 

C_DEPS += \
./PMSM_FOC/MCUInit/adc.d \
./PMSM_FOC/MCUInit/ccu4.d \
./PMSM_FOC/MCUInit/ccu8.d \
./PMSM_FOC/MCUInit/clock.d \
./PMSM_FOC/MCUInit/gpio.d \
./PMSM_FOC/MCUInit/math_init.d \
./PMSM_FOC/MCUInit/mcuinit.d \
./PMSM_FOC/MCUInit/uart.d \
./PMSM_FOC/MCUInit/wdt.d 


# Each subdirectory must supply rules for building sources it contributes
PMSM_FOC/MCUInit/%.o: ../PMSM_FOC/MCUInit/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM-GCC C Compiler'
	"$(TOOLCHAIN_ROOT)/bin/arm-none-eabi-gcc" -MMD -MT "$@" -DXMC1302_T038x0200 -I"$(PROJECT_LOC)/Libraries/XMCLib/inc" -I"$(PROJECT_LOC)/PMSM_FOC/MCUInit" -I"$(PROJECT_LOC)/PMSM_FOC/MIDSys" -I"$(PROJECT_LOC)/Libraries/CMSIS/Include" -I"$(PROJECT_LOC)/Libraries/CMSIS/Infineon/XMC1300_series/Include" -I"$(PROJECT_LOC)/Libraries" -I"$(PROJECT_LOC)/PMSM_FOC/FOCLib" -I"$(PROJECT_LOC)/PMSM_FOC/Configuration" -I"$(PROJECT_LOC)/PMSM_FOC/ControlModules" -I"$(PROJECT_LOC)/PMSM_FOC/Interrupts" -O3 -ffunction-sections -fdata-sections -Wall -std=gnu99 -Wa,-adhlns="$@.lst" -pipe -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d) $@" -mcpu=cortex-m0 -mthumb -g -o "$@" "$<" 
	@echo 'Finished building: $<'
	@echo.

