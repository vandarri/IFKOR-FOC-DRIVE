################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../PMSM_FOC/MIDSys/pmsm_foc_current_singleshunt.c \
../PMSM_FOC/MIDSys/pmsm_foc_current_threeshunt.c \
../PMSM_FOC/MIDSys/pmsm_foc_debug.c \
../PMSM_FOC/MIDSys/pmsm_foc_pwmsvm.c 

OBJS += \
./PMSM_FOC/MIDSys/pmsm_foc_current_singleshunt.o \
./PMSM_FOC/MIDSys/pmsm_foc_current_threeshunt.o \
./PMSM_FOC/MIDSys/pmsm_foc_debug.o \
./PMSM_FOC/MIDSys/pmsm_foc_pwmsvm.o 

C_DEPS += \
./PMSM_FOC/MIDSys/pmsm_foc_current_singleshunt.d \
./PMSM_FOC/MIDSys/pmsm_foc_current_threeshunt.d \
./PMSM_FOC/MIDSys/pmsm_foc_debug.d \
./PMSM_FOC/MIDSys/pmsm_foc_pwmsvm.d 


# Each subdirectory must supply rules for building sources it contributes
PMSM_FOC/MIDSys/%.o: ../PMSM_FOC/MIDSys/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM-GCC C Compiler'
	"$(TOOLCHAIN_ROOT)/bin/arm-none-eabi-gcc" -MMD -MT "$@" -DXMC1302_T038x0200 -I"$(PROJECT_LOC)/Libraries/XMCLib/inc" -I"$(PROJECT_LOC)/PMSM_FOC/MCUInit" -I"$(PROJECT_LOC)/PMSM_FOC/MIDSys" -I"$(PROJECT_LOC)/Libraries/CMSIS/Include" -I"$(PROJECT_LOC)/Libraries/CMSIS/Infineon/XMC1300_series/Include" -I"$(PROJECT_LOC)/Libraries" -I"$(PROJECT_LOC)/PMSM_FOC/FOCLib" -I"$(PROJECT_LOC)/PMSM_FOC/Configuration" -I"$(PROJECT_LOC)/PMSM_FOC/ControlModules" -I"$(PROJECT_LOC)/PMSM_FOC/Interrupts" -O3 -ffunction-sections -fdata-sections -Wall -std=gnu99 -Wa,-adhlns="$@.lst" -pipe -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d) $@" -mcpu=cortex-m0 -mthumb -g -o "$@" "$<" 
	@echo 'Finished building: $<'
	@echo.

