################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/vl53l4cd/modules/vl53l4cd_api.c \
../Drivers/vl53l4cd/modules/vl53l4cd_calibration.c 

C_DEPS += \
./Drivers/vl53l4cd/modules/vl53l4cd_api.d \
./Drivers/vl53l4cd/modules/vl53l4cd_calibration.d 

OBJS += \
./Drivers/vl53l4cd/modules/vl53l4cd_api.o \
./Drivers/vl53l4cd/modules/vl53l4cd_calibration.o 


# Each subdirectory must supply rules for building sources it contributes
Drivers/vl53l4cd/modules/%.o Drivers/vl53l4cd/modules/%.su Drivers/vl53l4cd/modules/%.cyclo: ../Drivers/vl53l4cd/modules/%.c Drivers/vl53l4cd/modules/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G491xx -c -I../Core/Inc -I../Drivers/vl53l4cd/modules -I"/home/lianexu/Documents/Projects/KAISTfingertip_firmware/Fingertip_KAIST/Drivers/vl53l4cd" -I../Drivers/vl53l4cd/porting -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-vl53l4cd-2f-modules

clean-Drivers-2f-vl53l4cd-2f-modules:
	-$(RM) ./Drivers/vl53l4cd/modules/vl53l4cd_api.cyclo ./Drivers/vl53l4cd/modules/vl53l4cd_api.d ./Drivers/vl53l4cd/modules/vl53l4cd_api.o ./Drivers/vl53l4cd/modules/vl53l4cd_api.su ./Drivers/vl53l4cd/modules/vl53l4cd_calibration.cyclo ./Drivers/vl53l4cd/modules/vl53l4cd_calibration.d ./Drivers/vl53l4cd/modules/vl53l4cd_calibration.o ./Drivers/vl53l4cd/modules/vl53l4cd_calibration.su

.PHONY: clean-Drivers-2f-vl53l4cd-2f-modules

