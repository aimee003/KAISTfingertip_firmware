################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/vl53l4cd/porting/platform.c 

C_DEPS += \
./Drivers/vl53l4cd/porting/platform.d 

OBJS += \
./Drivers/vl53l4cd/porting/platform.o 


# Each subdirectory must supply rules for building sources it contributes
Drivers/vl53l4cd/porting/%.o Drivers/vl53l4cd/porting/%.su Drivers/vl53l4cd/porting/%.cyclo: ../Drivers/vl53l4cd/porting/%.c Drivers/vl53l4cd/porting/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G491xx -c -I../Core/Inc -I../Drivers/vl53l4cd/modules -I"/home/lianexu/Documents/Projects/KAISTfingertip_firmware/Fingertip_KAIST/Drivers/vl53l4cd" -I../Drivers/vl53l4cd/porting -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-vl53l4cd-2f-porting

clean-Drivers-2f-vl53l4cd-2f-porting:
	-$(RM) ./Drivers/vl53l4cd/porting/platform.cyclo ./Drivers/vl53l4cd/porting/platform.d ./Drivers/vl53l4cd/porting/platform.o ./Drivers/vl53l4cd/porting/platform.su

.PHONY: clean-Drivers-2f-vl53l4cd-2f-porting

