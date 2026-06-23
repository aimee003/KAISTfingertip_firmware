################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Src/ForceSensor.cpp \
../Core/Src/fingertip.cpp \
../Core/Src/fingertip_net.cpp \
../Core/Src/math_ops.cpp \
../Core/Src/ms5849_funcs.cpp \
../Core/Src/sensor_interface.cpp 

C_SRCS += \
../Core/Src/BNO080.c \
../Core/Src/Quaternion.c \
../Core/Src/fdcan.c \
../Core/Src/gpio.c \
../Core/Src/i2c.c \
../Core/Src/main.c \
../Core/Src/ms5849.c \
../Core/Src/printing.c \
../Core/Src/spi.c \
../Core/Src/stm32g4xx_hal_msp.c \
../Core/Src/stm32g4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32g4xx.c \
../Core/Src/tim.c \
../Core/Src/usart.c 

C_DEPS += \
./Core/Src/BNO080.d \
./Core/Src/Quaternion.d \
./Core/Src/fdcan.d \
./Core/Src/gpio.d \
./Core/Src/i2c.d \
./Core/Src/main.d \
./Core/Src/ms5849.d \
./Core/Src/printing.d \
./Core/Src/spi.d \
./Core/Src/stm32g4xx_hal_msp.d \
./Core/Src/stm32g4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32g4xx.d \
./Core/Src/tim.d \
./Core/Src/usart.d 

OBJS += \
./Core/Src/BNO080.o \
./Core/Src/ForceSensor.o \
./Core/Src/Quaternion.o \
./Core/Src/fdcan.o \
./Core/Src/fingertip.o \
./Core/Src/fingertip_net.o \
./Core/Src/gpio.o \
./Core/Src/i2c.o \
./Core/Src/main.o \
./Core/Src/math_ops.o \
./Core/Src/ms5849.o \
./Core/Src/ms5849_funcs.o \
./Core/Src/printing.o \
./Core/Src/sensor_interface.o \
./Core/Src/spi.o \
./Core/Src/stm32g4xx_hal_msp.o \
./Core/Src/stm32g4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32g4xx.o \
./Core/Src/tim.o \
./Core/Src/usart.o 

CPP_DEPS += \
./Core/Src/ForceSensor.d \
./Core/Src/fingertip.d \
./Core/Src/fingertip_net.d \
./Core/Src/math_ops.d \
./Core/Src/ms5849_funcs.d \
./Core/Src/sensor_interface.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G491xx -c -I../Core/Inc -I../Drivers/vl53l4cd/modules -I"/home/lianexu/Documents/Projects/KAISTfingertip_firmware/Fingertip_KAIST/Drivers/vl53l4cd" -I../Drivers/vl53l4cd/porting -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.cpp Core/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G491xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/BNO080.cyclo ./Core/Src/BNO080.d ./Core/Src/BNO080.o ./Core/Src/BNO080.su ./Core/Src/ForceSensor.cyclo ./Core/Src/ForceSensor.d ./Core/Src/ForceSensor.o ./Core/Src/ForceSensor.su ./Core/Src/Quaternion.cyclo ./Core/Src/Quaternion.d ./Core/Src/Quaternion.o ./Core/Src/Quaternion.su ./Core/Src/fdcan.cyclo ./Core/Src/fdcan.d ./Core/Src/fdcan.o ./Core/Src/fdcan.su ./Core/Src/fingertip.cyclo ./Core/Src/fingertip.d ./Core/Src/fingertip.o ./Core/Src/fingertip.su ./Core/Src/fingertip_net.cyclo ./Core/Src/fingertip_net.d ./Core/Src/fingertip_net.o ./Core/Src/fingertip_net.su ./Core/Src/gpio.cyclo ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/gpio.su ./Core/Src/i2c.cyclo ./Core/Src/i2c.d ./Core/Src/i2c.o ./Core/Src/i2c.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/math_ops.cyclo ./Core/Src/math_ops.d ./Core/Src/math_ops.o ./Core/Src/math_ops.su ./Core/Src/ms5849.cyclo ./Core/Src/ms5849.d ./Core/Src/ms5849.o ./Core/Src/ms5849.su ./Core/Src/ms5849_funcs.cyclo ./Core/Src/ms5849_funcs.d ./Core/Src/ms5849_funcs.o ./Core/Src/ms5849_funcs.su ./Core/Src/printing.cyclo ./Core/Src/printing.d ./Core/Src/printing.o ./Core/Src/printing.su ./Core/Src/sensor_interface.cyclo ./Core/Src/sensor_interface.d ./Core/Src/sensor_interface.o ./Core/Src/sensor_interface.su ./Core/Src/spi.cyclo ./Core/Src/spi.d ./Core/Src/spi.o ./Core/Src/spi.su ./Core/Src/stm32g4xx_hal_msp.cyclo ./Core/Src/stm32g4xx_hal_msp.d ./Core/Src/stm32g4xx_hal_msp.o ./Core/Src/stm32g4xx_hal_msp.su ./Core/Src/stm32g4xx_it.cyclo ./Core/Src/stm32g4xx_it.d ./Core/Src/stm32g4xx_it.o ./Core/Src/stm32g4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32g4xx.cyclo ./Core/Src/system_stm32g4xx.d ./Core/Src/system_stm32g4xx.o ./Core/Src/system_stm32g4xx.su ./Core/Src/tim.cyclo ./Core/Src/tim.d ./Core/Src/tim.o ./Core/Src/tim.su ./Core/Src/usart.cyclo ./Core/Src/usart.d ./Core/Src/usart.o ./Core/Src/usart.su

.PHONY: clean-Core-2f-Src

