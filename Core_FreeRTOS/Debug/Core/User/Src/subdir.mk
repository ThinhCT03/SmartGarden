################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/User/Src/DHT11.c \
../Core/User/Src/DS3231.c \
../Core/User/Src/bh1750.c \
../Core/User/Src/i2c-lcd.c 

OBJS += \
./Core/User/Src/DHT11.o \
./Core/User/Src/DS3231.o \
./Core/User/Src/bh1750.o \
./Core/User/Src/i2c-lcd.o 

C_DEPS += \
./Core/User/Src/DHT11.d \
./Core/User/Src/DS3231.d \
./Core/User/Src/bh1750.d \
./Core/User/Src/i2c-lcd.d 


# Each subdirectory must supply rules for building sources it contributes
Core/User/Src/%.o Core/User/Src/%.su Core/User/Src/%.cyclo: ../Core/User/Src/%.c Core/User/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I"E:/STide/STMworkspace/LTN_Semaphore/Core/User/Src" -I"E:/STide/STMworkspace/LTN_Semaphore/Core/User/Inc" -I"E:/STide/STMworkspace/LTN_Semaphore/Core/User" -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-User-2f-Src

clean-Core-2f-User-2f-Src:
	-$(RM) ./Core/User/Src/DHT11.cyclo ./Core/User/Src/DHT11.d ./Core/User/Src/DHT11.o ./Core/User/Src/DHT11.su ./Core/User/Src/DS3231.cyclo ./Core/User/Src/DS3231.d ./Core/User/Src/DS3231.o ./Core/User/Src/DS3231.su ./Core/User/Src/bh1750.cyclo ./Core/User/Src/bh1750.d ./Core/User/Src/bh1750.o ./Core/User/Src/bh1750.su ./Core/User/Src/i2c-lcd.cyclo ./Core/User/Src/i2c-lcd.d ./Core/User/Src/i2c-lcd.o ./Core/User/Src/i2c-lcd.su

.PHONY: clean-Core-2f-User-2f-Src

