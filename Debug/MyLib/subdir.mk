################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../MyLib/DS18B20.c \
../MyLib/DWT_Delay.c \
../MyLib/MATH.c \
../MyLib/SX1278.c \
../MyLib/SX1278_hw.c \
../MyLib/Timer_Delay.c \
../MyLib/bmp280.c \
../MyLib/uart_printf.c 

OBJS += \
./MyLib/DS18B20.o \
./MyLib/DWT_Delay.o \
./MyLib/MATH.o \
./MyLib/SX1278.o \
./MyLib/SX1278_hw.o \
./MyLib/Timer_Delay.o \
./MyLib/bmp280.o \
./MyLib/uart_printf.o 

C_DEPS += \
./MyLib/DS18B20.d \
./MyLib/DWT_Delay.d \
./MyLib/MATH.d \
./MyLib/SX1278.d \
./MyLib/SX1278_hw.d \
./MyLib/Timer_Delay.d \
./MyLib/bmp280.d \
./MyLib/uart_printf.d 


# Each subdirectory must supply rules for building sources it contributes
MyLib/%.o MyLib/%.su MyLib/%.cyclo: ../MyLib/%.c MyLib/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I"D:/Embedded_Advanced/Project/LoRa_WSN/MyLib" -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-MyLib

clean-MyLib:
	-$(RM) ./MyLib/DS18B20.cyclo ./MyLib/DS18B20.d ./MyLib/DS18B20.o ./MyLib/DS18B20.su ./MyLib/DWT_Delay.cyclo ./MyLib/DWT_Delay.d ./MyLib/DWT_Delay.o ./MyLib/DWT_Delay.su ./MyLib/MATH.cyclo ./MyLib/MATH.d ./MyLib/MATH.o ./MyLib/MATH.su ./MyLib/SX1278.cyclo ./MyLib/SX1278.d ./MyLib/SX1278.o ./MyLib/SX1278.su ./MyLib/SX1278_hw.cyclo ./MyLib/SX1278_hw.d ./MyLib/SX1278_hw.o ./MyLib/SX1278_hw.su ./MyLib/Timer_Delay.cyclo ./MyLib/Timer_Delay.d ./MyLib/Timer_Delay.o ./MyLib/Timer_Delay.su ./MyLib/bmp280.cyclo ./MyLib/bmp280.d ./MyLib/bmp280.o ./MyLib/bmp280.su ./MyLib/uart_printf.cyclo ./MyLib/uart_printf.d ./MyLib/uart_printf.o ./MyLib/uart_printf.su

.PHONY: clean-MyLib

