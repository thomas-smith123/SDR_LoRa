################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Application/Periphery/UserConfig.c \
../Application/Periphery/lr_fhss_mac.c \
../Application/Periphery/sx126x.c \
../Application/Periphery/sx126x_driver_version.c \
../Application/Periphery/sx126x_hal.c \
../Application/Periphery/sx126x_lr_fhss.c 

OBJS += \
./Application/Periphery/UserConfig.o \
./Application/Periphery/lr_fhss_mac.o \
./Application/Periphery/sx126x.o \
./Application/Periphery/sx126x_driver_version.o \
./Application/Periphery/sx126x_hal.o \
./Application/Periphery/sx126x_lr_fhss.o 

C_DEPS += \
./Application/Periphery/UserConfig.d \
./Application/Periphery/lr_fhss_mac.d \
./Application/Periphery/sx126x.d \
./Application/Periphery/sx126x_driver_version.d \
./Application/Periphery/sx126x_hal.d \
./Application/Periphery/sx126x_lr_fhss.d 


# Each subdirectory must supply rules for building sources it contributes
Application/Periphery/%.o Application/Periphery/%.su Application/Periphery/%.cyclo: ../Application/Periphery/%.c Application/Periphery/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L031xx -c -I../../Core/Inc -I"C:/code/SDR_LoRa/lower_computer/lower_computer2/STM32CubeIDE/Application/Periphery" -I../../Drivers/STM32L0xx_HAL_Driver/Inc -I../../Drivers/STM32L0xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32L0xx/Include -I../../Drivers/CMSIS/Include -O1 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Application-2f-Periphery

clean-Application-2f-Periphery:
	-$(RM) ./Application/Periphery/UserConfig.cyclo ./Application/Periphery/UserConfig.d ./Application/Periphery/UserConfig.o ./Application/Periphery/UserConfig.su ./Application/Periphery/lr_fhss_mac.cyclo ./Application/Periphery/lr_fhss_mac.d ./Application/Periphery/lr_fhss_mac.o ./Application/Periphery/lr_fhss_mac.su ./Application/Periphery/sx126x.cyclo ./Application/Periphery/sx126x.d ./Application/Periphery/sx126x.o ./Application/Periphery/sx126x.su ./Application/Periphery/sx126x_driver_version.cyclo ./Application/Periphery/sx126x_driver_version.d ./Application/Periphery/sx126x_driver_version.o ./Application/Periphery/sx126x_driver_version.su ./Application/Periphery/sx126x_hal.cyclo ./Application/Periphery/sx126x_hal.d ./Application/Periphery/sx126x_hal.o ./Application/Periphery/sx126x_hal.su ./Application/Periphery/sx126x_lr_fhss.cyclo ./Application/Periphery/sx126x_lr_fhss.d ./Application/Periphery/sx126x_lr_fhss.o ./Application/Periphery/sx126x_lr_fhss.su

.PHONY: clean-Application-2f-Periphery

