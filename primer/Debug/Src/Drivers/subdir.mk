CPP_SRCS += \
../Src/Drivers/basicTimer.cpp \
../Src/Drivers/GPIO.cpp \
../Src/Drivers/ADC.cpp \
../Src/Drivers/USART.cpp \

CPP_DEPS += \
../Src/Drivers/basicTimer.d \
../Src/Drivers/GPIO.d \
../Src/Drivers/ADC.d \
../Src/Drivers/USART.d \


OBJS += \
./Src/Drivers/basicTimer.o \
./Src/Drivers/GPIO.o \
./Src/Drivers/ADC.o \
./Src/Drivers/USART.o \



Src/%.o: ../Src/Drivers/%.cpp Src/Drivers/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -c -I../Inc -I../Inc/Drivers -I ../Inc/Drivers/CMSIS/Device/ST/STM32F4xx/Include -I ../Inc/Drivers/CMSIS/Include -I ../Inc/Classes -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

Src/%.o: ../Src/Drivers/%.c Src/Drivers/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -c -I../Inc/ -I../Inc/Drivers -I ../Inc/Drivers/CMSIS/Device/ST/STM32F4xx/Include -I ../Inc/Drivers/CMSIS/Include -I ../Inc/Classes -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

