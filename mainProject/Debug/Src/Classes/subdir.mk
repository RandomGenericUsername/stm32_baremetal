CPP_SRCS += \
../Src/Classes/Vector.cpp \

CPP_DEPS += \
../Src/Classes/Vector.d \

OBJS += \
./Src/Classes/Vector.o \


Src/%.o: ../Src/Classes/%.cpp Src/Classes/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -c -I../Inc -I../Inc/Classes -I ../Inc/Drivers/CMSIS/Device/ST/STM32F4xx/Include -I ../Inc/Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

Src/%.o: ../Src/Classes/%.c Src/Classes/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -c -I../Inc/ -I../Inc/Classes -I ../Inc/Drivers/CMSIS/Device/ST/STM32F4xx/Include -I ../Inc/Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
