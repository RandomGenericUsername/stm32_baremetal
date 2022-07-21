C_SRCS += \
../Src/syscalls.c \
../Src/sysmem.c

C_DEPS += \
../Src/syscalls.d \
../Src/sysmem.d

CPP_SRCS += \
../Src/main.cpp

CPP_DEPS += \
../Src/main.d

OBJS += \
./Src/main.o \
./Src/syscalls.o \
./Src/sysmem.o 


Src/%.o: ../Src/%.cpp Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -c -I../Inc/ -I../Inc/Drivers -I ../Inc/Drivers/CMSIS/Device/ST/STM32F4xx/Include -I ../Inc/Drivers/CMSIS/Include -I ../Inc/Classes -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

Src/%.o: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -c -I../Inc/ -I../Inc/Drivers -I ../Inc/Drivers/CMSIS/Device/ST/STM32F4xx/Include -I ../Inc/Drivers/CMSIS/Include -I ../Inc/Classes -O0  -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

