CPP_SRCS += \
../Startup/stm32_startup.cpp \

CPP_DEPS += \
./Startup/stm32_startup.d \

OBJS += \
./Startup/stm32_startup.o \

Startup/%.o: ../Startup/%.cpp Startup/subdir.mk
	arm-none-eabi-g++ -mcpu=cortex-m4 -std=gnu++14 -g3 -c  -ffunction-sections -fdata-sections -fno-exceptions -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"
