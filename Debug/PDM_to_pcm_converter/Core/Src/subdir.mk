################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../PDM_to_pcm_converter/Core/Src/main.c 

OBJS += \
./PDM_to_pcm_converter/Core/Src/main.o 

C_DEPS += \
./PDM_to_pcm_converter/Core/Src/main.d 


# Each subdirectory must supply rules for building sources it contributes
PDM_to_pcm_converter/Core/Src/%.o PDM_to_pcm_converter/Core/Src/%.su PDM_to_pcm_converter/Core/Src/%.cyclo: ../PDM_to_pcm_converter/Core/Src/%.c PDM_to_pcm_converter/Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32F407xx -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I"C:/Users/Amal/RTOS_workspace/microphone_to_speaker_usin_freeRTOS/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Users/Amal/RTOS_workspace/microphone_to_speaker_usin_freeRTOS/PDM_to_pcm_converter/Core/Inc" -I"C:/Users/Amal/RTOS_workspace/microphone_to_speaker_usin_freeRTOS/PDM_to_pcm_converter/Middlewares/ST/STM32_Audio/Addons/PDM/Inc" -I"C:/Users/Amal/RTOS_workspace/microphone_to_speaker_usin_freeRTOS/PDM_to_pcm_converter/Middlewares/ST/STM32_Audio/Addons/PDM/Lib" -I"C:/Users/Amal/RTOS_workspace/microphone_to_speaker_usin_freeRTOS/PDM_to_pcm_converter/PDM2PCM/App" -I"C:/Users/Amal/RTOS_workspace/microphone_to_speaker_usin_freeRTOS/Inc" -I"C:/Users/Amal/RTOS_workspace/microphone_to_speaker_usin_freeRTOS/Drivers/STM32F4xx_HAL_Driver/Inc" -I"C:/Users/Amal/RTOS_workspace/microphone_to_speaker_usin_freeRTOS/Drivers/CMSIS/Include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-PDM_to_pcm_converter-2f-Core-2f-Src

clean-PDM_to_pcm_converter-2f-Core-2f-Src:
	-$(RM) ./PDM_to_pcm_converter/Core/Src/main.cyclo ./PDM_to_pcm_converter/Core/Src/main.d ./PDM_to_pcm_converter/Core/Src/main.o ./PDM_to_pcm_converter/Core/Src/main.su

.PHONY: clean-PDM_to_pcm_converter-2f-Core-2f-Src

