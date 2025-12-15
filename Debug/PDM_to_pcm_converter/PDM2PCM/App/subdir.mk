################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../PDM_to_pcm_converter/PDM2PCM/App/pdm2pcm.c 

OBJS += \
./PDM_to_pcm_converter/PDM2PCM/App/pdm2pcm.o 

C_DEPS += \
./PDM_to_pcm_converter/PDM2PCM/App/pdm2pcm.d 


# Each subdirectory must supply rules for building sources it contributes
PDM_to_pcm_converter/PDM2PCM/App/%.o PDM_to_pcm_converter/PDM2PCM/App/%.su PDM_to_pcm_converter/PDM2PCM/App/%.cyclo: ../PDM_to_pcm_converter/PDM2PCM/App/%.c PDM_to_pcm_converter/PDM2PCM/App/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32F407xx -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I"C:/Users/Amal/RTOS_workspace/microphone_to_speaker_usin_freeRTOS/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Users/Amal/RTOS_workspace/microphone_to_speaker_usin_freeRTOS/PDM_to_pcm_converter/Core/Inc" -I"C:/Users/Amal/RTOS_workspace/microphone_to_speaker_usin_freeRTOS/PDM_to_pcm_converter/Middlewares/ST/STM32_Audio/Addons/PDM/Inc" -I"C:/Users/Amal/RTOS_workspace/microphone_to_speaker_usin_freeRTOS/PDM_to_pcm_converter/Middlewares/ST/STM32_Audio/Addons/PDM/Lib" -I"C:/Users/Amal/RTOS_workspace/microphone_to_speaker_usin_freeRTOS/PDM_to_pcm_converter/PDM2PCM/App" -I"C:/Users/Amal/RTOS_workspace/microphone_to_speaker_usin_freeRTOS/Inc" -I"C:/Users/Amal/RTOS_workspace/microphone_to_speaker_usin_freeRTOS/Drivers/STM32F4xx_HAL_Driver/Inc" -I"C:/Users/Amal/RTOS_workspace/microphone_to_speaker_usin_freeRTOS/Drivers/CMSIS/Include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-PDM_to_pcm_converter-2f-PDM2PCM-2f-App

clean-PDM_to_pcm_converter-2f-PDM2PCM-2f-App:
	-$(RM) ./PDM_to_pcm_converter/PDM2PCM/App/pdm2pcm.cyclo ./PDM_to_pcm_converter/PDM2PCM/App/pdm2pcm.d ./PDM_to_pcm_converter/PDM2PCM/App/pdm2pcm.o ./PDM_to_pcm_converter/PDM2PCM/App/pdm2pcm.su

.PHONY: clean-PDM_to_pcm_converter-2f-PDM2PCM-2f-App

