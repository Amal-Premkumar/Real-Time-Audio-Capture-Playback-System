#include "main.h"
#include "stdio.h"
#include "core_cm4.h"
#include "pdm2pcm.h"
#include "pdm2pcm_glo.h"
#include "stm32f4xx_hal_i2s.h"
#include "stm32f4xx_hal_i2c.h"
#include "string.h"

#define I2S_BUFFER_SAMPLES  8000
#define PDM_INPUT_SAMPLES 125 * 64
#define PCM_OUTPUT_SAMPLES 125
int16_t pcm_mono_buffer[PCM_OUTPUT_SAMPLES];

#define PCM_stereo_Sample  (PCM_OUTPUT_SAMPLES *2)
int16_t PCM_stereo_buffer[PCM_stereo_Sample];

#define PCM_OUTPUT_SAMPLES   125
#define DECIM_FACTOR         64

// number of 16-bit PDM words per PCM block:
#define PDM_INPUT_WORDS      ((PCM_OUTPUT_SAMPLES * DECIM_FACTOR) / 16)   // 125*64/16 = 500

// I2S buffer has two halves: 500 + 500 words
#define I2S_BUFFER_SAMPLES   (PDM_INPUT_WORDS * 2)   // 1000

#define CS43L22_I2C_ADDR  (0x94)

I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_spi2_rx;

I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_spi3_tx;

I2C_HandleTypeDef hi2c1;

int16_t data_i2s[I2S_BUFFER_SAMPLES];
volatile uint8_t button_flag = 0;
volatile uint8_t start_stop_recording = 0;
volatile uint8_t half_i2s = 0;
volatile uint8_t full_i2s = 0;

// Function declarations (keep as is)
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_i2s2_Init(void);
static void Force_PB3_As_SWO(void);
static void MX_i2s3_init(void);
static void MX_i2c1_init(void);
static void AudioCodec_Write(uint8_t reg , uint8_t value);
void CS43L22_headphone_jack_init(void);

int main(void)
{
   HAL_Init();
    SystemClock_Config();

    __HAL_RCC_CRC_CLK_ENABLE();
    CRC->CR = CRC_CR_RESET;

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_i2s2_Init();
    MX_PDM2PCM_Init();
    MX_i2s3_init();
    MX_i2c1_init();

    CS43L22_headphone_jack_init();
    Force_PB3_As_SWO();

    HAL_I2S_DMAStop(&hi2s2);
    HAL_Delay(500);

    printf("System ready. Press USER button to start/stop I2S capture.\n");

    while (1)
    {
        // Handle button: start/stop capture
        if (button_flag)
        {
            button_flag = 0;

            if (start_stop_recording)
            {
                HAL_I2S_DMAStop(&hi2s2);
                HAL_I2S_DMAStop(&hi2s3);
                start_stop_recording = 0;
                half_i2s = 0;
                full_i2s = 0;
                printf("I2S capture stopped.\n");
            }
            else
            {
                start_stop_recording = 1;
                half_i2s = 0;
                full_i2s = 0;

                // Clear buffers
                memset(data_i2s, 0, sizeof(data_i2s));
                memset(PCM_stereo_buffer, 0, sizeof(PCM_stereo_buffer));

                // Start I2S2 RX (microphone)
                if (HAL_I2S_Receive_DMA(&hi2s2, (uint16_t *)data_i2s,
                                        I2S_BUFFER_SAMPLES) != HAL_OK)
                {
                    printf("Error starting I2S2 DMA receive.\n");
                }
                else
                {
                    printf("I2S2 RX started.\n");
                }

                HAL_Delay(10);

                // Start I2S3 TX (speaker) - circular DMA
                if (HAL_I2S_Transmit_DMA(&hi2s3, (uint16_t*)PCM_stereo_buffer,
                                         PCM_stereo_Sample) != HAL_OK)
                {
                    printf("Error starting I2S3 DMA transmit. err=0x%08lX\n", hi2s3.ErrorCode);
                }
                else
                {
                    printf("I2S3 TX started (DMA circular mode).\n");
                }
            }
        }

        // Only process audio when recording
        if (start_stop_recording)
        {
            /* ========== FIRST HALF OF I2S BUFFER ========== */
            if (half_i2s)
            {
                half_i2s = 0;

                // PDM -> PCM (first half)
                uint8_t db = MX_PDM2PCM_Process((uint16_t*)&data_i2s[0],
                                                (uint16_t*)pcm_mono_buffer);
                if (db != 0) {
                    printf("PDM filter error: %d\n", db);
                }

                // Volume calculation
                int32_t sum = 0;
                int32_t peak = 0;
                for (int i = 0; i < PCM_OUTPUT_SAMPLES; i++) {
                    int32_t abs_val = (pcm_mono_buffer[i] < 0)
                                      ? -pcm_mono_buffer[i]
                                      : pcm_mono_buffer[i];
                    sum += abs_val;
                    if (abs_val > peak) peak = abs_val;
                }
                int avg_pcm = sum / PCM_OUTPUT_SAMPLES;

                // LED VU meter
                if (avg_pcm > 2000) {
                    HAL_GPIO_WritePin(GPIOD, LD4_Pin, GPIO_PIN_SET);
                } else {
                    HAL_GPIO_WritePin(GPIOD, LD4_Pin, GPIO_PIN_RESET);
                }

                // Debug print once per second
                static uint32_t print_timer = 0;
                if (HAL_GetTick() - print_timer > 1000)
                {
                    print_timer = HAL_GetTick();
                    printf("Avg:%d Peak:%d PCM[0]=%d Raw[0]=0x%04X Raw[100]=0x%04X I2S3_State=%d\r\n",
                           avg_pcm, (int)peak,
                           pcm_mono_buffer[0],
                           (unsigned)data_i2s[0], (unsigned)data_i2s[100],
                           hi2s3.State);
                }

                for (int i = 0; i < PCM_OUTPUT_SAMPLES; i++) {
                    int32_t sam = (int32_t)pcm_mono_buffer[i];  // 2x gain

                    // Clip to prevent overflow
                    if (sam > 32767) sam = 32767;
                    if (sam < -32768) sam = -32768;

                    PCM_stereo_buffer[2*i]     = (int16_t)sam;  // L
                    PCM_stereo_buffer[2*i + 1] = (int16_t)sam;  // R
                }
            }

            if (full_i2s)
            {
                full_i2s = 0;

                // PDM -> PCM (second half)
                MX_PDM2PCM_Process((uint16_t*)&data_i2s[PDM_INPUT_WORDS],
                                   (uint16_t*)pcm_mono_buffer);

                // Mono -> Stereo
                for (int i = 0; i < PCM_OUTPUT_SAMPLES; i++) {
                    int32_t sam = (int32_t)pcm_mono_buffer[i];
                    if (sam > 32767) sam = 32767;
                    if (sam < -32768) sam = -32768;

                    PCM_stereo_buffer[2*i]     = (int16_t)sam;
                    PCM_stereo_buffer[2*i + 1] = (int16_t)sam;
                }
            }
        }
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 168;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType =
        RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
        RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }



    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
    PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
    PeriphClkInitStruct.PLLI2S.PLLI2SR = 5;   // these values depend on your target audio freq
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

}

static void MX_i2s2_Init(void)
{
    hi2s2.Instance = SPI2;
    hi2s2.Init.Mode = I2S_MODE_MASTER_RX;
    hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
    hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
    hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
    hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_32K;
    hi2s2.Init.CPOL = I2S_CPOL_HIGH;
    hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
    hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
    if (HAL_I2S_Init(&hi2s2) != HAL_OK)
    {
        Error_Handler();
    }
}

static void MX_i2s3_init(void){
	hi2s3.Instance = SPI3;
	hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
	hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
	hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
	hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
	hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
	hi2s3.Init.CPOL = I2S_CPOL_LOW;
	hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
	hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_16K;
	if(HAL_I2S_Init(&hi2s3)!= HAL_OK){
		 printf("I2S3 init error, err=0x%08lX\r\n", hi2s3.ErrorCode);
		Error_Handler();
	}
}

static void MX_i2c1_init(void){
	hi2c1.Instance = I2C1;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.ClockSpeed = 100000;  //100k
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.OwnAddress2 = 0;
	if(HAL_I2C_Init(&hi2c1)!= HAL_OK){
		Error_Handler();
	}
}

static void MX_DMA_Init(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();

    HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);

       HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 1, 0);  // Slightly lower priority
       HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(GPIOD,
                      LD4_Pin | LD3_Pin | LD5_Pin | LD6_Pin | Audio_RST_Pin,
                      GPIO_PIN_RESET);

    // PB10 = I2S2_CK
    GPIO_InitStruct.Pin = CLK_IN_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // PC3 = I2S2_SD (PDM_OUT)
    GPIO_InitStruct.Pin = PDM_OUT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);



    //PC7 = I2S3 MCK_pin( for headphone jack)
    GPIO_InitStruct.Pin = I2S3_MCK_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(I2S3_MCK_GPIO_Port, &GPIO_InitStruct);

    //PA4 = I2S3 WS
    GPIO_InitStruct.Pin = I2S3_WS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull =GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(I2S3_WS_GPIO_Port, &GPIO_InitStruct);

    //PC10 = I2S3 SCK  PC12 = I2S3 SD
    GPIO_InitStruct.Pin = GPIO_PIN_10 |GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull =GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);


    //I2C1 config for CS43L22 headphone jack control   //PB6=SCL PB9=SDA
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull =GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


    // User button
    GPIO_InitStruct.Pin = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = BOOT1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LD4_Pin | LD3_Pin | LD5_Pin | LD6_Pin | Audio_RST_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = Audio_SCL_Pin | Audio_SDA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

static void Force_PB3_As_SWO(void)
{
    /* Make sure GPIOB clock is enabled */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    /* PB3 = alternate function mode (10b) */
    GPIOB->MODER &= ~(3U << (3U * 2U));   // clear mode bits for PB3
    GPIOB->MODER |=  (2U << (3U * 2U));   // set to AF

    /* High speed (optional) */
    GPIOB->OSPEEDR |= (3U << (3U * 2U));

    /* No pull-up/pull-down */
    GPIOB->PUPDR &= ~(3U << (3U * 2U));

    /* Alternate function 0 (AF0 = JTDO/TRACESWO on PB3) */
    GPIOB->AFR[0] &= ~(0xFU << (3U * 4U));   // clear AF bits for PB3
    // no need to set a value, AF0 = 0
}

static void AudioCodec_Write(uint8_t reg , uint8_t value){

	HAL_StatusTypeDef sts = HAL_I2C_Mem_Write(&hi2c1,
			CS43L22_I2C_ADDR,
			reg,
			I2C_MEMADD_SIZE_8BIT,
			&value,
			1,
			100);

	if(sts != HAL_OK){
		 printf("I2S3 audio tx error: st=%d, err=0x%08lX, state=0x%02X\r\n",
					sts, hi2c1.ErrorCode, hi2c1.State);
}
}

void CS43L22_headphone_jack_init(void){
	HAL_GPIO_WritePin(Audio_RST_GPIO_Port, Audio_RST_Pin, GPIO_PIN_SET);
	HAL_Delay(10);

    AudioCodec_Write(0x00, 0x99);
    AudioCodec_Write(0x47, 0x80);
    AudioCodec_Write(0x32, 0x80);
    AudioCodec_Write(0x32, 0x00);
    AudioCodec_Write(0x00, 0x00);

    AudioCodec_Write(0x06, 0x04);  // I2S, 16-bit, slave mode

    //  Set headphone volumes
    AudioCodec_Write(0x20, 0xDF);  // Left headphone volume
    AudioCodec_Write(0x21, 0xDF);  // Right headphone volume

    //  Power up / enable outputs
    AudioCodec_Write(0x02, 0x9E);  // Power up, enable HP, etc.

    AudioCodec_Write(0x04, 0xAF);  // Main power/volume, unmute
}

int _write(int file, char *ptr, int len)
{
    (void)file;
    for (int i = 0; i < len; i++)
    {
        ITM_SendChar(*ptr++);
    }
    return len;
}


void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s)
{
	 if (hi2s->Instance == SPI2){
    full_i2s = 1;
}

}

void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
	if (hi2s->Instance == SPI2){
   half_i2s = 1;
}
}


void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
    if (hi2s->Instance == SPI3)
    { }
}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s)
{
    if (hi2s->Instance == SPI3)
    { }
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == B1_Pin)
    {
        button_flag = 1;
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}


#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
}
#endif
