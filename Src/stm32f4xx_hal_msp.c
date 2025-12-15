/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file         stm32f4xx_hal_msp.c
  * @brief        This file provides code for the MSP Initialization
  *               and de-Initialization codes.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

extern DMA_HandleTypeDef hdma_spi2_rx;
extern DMA_HandleTypeDef hdma_spi3_tx;  // Added for I2S3 TX

void HAL_i2c_MspInit(I2C_HandleTypeDef* hi2c1);

/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_0);
}

/**
* @brief I2C MSP Initialization
*/
void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c)
{
    if (hi2c->Instance == I2C1)
    {
        __HAL_RCC_I2C1_CLK_ENABLE();
    }
}

/**
* @brief I2S MSP Initialization
*/
void HAL_I2S_MspInit(I2S_HandleTypeDef* hi2s)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  if(hi2s->Instance==SPI2)
  {
    /** Initializes the peripherals clock */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
    PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
    PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* Peripheral clock enable */
    __HAL_RCC_SPI2_CLK_ENABLE();

    /* I2S2 DMA Init (RX - Microphone) */
    hdma_spi2_rx.Instance = DMA1_Stream3;
    hdma_spi2_rx.Init.Channel = DMA_CHANNEL_0;
    hdma_spi2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_spi2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi2_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_spi2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_spi2_rx.Init.Mode = DMA_CIRCULAR;
    hdma_spi2_rx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_spi2_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_spi2_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(hi2s, hdmarx, hdma_spi2_rx);
  }
  else if(hi2s->Instance == SPI3)
  {
    /* Peripheral clock enable */
    __HAL_RCC_SPI3_CLK_ENABLE();

    /* I2S3 DMA Init (TX - Speaker) */
    hdma_spi3_tx.Instance = DMA1_Stream5;
    hdma_spi3_tx.Init.Channel = DMA_CHANNEL_0;
    hdma_spi3_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_spi3_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi3_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_spi3_tx.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_spi3_tx.Init.Mode = DMA_CIRCULAR;
    hdma_spi3_tx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_spi3_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

    if (HAL_DMA_Init(&hdma_spi3_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(hi2s, hdmatx, hdma_spi3_tx);

    /* DMA interrupt init for I2S3 TX */
    HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  }
}

/**
* @brief I2S MSP De-Initialization
*/
void HAL_I2S_MspDeInit(I2S_HandleTypeDef* hi2s)
{
  if(hi2s->Instance==SPI2)
  {
    __HAL_RCC_SPI2_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOB, CLK_IN_Pin|GPIO_PIN_12|PDM_OUT_Pin);
    HAL_DMA_DeInit(hi2s->hdmarx);
  }
  else if(hi2s->Instance==SPI3)
  {
    __HAL_RCC_SPI3_CLK_DISABLE();
    HAL_DMA_DeInit(hi2s->hdmatx);
    HAL_NVIC_DisableIRQ(DMA1_Stream5_IRQn);
  }
}
