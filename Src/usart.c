/**
  ******************************************************************************
  * File Name          : USART.c
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usart.h" 
/* USER CODE END 0 */ 

UART_HandleTypeDef huartx;
DMA_HandleTypeDef  hdma_usart2_tx;
DMA_HandleTypeDef  hdma_usart2_rx; 
DMA_HandleTypeDef  hdma_usart3_tx;
DMA_HandleTypeDef  hdma_usart3_rx; 
#ifdef __GNUC__
		 /* With GCC, small printf (option LD Linker->Libraries->Small printf
		 set to 'Yes') calls __io_putchar() */
			#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
		 #else
			#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
		 #endif /* __GNUC__ */
		 
		 /**
		 * @brief  Retargets the C library printf function to the USART.
		 * @param  None
		 * @retval None
		 */
		PUTCHAR_PROTOTYPE
		{
		 /* Place your implementation of fputc here */
		 /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
		 HAL_UART_Transmit(&huartx, (uint8_t *)&ch, 1, 0xFFFF); 

		 return ch;
		} 
/* USER CODE BEGIN 0 */

void My_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct; 
	__HAL_RCC_DMA1_CLK_ENABLE();
  if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
  /* USER CODE END USART2_MspInit 0 */
    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();
      
    /**USART2 GPIO Configuration    
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART2 DMA Init */
    /* USART2_TX Init */
    hdma_usart2_tx.Instance = DMA1_Channel7;
    hdma_usart2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart2_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart2_tx.Init.Mode = DMA_NORMAL;
    hdma_usart2_tx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart2_tx) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart2_tx);

    /* USART2_RX Init */
    hdma_usart2_rx.Instance = DMA1_Channel6;
    hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart2_rx.Init.Mode = DMA_NORMAL; //单次模式(串口打印只能使用单次模式)
    hdma_usart2_rx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart2_rx) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart2_rx);

  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  } 
	if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */
    __HAL_RCC_GPIOB_CLK_ENABLE();
  /* USER CODE END USART3_MspInit 0 */
    /* USART3 clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();
  
    /**USART3 GPIO Configuration    
    PB10     ------> USART2_TX
    PB11     ------> USART2_RX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USART2 DMA Init */
    /* USART2_TX Init */
    hdma_usart3_tx.Instance = DMA1_Channel2;
    hdma_usart3_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart3_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart3_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart3_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart3_tx.Init.Mode = DMA_NORMAL;//单次模式(串口打印只能使用单次模式)
    hdma_usart3_tx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart3_tx) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart3_tx);

    /* USART2_RX Init */
    hdma_usart3_rx.Instance = DMA1_Channel3;
    hdma_usart3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart3_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart3_rx.Init.Mode = DMA_NORMAL;
    hdma_usart3_rx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart3_rx) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart3_rx);
   
  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }  
}

void My_UART_SetConfig(UART_HandleTypeDef *huart) {
	
 uint32_t tmpreg = 0x00U;

  /* Check the parameters */
  assert_param(IS_UART_BAUDRATE(huart->Init.BaudRate));
  assert_param(IS_UART_STOPBITS(huart->Init.StopBits));
  assert_param(IS_UART_PARITY(huart->Init.Parity));
  assert_param(IS_UART_MODE(huart->Init.Mode));

  /*------- UART-associated USART registers setting : CR2 Configuration ------*/
  /* Configure the UART Stop Bits: Set STOP[13:12] bits according 
   * to huart->Init.StopBits value */
  MODIFY_REG(huart->Instance->CR2, USART_CR2_STOP, huart->Init.StopBits);

  /*------- UART-associated USART registers setting : CR1 Configuration ------*/
  /* Configure the UART Word Length, Parity and mode: 
     Set the M bits according to huart->Init.WordLength value 
     Set PCE and PS bits according to huart->Init.Parity value
     Set TE and RE bits according to huart->Init.Mode value
     Set OVER8 bit according to huart->Init.OverSampling value */

#if defined(USART_CR1_OVER8)
  tmpreg |= (uint32_t)huart->Init.WordLength | huart->Init.Parity | huart->Init.Mode | huart->Init.OverSampling;
  MODIFY_REG(huart->Instance->CR1, 
             (uint32_t)(USART_CR1_M | USART_CR1_PCE | USART_CR1_PS | USART_CR1_TE | USART_CR1_RE | USART_CR1_OVER8), 
             tmpreg);
#else
  tmpreg |= (uint32_t)huart->Init.WordLength | huart->Init.Parity | huart->Init.Mode;
  MODIFY_REG(huart->Instance->CR1, 
             (uint32_t)(USART_CR1_M | USART_CR1_PCE | USART_CR1_PS | USART_CR1_TE | USART_CR1_RE), 
             tmpreg);
#endif /* USART_CR1_OVER8 */

  /*------- UART-associated USART registers setting : CR3 Configuration ------*/
  /* Configure the UART HFC: Set CTSE and RTSE bits according to huart->Init.HwFlowCtl value */
  MODIFY_REG(huart->Instance->CR3, (USART_CR3_RTSE | USART_CR3_CTSE), huart->Init.HwFlowCtl);

#if defined(USART_CR1_OVER8)
  /* Check the Over Sampling */
  if(huart->Init.OverSampling == UART_OVERSAMPLING_8)
  {
    /*-------------------------- USART BRR Configuration ---------------------*/
    if(huart->Instance == USART1)
    {
      huart->Instance->BRR = UART_BRR_SAMPLING8(HAL_RCC_GetPCLK2Freq(), huart->Init.BaudRate);
    }
    else
    {
      huart->Instance->BRR = UART_BRR_SAMPLING8(HAL_RCC_GetPCLK1Freq(), huart->Init.BaudRate);
    }
  }
  else
  {
    /*-------------------------- USART BRR Configuration ---------------------*/
    if(huart->Instance == USART1)
    {
      huart->Instance->BRR = UART_BRR_SAMPLING16(HAL_RCC_GetPCLK2Freq(), huart->Init.BaudRate);
    }
    else
    {
      huart->Instance->BRR = UART_BRR_SAMPLING16(HAL_RCC_GetPCLK1Freq(), huart->Init.BaudRate);
    }
  }
#else
  /*-------------------------- USART BRR Configuration ---------------------*/
  if(huart->Instance == USART1)
  {
    huart->Instance->BRR = UART_BRR_SAMPLING16(HAL_RCC_GetPCLK2Freq(), huart->Init.BaudRate);
  }
  else
  {
    huart->Instance->BRR = UART_BRR_SAMPLING16(HAL_RCC_GetPCLK1Freq(), huart->Init.BaudRate);
  }
#endif /* USART_CR1_OVER8 */
}
/* USART2 init function */

HAL_StatusTypeDef BSP_USARTX_Init(USART_TypeDef *USART_x,uint32_t Baud)
{  
	UART_HandleTypeDef *huart;
	
  huartx.Instance = USART_x; 
  huartx.Init.BaudRate = Baud;
  huartx.Init.WordLength = UART_WORDLENGTH_8B;
  huartx.Init.StopBits = UART_STOPBITS_1;
  huartx.Init.Parity = UART_PARITY_NONE;
  huartx.Init.Mode = UART_MODE_TX_RX;
  huartx.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huartx.Init.OverSampling = UART_OVERSAMPLING_16; 
	
	huart=&huartx;
//  if (HAL_UART_Init(&huartx) != HAL_OK)
//  {
//    _Error_Handler(__FILE__, __LINE__);
//  }  
	/* Check the UART handle allocation */
  if(huart == NULL)
  {
    return HAL_ERROR;
  }

  /* Check the parameters */
  if(huart->Init.HwFlowCtl != UART_HWCONTROL_NONE)
  {
    /* The hardware flow control is available only for USART1, USART2, USART3 */
    assert_param(IS_UART_HWFLOW_INSTANCE(huart->Instance));
    assert_param(IS_UART_HARDWARE_FLOW_CONTROL(huart->Init.HwFlowCtl));
  }
  else
  {
    assert_param(IS_UART_INSTANCE(huart->Instance));
  }
  assert_param(IS_UART_WORD_LENGTH(huart->Init.WordLength));
#if defined(USART_CR1_OVER8)
  assert_param(IS_UART_OVERSAMPLING(huart->Init.OverSampling));
#endif /* USART_CR1_OVER8 */
  
  if(huart->gState == HAL_UART_STATE_RESET)
  {  
    /* Allocate lock resource and initialize it */
    huart->Lock = HAL_UNLOCKED;

    /* Init the low level hardware */
    //HAL_UART_MspInit(huart);
		My_UART_MspInit(&huartx); 
  }

  huart->gState = HAL_UART_STATE_BUSY;

  /* Disable the peripheral */
  __HAL_UART_DISABLE(huart);
  
  /* Set the UART Communication parameters */
  My_UART_SetConfig(huart);
  
  /* In asynchronous mode, the following bits must be kept cleared: 
     - LINEN and CLKEN bits in the USART_CR2 register,
     - SCEN, HDSEL and IREN  bits in the USART_CR3 register.*/
  CLEAR_BIT(huart->Instance->CR2, (USART_CR2_LINEN | USART_CR2_CLKEN));
  CLEAR_BIT(huart->Instance->CR3, (USART_CR3_SCEN | USART_CR3_HDSEL | USART_CR3_IREN));
  
  /* Enable the peripheral */
  __HAL_UART_ENABLE(huart);
  
  /* Initialize the UART state */
  huart->ErrorCode = HAL_UART_ERROR_NONE;
  huart->gState= HAL_UART_STATE_READY;
  huart->RxState= HAL_UART_STATE_READY;
  
  return HAL_OK;
	
	
}


//void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
//{

//  if(uartHandle->Instance==USART2)
//  {
//  /* USER CODE BEGIN USART2_MspDeInit 0 */

//  /* USER CODE END USART2_MspDeInit 0 */
//    /* Peripheral clock disable */
//    __HAL_RCC_USART2_CLK_DISABLE();
//  
//    /**USART2 GPIO Configuration    
//    PA2     ------> USART2_TX
//    PA3     ------> USART2_RX 
//    */
//    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);

//    /* USART2 DMA DeInit */
//    HAL_DMA_DeInit(uartHandle->hdmatx);
//    HAL_DMA_DeInit(uartHandle->hdmarx);
//  /* USER CODE BEGIN USART2_MspDeInit 1 */

//  /* USER CODE END USART2_MspDeInit 1 */
//  }
//	 if(uartHandle->Instance==USART3)
//  {
//  /* USER CODE BEGIN USART2_MspDeInit 0 */

//  /* USER CODE END USART2_MspDeInit 0 */
//    /* Peripheral clock disable */
//    __HAL_RCC_USART3_CLK_DISABLE();
//  
//    /**USART2 GPIO Configuration    
//    PA2     ------> USART2_TX
//    PA3     ------> USART2_RX 
//    */
//    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10|GPIO_PIN_11);

//    /* USART2 DMA DeInit */
//    HAL_DMA_DeInit(uartHandle->hdmatx);
//    HAL_DMA_DeInit(uartHandle->hdmarx);
//  /* USER CODE BEGIN USART2_MspDeInit 1 */

//  /* USER CODE END USART2_MspDeInit 1 */
//  }
//	
//	 /*##-3- Disable the NVIC for UART ##########################################*/
//   HAL_NVIC_DisableIRQ(USARTx_IRQn);
//}  
void BSP_USRATX_DMA_Send(UART_HandleTypeDef *huart,uint8_t *string,uint16_t size)
{
 	uint32_t *temp=(uint32_t *)&string;  
	//等待串口发送完成
	while(!(huart->Instance->SR & UART_FLAG_TC)){
	
	} 
	__HAL_UART_CLEAR_FLAG(huart, UART_FLAG_TC);  
	//关闭DMA通道
	__HAL_DMA_DISABLE(huart->hdmatx); //CCR Bit0
  //关闭串口DMA 
  CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAT); //CR3 BIT7 
	 /* Clear all flags */
	huart->hdmatx->DmaBaseAddress->IFCR = (DMA_ISR_GIF1 << huart->hdmatx->ChannelIndex); 
	/* Configure DMA Channel data length */
	huart->hdmatx->Instance->CNDTR = size;  
	/* Configure DMA Channel destination address */
	huart->hdmatx->Instance->CPAR = (uint32_t)&huart->Instance->DR; 
	/* Configure DMA Channel source address */
	huart->hdmatx->Instance->CMAR = *temp; 
  //开启串口DMA
	SET_BIT(huart->Instance->CR3, USART_CR3_DMAT);	
	//开启DMA通道
	__HAL_DMA_ENABLE(huart->hdmatx);   
}
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
