/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
__IO uint32_t     Transfer_Direction = 0;
__IO uint32_t     Xfer_Complete = 0;
#define ln_ch 5 //chanels for adc

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern ADC_HandleTypeDef hadc;
extern TIM_HandleTypeDef htim3;
/* Buffer used for transmission */
uint16_t aTxBuffer[24] = {0};

/* Buffer used for reception */
uint16_t aRxBuffer[16] = {0};

uint16_t adc_buffer[ln_ch*2] = {0};

#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
/* Size of Transmission buffer */
uint16_t TXBUFFERSIZE = (COUNTOF(aTxBuffer));
/* Size of Reception buffer */
uint16_t RXBUFFERSIZE = (COUNTOF(aRxBuffer));
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t rxEnd = 1;
volatile uint32_t OwnAddr = 0;
uint8_t en1 = 0, en2 = 0, en3 = 0;
uint8_t adc_Flag = 0;
uint32_t PWM1 = 0, PWM2 = 0, PWM3 = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

	IN_GPIO_Init();

	//Bit0
	if (LL_GPIO_IsInputPinSet(A0_PWM_CH4_GPIO_Port, A0_PWM_CH4_Pin))
	{SET_BIT(OwnAddr,1<<0);}
	else {CLEAR_BIT(OwnAddr,1<<0);}
	//Bit1
	if (LL_GPIO_IsInputPinSet(A1_PWM_CH2_GPIO_Port, A1_PWM_CH2_Pin))
	{SET_BIT(OwnAddr,1<<1);}
	else {CLEAR_BIT(OwnAddr,1<<1);}
	//Bit2
	if (LL_GPIO_IsInputPinSet(A2_PWM_CH1_GPIO_Port, A2_PWM_CH1_Pin))
	{SET_BIT(OwnAddr,1<<2);}
	else {CLEAR_BIT(OwnAddr,1<<2);}
	//Bit3
	if (LL_GPIO_IsInputPinSet(A3_LED_GPIO_Port, A3_LED_Pin))
	{SET_BIT(OwnAddr,1<<3);}
	else {CLEAR_BIT(OwnAddr,1<<3);}
	//Bit4
	//SET_BIT(OwnAddr,1<<4);

	LL_GPIO_DeInit(A0_PWM_CH4_GPIO_Port);
	LL_GPIO_DeInit(A1_PWM_CH2_GPIO_Port);
	LL_GPIO_DeInit(A3_LED_GPIO_Port);


  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);


	if(HAL_ADCEx_Calibration_Start(&hadc) != HAL_OK){
		Error_Handler();
	}


	if(HAL_ADC_Start_DMA(&hadc, (uint32_t*)&adc_buffer, ln_ch) != HAL_OK){
		Error_Handler();
	}

	HAL_MultiProcessor_EnableMuteMode(&huart1);
	HAL_MultiProcessor_EnterMuteMode(&huart1);
	HAL_UART_Receive_DMA(&huart1, (uint8_t*)aRxBuffer, 16);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		if(rxEnd == 1){
			rxEnd = 0;
			PWM1 = aRxBuffer[1] |(aRxBuffer[2] << 8)|(aRxBuffer[3] << 16)|(aRxBuffer[4] << 24);
			en1 = aRxBuffer[5];
			PWM2 = aRxBuffer[6] |(aRxBuffer[7] << 8)|(aRxBuffer[8] << 16)|(aRxBuffer[9] << 24);
			en2 = aRxBuffer[10];
			PWM3 = aRxBuffer[11] |(aRxBuffer[12] << 8)|(aRxBuffer[13] << 16)|(aRxBuffer[14] << 24);
			en3 = aRxBuffer[15];

			for (int i = 0; i < RXBUFFERSIZE; ++i) {
				aRxBuffer[i] = 0;
			}
		}

		if(en1) htim3.Instance->CCR4 = PWM1;
		else htim3.Instance->CCR4 = 0;

		if(en2) htim3.Instance->CCR2 = PWM2;
		else htim3.Instance->CCR2 = 0;

		if(en3) htim3.Instance->CCR1 = PWM3;
		else htim3.Instance->CCR1 = 0;



		if (adc_Flag){
			//обработка

			// Запретить прерывания IRQ
			__disable_irq ();

			aTxBuffer[4] = adc_buffer[2]&0xFF;
			aTxBuffer[5] = (adc_buffer[2]>>8)&0xFF;

			aTxBuffer[11] = adc_buffer[1]&0xFF;
			aTxBuffer[12] = (adc_buffer[1]>>8)&0xFF;

			aTxBuffer[18] = adc_buffer[0]&0xFF;
			aTxBuffer[19] = (adc_buffer[0]>>8)&0xFF;

			aTxBuffer[0] = htim3.Instance->CCR4 & 0xFF;
			aTxBuffer[1] = (htim3.Instance->CCR4 >> 8) & 0xFF;
			aTxBuffer[2] = (htim3.Instance->CCR4 >> 16) & 0xFF;
			aTxBuffer[3] = (htim3.Instance->CCR4 >> 24) & 0xFF;
			aTxBuffer[6] = en1;

			aTxBuffer[7] = htim3.Instance->CCR2 & 0xFF;
			aTxBuffer[8] = (htim3.Instance->CCR2 >> 8) & 0xFF;
			aTxBuffer[9] = (htim3.Instance->CCR2 >> 16) & 0xFF;
			aTxBuffer[10] = (htim3.Instance->CCR2 >> 24) & 0xFF;
			aTxBuffer[13] = en2;

			aTxBuffer[14] = htim3.Instance->CCR1 & 0xFF;
			aTxBuffer[15] = (htim3.Instance->CCR1 >> 8) & 0xFF;
			aTxBuffer[16] = (htim3.Instance->CCR1 >> 16) & 0xFF;
			aTxBuffer[17] = (htim3.Instance->CCR1 >> 24) & 0xFF;
			aTxBuffer[20] = en3;
			aTxBuffer[21] = OwnAddr;
			aTxBuffer[22] = 10; // Type
			aTxBuffer[23] = 0;

			// Разрешить прерывания IRQ
			__enable_irq ();
			HAL_Delay(1);
			adc_Flag = 0;
			if(HAL_ADC_Start_DMA(&hadc, (uint32_t*)&adc_buffer, ln_ch) != HAL_OK){
				Error_Handler();
			}
		}
		HAL_Delay(1);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_1)
  {
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_HSI14_Enable();

   /* Wait till HSI14 is ready */
  while(LL_RCC_HSI14_IsReady() != 1)
  {

  }
  LL_RCC_HSI14_SetCalibTrimming(16);
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2, LL_RCC_PLL_MUL_12);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_SetSystemCoreClock(48000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
  LL_RCC_HSI14_EnableADCControl();
  LL_RCC_SetUSARTClockSource(LL_RCC_USART1_CLKSOURCE_PCLK1);
}

/* USER CODE BEGIN 4 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	HAL_GPIO_TogglePin(A3_LED_GPIO_Port, A3_LED_Pin);
	//HAL_UART_Receive_IT(huart, (uint8_t*)aRxBuffer, 15);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

	rxEnd = 1;
	HAL_GPIO_TogglePin(A3_LED_GPIO_Port, A3_LED_Pin);
	HAL_UART_Transmit_IT(huart, (uint8_t*)aTxBuffer, 24);
	HAL_UART_Receive_DMA(&huart1, (uint8_t*)aRxBuffer, 16);

}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
        if (huart == &huart1)
        {
                uint32_t er = HAL_UART_GetError(&huart1);

                if (er & HAL_UART_ERROR_PE)
                {
                	__HAL_UART_CLEAR_PEFLAG(&huart1);
                }
                if (er & HAL_UART_ERROR_NE)
                {
                	__HAL_UART_CLEAR_NEFLAG(&huart1);
                }
                if (er & HAL_UART_ERROR_FE)
                {
                	__HAL_UART_CLEAR_FEFLAG(&huart1);
                }
                if (er & HAL_UART_ERROR_ORE)
                {
                	__HAL_UART_CLEAR_OREFLAG(&huart1);
                }
                if (er & HAL_UART_ERROR_DMA)
                {
                	__HAL_UART_CLEAR_NEFLAG(&huart1);
                }
                huart->ErrorCode = HAL_UART_ERROR_NONE;
        }
}
/** Configure pins as
 */
void IN_GPIO_Init(void)
{

	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOF);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

	/**/
	LL_GPIO_ResetOutputPin(A0_PWM_CH4_GPIO_Port, A0_PWM_CH4_Pin);

	/**/
	LL_GPIO_ResetOutputPin(A1_PWM_CH2_GPIO_Port, A1_PWM_CH2_Pin);

	/**/
	LL_GPIO_ResetOutputPin(A2_PWM_CH1_GPIO_Port, A2_PWM_CH1_Pin);

	/**/
	LL_GPIO_ResetOutputPin(A3_LED_GPIO_Port, A3_LED_Pin);

	/**/
	GPIO_InitStruct.Pin = A0_PWM_CH4_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(A0_PWM_CH4_GPIO_Port, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = A1_PWM_CH2_Pin;
	LL_GPIO_Init(A1_PWM_CH2_GPIO_Port, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = A1_PWM_CH2_Pin;
	LL_GPIO_Init(A2_PWM_CH1_GPIO_Port, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = A3_LED_Pin;
	LL_GPIO_Init(A3_LED_GPIO_Port, &GPIO_InitStruct);

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	/* This is called after the conversion is completed */

	HAL_ADC_Stop_DMA(hadc);
	adc_Flag = 1;


}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	NVIC_SystemReset();
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{

	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
