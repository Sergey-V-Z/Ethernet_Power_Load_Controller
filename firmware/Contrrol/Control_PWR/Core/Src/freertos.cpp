/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "flash_spi.h"
#include "LED.h"
#include "lwip.h"
using namespace std;
#include <string>
#include "api.h"
#include <iostream>
#include <vector>
//#include "hcsr04_driver.h"
#include "device_API.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define START_ADR_I2C 32
#define MAX_ADR_I2C 15
#define MAX_CH_NAME 45
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

extern settings_t settings;
uint16_t sensBuff[8] = {0};
uint8_t sensState = 255; // битовое поле

uint32_t freqSens = HAL_RCC_GetHCLKFreq()/30000u;
uint32_t pwmSens;

uint16_t adc_buffer[24] = {0};
uint16_t adc_buffer2[24] = {0};

uint8_t TX_buff[15]={0}; // i2c buff
uint8_t RX_buff[21]={0}; // i2c buff

extern led LED_IPadr;
extern led LED_error;
extern led LED_OSstart;

//структуры для netcon
extern struct netif gnetif;

//TCP_IP
string strIP;
string in_str;

//переменные для обшей работы
uint32_t Start = 0;
extern I2C_HandleTypeDef hi2c1;
extern flash mem_spi;

//переменные для тестов

uint8_t txRedy = 1;

/* USER CODE END Variables */
osThreadId MainTaskHandle;
osThreadId LEDHandle;
osThreadId ethTasHandle;
osMutexId s2DistanceMutexHandle;
osMutexId mutexADCHandle;
osMutexId s1DistanceMutexHandle;
osMutexId setMutexHandle;
osSemaphoreId ADC_endHandle;
osSemaphoreId ADC_end2Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
// extern "C"
/* USER CODE END FunctionPrototypes */

void mainTask(void const * argument);
void led(void const * argument);
void eth_Task(void const * argument);

extern void MX_LWIP_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
extern "C" void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
	*ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
	*ppxIdleTaskStackBuffer = &xIdleStack[0];
	*pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
	/* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* definition and creation of s2DistanceMutex */
  osMutexDef(s2DistanceMutex);
  s2DistanceMutexHandle = osMutexCreate(osMutex(s2DistanceMutex));

  /* definition and creation of mutexADC */
  osMutexDef(mutexADC);
  mutexADCHandle = osMutexCreate(osMutex(mutexADC));

  /* definition and creation of s1DistanceMutex */
  osMutexDef(s1DistanceMutex);
  s1DistanceMutexHandle = osMutexCreate(osMutex(s1DistanceMutex));

  /* definition and creation of setMutex */
  osMutexDef(setMutex);
  setMutexHandle = osMutexCreate(osMutex(setMutex));

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of ADC_end */
  osSemaphoreDef(ADC_end);
  ADC_endHandle = osSemaphoreCreate(osSemaphore(ADC_end), 1);

  /* definition and creation of ADC_end2 */
  osSemaphoreDef(ADC_end2);
  ADC_end2Handle = osSemaphoreCreate(osSemaphore(ADC_end2), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of MainTask */
  osThreadDef(MainTask, mainTask, osPriorityNormal, 0, 256);
  MainTaskHandle = osThreadCreate(osThread(MainTask), NULL);

  /* definition and creation of LED */
  osThreadDef(LED, led, osPriorityNormal, 0, 128);
  LEDHandle = osThreadCreate(osThread(LED), NULL);

  /* definition and creation of ethTas */
  osThreadDef(ethTas, eth_Task, osPriorityNormal, 0, 768);
  ethTasHandle = osThreadCreate(osThread(ethTas), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_mainTask */
/**
 * @brief  Function implementing the MainTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_mainTask */
void mainTask(void const * argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();
  /* USER CODE BEGIN mainTask */

	HAL_StatusTypeDef status1;


	// Сбросим карту адрессов
	/*
	I2C_Map.CountAddresI2C = 0;
	for (int var = 0; var < 128; ++var) {
		I2C_Map.I2C_addr[var] = 0;
	}
	 */

	/*
	// Сканируем I2C и заносим в карту
	for(int i=1; i<128; i++)
	{
		int ret = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i<<1), 1, 5);
		if (ret != HAL_OK) // No ACK Received At That Address
		{  }
		else if(ret == HAL_OK)
		{
			I2C_Map.I2C_addr[I2C_Map.CountAddresI2C] = (uint16_t)(i<<1);
			I2C_Map.CountAddresI2C ++;
		}
	}
	 */
	/* Infinite loop */
	for(;;)
	{

		//status1 = HAL_I2C_IsDeviceReady(&hi2c1, 0x40 << 1, 3, 100);
		if(1){
			txRedy = 0;

			for (int var = 0; var < 45; ++var) {

				if((settings.Global_I2C[var].i2c_addr.I2C_addr >= START_ADR_I2C) && (settings.Global_I2C[var].i2c_addr.I2C_addr <= (START_ADR_I2C + MAX_ADR_I2C))){
					// запрос данных
					status1 = HAL_I2C_Master_Receive(&hi2c1,
							(uint16_t)settings.Global_I2C[var].i2c_addr.I2C_addr << 1,
							RX_buff,
							21, 100);

					if(status1 != HAL_OK){
						LED_error.LEDon();
					}
					else{
						LED_error.LEDoff();
					}

					//буффер рассовываем по переменным (переделать в указатели)

					// ch1 *******************************************************
					settings.Global_I2C[var].i2c_addr.led_Sett.PWM =
							RX_buff[0] | (RX_buff[1] << 8) | (RX_buff[2] << 16) | (RX_buff[3] << 24);

					settings.Global_I2C[var].i2c_addr.led_Sett.Current =
							RX_buff[4] | (RX_buff[5] << 8);

					settings.Global_I2C[var].i2c_addr.led_Sett.IsOn =
							RX_buff[6];

					// ch2 *******************************************************
					++var;
					settings.Global_I2C[var].i2c_addr.led_Sett.PWM =
							RX_buff[7] | (RX_buff[8] << 8) | (RX_buff[9] << 16) | (RX_buff[10] << 24);

					settings.Global_I2C[var].i2c_addr.led_Sett.Current =
							RX_buff[11] | (RX_buff[12] << 8);

					settings.Global_I2C[var].i2c_addr.led_Sett.IsOn =
							RX_buff[13];

					// ch3 *******************************************************
					++var;
					settings.Global_I2C[var].i2c_addr.led_Sett.PWM =
							RX_buff[14] | (RX_buff[15] << 8) | (RX_buff[16] << 16) | (RX_buff[17] << 24);

					settings.Global_I2C[var].i2c_addr.led_Sett.Current =
							RX_buff[18] | (RX_buff[19] << 8);

					settings.Global_I2C[var].i2c_addr.led_Sett.IsOn =
							RX_buff[20];

					//передаем данные в устройство

					// ch1 *******************************************************
					var = var - 2;

					TX_buff[0] = settings.Global_I2C[var].i2c_addr.led_Sett.PWM_out & 0xFF;
					TX_buff[1] = (settings.Global_I2C[var].i2c_addr.led_Sett.PWM_out >> 8) & 0xFF;
					TX_buff[2] = (settings.Global_I2C[var].i2c_addr.led_Sett.PWM_out >> 16) & 0xFF;
					TX_buff[3] = (settings.Global_I2C[var].i2c_addr.led_Sett.PWM_out >> 24) & 0xFF;

					TX_buff[4] = settings.Global_I2C[var].i2c_addr.led_Sett.On_off;

					// ch2 *******************************************************
					++var;
					TX_buff[5] = settings.Global_I2C[var].i2c_addr.led_Sett.PWM_out & 0xFF;
					TX_buff[6] = (settings.Global_I2C[var].i2c_addr.led_Sett.PWM_out >> 8) & 0xFF;
					TX_buff[7] = (settings.Global_I2C[var].i2c_addr.led_Sett.PWM_out >> 16) & 0xFF;
					TX_buff[8] = (settings.Global_I2C[var].i2c_addr.led_Sett.PWM_out >> 24) & 0xFF;

					TX_buff[9] = settings.Global_I2C[var].i2c_addr.led_Sett.On_off;

					// ch3 *******************************************************
					++var;
					TX_buff[10] = settings.Global_I2C[var].i2c_addr.led_Sett.PWM_out & 0xFF;
					TX_buff[11] = (settings.Global_I2C[var].i2c_addr.led_Sett.PWM_out >> 8) & 0xFF;
					TX_buff[12] = (settings.Global_I2C[var].i2c_addr.led_Sett.PWM_out >> 16) & 0xFF;
					TX_buff[13] = (settings.Global_I2C[var].i2c_addr.led_Sett.PWM_out >> 24) & 0xFF;

					TX_buff[14] = settings.Global_I2C[var].i2c_addr.led_Sett.On_off;

					osDelay(5);
					status1 = HAL_I2C_Master_Transmit(&hi2c1,
							(uint16_t)settings.Global_I2C[var].i2c_addr.I2C_addr << 1,
							TX_buff,
							15, 100);
					osDelay(5);
				}

			}

		}

		osDelay(100);
	}
  /* USER CODE END mainTask */
}

/* USER CODE BEGIN Header_led */
/**
 * @brief Function implementing the LED thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_led */
void led(void const * argument)
{
  /* USER CODE BEGIN led */
	/* Infinite loop */
	HAL_GPIO_WritePin(R_GPIO_Port, R_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_SET);

	LED_IPadr.Init(G_GPIO_Port, G_Pin);
	LED_error.Init(R_GPIO_Port, R_Pin);
	LED_OSstart.Init(B_GPIO_Port, B_Pin);

	LED_IPadr.setParameters(mode::ON_OFF);
	LED_error.setParameters(mode::ON_OFF);
	LED_OSstart.setParameters(mode::BLINK, 2000, 100);
	LED_OSstart.LEDon();

	//uint32_t tickcount = osKernelSysTick();// переменная для точной задержки
	/* Infinite loop */
	for(;;)
	{
		LED_IPadr.poll();
		LED_error.poll();
		LED_OSstart.poll();

		if(Start == 1){
			Start = 0;

		}
		if(Start == 2){
			Start = 0;


		}
		if(Start == 3){
			Start = 0;


		}
		if(Start == 4){
			Start = 0;

		}
		if(Start == 5){
			Start = 0;

		}
		if(Start == 6){
			Start = 0;

		}
		if(Start == 7){
			Start = 0;

		}
		osDelay(1);
		//taskYIELD();
		//osDelayUntil(&tickcount, 1); // задача будет вызываься ровро через 1 милисекунду
	}
  /* USER CODE END led */
}

/* USER CODE BEGIN Header_eth_Task */
/**
 * @brief Function implementing the ethTas thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_eth_Task */
void eth_Task(void const * argument)
{
  /* USER CODE BEGIN eth_Task */

	while(gnetif.ip_addr.addr == 0){osDelay(1);}	//ждем получение адреса
	LED_IPadr.LEDon();
	osDelay(1000);
	LED_IPadr.LEDoff();
	strIP = ip4addr_ntoa(&gnetif.ip_addr);

	//структуры для netcon
	struct netconn *conn;
	struct netconn *newconn;
	struct netbuf *netbuf;
	volatile err_t err, accept_err;
	//ip_addr_t local_ip;
	//ip_addr_t remote_ip;
	void 		*in_data = NULL;
	uint16_t 		data_size = 0;


	/* Infinite loop */
	for(;;)
	{

		conn = netconn_new(NETCONN_TCP);
		if (conn!=NULL)
		{
			err = netconn_bind(conn,NULL,81);//assign port number to connection
			if (err==ERR_OK)
			{
				netconn_listen(conn);//set port to listening mode
				while(1)
				{
					accept_err=netconn_accept(conn,&newconn);//suspend until new connection
					if (accept_err==ERR_OK)
					{
						//LED_IPadr.LEDon();
						while ((accept_err=netconn_recv(newconn,&netbuf))==ERR_OK)//работаем до тех пор пока клиент не разорвет соеденение
						{

							do
							{
								netbuf_data(netbuf,&in_data,&data_size);//get pointer and data size of the buffer
								in_str.assign((char*)in_data, data_size);//copy in string
								/*-----------------------------------------------------------------------------------------------------------------------------*/

								string resp = Сommand_execution(in_str);

								netconn_write(newconn, resp.c_str(), resp.size(), NETCONN_COPY);

							} while (netbuf_next(netbuf) >= 0);
							netbuf_delete(netbuf);

						}
						netconn_close(newconn);
						netconn_delete(newconn);
						//LED_IPadr.LEDoff();
					} else netconn_delete(newconn);
					osDelay(20);
				}
			}
		}
		osDelay(1);
	}
  /* USER CODE END eth_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void HAL_TIM_IC_CaptureCallback (TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM4) {
		//Sensor2._acknowledgeChannelCapture();
	}
	if (htim->Instance == TIM3) {
		//Sensor1._acknowledgeChannelCapture();
	}
}


void HAL_I2C_MasterTxCpltCallback (I2C_HandleTypeDef * hi2c)
{
	txRedy = 1;
	// TX Done .. Do Something!
}

void HAL_I2C_MasterRxCpltCallback (I2C_HandleTypeDef * hi2c)
{
	// RX Done .. Do Something!
}


/*
 * функция установки нового устройства
 * Addr - I2C адрес
 * CH	- один из трех канвлов
 * Name	- глобальное имя от 1 до 45
 */
int set_i2c_dev(uint8_t Addr, uint8_t CH, uint8_t Name){
	uint8_t ret = 0;
	// проверка входных данных
	if(CH > 2){
		return 1;
	}
	if((Name > 44)){
		return 2;
	}
	//если вышли за диапазон
	if((Addr <  START_ADR_I2C) || (Addr > (START_ADR_I2C + MAX_ADR_I2C))){
		return 3;
	}

	// если в ячейке записанно число которое входит в диапазон
	if ((settings.Global_I2C[Name].i2c_addr.I2C_addr >= START_ADR_I2C) &&
			(settings.Global_I2C[Name].i2c_addr.I2C_addr <= (START_ADR_I2C + MAX_ADR_I2C))) {
		if(settings.Global_I2C[Name].i2c_addr.led_Sett.Name_ch < 44){
			return 4;
		}
	}

	//mem_spi.W25qxx_EraseSector(0);
	// записываем данные в память и сохраняем на флешку
	settings.Global_I2C[Name].i2c_addr.I2C_addr = Addr;
	settings.Global_I2C[Name].Channel_number = CH;
	settings.Global_I2C[Name].i2c_addr.led_Sett.Name_ch = Name;

	settings.Global_I2C[Name].i2c_addr.led_Sett.Current = 0;
	settings.Global_I2C[Name].i2c_addr.led_Sett.IsOn = 0;
	settings.Global_I2C[Name].i2c_addr.led_Sett.On_off = 0;
	settings.Global_I2C[Name].i2c_addr.led_Sett.PWM = 0;
	settings.Global_I2C[Name].i2c_addr.led_Sett.PWM_out = 0;
	//mem_spi.Write(settings);

	return ret;
}

/*
 * функция удаления устройства
 * Addr - I2C адрес
 * CH	- один из трех канвлов
 * Name	- глобальное имя от 1 до 45
 */
int del_i2c_dev(uint8_t Name){
	uint8_t ret = 0;

	if((Name > 44)){
		return -2;
	}

	//mem_spi.W25qxx_EraseSector(0);
	// записываем данные в память и сохраняем на флешку
	settings.Global_I2C[Name].i2c_addr.I2C_addr = 0xff;
	settings.Global_I2C[Name].Channel_number = 0xff;
	settings.Global_I2C[Name].TypePCB = NoInit;

	settings.Global_I2C[Name].i2c_addr.led_Sett.Name_ch = 0xff;
	settings.Global_I2C[Name].i2c_addr.led_Sett.Current = 0xff;
	settings.Global_I2C[Name].i2c_addr.led_Sett.IsOn = 0xff;
	settings.Global_I2C[Name].i2c_addr.led_Sett.On_off = 0xff;
	settings.Global_I2C[Name].i2c_addr.led_Sett.PWM = 0xff;
	settings.Global_I2C[Name].i2c_addr.led_Sett.PWM_out = 0xff;
	//mem_spi.Write(settings);



	return ret;
}
/* USER CODE END Application */
