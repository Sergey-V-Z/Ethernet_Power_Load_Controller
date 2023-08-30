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
#include <stdio.h>
#include <string.h>
#include "flash_spi.h"
#include "LED.h"
#include "lwip.h"
using namespace std;
#include <string>
#include "api.h"
#include <iostream>
#include <vector>
#include "device_API.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

extern settings_t settings;
extern chName_t NameCH[MAX_CH_NAME];
uint16_t sensBuff[8] = {0};
uint8_t sensState = 255; // битовое поле
bool rx_end = 1;

uint32_t freqSens = HAL_RCC_GetHCLKFreq()/30000u;
uint32_t pwmSens;

uint16_t adc_buffer[24] = {0};
uint16_t adc_buffer2[24] = {0};

uint16_t TX_buff[15]={0}; // buff
uint16_t RX_buff[24]={0}; // buff

extern led LED_IPadr;
extern led LED_error;
extern led LED_OSstart;

//структуры для netcon
extern struct netif gnetif;

//TCP_IP
string strIP;
string in_str;

//TCP for ModBUS
uint8_t         input_data[129] = {0};
uint8_t 		response[129] = {0};
uint32_t		SizeInModBus = 0;
struct netconn 	connectionForModBUS , newconnectionForModBUS;
struct netconn 	*connMB = &connectionForModBUS, *newconnMB = &newconnectionForModBUS; //contains info about connection inc. type, port, buf pointers etc.

//переменные переферии
uint32_t Start = 0;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern flash mem_spi;


//переменные для тестов

uint8_t txRedy = 1;


/* USER CODE END Variables */
osThreadId MainTaskHandle;
osThreadId LEDHandle;
osThreadId ethTasHandle;
osThreadId MBRTUTaskHandle;
osThreadId MBETHTaskHandle;
osSemaphoreId ADC_endHandle;
osSemaphoreId ADC_end2Handle;
osSemaphoreId Resive_USARTHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
// extern "C"
/* USER CODE END FunctionPrototypes */

void mainTask(void const * argument);
void led(void const * argument);
void eth_Task(void const * argument);
void mbrtuTask(void const * argument);
void mbethTask(void const * argument);

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

	/* definition and creation of Resive_USART */
	osSemaphoreDef(Resive_USART);
	Resive_USARTHandle = osSemaphoreCreate(osSemaphore(Resive_USART), 1);

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

	/* definition and creation of MBRTUTask */
	osThreadDef(MBRTUTask, mbrtuTask, osPriorityNormal, 0, 256);
	MBRTUTaskHandle = osThreadCreate(osThread(MBRTUTask), NULL);

	/* definition and creation of MBETHTask */
	osThreadDef(MBETHTask, mbethTask, osPriorityNormal, 0, 512);
	MBETHTaskHandle = osThreadCreate(osThread(MBETHTask), NULL);

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
	//uint8_t channelForName = 0;
	uint16_t Address = 0;
	/* Infinite loop */
	for(;;)
	{

		if(1){
			for (int var = 0; var < MAX_ADR_DEV; ++var) {

				if((settings.devices[var].Addr >= START_ADR_I2C) && (settings.devices[var].Addr <= (START_ADR_I2C + MAX_ADR_DEV))){

					for (int i = 0; i < 15; ++i) {
						TX_buff[i] = 0;
					}
					//передаем данные в устройство
					// ch0 *******************************************************

					TX_buff[0] = settings.devices[var].ch[0].PWM_out & 0xFF;
					TX_buff[1] = (settings.devices[var].ch[0].PWM_out >> 8) & 0xFF;
					TX_buff[2] = (settings.devices[var].ch[0].PWM_out >> 16) & 0xFF;
					TX_buff[3] = (settings.devices[var].ch[0].PWM_out >> 24) & 0xFF;

					TX_buff[4] = settings.devices[var].ch[0].On_off;

					// ch1 *******************************************************
					TX_buff[5] = settings.devices[var].ch[1].PWM_out & 0xFF;
					TX_buff[6] = (settings.devices[var].ch[1].PWM_out >> 8) & 0xFF;
					TX_buff[7] = (settings.devices[var].ch[1].PWM_out >> 16) & 0xFF;
					TX_buff[8] = (settings.devices[var].ch[1].PWM_out >> 24) & 0xFF;

					TX_buff[9] = settings.devices[var].ch[1].On_off;

					// ch2 *******************************************************
					TX_buff[10] = settings.devices[var].ch[2].PWM_out & 0xFF;
					TX_buff[11] = (settings.devices[var].ch[2].PWM_out >> 8) & 0xFF;
					TX_buff[12] = (settings.devices[var].ch[2].PWM_out >> 16) & 0xFF;
					TX_buff[13] = (settings.devices[var].ch[2].PWM_out >> 24) & 0xFF;

					TX_buff[14] = settings.devices[var].ch[2].On_off;


					status1 = HAL_UART_Receive_DMA(&huart1, (uint8_t*)RX_buff, 24);// Read data
					Address =  settings.devices[var].Addr  | 0x0100;
					status1 = HAL_UART_Transmit(&huart1, (uint8_t*)&Address, 1, 10);
					status1 = HAL_UART_Transmit(&huart1, (uint8_t*)TX_buff, 15, 10);
					if(status1 != HAL_OK){
						settings.devices[var].ERR_counter ++;
						settings.devices[var].last_ERR = status1;
						LED_error.LEDon();
						continue;
					}
					else{
						settings.devices[var].last_ERR = status1;
						LED_error.LEDoff();
					}

					for (int i = 0; i < 10; ++i) {
						if(rx_end){
							rx_end = 0;
							//буффер рассовываем по переменным (переделать в указатели)

							// ch0 *******************************************************

							settings.devices[var].ch[0].PWM =
									RX_buff[0] | (RX_buff[1] << 8) | (RX_buff[2] << 16) | (RX_buff[3] << 24);

							settings.devices[var].ch[0].Current = RX_buff[4] | (RX_buff[5] << 8);

							settings.devices[var].ch[0].IsOn = RX_buff[6];

							// ch1 *******************************************************

							settings.devices[var].ch[1].PWM =
									RX_buff[7] | (RX_buff[8] << 8) | (RX_buff[9] << 16) | (RX_buff[10] << 24);

							settings.devices[var].ch[1].Current = RX_buff[11] | (RX_buff[12] << 8);

							settings.devices[var].ch[1].IsOn = RX_buff[13];

							// ch2 *******************************************************

							settings.devices[var].ch[2].PWM =
									RX_buff[14] | (RX_buff[15] << 8) | (RX_buff[16] << 16) | (RX_buff[17] << 24);

							settings.devices[var].ch[2].Current = RX_buff[18] | (RX_buff[19] << 8);

							settings.devices[var].ch[2].IsOn = RX_buff[20];

							settings.devices[var].TypePCB = (PCBType)RX_buff[22];
							settings.devices[var].AddrFromDev = RX_buff[21];
							break;
						}else{
							osDelay(1);
						}

					}
				}

			}
			osDelay(1);
		}

		//osDelay(10);
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
						LED_IPadr.LEDon();
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
						LED_IPadr.LEDoff();
					} else netconn_delete(newconn);
					osDelay(20);
				}
			}
		}
		osDelay(1);
	}
	/* USER CODE END eth_Task */
}

/* USER CODE BEGIN Header_mbrtuTask */
/**
 * @brief Function implementing the MBRTUTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_mbrtuTask */
void mbrtuTask(void const * argument)
{
	/* USER CODE BEGIN mbrtuTask */
	HAL_StatusTypeDef status1;

	status1 = HAL_UARTEx_ReceiveToIdle_IT(&huart2, response, 128);// Read data

	/* Infinite loop */
	for(;;)
	{

		// ждем event от USART
		osSemaphoreWait(Resive_USARTHandle,osWaitForever);
		if(newconnMB->type == netconn_type::NETCONN_TCP){
			netconn_write(newconnMB,response,SizeInModBus,NETCONN_COPY);
			SizeInModBus = 0;
		}

		//osDelay(1000);
	}
	/* USER CODE END mbrtuTask */
}

/* USER CODE BEGIN Header_mbethTask */
/**
 * @brief Function implementing the MBETHTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_mbethTask */
void mbethTask(void const * argument)
{
	/* USER CODE BEGIN mbethTask */
	while(gnetif.ip_addr.addr == 0){osDelay(1);}	//ждем получение адреса

	//TCP connection vars
	err_t                err, accept_err;
	struct netbuf        buffer;
	struct netbuf 	*buf = &buffer; //bufferized input data
	void 		*in_data = NULL;
	uint16_t 		data_size = 0;

	//osSemaphoreWait(ModBusEndHandle,1000);
	//int32_t SemRet = 0;
	//sizeH = xPortGetMinimumEverFreeHeapSize();
	/* Infinite loop */
	for(;;)
	{
		connMB = netconn_new(NETCONN_TCP);
		if (connMB!=NULL)
		{
			err = netconn_bind(connMB,NULL,8010);//assign port number to connection
			if (err==ERR_OK)
			{
				netconn_listen(connMB);//set port to listening mode
				while(1)
				{
					accept_err=netconn_accept(connMB,&newconnMB);//suspend until new connection
					if (accept_err==ERR_OK)
					{
						while (netconn_recv(newconnMB,&buf)==ERR_OK)//suspend until data received
						{
							do
							{
								netbuf_data(buf,&in_data,&data_size);//get pointer and data size of the buffer
								memcpy((void*)input_data,in_data,data_size);

								//netconn_write(newconnMB,response,4,NETCONN_COPY);
								HAL_GPIO_WritePin(DE_M_GPIO_Port, DE_M_Pin, GPIO_PIN_SET); //включить на передачу
								HAL_UART_Transmit(&huart2, input_data, data_size, 100); //Отправляем данные в USART
								HAL_GPIO_WritePin(DE_M_GPIO_Port, DE_M_Pin, GPIO_PIN_RESET); //включить на прием

							} while (netbuf_next(buf) >= 0);
							netbuf_delete(buf);
						}
						netconn_close(newconnMB);
						netconn_delete(newconnMB);
					} else netconn_delete(newconnMB);
					osDelay(20);
				}
			}
		}
		osDelay(1000);
	}
	/* USER CODE END mbethTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){

	if(huart->Instance == USART1){

	}
	if(huart->Instance == USART2){

	}

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

	if(huart->Instance == USART1){
		HAL_UART_DMAStop(huart);
		rx_end = 1;
	}
	if(huart->Instance == USART2){


	}

}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
	if(huart->Instance == USART2){
		SizeInModBus = Size;
		HAL_UARTEx_ReceiveToIdle_IT(&huart2, response, 128);// Read data
		osSemaphoreRelease(Resive_USARTHandle);
	}
}
/* USER CODE END Application */
