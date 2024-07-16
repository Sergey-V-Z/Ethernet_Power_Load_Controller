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
#include <iomanip>
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

void actoin_ip(cJSON *obj, bool save);
void actoin_ch_set(cJSON *obj);
void actoin_cmd(cJSON *obj);
void actoin_settings_data(cJSON *obj);

//структуры для netcon
extern struct netif gnetif;

//TCP_IP
string strIP;
string in_str;

// обмен данными с компом
uint8_t message_rx[message_RX_LENGTH];
uint8_t UART6_rx[UART6_RX_LENGTH];
uint16_t indx_message_rx = 0;
uint16_t indx_UART6_rx = 0;
uint16_t Size_message = 0;
uint16_t Start_index = 0;

//TCP for ModBUS
uint8_t         input_tcp_data[256] = {0};
uint8_t 		response[260] = {0};
uint8_t 		Modbut_to_TCP[260] = {0};
uint16_t		SizeInModBus = 0;
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
osThreadId uart_taskHandle;
osMessageQId rxDataUART2Handle;
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
void uart_Task(void const * argument);

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

  /* Create the queue(s) */
  /* definition and creation of rxDataUART2 */
  osMessageQDef(rxDataUART2, 16, uint8_t);
  rxDataUART2Handle = osMessageCreate(osMessageQ(rxDataUART2), NULL);

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

  /* definition and creation of uart_task */
  osThreadDef(uart_task, uart_Task, osPriorityNormal, 0, 1024);
  uart_taskHandle = osThreadCreate(osThread(uart_task), NULL);

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
						STM_LOG("Connect open");
						while ((accept_err=netconn_recv(newconn,&netbuf))==ERR_OK)//работаем до тех пор пока клиент не разорвет соеденение
						{

							do
							{
								netbuf_data(netbuf,&in_data,&data_size);//get pointer and data size of the buffer
								in_str.assign((char*)in_data, data_size);//copy in string
								/*-----------------------------------------------------------------------------------------------------------------------------*/
								STM_LOG("Get CMD %s", in_str.c_str());

								if (!in_str.empty()) {
									string resp = Сommand_execution(in_str);
									netconn_write(newconn, resp.c_str(), resp.size(), NETCONN_COPY);
								}

							} while (netbuf_next(netbuf) >= 0);
							netbuf_delete(netbuf);

						}
						netconn_close(newconn);
						netconn_delete(newconn);
						STM_LOG("Connect close");
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
    uint16_t usCRC16;
	/* Infinite loop */
	for(;;)
	{

		// ждем event от USART
		osSemaphoreWait(Resive_USARTHandle,osWaitForever);
		// если соеденение все еще установленно
		if(newconnMB->type == netconn_type::NETCONN_TCP){
			// проверяем crc
			usCRC16 = usMBCRC16(response, SizeInModBus-2);
			if(response[SizeInModBus-2] != ( uint8_t )( usCRC16 & 0xFF )){
				continue;
			}
			if(response[SizeInModBus-1] != ( uint8_t )( usCRC16 >> 8 )){
				continue;
			}
			// собираем сообщение
			Modbut_to_TCP[0] = 0;
			Modbut_to_TCP[1] = 0;
			Modbut_to_TCP[2] = 0;
			Modbut_to_TCP[3] = 0;
			Modbut_to_TCP[4] = (uint8_t) ((SizeInModBus-2) >> 8);
			Modbut_to_TCP[5] = (uint8_t) ((SizeInModBus-2) & 0xFF);

			memcpy(&Modbut_to_TCP[6], response, SizeInModBus-2);

			netconn_write(newconnMB, Modbut_to_TCP, 4+(SizeInModBus), NETCONN_COPY);
			SizeInModBus = 0;
		}else{
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
	uint8_t			mb_tcp_head[6];
	uint16_t usCRC16;
	//osSemaphoreWait(ModBusEndHandle,1000);
	//int32_t SemRet = 0;
	//sizeH = xPortGetMinimumEverFreeHeapSize();
	/* Infinite loop */
	for(;;)
	{
		connMB = netconn_new(NETCONN_TCP);
		if (connMB!=NULL)
		{
			err = netconn_bind(connMB,NULL,502);//assign port number to connection
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


								// вырезать данные для модбаса
								memcpy(mb_tcp_head, in_data, 6); // копируем заголовок
								memcpy((void*)input_tcp_data, ((uint8_t*)in_data)+6, data_size-6);

								uint16_t len = mb_tcp_head[4] << 8;
								len |= mb_tcp_head[5];

								// проверить пакет на длинну
								if( len >= 254 ){
									//netconn_write(newconnMB,response,4,NETCONN_COPY);
									return;
								}

								// расчитать crc
								usCRC16 = usMBCRC16(input_tcp_data,data_size-6);
								input_tcp_data[data_size-6] = ( uint8_t )( usCRC16 & 0xFF );
								input_tcp_data[data_size-5] = ( uint8_t )( usCRC16 >> 8 );

								// отправить

								HAL_GPIO_WritePin(DE_M_GPIO_Port, DE_M_Pin, GPIO_PIN_SET); //включить на передачу
								HAL_UART_Transmit(&huart2, input_tcp_data, data_size-4, 100); //Отправляем данные в USART
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

/* USER CODE BEGIN Header_uart_Task */
/**
* @brief Function implementing the uart_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_uart_Task */
void uart_Task(void const * argument)
{
  /* USER CODE BEGIN uart_Task */
	 /* USER CODE BEGIN uart_Task */
		//HAL_UART_Receive_DMA(&huart2, UART2_rx, UART2_RX_LENGTH);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart6, UART6_rx, UART6_RX_LENGTH);
		__HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT);
		//__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_TC);
		/* Infinite loop */
		for (;;) {
			// ожидать собщение
			osMessageGet(rxDataUART2Handle, osWaitForever);
			//uint32_t message_len = strlen((char*) message_rx);
			//HAL_UART_Transmit(&huart2, message_rx, message_len, HAL_MAX_DELAY);

			// парсим  json
			cJSON *json = cJSON_Parse((char*) message_rx);
			if (json != NULL) {
				cJSON *id = cJSON_GetObjectItemCaseSensitive(json, "id");
				cJSON *name_device = cJSON_GetObjectItemCaseSensitive(json, "name_device");
				cJSON *type_data = cJSON_GetObjectItemCaseSensitive(json, "type_data");
				cJSON *save_settings = cJSON_GetObjectItemCaseSensitive(json, "save_settings");
				cJSON *obj = cJSON_GetObjectItemCaseSensitive(json, "obj");

			if (cJSON_IsNumber(id) && cJSON_GetNumberValue(id) == ID_CTRL) {
				bool save_set = false;
				if (cJSON_IsTrue(save_settings)) {
					save_set = true;
				} else {
					save_set = false;
				}

				if (cJSON_IsNumber(type_data)) {
					switch (type_data->valueint) {
					case 1: // ip settings
						actoin_ip(obj, save_set);
						break;
					case 2: // chanels settings
						actoin_ch_set(obj);
						break;
					case 3:
						actoin_cmd(obj);
						break;
					case 4:
						actoin_settings_data(obj);
						//STM_LOG("Empty type_data num");
						break;
					default:
						STM_LOG("data type not registered");
						break;
					}
				} else {
					STM_LOG("Invalid type data");
				}
			} else {
				STM_LOG("id not valid");
			}

			cJSON_Delete(json);
		} else {
			STM_LOG("Invalid JSON");
		}
			osDelay(1);
		}
  /* USER CODE END uart_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void actoin_ip(cJSON *obj, bool save)
{
	cJSON *j_IP = cJSON_GetObjectItemCaseSensitive(obj, "IP");
	cJSON *j_setIP = cJSON_GetObjectItemCaseSensitive(obj, "setIP");
	cJSON *j_MAC = cJSON_GetObjectItemCaseSensitive(obj, "MAC");
	cJSON *j_setMAC = cJSON_GetObjectItemCaseSensitive(obj, "setMAC");
	cJSON *j_GATEWAY = cJSON_GetObjectItemCaseSensitive(obj, "GATEWAY");
	cJSON *j_setGATEWAY = cJSON_GetObjectItemCaseSensitive(obj, "setGATEWAY");
	cJSON *j_MASK = cJSON_GetObjectItemCaseSensitive(obj, "MASK");
	cJSON *j_setMASK = cJSON_GetObjectItemCaseSensitive(obj, "setMASK");
	cJSON *j_DNS = cJSON_GetObjectItemCaseSensitive(obj, "DNS");
	cJSON *j_setDNS = cJSON_GetObjectItemCaseSensitive(obj, "setDNS");
	cJSON *j_DHCP = cJSON_GetObjectItemCaseSensitive(obj, "DHCP");
	cJSON *j_setDHCP = cJSON_GetObjectItemCaseSensitive(obj, "setDHCP");

	try {
		if ((j_setIP != NULL) && cJSON_IsTrue(j_setIP)) {
			char sep = '.';
			std::string s = j_IP->valuestring;
			if (!s.empty()) {
				std::string sepIP[4];
				int i = 0;

				for (size_t p = 0, q = 0; (p != s.npos) || (i < 4); p = q, i++)
					sepIP[i] = s.substr(p + (p != 0),
							(q = s.find(sep, p + 1)) - p - (p != 0));

				// записать новые данные в во временную переменную и после проверки на исключение выставить в настройки
				settings.saveIP.ip[0] = std::stoi(sepIP[0].c_str());
				settings.saveIP.ip[1] = std::stoi(sepIP[1].c_str());
				settings.saveIP.ip[2] = std::stoi(sepIP[2].c_str());
				settings.saveIP.ip[3] = std::stoi(sepIP[3].c_str());
			}

			//settings.saveIP.ip[0] = std::stoi();
		}

		if ((j_setMAC != NULL) && cJSON_IsTrue(j_setMAC)) {
			char sep = ':';
			std::string s = j_MAC->valuestring;
			if (!s.empty()) {
				std::string sepMAC[6];
				int i = 0;

				for (size_t p = 0, q = 0; (p != s.npos) || (i < 6); p = q, i++)
					sepMAC[i] = s.substr(p + (p != 0),
							(q = s.find(sep, p + 1)) - p - (p != 0));

				size_t pos = 0;
				settings.MAC[0] = std::stoi(sepMAC[0].c_str(), &pos, 16);
				settings.MAC[1] = std::stoi(sepMAC[1].c_str(), &pos, 16);
				settings.MAC[2] = std::stoi(sepMAC[2].c_str(), &pos, 16);
				settings.MAC[3] = std::stoi(sepMAC[3].c_str(), &pos, 16);
				settings.MAC[4] = std::stoi(sepMAC[4].c_str(), &pos, 16);
				settings.MAC[5] = std::stoi(sepMAC[5].c_str(), &pos, 16);
			}
		}

		if ((j_setGATEWAY != NULL) && cJSON_IsTrue(j_setGATEWAY)) {
			char sep = '.';
			std::string s = j_GATEWAY->valuestring;
			if (!s.empty()) {
				std::string sepGATEWAY[4];
				int i = 0;

				for (size_t p = 0, q = 0; (p != s.npos) || (i < 4); p = q, i++)
					sepGATEWAY[i] = s.substr(p + (p != 0),
							(q = s.find(sep, p + 1)) - p - (p != 0));

				settings.saveIP.gateway[0] = std::stoi(sepGATEWAY[0].c_str());
				settings.saveIP.gateway[1] = std::stoi(sepGATEWAY[1].c_str());
				settings.saveIP.gateway[2] = std::stoi(sepGATEWAY[2].c_str());
				settings.saveIP.gateway[3] = std::stoi(sepGATEWAY[3].c_str());
			}
		}

		if ((j_setMASK != NULL) && cJSON_IsTrue(j_setMASK)) {
			char sep = '.';
			std::string s = j_MASK->valuestring;
			if (!s.empty()) {
				std::string sepMASK[4];
				int i = 0;

				for (size_t p = 0, q = 0; (p != s.npos) || (i < 4); p = q, i++)
					sepMASK[i] = s.substr(p + (p != 0),
							(q = s.find(sep, p + 1)) - p - (p != 0));

				settings.saveIP.mask[0] = std::stoi(sepMASK[0].c_str());
				settings.saveIP.mask[1] = std::stoi(sepMASK[1].c_str());
				settings.saveIP.mask[2] = std::stoi(sepMASK[2].c_str());
				settings.saveIP.mask[3] = std::stoi(sepMASK[3].c_str());
			}
		}

		if ((j_setDNS != NULL) && cJSON_IsTrue(j_setDNS)) {
			char sep = '.';
			std::string s = j_DNS->valuestring;
			if (!s.empty()) {
				std::string sepDNS[4];
				int i = 0;

				for (size_t p = 0, q = 0; (p != s.npos) || (i < 4); p = q, i++)
					sepDNS[i] = s.substr(p + (p != 0),
							(q = s.find(sep, p + 1)) - p - (p != 0));

				//settings.[0] = std::stoi(sepDNS[0].c_str());
				//settings.[1] = std::stoi(sepDNS[1].c_str());
				//settings.[2] = std::stoi(sepDNS[2].c_str());
				//settings.[3] = std::stoi(sepDNS[3].c_str());

			}

		}

		if ((j_setDHCP != NULL) && cJSON_IsTrue(j_setDHCP)) {
			if (cJSON_IsTrue(j_DHCP)) {
				settings.DHCPset = 1;
			} else {
				settings.DHCPset = 0;
			}
		}

		STM_LOG("Settings set successful");
		// сохранение
		if (save) {
			STM_LOG("Save settings");
			mem_spi.W25qxx_EraseSector(0);
			osDelay(5);
			mem_spi.Write(settings);
		}

		// отправить ответ на хост
	} catch (...) {
		//ex.what()
		STM_LOG("err argument in motor parametrs");
		//return;
	}
}

void actoin_ch_set(cJSON *obj)
{
	if(settings.devices_depth != 0)
	{
		cJSON *j_out_obj = cJSON_CreateObject();
		cJSON *j_arr_obj = cJSON_CreateArray();

		for (int var = 0; var < settings.devices_depth; ++var) {
			cJSON *temp_obj = cJSON_CreateObject();
			uint8_t c = NameCH[var].Channel_number;
			cJSON_AddNumberToObject(temp_obj, "num", NameCH[var].dev->ch[c].Name_ch);
			cJSON_AddNumberToObject(temp_obj, "dev_addr", NameCH[var].dev->Addr);
			cJSON_AddNumberToObject(temp_obj, "ch_dev", c);

			cJSON_AddItemToArray(j_arr_obj, temp_obj);
		}

		cJSON_AddNumberToObject(j_out_obj, "id", ID_CTRL);
		cJSON_AddStringToObject(j_out_obj, "name_device", NAME);
		cJSON_AddNumberToObject(j_out_obj, "type_data", 2);
		cJSON_AddItemToObject(j_out_obj, "obj", j_arr_obj);

		char *out_str = cJSON_Print(j_out_obj);
		STM_LOG(out_str);

		free(out_str);
		cJSON_Delete(j_arr_obj);
		cJSON_Delete(j_out_obj);
	}
	else
	{
		STM_LOG("empty channels");
	}

}

void actoin_cmd(cJSON *obj)
{
	cJSON *id_cmd = cJSON_GetObjectItemCaseSensitive(obj, "id_cmd");

	int key = cJSON_GetNumberValue(id_cmd);

	switch (key) {
	case 1:{ //auto set
		cJSON *Addr_start = cJSON_GetObjectItemCaseSensitive(obj, "Addr_start");
		cJSON *Count_dev = cJSON_GetObjectItemCaseSensitive(obj, "Count_dev");

		int addres = cJSON_GetNumberValue(Addr_start);
		int count = cJSON_GetNumberValue(Count_dev);

		setRange_i2c_dev(addres, count);
		mem_spi.W25qxx_EraseSector(0);
		osDelay(5);
		mem_spi.Write(settings);

		STM_LOG("auto set end");
		break;
	}
	case 2:{ // add device
		cJSON *Num = cJSON_GetObjectItemCaseSensitive(obj, "Num");
		cJSON *Dev_addr = cJSON_GetObjectItemCaseSensitive(obj, "Dev_addr");
		cJSON *CH_dev = cJSON_GetObjectItemCaseSensitive(obj, "CH_dev");

		int num = cJSON_GetNumberValue(Num);
		int dev_addr = cJSON_GetNumberValue(Dev_addr);
		int ch_dev = cJSON_GetNumberValue(CH_dev);

		int ret = set_i2c_dev(dev_addr, ch_dev, num);
		switch (ret) {
		case 1:
			STM_LOG("Error not valid chanel data");
			break;
		case 2:
			STM_LOG("Error not empty cell");
			break;
		case 3:
			STM_LOG("Error not valid addres");
			break;
		case 4:
			STM_LOG("err not empty cell");
			break;

		}

		break;
	}
	case 3:{ // del device
		cJSON *Num = cJSON_GetObjectItemCaseSensitive(obj, "Num");
		int num = cJSON_GetNumberValue(Num);

		//del_Name_dev(num);
		STM_LOG("err empty cmd");
		break;
	}
	case 4:{ // on_off chanel
		cJSON *Num = cJSON_GetObjectItemCaseSensitive(obj, "Num");
		cJSON *PWM = cJSON_GetObjectItemCaseSensitive(obj, "PWM");
		cJSON *On = cJSON_GetObjectItemCaseSensitive(obj, "On");
		cJSON *All = cJSON_GetObjectItemCaseSensitive(obj, "All");

		int num = cJSON_GetNumberValue(Num);
		int pwm = cJSON_GetNumberValue(PWM);
		bool on = cJSON_IsTrue(On);
		bool all = cJSON_IsTrue(All);

		if (all) {
			for (int name = 0; name < MAX_CH_NAME; ++name) {
				if(NameCH[name].dev != NULL){
					uint8_t c = NameCH[name].Channel_number; // get channel number for this name
					NameCH[name].dev->ch[c].PWM_out = pwm;
					NameCH[name].dev->ch[c].On_off = on;
				}
			}
			STM_LOG("OK");
		} else {
			// mode set one channel
			if(NameCH[num].dev != NULL){
				uint8_t c = NameCH[num].Channel_number; // get channel number for this name
				NameCH[num].dev->ch[c].PWM_out = pwm;
				NameCH[num].dev->ch[c].On_off = on;
				STM_LOG("OK");
			}else{
				STM_LOG("NULL ptr dev");
			}
		}
		break;
	}
	case 5:{ // reboot
		STM_LOG("Rebooting...");
		osDelay(3000);
		NVIC_SystemReset();
		break;
	}
	default:{
		STM_LOG("Error id_cmd");
		break;
	}
	}
}
void actoin_settings_data(cJSON *obj)
{
	cJSON *j_all_settings_obj = cJSON_CreateObject();
	cJSON *obj_ch = cJSON_CreateArray();
	cJSON *obj_ip = cJSON_CreateObject();

	cJSON_AddNumberToObject(j_all_settings_obj, "id", ID_CTRL);
	cJSON_AddStringToObject(j_all_settings_obj, "name_device", NAME);
	cJSON_AddNumberToObject(j_all_settings_obj, "type_data", 4);

	// настройки ip
	string srtIP_to_host = std::to_string(settings.saveIP.ip[0])+"."+
							std::to_string(settings.saveIP.ip[1])+"."+
							std::to_string(settings.saveIP.ip[2])+ "."+
							std::to_string(settings.saveIP.ip[3]);
	cJSON_AddStringToObject(obj_ip, "IP", srtIP_to_host.c_str());

	char srtMAC_to_host[100];
	sprintf(srtMAC_to_host,"%x:%x:%x:%x:%x:%x",settings.MAC[0],settings.MAC[1],settings.MAC[2],
												settings.MAC[3],settings.MAC[4],settings.MAC[5]);
	cJSON_AddStringToObject(obj_ip, "MAC", srtMAC_to_host);

	string srtGATEWAY_to_host = std::to_string(settings.saveIP.gateway[0])+"."+
							std::to_string(settings.saveIP.gateway[1])+"."+
							std::to_string(settings.saveIP.gateway[2])+"."+
							std::to_string(settings.saveIP.gateway[3]);
	cJSON_AddStringToObject(obj_ip, "GATEWAY", srtGATEWAY_to_host.c_str());

	string srtMASK_to_host = std::to_string(settings.saveIP.mask[0])+"."+
							std::to_string(settings.saveIP.mask[1])+"."+
							std::to_string(settings.saveIP.mask[2])+"."+
							std::to_string(settings.saveIP.mask[3]);
	cJSON_AddStringToObject(obj_ip, "MASK", srtMASK_to_host.c_str());

	cJSON_AddStringToObject(obj_ip, "DNS", "0.0.0.0");

	if(settings.DHCPset)
	{
		cJSON_AddTrueToObject(obj_ip, "DHCP");
	}
	else
	{
		cJSON_AddFalseToObject(obj_ip, "DHCP");
	}


	// настройки каналов
	for (int name = 0; name < MAX_CH_NAME; ++name) {

		if (NameCH[name].dev != NULL) {
			cJSON *temp_obj = cJSON_CreateObject();

			uint8_t c = NameCH[name].Channel_number; // get channel number for this name

			cJSON_AddNumberToObject(temp_obj, "num", NameCH[name].dev->ch[c].Name_ch);
			cJSON_AddNumberToObject(temp_obj, "dev_addr", NameCH[name].dev->Addr);
			cJSON_AddNumberToObject(temp_obj, "ch_dev", NameCH[name].Channel_number);
			cJSON_AddNumberToObject(temp_obj, "PWM", NameCH[name].dev->ch[c].PWM_out);

			cJSON_AddItemToArray(obj_ch, temp_obj);

			//cJSON_Delete(temp_obj);
		} else {
			//STM_LOG("Error id_cmd");
			break;
		}
	}

	// отрпавка на хост
	cJSON_AddItemToObject(j_all_settings_obj, "obj_ch", obj_ch);
	cJSON_AddItemToObject(j_all_settings_obj, "obj_ip", obj_ip);

	char *str_to_host = cJSON_Print(j_all_settings_obj);

	STM_LOG("%s", str_to_host);

	cJSON_free(str_to_host);
	//cJSON_Delete(obj_ch);
	//cJSON_Delete(obj_ip);
	cJSON_Delete(j_all_settings_obj);

}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{

	if(huart->Instance == USART1){

	}
	if(huart->Instance == USART2){

	}

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	if(huart->Instance == USART1){
		HAL_UART_DMAStop(huart);
		rx_end = 1;
	}
	if(huart->Instance == USART2){


	}

}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if(huart->Instance == USART2){
		SizeInModBus = Size;
		HAL_UARTEx_ReceiveToIdle_IT(&huart2, response, 128);// Read data
		osSemaphoreRelease(Resive_USARTHandle);
	}
	if (huart->Instance == USART6) {

		while ( __HAL_UART_GET_FLAG(huart, UART_FLAG_TC) != SET) {
		};

		uint16_t Size_Data = Size - Start_index;

		HAL_UART_RxEventTypeTypeDef rxEventType;
		rxEventType = HAL_UARTEx_GetRxEventType(huart);
		switch (rxEventType) {
		case HAL_UART_RXEVENT_IDLE:
			//STM_LOG( "IDLE. Size:%d sd:%d sti:%d", Size, Size_Data, Start_index);
			// копировать с индекса сообщения
			memcpy(&message_rx[indx_message_rx], &UART6_rx[Start_index],
					Size_Data);

			//|| (message_rx[indx_message_rx + Size_Data - 1] == '\n')
			if ((message_rx[indx_message_rx + Size_Data - 1] == '\r')
					|| (message_rx[indx_message_rx + Size_Data - 1] == 0)) {
				message_rx[indx_message_rx + Size_Data] = 0;
				// выдать сигнал
				osMessagePut(rxDataUART2Handle, (uint32_t) indx_message_rx, 0);
				Size_message = 0;
				// обнулить индекс сообщения
				indx_message_rx = 0;
			} else {
				indx_message_rx += Size_Data;
			}

			Start_index = Size;

			//STM_LOG( "\n" );
			break;

		case HAL_UART_RXEVENT_HT:
			//STM_LOG( "HT Size:%d sd:%d sti:%d", Size, Size_Data, Start_index);
			break;

		case HAL_UART_RXEVENT_TC:
			//STM_LOG( "TC Size:%d sd:%d sti:%d", Size, Size_Data, Start_index);
			// скопировать в начало буфера
			memcpy(&message_rx[indx_message_rx], &UART6_rx[Start_index],
					Size_Data);
			// сохронить индекс сообщения
			indx_message_rx += Size_Data;
			Start_index = 0;
			break;

		default:
			STM_LOG("???");
			break;
		}

		HAL_UARTEx_ReceiveToIdle_DMA(huart, UART6_rx, UART6_RX_LENGTH);
		__HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT);
		//usart_rx_check(Size);
	}
}
/* USER CODE END Application */
