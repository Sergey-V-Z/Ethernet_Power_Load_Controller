/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "lwip.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "flash_spi.h"
#include "Delay_us_DWT.h"
#include "LED.h"
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

/* USER CODE BEGIN PV */
settings_t settings = {0, 0x0E};
chName_t NameCH[MAX_CH_NAME];

uint32_t count_tic = 0; //для замеров времени выполнения кода

led LED_IPadr;
led LED_error;
led LED_OSstart;


//for i2c
//g_stat_t I2C_net[45];

bool resetSettings = false;

// for SPI Flash
extern SPI_HandleTypeDef hspi3;
pins_spi_t ChipSelect = {SPI3_CS_GPIO_Port, SPI3_CS_Pin};
pins_spi_t WriteProtect = {WP_GPIO_Port, WP_Pin};
pins_spi_t Hold = {HOLD_GPIO_Port, HOLD_Pin};
flash mem_spi;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
uint8_t ReadStraps();
void finishedBlink();
void timoutBlink();
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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  	  STM_LOG("Start CTRL_LED. %f", 0.01);

	HAL_GPIO_WritePin(R_GPIO_Port, R_Pin, GPIO_PIN_SET); // PC15 VD4
	HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_SET); // PC13 VD2
	HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, GPIO_PIN_SET); // PC14 VD3


	uint8_t endMAC = 0, IP = 100;
	// работаем с настройками из флешки
	HAL_GPIO_WritePin(eth_NRST_GPIO_Port, eth_NRST_Pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(HOLD_GPIO_Port, HOLD_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(WP_GPIO_Port, WP_Pin, GPIO_PIN_SET);

	mem_spi.Init(&hspi3, 0, ChipSelect, WriteProtect, Hold, false);

	mem_spi.Read(&settings);

	// если установлен джампер set
	// заходим в режим настройки
	bool StartSettings = false;
	for (int var = 0; var < 5; ++var) {

		if(HAL_GPIO_ReadPin(MAC_IP_Pin_GPIO_Port, MAC_IP_Pin_Pin)){
			StartSettings = true;
		}else{
			StartSettings = false;
			break;
		}

		HAL_Delay(30);
	}

	//режим настройки
	if(StartSettings){
		StartSettings = false;
		HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, GPIO_PIN_RESET); // PC14 VD3

		endMAC = ReadStraps();
		HAL_Delay(300);
		HAL_GPIO_WritePin(R_GPIO_Port, R_Pin, GPIO_PIN_RESET); // PC13 VD2

		// ждем снятия джампера или таймаута
		int time;
		bool Settings;
		for (time = 0; time < 600; ++time) {

			if(HAL_GPIO_ReadPin(MAC_IP_Pin_GPIO_Port, MAC_IP_Pin_Pin)){
				Settings = true;
			}else{
				Settings = false;
				break;
			}

			HAL_Delay(100);
		}

		if(!Settings){ // if pin settings is 0
			IP = ReadStraps();
			HAL_Delay(300);
			HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_RESET); // PC15 VD4

			if((settings.version != 0) | (settings.version != 0xFF)){ // если считанные настройки не пусты то перезаписываем только изменения

				if(endMAC != 0xFF){
					settings.MAC[5] = endMAC;
				}

				if(IP != 0xFF){
					settings.saveIP.ip[3] = IP;
				}

				mem_spi.W25qxx_EraseSector(0);
				mem_spi.Write(settings);
				mem_spi.Read(&settings);

				finishedBlink();

				// and reset system
				HAL_Delay(500);
				NVIC_SystemReset();
			}else{
				resetSettings = true; // else reset all settings
			}
		}else{

			timoutBlink();
			// and reset system
			HAL_Delay(1000);
			NVIC_SystemReset();
		}


	}

	if ((settings.version == 0) | (settings.version == 0xFF) | resetSettings)
	{
		STM_LOG("Start reset settings");

		settings.isON_from_settings = false;
		settings.IP_end_from_settings = 1;

		settings.DHCPset = true;
		settings.devices_depth = 0;

		settings.saveIP.ip[0] = 192;
		settings.saveIP.ip[1] = 168;
		settings.saveIP.ip[2] = 1;
		settings.saveIP.ip[3] = IP;

		settings.saveIP.mask[0] = 255;
		settings.saveIP.mask[1] = 255;
		settings.saveIP.mask[2] = 255;
		settings.saveIP.mask[3] = 0;

		settings.saveIP.gateway[0] = 192;
		settings.saveIP.gateway[1] = 168;
		settings.saveIP.gateway[2] = 1;
		settings.saveIP.gateway[3] = 1;

		settings.MAC[0] = 0x44;
		settings.MAC[1] = 0x84;
		settings.MAC[2] = 0x23;
		settings.MAC[3] = 0x84;
		settings.MAC[4] = 0x44;
		settings.MAC[5] = endMAC;

		settings.version = CURENT_VERSION;

		//setRange_i2c_dev(16, 8);
		mem_spi.W25qxx_EraseSector(0);
		mem_spi.Write(settings);
		mem_spi.Read(&settings);

		finishedBlink();
	}

	// настройка первоночального состояния канналов
	if (settings.isON_from_settings) { // если состояние нужно взять из настроек
		// ничего не делаем состояния уже загруженны
	} else {

		//иначе выключаем все канналы

	}

	// reset link
	for (int var = 0; var <= MAX_CH_NAME; ++var) {
		NameCH[var].dev = NULL;
		NameCH[var].Channel_number = 0xff;
	}

	// linking the channel name with the device and channel number
	for (int var = 0; var <= MAX_ADR_DEV; ++var) {
		// check device address
		if((settings.devices[var].Addr >= START_ADR_I2C) &&
				(settings.devices[var].Addr <= (START_ADR_I2C + MAX_ADR_DEV))){
			for (int i = 0; i < 3; ++i) {
				NameCH[settings.devices[var].ch[i].Name_ch].dev = &settings.devices[var];
				NameCH[settings.devices[var].ch[i].Name_ch].Channel_number = i;
			}
		}
	}


	mem_spi.SetUsedInOS(true); // switch to use in OS
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/* USER CODE BEGIN 4 */
uint8_t ReadStraps(){
	uint8_t tempStraps;

	//Bit0
	if (HAL_GPIO_ReadPin(MAC_b0_GPIO_Port, MAC_b0_Pin)) SET_BIT(tempStraps,1<<0);
	else CLEAR_BIT(tempStraps,1<<0);
	//Bit1
	if (HAL_GPIO_ReadPin(MAC_b1_GPIO_Port, MAC_b1_Pin)) SET_BIT(tempStraps,1<<1);
	else CLEAR_BIT(tempStraps,1<<1);
	//Bit2
	if (HAL_GPIO_ReadPin(MAC_b2_GPIO_Port, MAC_b2_Pin)) SET_BIT(tempStraps,1<<2);
	else CLEAR_BIT(tempStraps,1<<2);
	//Bit3
	if (HAL_GPIO_ReadPin(MAC_b3_GPIO_Port, MAC_b3_Pin)) SET_BIT(tempStraps,1<<3);
	else CLEAR_BIT(tempStraps,1<<3);
	//Bit4
	if (HAL_GPIO_ReadPin(MAC_b4_GPIO_Port, MAC_b4_Pin)) SET_BIT(tempStraps,1<<4);
	else CLEAR_BIT(tempStraps,1<<4);
	//Bit5
	if (HAL_GPIO_ReadPin(MAC_b5_GPIO_Port, MAC_b5_Pin)) SET_BIT(tempStraps,1<<5);
	else CLEAR_BIT(tempStraps,1<<5);
	//Bit6
	if (HAL_GPIO_ReadPin(MAC_b6_GPIO_Port, MAC_b6_Pin)) SET_BIT(tempStraps,1<<6);
	else CLEAR_BIT(tempStraps,1<<6);
	//Bit7
	if (HAL_GPIO_ReadPin(MAC_b7_GPIO_Port, MAC_b7_Pin)) SET_BIT(tempStraps,1<<7);
	else CLEAR_BIT(tempStraps,1<<7);

	return tempStraps;
}

void finishedBlink(){
#define  timeBetween 300

	// finished blink
	HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_SET); // PC15 VD4
	HAL_GPIO_WritePin(R_GPIO_Port, R_Pin, GPIO_PIN_SET); // PC13 VD2
	HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, GPIO_PIN_SET); // PC14 VD3


	for (int var = 0; var < 5; ++var) {
		HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, GPIO_PIN_RESET); // PC14 VD3
		HAL_Delay(timeBetween);
		HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, GPIO_PIN_SET); // PC14 VD3
		HAL_GPIO_WritePin(R_GPIO_Port, R_Pin, GPIO_PIN_RESET); // PC13 VD2
		HAL_Delay(timeBetween);
		HAL_GPIO_WritePin(R_GPIO_Port, R_Pin, GPIO_PIN_SET); // PC13 VD2
		HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_RESET); // PC15 VD4
		HAL_Delay(timeBetween);
		HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_SET); // PC15 VD4

	}

	HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_SET); // PC15 VD4
	HAL_GPIO_WritePin(R_GPIO_Port, R_Pin, GPIO_PIN_SET); // PC13 VD2
	HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, GPIO_PIN_SET); // PC14 VD3
}

void timoutBlink(){
	// timOut plink  all
	HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_RESET); // PC15 VD4
	HAL_GPIO_WritePin(R_GPIO_Port, R_Pin, GPIO_PIN_RESET); // PC13 VD2
	HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, GPIO_PIN_RESET); // PC14 VD3
	for (int var = 0; var < 5; ++var) {
		HAL_GPIO_TogglePin(B_GPIO_Port, B_Pin); // PC15 VD4
		HAL_GPIO_TogglePin(R_GPIO_Port, R_Pin); // PC13 VD2
		HAL_GPIO_TogglePin(G_GPIO_Port, G_Pin); // PC14 VD3
		HAL_Delay(800);
	}
}


/*
 * функция установки нового устройства
 * Addr - I2C адрес
 * CH	- один из трех канвлов
 * Name	- глобальное имя от 1 до 45
 */
int set_i2c_dev(uint8_t Addr, uint8_t CH, uint8_t Name){
	uint8_t ret = 0, dev = (Addr - START_ADR_I2C);

	// проверка входных данных
	if(CH > 2){
		return 1;
	}
	if((Name > MAX_CH_NAME)){
		return 2;
	}
	//если вышли за диапазон
	if((Addr <  START_ADR_I2C) || (Addr > (START_ADR_I2C + MAX_ADR_DEV))){
		return 3;
	}

	//mem_spi.W25qxx_EraseSector(0);
	NameCH[Name].dev = &settings.devices[dev];
	NameCH[Name].Channel_number = CH;

	// записываем данные в память и сохраняем на флешку
	settings.devices[dev].Addr = Addr;
	settings.devices[dev].AddrFromDev = 0;
	settings.devices[dev].ch[CH].Name_ch = Name;
	settings.devices[dev].ERR_counter = 0;
	settings.devices[dev].last_ERR = 0;
	settings.devices[dev].TypePCB = PCBType::NoInit ;

	settings.devices[dev].ch[CH].Current = 0;
	settings.devices[dev].ch[CH].IsOn = 0;
	settings.devices[dev].ch[CH].On_off = 0;
	settings.devices[dev].ch[CH].PWM = 0;
	settings.devices[dev].ch[CH].PWM_out = 0;
	//mem_spi.Write(settings);

	return ret;
}

/*
 * функция удаления устройства
 * Addr - I2C адрес
 * CH	- один из трех канвлов
 * Name	- глобальное имя от 1 до 45
 */
int del_Name_dev(uint8_t Name){
	uint8_t ret = 0;

	if((Name > 44)){
		return -2;
	}

	//mem_spi.W25qxx_EraseSector(0);
	// записываем данные в память и сохраняем на флешку
	//NameCH[Name].dev = &settings.devices[dev];
	uint8_t CH = NameCH[Name].Channel_number;

	// записываем данные в память и сохраняем на флешку
	NameCH[Name].dev->Addr = 0xff;
	NameCH[Name].dev->AddrFromDev = 0xff;
	NameCH[Name].dev->ch[CH].Name_ch = 0xff;
	NameCH[Name].dev->ERR_counter = 0xffffffff;
	NameCH[Name].dev->last_ERR = 0xffffffff;
	NameCH[Name].dev->TypePCB = PCBType::NoInit;

	NameCH[Name].dev->ch[CH].Current = 0xffff;
	NameCH[Name].dev->ch[CH].IsOn = 0xff;
	NameCH[Name].dev->ch[CH].On_off = 0xff;
	NameCH[Name].dev->ch[CH].PWM = 0xffffffff;
	NameCH[Name].dev->ch[CH].PWM_out = 0xffffffff;

	NameCH[Name].dev = NULL;
	NameCH[Name].Channel_number = 0xff;
	//mem_spi.Write(settings);



	return ret;
}

void setRange_i2c_dev(uint8_t startAddres, uint8_t quantity){
	// Clear all
	cleanAll_i2c_dev();

	uint8_t name_num = 0;
	for (int var = 0; var < quantity; ++var) {
		for (int ch = 0; ch < 3; ++ch) {
			set_i2c_dev(startAddres + var, ch, name_num);
			++name_num;
			settings.devices_depth++;
		}
	}
}

void cleanAll_i2c_dev(){
	// Clear all
	for (int var = 0; var <= MAX_CH_NAME; ++var) {
		del_Name_dev(var);
	}
	del_all_dev();
}

void del_all_dev() {
	for (int var = 0; var < MAX_ADR_DEV; ++var) {

		settings.devices[var].Addr = 0xff;
		settings.devices[var].AddrFromDev = 0xff;
		settings.devices[var].ERR_counter = 0xffffffff;
		settings.devices[var].last_ERR = 0xffffffff;
		settings.devices[var].TypePCB = PCBType::NoInit;

		for (int CH = 0; CH < 3; ++CH) {
			settings.devices[var].ch[CH].Current = 0xffff;
			settings.devices[var].ch[CH].IsOn = 0xff;
			settings.devices[var].ch[CH].On_off = 0xff;
			settings.devices[var].ch[CH].PWM = 0xffffffff;
			settings.devices[var].ch[CH].PWM_out = 0xffffffff;
			settings.devices[var].ch[CH].Name_ch = 0xff;
		}

	}
	settings.devices_depth = 0;
}

uint8_t log_tx_buffer[LOG_TX_BUF_SIZE+2];

void STM_LOG(const char* format, ...)
{
	while(DBG_PORT.gState != HAL_UART_STATE_READY);
	va_list args;
	int size = 0;

	va_start(args, format);
	//vsprintf((char *)log_tx_buffer, format, args);
	size = vsnprintf((char *)log_tx_buffer, LOG_TX_BUF_SIZE, format, args);
	va_end(args);

	// добавить \r
	log_tx_buffer[size] = '\r';
	log_tx_buffer[size + 1] = 0;

	HAL_UART_Transmit_DMA(&DBG_PORT, log_tx_buffer, strlen((const char *)log_tx_buffer));
}

static const uint8_t aucCRCHi[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40
};

static const uint8_t aucCRCLo[] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7,
    0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E,
    0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9,
    0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
    0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32,
    0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D,
    0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38,
    0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF,
    0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1,
    0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
    0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB,
    0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA,
    0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
    0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97,
    0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E,
    0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89,
    0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83,
    0x41, 0x81, 0x80, 0x40
};

uint16_t usMBCRC16(uint8_t *pucFrame, uint16_t usLen)
{
	uint8_t           ucCRCHi = 0xFF;
	uint8_t           ucCRCLo = 0xFF;
    int             iIndex;

    while( usLen-- )
    {
        iIndex = ucCRCLo ^ *( pucFrame++ );
        ucCRCLo = ( uint8_t )( ucCRCHi ^ aucCRCHi[iIndex] );
        ucCRCHi = aucCRCLo[iIndex];
    }
    return ( uint16_t )( ucCRCHi << 8 | ucCRCLo );
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	STM_LOG("Error handler");
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
