/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cJSON.h"
#include <stdarg.h>
#include <string.h>
#include "stdio.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;
extern SPI_HandleTypeDef hspi3;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define ID_STRING " Controll ver1.3"

#define START_ADR_I2C 1
#define MAX_ADR_DEV 16 // 16 with 0
#define MAX_CH_NAME MAX_ADR_DEV * 3 // 48

#define ARRAY_LEN(x)            (sizeof(x) / sizeof((x)[0]))

#define DBG_PORT huart6
#define LOG_TX_BUF_SIZE 1024*4

#define CURENT_VERSION 46
#define ID_CTRL 1
#define NAME "pwr controller"
#define LWIP_DHCP 1

#define UART6_RX_LENGTH 512
#define message_RX_LENGTH 512

#define NVS_KEY_BRIDGE "br"  // Вместо "bridge"
#define LOG_ERR "Err: "      // Вместо "Error: "
#define LOG_OK "OK"          // Вместо "Success" или "OK: "

// NVS key

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
int set_i2c_dev(uint8_t Addr, uint8_t CH, uint8_t Name);
int del_Name_dev(uint8_t Name);
void del_all_dev();
void setRange_i2c_dev(uint8_t startAddres, uint8_t quantity);
void cleanAll_i2c_dev();
uint8_t ReadStraps();
void finishedBlink();
void timoutBlink();

uint16_t usMBCRC16(uint8_t *pucFrame, uint16_t usLen);

void STM_LOG(const char* format, ...);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MAC_IP_Pin_Pin GPIO_PIN_2
#define MAC_IP_Pin_GPIO_Port GPIOE
#define R_Pin GPIO_PIN_13
#define R_GPIO_Port GPIOC
#define G_Pin GPIO_PIN_14
#define G_GPIO_Port GPIOC
#define B_Pin GPIO_PIN_15
#define B_GPIO_Port GPIOC
#define eth_NRST_Pin GPIO_PIN_0
#define eth_NRST_GPIO_Port GPIOA
#define MAC_b0_Pin GPIO_PIN_8
#define MAC_b0_GPIO_Port GPIOD
#define MAC_b1_Pin GPIO_PIN_9
#define MAC_b1_GPIO_Port GPIOD
#define MAC_b2_Pin GPIO_PIN_10
#define MAC_b2_GPIO_Port GPIOD
#define MAC_b3_Pin GPIO_PIN_11
#define MAC_b3_GPIO_Port GPIOD
#define MAC_b4_Pin GPIO_PIN_12
#define MAC_b4_GPIO_Port GPIOD
#define MAC_b5_Pin GPIO_PIN_13
#define MAC_b5_GPIO_Port GPIOD
#define MAC_b6_Pin GPIO_PIN_14
#define MAC_b6_GPIO_Port GPIOD
#define MAC_b7_Pin GPIO_PIN_15
#define MAC_b7_GPIO_Port GPIOD
#define SPI3_CS_Pin GPIO_PIN_15
#define SPI3_CS_GPIO_Port GPIOA
#define WP_Pin GPIO_PIN_0
#define WP_GPIO_Port GPIOD
#define HOLD_Pin GPIO_PIN_1
#define HOLD_GPIO_Port GPIOD
#define DE_M_Pin GPIO_PIN_4
#define DE_M_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */
typedef enum {
	NoInit = 0,
	LED_DRV = 10,
	RELE = 20,
	MOSFET_6CH = 30,
}PCBType;

typedef struct
{
	uint32_t 	PWM;			// шим на каннале
	uint32_t 	PWM_out;		// шим на каннале для передачи
	uint16_t 	Current;		// ток в канале
	uint8_t 	IsOn;			// включился ?
	uint8_t 	On_off;			// включение или выключение канаала
	uint8_t		Name_ch;		// "имя" каннала цыфра (1-45) в общей системе
}chanel;

typedef struct
{
	chanel 		ch[3];
	uint8_t 	Addr;		// адрес на котором рассположен канналы
	uint8_t 	AddrFromDev;
	uint32_t	ERR_counter;
	uint32_t	last_ERR;
	PCBType 	TypePCB;		// тип платы считывается с самой платы (группы)
}DEV_t;

typedef struct
{
	DEV_t* 		dev;
	uint8_t		Channel_number;	// номер канала в пределах одного i2c
}chName_t;

typedef struct
{
	uint8_t 	ip[4];// = {192, 168, 0, 2};
	uint8_t		mask[4];//  = {255, 255, 255, 0};
	uint8_t 	gateway[4];// = {192, 168, 0, 1};
}setIP_t;

#pragma pack(push, 1)
typedef struct
{
	DEV_t devices[MAX_ADR_DEV];
	uint8_t devices_depth;
	uint8_t	MAC[6];
	uint8_t isON_from_settings;
	uint8_t IP_end_from_settings;
	setIP_t	saveIP;
	uint8_t DHCPset;
	uint8_t version;

}settings_t;
#pragma pack(pop)

typedef enum {
	RTU = 0,
	STREAMER = 1,
}mode_bridge_t;

typedef struct
{
	mode_bridge_t mode_rs485;
	uint16_t port;
	UART_HandleTypeDef *RS485;
	uint32_t reserv1;
	uint32_t reserv2;
	uint32_t reserv3;
	uint32_t reserv4;
	uint32_t reserv5;

}set_bridge_t;

// Структура заголовка ModbusTCP
typedef struct {
    uint16_t transaction_id;
    uint16_t protocol_id;
    uint16_t length;
    uint8_t unit_id;
} ModbusTCPHeader;

extern set_bridge_t bridge_sett;

// Расчёт CRC16 для ModbusRTU
uint16_t calculate_crc16(uint8_t *buffer, int length);

// Проверка CRC входящего RTU пакета
uint8_t verify_rtu_crc(uint8_t *rtu_data, int rtu_length);

// Преобразование ModbusTCP в ModbusRTU
int tcp_to_rtu(uint8_t *tcp_data, int tcp_length, uint8_t *rtu_data, uint16_t *transaction_id);

// Преобразование ModbusRTU в ModbusTCP
int rtu_to_tcp(uint8_t *rtu_data, int rtu_length, uint8_t *tcp_data, uint16_t transaction_id);

// Вспомогательная функция для вывода содержимого пакета
void print_packet(uint8_t *data, int length, const char *prefix);


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
