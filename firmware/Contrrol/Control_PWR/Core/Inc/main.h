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
#include "stdio.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define ID_STRING " Controll LED ETH ver1.2 11.07.23"

#define START_ADR_I2C 1
#define MAX_ADR_DEV 16 // 16 with 0
#define MAX_CH_NAME MAX_ADR_DEV * 3 // 48
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

typedef struct
{
	DEV_t devices[MAX_ADR_DEV];
	uint8_t	MAC[6];
	uint8_t isON_from_settings;
	uint8_t IP_end_from_settings;
	setIP_t	saveIP;
	uint8_t DHCPset;
	uint8_t version;

}settings_t;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
