/*
 * settings.h
 *
 *  Created on: Nov 6, 2024
 *      Author: Ierixon-HP
 */
#ifndef SRC_SETTINGS_H_
#define SRC_SETTINGS_H_

#include "main.h"
#include "cmsis_os.h"
#include "flash_nvs.h"

bool initAndMigrateStorage(flash *storage, settings_t &settings, FlashNVS** nvsStorage);
bool readBridgeSettings(FlashNVS* nvsStorage, set_bridge_t& settings, UART_HandleTypeDef *default_serial);
bool writeBridgeSettings(FlashNVS* nvsStorage, const set_bridge_t& settings);
const char* getModeStr(mode_bridge_t mode);
bool saveSettingsToNVS(FlashNVS* nvsStorage, const settings_t& settings);

#endif /* SRC_SETTINGS_H_ */
