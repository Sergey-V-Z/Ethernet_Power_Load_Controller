/*
 * settings.cpp
 *
 *  Created on: Nov 6, 2024
 *      Author: Ierixon-HP
 */
/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "settings.h"
#include "flash_nvs.h"


bool readBridgeSettings(FlashNVS* nvsStorage, set_bridge_t& settings, UART_HandleTypeDef *default_serial) {
    if (!nvsStorage) {
        STM_LOG(LOG_ERR "NVS null");
        return false;
    }

    uint32_t size = sizeof(settings);
    FlashNVS::ErrorCode err = nvsStorage->getBlob(NVS_KEY_BRIDGE, &settings, &size);

    if (err == FlashNVS::OK && size == sizeof(settings)) {
        // Проверяем валидность mode_rs485
        if (settings.mode_rs485 == RTU || settings.mode_rs485 == STREAMER) {
            STM_LOG(LOG_OK " m:%d p:%u", settings.mode_rs485, settings.port);
            return true;
        }
    }

    // Загружаем дефолтные значения если данные невалидны
    //memcpy(&settings, &DEFAULT_BRIDGE_SETTINGS, sizeof(settings));
    settings.mode_rs485 = mode_bridge_t::RTU;
    settings.RS485 = default_serial;
    settings.port = 502;
    settings.reserv1 = 0;
    settings.reserv2 = 0;
    settings.reserv3 = 0;
    settings.reserv4 = 0;
    settings.reserv5 = 0;

    // Сохраняем дефолтные значения
    err = nvsStorage->setBlob(NVS_KEY_BRIDGE, &settings, sizeof(settings));
    if (err != FlashNVS::OK) {
        STM_LOG(LOG_ERR "def");
        return false;
    }

    STM_LOG(LOG_OK " def");
    return true;
}

bool writeBridgeSettings(FlashNVS* nvsStorage, const set_bridge_t& settings) {
    if (!nvsStorage) {
        STM_LOG(LOG_ERR "NVS null");
        return false;
    }

    // Проверяем валидность режима
    if (settings.mode_rs485 != RTU && settings.mode_rs485 != STREAMER) {
        STM_LOG(LOG_ERR "mode");
        return false;
    }

    FlashNVS::ErrorCode err = nvsStorage->setBlob(NVS_KEY_BRIDGE, &settings, sizeof(settings));
    if (err != FlashNVS::OK) {
        STM_LOG(LOG_ERR "save");
        return false;
    }

    STM_LOG(LOG_OK " m:%d", settings.mode_rs485);
    return true;
}

// Вспомогательная функция получения строкового описания режима
const char* getModeStr(mode_bridge_t mode) {
    return (mode == RTU) ? "RTU" : "STR";
}

bool initAndMigrateStorage(flash* storage, settings_t& settings, FlashNVS** nvsStorage) {
    STM_LOG("Init storage...");

    // 1. Инициализация флеш
    pins_spi_t ChipSelect = {SPI3_CS_GPIO_Port, SPI3_CS_Pin};
    pins_spi_t WriteProtect = {WP_GPIO_Port, WP_Pin};
    pins_spi_t Hold = {HOLD_GPIO_Port, HOLD_Pin};

    if (!storage->Init(&hspi3, 0, ChipSelect, WriteProtect, Hold, true)) {
        STM_LOG("Error: Failed to initialize flash");
        return false;
    }

    // 2. Создаем и инициализируем NVS
    try {
        *nvsStorage = new FlashNVS(storage);
        if ((*nvsStorage)->init() != FlashNVS::OK) {
            STM_LOG("Error: Failed to initialize NVS");
            delete *nvsStorage;
            *nvsStorage = nullptr;
            return false;
        }
    } catch (...) {
        STM_LOG("Error: Failed to create NVS instance");
        return false;
    }

    // 3. Пытаемся прочитать данные из NVS
    uint32_t settingsSize = sizeof(settings);
    FlashNVS::ErrorCode nvs_err = (*nvsStorage)->getBlob("settings", &settings, &settingsSize);

    if (nvs_err == FlashNVS::OK) {
        // Проверяем валидность данных из NVS
        if (settings.version != 0 && settings.version != 0xFF) {
            STM_LOG("Valid settings found in NVS storage");
            STM_LOG("Settings: v%d, DHCP: %d, IP: %d.%d.%d.%d",
                    settings.version,
                    settings.DHCPset,
                    settings.saveIP.ip[0],
                    settings.saveIP.ip[1],
                    settings.saveIP.ip[2],
                    settings.saveIP.ip[3]);
            return true;
        }
    }

    // 4. Если данные в NVS невалидны или отсутствуют - читаем старым способом
    STM_LOG("NVS data not found or invalid, reading from old storage");
    storage->Read(&settings);

    // 5. Проверяем валидность данных
    bool needDefaultSettings = false;
    if ((settings.version == 0) || (settings.version == 0xFF)) {
        STM_LOG("Invalid settings detected, using defaults");
        needDefaultSettings = true;
    }

    // 6. Если данные невалидны - заполняем значениями по умолчанию
    if (needDefaultSettings) {
        settings.isON_from_settings = false;
        settings.IP_end_from_settings = 1;
        settings.DHCPset = true;
        settings.devices_depth = 0;

        settings.saveIP.ip[0] = 192;
        settings.saveIP.ip[1] = 168;
        settings.saveIP.ip[2] = 1;
        settings.saveIP.ip[3] = 100;

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
        settings.MAC[5] = 0x01;

        settings.version = CURENT_VERSION;
    }

    // 7. Сохраняем данные в NVS
    STM_LOG("Saving settings to NVS...");
    FlashNVS::ErrorCode err = (*nvsStorage)->setBlob("settings", &settings, sizeof(settings));
    if (err != FlashNVS::OK) {
        STM_LOG("Failed to save settings to NVS, error: %d", err);
        delete *nvsStorage;
        *nvsStorage = nullptr;
        return false;
    }

    STM_LOG("Storage initialized successfully");
    STM_LOG("Settings: v%d, DHCP: %d, IP: %d.%d.%d.%d",
            settings.version,
            settings.DHCPset,
            settings.saveIP.ip[0],
            settings.saveIP.ip[1],
            settings.saveIP.ip[2],
            settings.saveIP.ip[3]);

    return true;
}

bool saveSettingsToNVS(FlashNVS* nvsStorage, const settings_t& settings) {
    if (!nvsStorage) {
        STM_LOG(LOG_ERR "NVS storage not initialized");
        return false;
    }

    // Get settings structure size
    uint32_t settingsSize = sizeof(settings);

    // Try to save settings to NVS
    FlashNVS::ErrorCode err = nvsStorage->setBlob("settings", &settings, settingsSize);

    if (err != FlashNVS::OK) {
        STM_LOG(LOG_ERR "Failed to save settings to NVS, error: %d", err);
        return false;
    }

    // Verify saved data
    settings_t verifySettings;
    uint32_t readSize = settingsSize;
    err = nvsStorage->getBlob("settings", &verifySettings, &readSize);

    if (err != FlashNVS::OK || readSize != settingsSize) {
        STM_LOG(LOG_ERR "Error verifying saved settings");
        return false;
    }

    // Compare saved data with original
    if (memcmp(&settings.MAC, &verifySettings.MAC, sizeof(settings.MAC)) != 0) {
        STM_LOG(LOG_ERR "Data verification failed - data mismatch");
        return false;
    }

    if (memcmp(&settings.DHCPset, &verifySettings.DHCPset, sizeof(settings.DHCPset)) != 0) {
        STM_LOG(LOG_ERR "Data verification failed - data mismatch");
        return false;
    }

    if (memcmp(&settings.saveIP, &verifySettings.saveIP, sizeof(settings.saveIP)) != 0) {
        STM_LOG(LOG_ERR "Data verification failed - data mismatch");
        return false;
    }

    if (memcmp(&settings.version, &verifySettings.version, sizeof(settings.version)) != 0) {
        STM_LOG(LOG_ERR "Data verification failed - data mismatch");
        return false;
    }

    STM_LOG("Settings successfully saved to NVS");
    return true;
}
