#include "flash_nvs.h"

uint8_t FlashNVS::sectorBuffer[SECTOR_SIZE] __attribute__((section(".ccmram")));

FlashNVS::FlashNVS(flash* driver) : flashDriver(driver) {
    memset(&cache, 0, sizeof(cache));
    memset(sectorBuffer, 0xFF, SECTOR_SIZE);
}

FlashNVS::~FlashNVS() {
}

void FlashNVS::loadSectorToBuffer(uint32_t sector) {
    // Читаем сектор во временный буфер
    memset(sectorBuffer, 0xFF, SECTOR_SIZE);
    flashDriver->W25qxx_ReadSector(sectorBuffer, sector, 0, SECTOR_SIZE);

    // Обновляем кэш заголовка
    PageHeader* header = (PageHeader*)sectorBuffer;
    //STM_LOG("loadSectorToBuffer: sector=%lu, magic=0x%08lX, count=%d, crc=0x%04X", sector, header->magicNumber, header->count, header->crc);
}

void FlashNVS::saveSectorFromBuffer(uint32_t sector) {
    flashDriver->W25qxx_EraseSector(sector);
    HAL_Delay(1);
    flashDriver->W25qxx_WriteSector(sectorBuffer, sector, 0, SECTOR_SIZE);
    HAL_Delay(1);

    uint8_t* verifyBuffer = new uint8_t[SECTOR_SIZE];
    if (verifyBuffer) {
        flashDriver->W25qxx_ReadSector(verifyBuffer, sector, 0, SECTOR_SIZE);

        if (memcmp(verifyBuffer, sectorBuffer, SECTOR_SIZE) != 0) {
            STM_LOG(LOG_ERR "Failed to write sector %lu", sector);

            flashDriver->W25qxx_EraseSector(sector);
            HAL_Delay(1);
            flashDriver->W25qxx_WriteSector(sectorBuffer, sector, 0, SECTOR_SIZE);
            HAL_Delay(1);

            flashDriver->W25qxx_ReadSector(verifyBuffer, sector, 0, SECTOR_SIZE);
            if (memcmp(verifyBuffer, sectorBuffer, SECTOR_SIZE) != 0) {
                STM_LOG(LOG_ERR "Sector %lu verification failed after retry", sector);
            }
        }
        delete[] verifyBuffer;
    }
}

FlashNVS::ErrorCode FlashNVS::init(uint32_t startSector) {
    currentSector = startSector;
    dataSector = startSector + 1;

    // Читаем заголовок страницы
    loadSectorToBuffer(currentSector);
    memcpy(&cache.header, sectorBuffer, sizeof(PageHeader));
    
    STM_LOG("Init: Read header from sector %lu", currentSector);

    // Проверяем валидность страницы
    if (!isPageValid(&cache.header)) {
        STM_LOG("Init: Invalid page, creating new");

        memset(&cache.header, 0, sizeof(PageHeader));
        cache.header.magicNumber = MAGIC_NUMBER;
        cache.header.count = 0;
        cache.header.sectorNumber = currentSector;
        cache.header.crc = calculateCRC(&cache.header, offsetof(PageHeader, crc));

        memset(sectorBuffer, 0xFF, SECTOR_SIZE);
        memcpy(sectorBuffer, &cache.header, sizeof(PageHeader));

        saveSectorFromBuffer(currentSector);
        STM_LOG("Init: New page created with CRC=0x%04X", cache.header.crc);
    }
    else if (cache.header.count > MAX_ENTRIES_PER_PAGE/2) {
        // Если занято больше половины записей, выполняем уплотнение
        STM_LOG("Init: Too many entries (%d), performing compaction", cache.header.count);
        ErrorCode err = compactSector();
        if (err != OK) {
            STM_LOG("Init: Compaction failed with error %d", err);
            return err;
        }
        STM_LOG("Init: Compaction completed, entries reduced to %d", cache.header.count);
    }

    STM_LOG("Init: Sector ready, entries: %d", cache.header.count);
    return OK;
}

uint32_t FlashNVS::findNextFreeSector() {
    uint8_t tempBuffer[256];  // Используем маленький буфер для проверки начала сектора

    STM_LOG("Searching for free sector starting from %lu", dataSector);

    for (uint32_t sector = dataSector; sector < dataSector + 16; sector++) {
        // Читаем начало сектора
        flashDriver->W25qxx_ReadSector(tempBuffer, sector, 0, sizeof(tempBuffer));

        bool isEmpty = true;
        // Проверяем, содержит ли сектор только 0xFF
        for (uint32_t i = 0; i < sizeof(tempBuffer); i++) {
            if (tempBuffer[i] != 0xFF) {
                isEmpty = false;
                break;
            }
        }

        if (isEmpty) {
            STM_LOG("Found free sector: %lu", sector);
            return sector;
        }
    }

    STM_LOG("No free sectors found");
    return UINT32_MAX;
}

FlashNVS::ErrorCode FlashNVS::setString(const char* key, const char* value) {
    size_t length = strlen(value) + 1;
    return writeEntry(key, value, length, TYPE_STR);
}

FlashNVS::ErrorCode FlashNVS::getString(const char* key, char* out_value, uint32_t* length) {
    Entry entry;
    uint32_t offset;
    ErrorCode err = findEntry(key, &entry, &offset);

    if (err != OK) return err;
    if (entry.type != TYPE_STR) return ERROR_TYPE_MISMATCH;

    return readEntry(&entry, out_value, length);
}

FlashNVS::ErrorCode FlashNVS::setBlob(const char* key, const void* data, uint32_t length) {
    if (!data || !key || length == 0) {
        STM_LOG(LOG_ERR "Invalid parameters");
        return ERROR_INVALID_SIZE;
    }

    loadSectorToBuffer(currentSector);
    bool entryExists = false;
    uint32_t existingEntryIndex = 0;
    uint32_t existingDataOffset = sizeof(PageHeader) + MAX_ENTRIES_PER_PAGE * sizeof(Entry);
    uint32_t entryOffset = sizeof(PageHeader);

    for (uint16_t i = 0; i < cache.header.count; i++) {
        const Entry* currentEntry = (const Entry*)(sectorBuffer + entryOffset);
        if (strncmp(currentEntry->key, key, NVS_KEY_MAX_LENGTH) == 0) {
            entryExists = true;
            existingEntryIndex = i;
            break;
        }
        entryOffset += sizeof(Entry);
        existingDataOffset += currentEntry->length;
    }

    Entry newEntry;
    memset(&newEntry, 0, sizeof(Entry));
    strncpy(newEntry.key, key, sizeof(newEntry.key) - 1);
    newEntry.type = TYPE_BLOB;
    newEntry.length = length;
    newEntry.crc = calculateCRC(data, length);

    uint32_t newDataOffset;
    if (entryExists) {
        entryOffset = sizeof(PageHeader) + existingEntryIndex * sizeof(Entry);
        newDataOffset = existingDataOffset;
    } else {
        entryOffset = sizeof(PageHeader) + cache.header.count * sizeof(Entry);
        newDataOffset = sizeof(PageHeader) + MAX_ENTRIES_PER_PAGE * sizeof(Entry);
        uint32_t currentOffset = sizeof(PageHeader);
        for (uint16_t i = 0; i < cache.header.count; i++) {
            const Entry* currentEntry = (const Entry*)(sectorBuffer + currentOffset);
            newDataOffset += currentEntry->length;
            currentOffset += sizeof(Entry);
        }
    }

    if (newDataOffset + length > SECTOR_SIZE) {
        STM_LOG(LOG_ERR "Not enough space in sector");
        return ERROR_NO_SPACE;
    }

    memcpy(sectorBuffer + entryOffset, &newEntry, sizeof(Entry));
    memcpy(sectorBuffer + newDataOffset, data, length);

    if (!entryExists) {
        cache.header.count++;
    }

    cache.header.crc = calculateCRC(&cache.header, offsetof(PageHeader, crc));
    memcpy(sectorBuffer, &cache.header, sizeof(PageHeader));

    flashDriver->W25qxx_EraseSector(currentSector);
    HAL_Delay(10);
    flashDriver->W25qxx_WriteSector(sectorBuffer, currentSector, 0, SECTOR_SIZE);
    HAL_Delay(10);

    uint8_t* verifyBuffer = new uint8_t[SECTOR_SIZE];
    if (!verifyBuffer) {
        return ERROR_NO_SPACE;
    }

    flashDriver->W25qxx_ReadSector(verifyBuffer, currentSector, 0, SECTOR_SIZE);
    if (memcmp(sectorBuffer, verifyBuffer, SECTOR_SIZE) != 0) {
        STM_LOG(LOG_ERR "Sector verification failed");
        delete[] verifyBuffer;
        return ERROR_WRITE;
    }

    delete[] verifyBuffer;
    return OK;
}

FlashNVS::ErrorCode FlashNVS::getBlob(const char* key, void* data, uint32_t* length) {
    Entry entry;
    uint32_t offset;
    ErrorCode err = findEntry(key, &entry, &offset);

    if (err != OK) {
        return err;
    }

    if (entry.type != TYPE_BLOB) {
        STM_LOG(LOG_ERR "Wrong type %d for key '%s'", entry.type, key);
        return ERROR_TYPE_MISMATCH;
    }

    return readEntry(&entry, data, length);
}

FlashNVS::ErrorCode FlashNVS::writeEntry(const char* key, const void* value, uint32_t length, DataType type) {
    // Загружаем текущий сектор в буфер
    loadSectorToBuffer(currentSector);

    // Проверяем, нужно ли выполнить уплотнение
    if (cache.header.count >= MAX_ENTRIES_PER_PAGE) {
        ErrorCode err = compactSector();
        if (err != OK) return err;
        loadSectorToBuffer(currentSector);
    }

    // Подготавливаем запись
    Entry entry;
    strncpy(entry.key, key, NVS_KEY_MAX_LENGTH);
    entry.type = type;
    entry.length = length;
    entry.crc = calculateCRC(value, length);

    // Вычисляем смещения
    uint32_t entryOffset = sizeof(PageHeader) + cache.header.count * sizeof(Entry);
    uint32_t dataOffset = sizeof(PageHeader) + MAX_ENTRIES_PER_PAGE * sizeof(Entry);

    // Копируем данные в буфер
    memcpy(sectorBuffer + entryOffset, &entry, sizeof(Entry));
    memcpy(sectorBuffer + dataOffset, value, length);

    // Обновляем заголовок
    cache.header.count++;
    cache.header.crc = calculateCRC(&cache.header, sizeof(PageHeader) - sizeof(uint16_t));
    memcpy(sectorBuffer, &cache.header, sizeof(PageHeader));

    // Записываем буфер в сектор
    saveSectorFromBuffer(currentSector);

    return OK;
}

FlashNVS::ErrorCode FlashNVS::findEntry(const char* key, Entry* entry, uint32_t* offset) {
    loadSectorToBuffer(currentSector);
    PageHeader* header = (PageHeader*)sectorBuffer;

    if (!isPageValid(header)) {
        STM_LOG(LOG_ERR "Invalid header in sector %lu", currentSector);
        return ERROR_NOT_FOUND;
    }

    uint32_t entryOffset = sizeof(PageHeader);
    for (uint16_t i = 0; i < header->count; i++) {
        Entry* currentEntry = (Entry*)(sectorBuffer + entryOffset);
        if (strncmp(currentEntry->key, key, NVS_KEY_MAX_LENGTH) == 0) {
            memcpy(entry, currentEntry, sizeof(Entry));
            *offset = entryOffset;
            return OK;
        }
        entryOffset += sizeof(Entry);
    }

    return ERROR_NOT_FOUND;
}

FlashNVS::ErrorCode FlashNVS::readEntry(const Entry* entry, void* out_value, uint32_t* length) {
    if (*length < entry->length) {
        *length = entry->length;
        return ERROR_INVALID_SIZE;
    }

    loadSectorToBuffer(currentSector);
    uint32_t dataOffset = sizeof(PageHeader) + MAX_ENTRIES_PER_PAGE * sizeof(Entry);
    uint32_t entryOffset = sizeof(PageHeader);

    for (uint16_t i = 0; i < cache.header.count; i++) {
        const Entry* currentEntry = (const Entry*)(sectorBuffer + entryOffset);
        if (strncmp(currentEntry->key, entry->key, NVS_KEY_MAX_LENGTH) == 0) {
            memcpy(out_value, sectorBuffer + dataOffset, entry->length);
            uint16_t calculatedCRC = calculateCRC(out_value, entry->length);

            if (calculatedCRC != entry->crc) {
                STM_LOG(LOG_ERR "CRC verification failed");
                return ERROR_CRC;
            }

            *length = entry->length;
            return OK;
        }
        entryOffset += sizeof(Entry);
        dataOffset += currentEntry->length;
    }

    return ERROR_NOT_FOUND;
}

FlashNVS::ErrorCode FlashNVS::compactSector() {
    uint8_t* tempBuffer = new uint8_t[SECTOR_SIZE];
    if (!tempBuffer) {
        STM_LOG(LOG_ERR "Failed to allocate temporary buffer");
        return ERROR_NO_SPACE;
    }

    memset(tempBuffer, 0xFF, SECTOR_SIZE);

    PageHeader newHeader;
    newHeader.magicNumber = MAGIC_NUMBER;
    newHeader.count = 0;
    newHeader.sectorNumber = currentSector;

    uint32_t newEntryOffset = sizeof(PageHeader);
    uint32_t newDataOffset = sizeof(PageHeader) + MAX_ENTRIES_PER_PAGE * sizeof(Entry);

    char processedKeys[MAX_ENTRIES_PER_PAGE][NVS_KEY_MAX_LENGTH] = {0};
    int processedCount = 0;

    for (int16_t i = cache.header.count - 1; i >= 0; i--) {
        const Entry* currentEntry = (const Entry*)(sectorBuffer + sizeof(PageHeader) + i * sizeof(Entry));

        bool isProcessed = false;
        for (int j = 0; j < processedCount; j++) {
            if (strncmp(currentEntry->key, processedKeys[j], NVS_KEY_MAX_LENGTH) == 0) {
                isProcessed = true;
                break;
            }
        }

        if (isProcessed) {
            continue;
        }

        uint32_t oldDataOffset = sizeof(PageHeader) + MAX_ENTRIES_PER_PAGE * sizeof(Entry);
        for (uint16_t j = 0; j < i; j++) {
            const Entry* prevEntry = (Entry*)(sectorBuffer + sizeof(PageHeader) + j * sizeof(Entry));
            oldDataOffset += prevEntry->length;
        }

        memcpy(tempBuffer + newEntryOffset, currentEntry, sizeof(Entry));
        memcpy(tempBuffer + newDataOffset, sectorBuffer + oldDataOffset, currentEntry->length);

        strncpy(processedKeys[processedCount++], currentEntry->key, NVS_KEY_MAX_LENGTH);

        newEntryOffset += sizeof(Entry);
        newDataOffset += currentEntry->length;
        newHeader.count++;
    }

    newHeader.crc = calculateCRC(&newHeader, offsetof(PageHeader, crc));
    memcpy(tempBuffer, &newHeader, sizeof(PageHeader));

    flashDriver->W25qxx_EraseSector(currentSector);
    flashDriver->W25qxx_WriteSector(tempBuffer, currentSector, 0, SECTOR_SIZE);

    loadSectorToBuffer(currentSector);
    if (memcmp(tempBuffer, sectorBuffer, SECTOR_SIZE) != 0) {
        delete[] tempBuffer;
        STM_LOG(LOG_ERR "Compaction verification failed");
        return ERROR_WRITE;
    }

    delete[] tempBuffer;
    memcpy(&cache.header, &newHeader, sizeof(PageHeader));

    return OK;
}

bool FlashNVS::isPageValid(const PageHeader* header) {
    if (!header) {
        STM_LOG("Header is NULL");
        return false;
    }

    //STM_LOG("Header check: magic=0x%08lX (expected 0x%08lX), count=%d", header->magicNumber, MAGIC_NUMBER, header->count);

    if (header->magicNumber != MAGIC_NUMBER) {
        STM_LOG("Invalid magic number");
        return false;
    }

    if (header->count > MAX_ENTRIES_PER_PAGE) {
        STM_LOG("Invalid entry count: %d > %d", header->count, MAX_ENTRIES_PER_PAGE);
        return false;
    }

    // Вычисляем CRC без учета поля crc
    uint16_t calculatedCRC = calculateCRC(header, offsetof(PageHeader, crc));
    if (calculatedCRC != header->crc) {
        STM_LOG("Invalid CRC: calculated=0x%04X, stored=0x%04X", calculatedCRC, header->crc);
        return false;
    }

    //STM_LOG("Header is valid");
    return true;
}

bool FlashNVS::verifyData(const void* data, size_t length, uint16_t expectedCrc) {
    if (!data || length == 0) {
        return false;
    }

    uint16_t calculatedCRC = calculateCRC(data, length);
    STM_LOG("CRC verification: calculated=0x%04X, expected=0x%04X",
            calculatedCRC, expectedCrc);
    return calculatedCRC == expectedCrc;
}

uint16_t FlashNVS::calculateCRC(const void* data, size_t length) {
    uint16_t crc = 0xFFFF;
    const uint8_t* p = (const uint8_t*)data;
    
    while (length--) {
        uint8_t byte = *p++;
        crc ^= byte;
        for (int i = 0; i < 8; i++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;  // Полином для CRC-16-MODBUS
            } else {
                crc = crc >> 1;
            }
        }
    }
    
    return crc;
}
