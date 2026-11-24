
/*
 * @file        ee.c
 * @brief       EEPROM Emulation library
 * @author      Nima Askari
 * @version     4.0.0
 * @license     See the LICENSE file in the root folder.
 *
 * @note        All my libraries are dual-licensed.
 *              Please review the licensing terms before using them.
 *              For any inquiries, feel free to contact me.
 *
 * @github      https://www.github.com/nimaltd
 * @linkedin    https://www.linkedin.com/in/nimaltd
 * @youtube     https://www.youtube.com/@nimaltd
 * @instagram   https://instagram.com/github.nimaltd
 *
 * Copyright (C) 2025 Nima Askari - NimaLTD. All rights reserved.
 */

/*************************************************************************************************/
/** Includes **/
/*************************************************************************************************/

#include "ee_config.h"
#include "ee.h"
#include <string.h>
#include <stddef.h>

//NEW ******************************************************************************************************

//CRC 32 calculation
#define EE_RECORD_CRC_OFFSET \
    ((uint32_t)offsetof(ee_record_t, crc))

static bool EE_IsSlotErased(uint32_t address);
static bool EE_IsSectorErased(uint32_t sector_index);

static uint32_t EE_CalculateCRC(const uint8_t *data, uint32_t length)
{
    uint32_t crc = 0xFFFFFFFFU;

    for (uint32_t i = 0; i < length; i++)
    {
        crc ^= data[i];

        for (uint32_t bit = 0; bit < 8; bit++)
        {
            if (crc & 1U)
            {
                crc = (crc >> 1) ^ 0xEDB88320U;
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    return ~crc;
}

//record validation
/*
 * Checks the record identifier and version.
 * Rejects records that weren't fully committed.
 * Recalculates the CRC and compares it with the stored CRC.
 * Checks that both sensors have valid lower and upper calibration limits.
 */
static bool EE_IsRecordValid(const ee_record_t *record)
{
    if (record->magic != EE_RECORD_MAGIC)
        return false;

    if (record->version != EE_RECORD_VERSION)
        return false;

    if (record->commit != EE_RECORD_COMMIT)
        return false;

    if (record->crc != EE_CalculateCRC(
            (const uint8_t *)record,
            EE_RECORD_CRC_OFFSET))
        return false;

    /* Validate sensor 1 calibration */
    if (record->calibration[0] >= record->calibration[2])
        return false;

    /* Validate sensor 2 calibration */
    if (record->calibration[1] >= record->calibration[3])
        return false;

    if (record->calibration[2] > 4095U ||
        record->calibration[3] > 4095U)
        return false;

    return true;
}



/**
 * @brief Programs one 32-bit word into Flash memory.
 * @param address Destination Flash address.
 * @param data 32-bit value to program.
 * @return true if programming and readback verification succeed.
 *
 * Note: Flash must already be unlocked before calling this function.
 */
static bool EE_ProgramWord(uint32_t address, uint32_t data)
{
    /* Flash word programming requires 4-byte alignment */
    if ((address & 0x03U) != 0U)
    {
        return false;
    }

    /* Never attempt to overwrite a previously programmed Flash word. */
    if (*(__IO uint32_t *)address != 0xFFFFFFFFU)
    {
        return false;
    }

    /* An erased word already contains this value. */
    if (data == 0xFFFFFFFFU)
    {
        return true;
    }

    /* Program one 32-bit word */
    if (HAL_FLASH_Program(
            FLASH_TYPEPROGRAM_WORD,
            address,
            (uint64_t)data) != HAL_OK)
    {
        return false;
    }

    /* Verify the programmed value */
    if (*(__IO uint32_t *)address != data)
    {
        return false;
    }

    return true;
}


/**
 * @brief Writes a complete record to Flash memory.
 * @param address Destination Flash address.
 * @param record Pointer to the prepared record.
 * @return true if the record is successfully written and validated.
 *
 * Note: Flash must already be unlocked before calling this function.
 */
static bool EE_WriteRecord(uint32_t address, const ee_record_t *record)
{


    /* Check arguments, alignment, and that the entire slot is unused. */
    if (record == NULL || (address & 0x03U) != 0U)
    {
        return false;
    }

    if (!EE_IsSlotErased(address))
    {
        return false;
    }

    const uint32_t *words = (const uint32_t *)record;

    /* Ensure the record is valid before writing */
    if (!EE_IsRecordValid(record))
    {
        return false;
    }

    /* Write words 0 through 8 (36 bytes) */
    for (uint32_t i = 0; i < 9U; i++)
    {
        if (!EE_ProgramWord(address + (i * 4U), words[i]))
        {
            return false;
        }
    }

    /* Write commit marker LAST */
    if (!EE_ProgramWord(
            address + offsetof(ee_record_t, commit),
            EE_RECORD_COMMIT))
    {
        return false;
    }

    /* Validate the completed record directly from Flash */
    if (!EE_IsRecordValid((const ee_record_t *)address))
    {
        return false;
    }

    return true;
}

/**
 * @brief Finds the newest valid calibration record across both Flash sectors.
 * @param latest_record Output copy of the newest valid record.
 * @param latest_address Output Flash address of that record.
 * @return true if a valid record was found.
 */
static bool EE_FindLatestRecord(
    ee_record_t *latest_record,
    uint32_t *latest_address)
{
    bool found = false;
    ee_record_t candidate_record;

    const uint32_t sector_addresses[2] =
    {
        EE_STORAGE_SECTOR_A_ADDRESS,
        EE_STORAGE_SECTOR_B_ADDRESS
    };

    const uint32_t sector_sizes[2] =
    {
        EE_STORAGE_SECTOR_A_SIZE,
        EE_STORAGE_SECTOR_B_SIZE
    };

    if (latest_record == NULL || latest_address == NULL)
    {
        return false;
    }

    /* Scan both storage sectors */
    for (uint32_t sector = 0; sector < 2U; sector++)
    {
        uint32_t start_address = sector_addresses[sector];
        uint32_t sector_size = sector_sizes[sector];

        /* Scan every possible record slot */
        for (uint32_t offset = 0;
             offset + sizeof(ee_record_t) <= sector_size;
             offset += sizeof(ee_record_t))
        {
            uint32_t address = start_address + offset;

            const ee_record_t *flash_record =
                (const ee_record_t *)address;

            /* Ignore invalid or incomplete records */
            if (!EE_IsRecordValid(flash_record))
            {
                continue;
            }

            /* Copy the candidate from Flash */
            memcpy(
                &candidate_record,
                flash_record,
                sizeof(ee_record_t));

            /* Select the newest sequence number */
            if (!found ||
                (int32_t)(candidate_record.sequence -
                          latest_record->sequence) > 0)
            {
                *latest_record = candidate_record;
                *latest_address = address;
                found = true;
            }
        }
    }

    return found;
}


/**
 * @brief Checks whether a Flash record slot is completely erased.
 */
static bool EE_IsSlotErased(uint32_t address)
{
    const uint32_t *words = (const uint32_t *)address;

    for (uint32_t i = 0; i < sizeof(ee_record_t) / 4U; i++)
    {
        if (words[i] != 0xFFFFFFFFU)
        {
            return false;
        }
    }

    return true;
}

/**
 * @brief Finds an unused record slot inside one verified active sector.
 *
 * @param free_address Output address of the unused slot.
 * @param sector_index Sector to search: 0 for sector A, 1 for sector B.
 * @return true if an unused slot was found.
 */
static bool EE_FindFreeSlotInSector(
    uint32_t sector_index,
    uint32_t *free_address)
{
    const uint32_t sector_addresses[2] =
    {
        EE_STORAGE_SECTOR_A_ADDRESS,
        EE_STORAGE_SECTOR_B_ADDRESS
    };

    const uint32_t sector_sizes[2] =
    {
        EE_STORAGE_SECTOR_A_SIZE,
        EE_STORAGE_SECTOR_B_SIZE
    };

    if (free_address == NULL || sector_index >= 2U)
    {
        return false;
    }

    for (uint32_t offset = 0;
         offset + sizeof(ee_record_t) <= sector_sizes[sector_index];
         offset += sizeof(ee_record_t))
    {
        uint32_t address = sector_addresses[sector_index] + offset;

        if (EE_IsSlotErased(address))
        {
            *free_address = address;
            return true;
        }
    }

    return false;
}

/**
 * @brief Checks whether an entire storage sector is erased.
 */
static bool EE_IsSectorErased(uint32_t sector_index)
{
    const uint32_t sector_addresses[2] =
    {
        EE_STORAGE_SECTOR_A_ADDRESS,
        EE_STORAGE_SECTOR_B_ADDRESS
    };

    const uint32_t sector_sizes[2] =
    {
        EE_STORAGE_SECTOR_A_SIZE,
        EE_STORAGE_SECTOR_B_SIZE
    };

    if (sector_index >= 2U)
    {
        return false;
    }

    const uint32_t *flash_words =
        (const uint32_t *)sector_addresses[sector_index];

    for (uint32_t i = 0U;
         i < sector_sizes[sector_index] / sizeof(uint32_t);
         i++)
    {
        if (flash_words[i] != 0xFFFFFFFFU)
        {
            return false;
        }
    }

    return true;
}


/**
 * @brief Erases one of the two calibration storage sectors.
 * @param sector_index 0 for sector A, 1 for sector B.
 * @return true if the erase succeeds and the sector is verified blank.
 *
 * Warning: The caller must ensure this sector does not contain
 * the only valid copy of the latest calibration record.
 */
static bool EE_EraseStorageSector(uint32_t sector_index)
{
    const uint32_t sector_numbers[2] =
    {
        EE_STORAGE_SECTOR_A_NUMBER,
        EE_STORAGE_SECTOR_B_NUMBER
    };

    FLASH_EraseInitTypeDef erase_config = {0};
    uint32_t sector_error = 0xFFFFFFFFU;
    bool success = false;

    if (sector_index >= 2U)
    {
        return false;
    }

    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        return false;
    }

    erase_config.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase_config.Banks = FLASH_BANK_1;
    erase_config.Sector = sector_numbers[sector_index];
    erase_config.NbSectors = 1U;
    erase_config.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    if (HAL_FLASHEx_Erase(&erase_config, &sector_error) == HAL_OK &&
        sector_error == 0xFFFFFFFFU)
    {
        success = true;
    }

    HAL_FLASH_Lock();

    if (!success)
    {
        return false;
    }

    /* Verify that every word in the sector is erased. */
    return EE_IsSectorErased(sector_index);
}

/**
 * @brief Prepares a calibration record in RAM before writing it to Flash.
 * @param record Destination record.
 * @param sequence Record sequence number.
 * @param calibration Four calibration values.
 * @return true if the prepared record is valid.
 */
static bool EE_PrepareRecord(
    ee_record_t *record,
    uint32_t sequence,
    const uint32_t calibration[4])
{
    if (record == NULL || calibration == NULL)
    {
        return false;
    }

    record->magic = EE_RECORD_MAGIC;
    record->version = EE_RECORD_VERSION;
    record->sequence = sequence;

    memcpy(
        record->calibration,
        calibration,
        sizeof(record->calibration));

    record->reserved = 0xFFFFFFFFU;

    /* Calculate CRC over all fields before the CRC field */
    record->crc = EE_CalculateCRC(
        (const uint8_t *)record,
        EE_RECORD_CRC_OFFSET);

    /* This marker is written to Flash separately, last */
    record->commit = EE_RECORD_COMMIT;

    return EE_IsRecordValid(record);
}




/*************************************************************************************************/
/** Private Definitions **/
/*************************************************************************************************/

/*************************************************************************************************/
/** Private Variables **/
/*************************************************************************************************/

static ee_t ee_handle;

/*************************************************************************************************/
/** Function Implementations **/
/*************************************************************************************************/

/*************************************************************************************************/
/** NEW EEINIT
 * @brief Initializes the Flash calibration storage module.
 * @param[in] data Pointer to the application calibration data.
 * @param[in] size Size of the application data in bytes.
 * @return true if initialization succeeds.
 */
bool ee_init(void *data, uint32_t size)
{
    assert_param(data != NULL);
    assert_param(size != 0U);

    /* Reset the storage handle */
    ee_handle.size = 0U;
    ee_handle.data = NULL;

    /* This record format stores exactly four uint32_t values */
    if (data == NULL || size != (4U * sizeof(uint32_t)))
    {
        return false;
    }

    /* Configure the two reserved Flash sectors */
    ee_handle.bank_number = FLASH_BANK_1;
    ee_handle.page_sector_number = EE_STORAGE_SECTOR_A_NUMBER;
    ee_handle.page_sector_size =
        EE_STORAGE_SECTOR_A_SIZE + EE_STORAGE_SECTOR_B_SIZE;
    ee_handle.address = EE_STORAGE_SECTOR_A_ADDRESS;

    ee_handle.size = size;
    ee_handle.data = (uint8_t *)data;

    return true;
}

/*************************************************************************************************/
/** NEW EECAPACITY
 * @brief Retrieves the total Flash storage capacity.
 * @return Total reserved Flash storage size in bytes.
 */
uint32_t ee_capacity(void)
{
    return EE_STORAGE_SECTOR_A_SIZE +
           EE_STORAGE_SECTOR_B_SIZE;
}

/*************************************************************************************************/
/** NEW EEFORMAT
 * @brief Erases both Flash sectors reserved for calibration storage.
 *
 * @retval true if both sectors are erased successfully, false otherwise.
 */
bool ee_format(void)
{
    bool sector_a_success;
    bool sector_b_success;

#ifdef HAL_ICACHE_MODULE_ENABLED
    HAL_ICACHE_Disable();
#endif

    /* Erase storage sector A (Sector 4) */
    sector_a_success = EE_EraseStorageSector(0U);

    /* Erase storage sector B (Sector 5) */
    sector_b_success = EE_EraseStorageSector(1U);

#ifdef HAL_ICACHE_MODULE_ENABLED
    HAL_ICACHE_Enable();
#endif

    return sector_a_success && sector_b_success;
}

/** NEW EEREAD
 * @brief Reads the newest valid calibration record from Flash.
 *
 * Searches both storage sectors for the newest valid record.
 * If no new-format record exists, attempts to load legacy
 * calibration data from the original storage address.
 */
ee_read_status_t ee_read(void)
{
    ee_record_t latest_record;
    uint32_t latest_address = 0U;
    uint32_t calibration[4];

    if (ee_handle.data == NULL ||
        ee_handle.size < sizeof(calibration))
    {
        return EE_READ_NOT_INITIALIZED;
    }

#ifdef HAL_ICACHE_MODULE_ENABLED
    HAL_ICACHE_Disable();
#endif

    /* First, search for the newest valid record */
    if (EE_FindLatestRecord(&latest_record, &latest_address))
    {
        memcpy(
            ee_handle.data,
            latest_record.calibration,
            sizeof(latest_record.calibration));

#ifdef HAL_ICACHE_MODULE_ENABLED
        HAL_ICACHE_Enable();
#endif
        return EE_READ_VALID_RECORD;
    }
    else
    {
        /*
         * No new-format record was found.
         * Try loading the legacy raw calibration data
         * from the original sector 5 storage address.
         */
        const uint32_t *legacy_data =
            (const uint32_t *)EE_STORAGE_SECTOR_B_ADDRESS;

        memcpy(calibration, legacy_data, sizeof(calibration));

        /* Validate legacy calibration before accepting it */
        if (calibration[0] < calibration[2] &&
            calibration[1] < calibration[3] &&
            calibration[2] <= 4095U &&
            calibration[3] <= 4095U)
        {
            memcpy(
                ee_handle.data,
                calibration,
                sizeof(calibration));

#ifdef HAL_ICACHE_MODULE_ENABLED
            HAL_ICACHE_Enable();
#endif
            return EE_READ_VALID_LEGACY;
        }
    }

#ifdef HAL_ICACHE_MODULE_ENABLED
    HAL_ICACHE_Enable();
#endif
    return EE_READ_NO_VALID_RECORD;
}

/** NEW EEWRITE
 * @brief Saves calibration data to Flash using append-only records.
 *
 * Writes a new record without erasing the sector containing the
 * latest valid calibration. When the active sector is full, the
 * alternate sector is erased and verified, the latest record is
 * copied, and then the new calibration is committed.
 */
ee_write_status_t ee_write_data(const void *data, uint32_t size)
{
    ee_record_t latest_record;
    ee_record_t new_record;
    ee_record_t copy_record;

    uint32_t latest_address = 0U;
    uint32_t free_address = 0U;
    uint32_t calibration[4];

    bool found_latest;

    if (ee_handle.data == NULL ||
        data == NULL ||
        size != sizeof(calibration))
    {
        return EE_WRITE_FAILED;
    }

    /* Copy the candidate so active application RAM can remain unchanged. */
    memcpy(calibration, data, sizeof(calibration));

    /* Validate the calibration before attempting to save */
    if (calibration[0] >= calibration[2] ||
        calibration[1] >= calibration[3] ||
        calibration[2] > 4095U ||
        calibration[3] > 4095U)
    {
        return EE_WRITE_FAILED;
    }

#ifdef HAL_ICACHE_MODULE_ENABLED
    HAL_ICACHE_Disable();
#endif

    found_latest = EE_FindLatestRecord(
        &latest_record,
        &latest_address);

    if (found_latest)
    {
        uint32_t active_sector =
            (latest_address >= EE_STORAGE_SECTOR_B_ADDRESS) ? 1U : 0U;

        /*
         * Normal case: append only inside the active sector. The alternate
         * sector is never trusted merely because part of it reads erased;
         * this prevents reuse after an interrupted sector erase.
         */
        if (EE_FindFreeSlotInSector(active_sector, &free_address))
        {
            if (!EE_PrepareRecord(
                    &new_record,
                    latest_record.sequence + 1U,
                    calibration))
            {
#ifdef HAL_ICACHE_MODULE_ENABLED
                HAL_ICACHE_Enable();
#endif
                return EE_WRITE_FAILED;
            }

            if (HAL_FLASH_Unlock() != HAL_OK)
            {
#ifdef HAL_ICACHE_MODULE_ENABLED
                HAL_ICACHE_Enable();
#endif
                return EE_WRITE_FAILED;
            }

            bool success = EE_WriteRecord(free_address, &new_record);

            HAL_FLASH_Lock();

#ifdef HAL_ICACHE_MODULE_ENABLED
            HAL_ICACHE_Enable();
#endif
            return success ? EE_WRITE_OK : EE_WRITE_FAILED;
        }

        uint32_t destination_sector = 1U - active_sector;

        uint32_t destination_address =
            (destination_sector == 0U)
                ? EE_STORAGE_SECTOR_A_ADDRESS
                : EE_STORAGE_SECTOR_B_ADDRESS;

        /*
         * Erase the destination sector while the old latest record
         * remains safely stored in the other sector.
         */
        if (!EE_EraseStorageSector(destination_sector))
        {
#ifdef HAL_ICACHE_MODULE_ENABLED
            HAL_ICACHE_Enable();
#endif
            return EE_WRITE_FAILED;
        }

        /* Copy the latest known-good calibration into the new sector */
        if (!EE_PrepareRecord(
                &copy_record,
                latest_record.sequence + 1U,
                latest_record.calibration))
        {
#ifdef HAL_ICACHE_MODULE_ENABLED
            HAL_ICACHE_Enable();
#endif
            return EE_WRITE_FAILED;
        }

        if (!EE_PrepareRecord(
                &new_record,
                latest_record.sequence + 2U,
                calibration))
        {
#ifdef HAL_ICACHE_MODULE_ENABLED
            HAL_ICACHE_Enable();
#endif
            return EE_WRITE_FAILED;
        }

        if (HAL_FLASH_Unlock() != HAL_OK)
        {
#ifdef HAL_ICACHE_MODULE_ENABLED
            HAL_ICACHE_Enable();
#endif
            return EE_WRITE_FAILED;
        }

        /* First preserve the previous calibration */
        bool copy_success =
            EE_WriteRecord(destination_address, &copy_record);

        /* Then commit the newly requested calibration */
        bool write_success = false;

        if (copy_success)
        {
            write_success = EE_WriteRecord(
                destination_address + sizeof(ee_record_t),
                &new_record);
        }

        HAL_FLASH_Lock();

        if (!write_success)
        {
#ifdef HAL_ICACHE_MODULE_ENABLED
            HAL_ICACHE_Enable();
#endif
            return EE_WRITE_FAILED;
        }

        /*
         * The new calibration is now safely committed.
         * The old sector can be erased to prepare for future writes.
         */
        bool cleanup_success = EE_EraseStorageSector(active_sector);

#ifdef HAL_ICACHE_MODULE_ENABLED
        HAL_ICACHE_Enable();
#endif
        return cleanup_success
            ? EE_WRITE_OK
            : EE_WRITE_OK_CLEANUP_FAILED;
    }
    else
    {
        /*
         * No new-format record exists. Sector A is the initial active sector.
         * It must be fully erased and verified before use; this also safely
         * handles a reset during an earlier attempted erase. Legacy data, when
         * present, remains preserved in sector B during this migration.
         */
        if (!EE_IsSectorErased(0U) && !EE_EraseStorageSector(0U))
        {
#ifdef HAL_ICACHE_MODULE_ENABLED
            HAL_ICACHE_Enable();
#endif
            return EE_WRITE_FAILED;
        }

        if (!EE_PrepareRecord(&new_record, 1U, calibration))
        {
#ifdef HAL_ICACHE_MODULE_ENABLED
            HAL_ICACHE_Enable();
#endif
            return EE_WRITE_FAILED;
        }

        if (HAL_FLASH_Unlock() != HAL_OK)
        {
#ifdef HAL_ICACHE_MODULE_ENABLED
            HAL_ICACHE_Enable();
#endif
            return EE_WRITE_FAILED;
        }

        bool success = EE_WriteRecord(
            EE_STORAGE_SECTOR_A_ADDRESS,
            &new_record);

        HAL_FLASH_Lock();

#ifdef HAL_ICACHE_MODULE_ENABLED
        HAL_ICACHE_Enable();
#endif
        return success ? EE_WRITE_OK : EE_WRITE_FAILED;
    }
}

bool ee_write(void)
{
    return ee_write_data(ee_handle.data, ee_handle.size) != EE_WRITE_FAILED;
}

/*************************************************************************************************/
/** End of File **/
/*************************************************************************************************/
