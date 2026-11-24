
/*
 * @file        ee.h
 * @brief       EEPROM Emulation Library
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

#ifndef _EE_H_
#define _EE_H_

#ifdef __cplusplus
extern "C"
{
#endif

/*************************************************************************************************/
/** Includes **/
/*************************************************************************************************/

#include <stdbool.h>
#include "main.h"


//NEW
#define EE_RECORD_MAGIC       0x45454341U
#define EE_RECORD_VERSION     1U
#define EE_RECORD_COMMIT      0xC04D17EDU

typedef struct
{
    uint32_t magic;
    uint32_t version;
    uint32_t sequence;
    uint32_t calibration[4];
    uint32_t reserved;
    uint32_t crc;
    uint32_t commit;
} ee_record_t;

typedef char ee_record_size_check[(sizeof(ee_record_t) == 40U) ? 1 : -1];

typedef enum
{
    EE_READ_NOT_INITIALIZED = 0,
    EE_READ_NO_VALID_RECORD,
    EE_READ_VALID_RECORD,
    EE_READ_VALID_LEGACY
} ee_read_status_t;

typedef enum
{
    EE_WRITE_FAILED = 0,
    EE_WRITE_OK,
    EE_WRITE_OK_CLEANUP_FAILED
} ee_write_status_t;


//OLD
/*************************************************************************************************/
/** Typedef/Struct/Enum **/
/*************************************************************************************************/

/*************************************************************************************************/
/* Handle struct of EEPROM */
typedef struct
{
  uint8_t                *data;
  uint32_t               size;
  uint32_t               page_sector_size;
  uint32_t               address;
  uint8_t                page_sector_number;
#if (defined FLASH_BANK_1) || (defined FLASH_BANK_2)
  uint8_t                bank_number;
#endif

} ee_t;

/*************************************************************************************************/
/** API Functions **/
/*************************************************************************************************/

/* Initializes the EEPROM emulation module */
bool      ee_init(void *data, uint32_t size);

/* Retrieves the capacity of the EEPROM emulation area */
uint32_t  ee_capacity(void);

/* Formats the EEPROM emulation area */
bool      ee_format(void);

/* Reads data from the EEPROM emulation area */
ee_read_status_t ee_read(void);

/* Writes the supplied data without modifying the configured RAM buffer */
ee_write_status_t ee_write_data(const void *data, uint32_t size);

/* Writes the configured RAM buffer (backward-compatible API) */
bool      ee_write(void);

/*************************************************************************************************/
/** End of File **/
/*************************************************************************************************/

#ifdef __cplusplus
}
#endif
#endif
