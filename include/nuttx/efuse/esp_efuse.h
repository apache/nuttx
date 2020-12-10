/****************************************************************************
 * include/nuttx/efuse/esp_efuse.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_EFUSE_EFUSE_H
#define __INCLUDE_NUTTX_EFUSE_EFUSE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>

#include <signal.h>

#ifdef CONFIG_ESP32_EFUSE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Command:     EFUSEIOC_READ_FIELD_BLOB
 * Description: Read a blob of bits from an efuse field.
 * Arguments:   A structure containing the field[], a dst pointer, and size
 *              of bits to be read from efuses.
 * Return:      Zero (OK) on success.  Minus one will be returned on failure
 *              with the errno value set appropriately.
 */

#define EFUSEIOC_READ_FIELD_BLOB      _EFUSEIOC(0x0001)

/* Command:     EFUSEIOC_READ_FIELD_BIT
 * Description: Read of state of an efuse bit.
 * Arguments:   A structure containing the field to be read.
 * Return:      Zero (OK) on success.  Minus one will be returned on failure
 *              with the errno value set appropriately.
 */

#define EFUSEIOC_READ_FIELD_BIT       _EFUSEIOC(0x0002)

/* Command:     EFUSEIOC_WRITE_FIELD_BLOB
 * Description: Write a blob of bits to an efuse's field
 * Arguments:   A structure containing the field[], the src memory and the
 *              amount of bits to write.
 * Return:      Zero (OK) on success.  Minus one will be returned on failure
 *              with the errno value set appropriately.
 */

#define EFUSEIOC_WRITE_FIELD_BLOB     _EFUSEIOC(0x0003)

/* Command:     EFUSEIOC_WRITE_FIELD_BIT
 * Description: Write a bit to an efuse (burn it).
 * Arguments:   A structure containing bit field.
 * Return:      Zero (OK) on success.  Minus one will be returned on failure
 *              with the errno value set appropriately.
 */

#define EFUSEIOC_WRITE_FIELD_BIT      _EFUSEIOC(0x0004)

/* Command:     EFUSEIOC_READ_REG
 * Description: Read an efuse register.
 * Arguments:   A structure containing the block number and the register to
 *              be read.
 * Return:      Zero (OK) on success.  Minus one will be returned on failure
 *              with the errno value set appropriately.
 */

#define EFUSEIOC_READ_REG             _EFUSEIOC(0x0005)

/* Command:     EFUSEIOC_WRITE_REG
 * Description: Write an efuse register.
 * Arguments:   A structure containing the block number, the register to
 *              write and value to be written.
 * Return:      Zero (OK) on success.  Minus one will be returned on failure
 *              with the errno value set appropriately.
 */

#define EFUSEIOC_WRITE_REG            _EFUSEIOC(0x0006)

/* Command:     EFUSEIOC_READ_BLOCK
 * Description: Read a key from efuse's block.
 * Arguments:   A structure containing the block number, the dst_key pointer,
 *              the offset in bits and the amount of bits to read.
 * Return:      Zero (OK) on success.  Minus one will be returned on failure
 *              with the errno value set appropriately.
 */

#define EFUSEIOC_READ_BLOCK           _EFUSEIOC(0x0007)

/* Command:     EFUSEIOC_WRITE_BLOCK
 * Description: Write a key to efuse's block.
 * Arguments:   A structure containing the block number, the src_key pointer,
 *              the offset in bits and the amount of bits to write.
 * Return:      Zero (OK) on success.  Minus one will be returned on failure
 *              with the errno value set appropriately.
 */

#define EFUSEIOC_WRITE_BLOCK          _EFUSEIOC(0x0008)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Structure eFuse field */

struct esp_efuse_desc_s
{
  uint32_t   efuse_block:8;  /* Block of eFuse */
  uint8_t    bit_start;      /* Start bit [0..255] */
  uint16_t   bit_count;      /* Length of bit field [1..-] */
};

/* Type definition for an eFuse field */

typedef struct esp_efuse_desc_s esp_efuse_desc_t;

/* Structure range address by blocks */

typedef struct
{
  uint32_t start;
  uint32_t end;
} esp_efuse_range_addr_t;

/* Structs with the parameters passed to the IOCTLs */

struct esp_efuse_par
{
  uint32_t block;
  uint32_t reg;
  uint32_t *data;
  size_t   bit_offset;
  size_t   bit_size;
};

struct esp_efuse_par_id
{
  esp_efuse_desc_t **field;
  uint8_t *data;
  size_t  size;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int esp_efuse_init(void);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_ESP32_EFUSE */
#endif /* __INCLUDE_NUTTX_EFUSE_EFUSE_H */
