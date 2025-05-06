/****************************************************************************
 * arch/arm64/include/imx9/imx9_romapi.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __ARCH_ARM64_INCLUDE_IMX9_IMX9_ROMAPI_H
#define __ARCH_ARM64_INCLUDE_IMX9_IMX9_ROMAPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* infor_type values for imx9_rom_api_query_boot_infor(). */

#define QUERY_ROM_VER    1
#define QUERY_BOOT_DEV   2
#define QUERY_PAGE_SZ    3
#define QUERY_IVT_OFF    4
#define QUERY_BOOT_STAGE 5
#define QUERY_IMG_OFF    6

/* Native return values from ROM API. */

#define  ROMAPI_INVALID_PARA         1
#define  ROMAPI_READ_FAILURE         2
#define  ROMAPI_DEVICE_NOT_READY     3
#define  ROMAPI_OUT_OF_DEVICE_MEMORY 4
#define  ROMAPI_OK                   0xf0

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Boot device types returned by ROM API (i.MX93). */

enum imx9_romapi_bootdev_e
{
  BOOTDEV_SD           = 1,
  BOOTDEV_MMC          = 2,
  BOOTDEV_NAND         = 3,
  BOOTDEV_FLEXSPI_NOR  = 4,
  BOOTDEV_SPI_NOR      = 6,
  BOOTDEV_FLEXSPI_NAND = 8,
  BOOTDEV_USB          = 0xe,
  BOOTDEV_MEM_DEV      = 0xf,
  BOOTDEV_INVALID      = 0xff
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: imx9_romapi_load_image
 *
 * Description:
 *   Use the i.MX9 ROM API, to copy data from the boot device to memory.
 *
 * Input Parameters:
 *   dest - Copy destination.
 *   offset - Offset from boot device to copy from.
 *   size - Size in bytes to copy.
 *
 * Returned Value:
 *   ROMAPI_OK - Success.
 *   ROMAPI_INVALID_PARA - Invalid parameters.
 *   ROMAPI_READ_FAILURE - Low level device read failure.
 *   ROMAPI_DEVICE_NOT_READY - The device isn't initialized or is invalid.
 *   ROMAPI_OUT_OF_DEVICE_MEMORY - Attempted to read data outside of the
 *   boot device's memory.
 *
 ****************************************************************************/

uint32_t imx9_romapi_load_image(uint8_t *dest, uint32_t offset,
                                uint32_t size);

/****************************************************************************
 * Name: imx9_romapi_query_boot_infor
 *
 * Description:
 *   Query information from the ROM API.
 *
 * Input Parameters:
 *   infor_type - Type of information to query.
 *   infor -  Points to the word to save the queried information.
 *
 * Returned Value:
 *   ROMAPI_OK - Success.
 *   ROMAPI_INVALID_PARA - Invalid parameters.
 *
 ****************************************************************************/

uint32_t imx9_romapi_query_boot_infor(uint32_t infor_type, uint32_t *infor);

/****************************************************************************
 * Name: imx9_romapi_get_boot_device
 *
 * Description:
 *   Get the boot device type.
 *
 * Returned Value:
 *   The device type.
 *
 ****************************************************************************/

enum imx9_romapi_bootdev_e imx9_romapi_get_boot_device(void);

/****************************************************************************
 * Name: imx9_romapi_get_seq_cntr_base
 *
 * Description:
 *   Get the offset in bytes from the boot device of the cntr_idx container.
 *   The containers are assumed to be sequential in the boot device's
 *   memory.
 *
 * Input Parameters:
 *   image_offset - Image offset of the first container in the boot device.
 *   pagesize - Pagesize of the boot device.
 *   cntr_idx - Index of the container.
 *
 * Returned Value:
 *   The offset from the boot device in bytes of the cntr_idx container.
 *   -1 is returned in case of error.
 *
 ****************************************************************************/

int64_t imx9_romapi_get_seq_cntr_base(uint32_t image_offset,
                                      uint32_t pagesize,
                                      uint8_t cntr_idx);
#endif /* __ARCH_ARM64_INCLUDE_IMX9_IMX9_ROMAPI_H */
