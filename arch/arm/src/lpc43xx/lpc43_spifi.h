/****************************************************************************
 * arch/arm/src/lpc43xx/lpc43_spifi.h
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

#ifndef __ARCH_ARM_SRC_LPC43XX_LPC43_SPIFI_H
#define __ARCH_ARM_SRC_LPC43XX_LPC43_SPIFI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/mtd/mtd.h>

#include "chip.h"
#include "hardware/lpc43_spifi.h"

#ifdef CONFIG_LPC43_SPIFI

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SPIFI Configuration ******************************************************/

/* This logic supports some special options that can be used to create an
 * MTD device on the SPIFI FLASH.
 *
 *    CONFIG_LPC43_SPIFI - Enable SPIFI support
 *
 * SPIFI device geometry:
 *
 *   CONFIG_SPIFI_OFFSET - Offset the beginning of the block driver this many
 *     bytes into the device address space.  This offset must be an exact
 *     multiple of the erase block size (CONFIG_SPIFI_BLKSIZE). Default 0.
 *   CONFIG_SPIFI_BLKSIZE - The size of one device erase block.  If not
 *     defined then the driver will try to determine the correct erase block
 *     size by examining that data returned from spifi_initialize (which
 *     sometimes seems bad).
 *
 * Other SPIFI options
 *
 *   CONFIG_SPIFI_LIBRARY - Don't use the LPC43xx ROM routines but, instead,
 *     use an external library implementation of the SPIFI interface.
 *   CONFIG_SPIFI_SECTOR512 - If defined, then the driver will report a more
 *     FAT friendly 512 byte sector size and will manage the
 *     read-modify-write operations on the larger erase block.
 *   CONFIG_SPIFI_READONLY - Define to support only read-only operations.
 */

#ifndef CONFIG_SPIFI_OFFSET
#  define CONFIG_SPIFI_OFFSET 0
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: lpc43_spifi_initialize
 *
 * Description:
 *   Create an initialized MTD device instance for the SPIFI device.  MTD
 *   devices are not registered in the file system, but are created as
 *   instances that can be bound to other functions (such as a block or
 *   character driver front end).
 *
 *   SPIFI interface clocking is configured per settings in the board.h file.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   On success, a reference to the initialized MTD device instance is
 *   returned;  NULL is returned on any failure.
 *
 ****************************************************************************/

struct mtd_dev_s *lpc43_spifi_initialize(void);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_LPC43_SPIFI */
#endif /* __ARCH_ARM_SRC_LPC43XX_LPC43_SPIFI_H */
