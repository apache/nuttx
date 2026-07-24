/****************************************************************************
 * arch/arm/src/rp23xx/rp23xx_psram.h
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

#ifndef __ARCH_ARM_SRC_RP23XX_RP23XX_PSRAM_H
#define __ARCH_ARM_SRC_RP23XX_RP23XX_PSRAM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stddef.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The QSPI PSRAM is mapped through QMI chip select 1 at this address */

#define RP23XX_PSRAM_BASE  0x11000000

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: rp23xx_psramconfig
 *
 * Description:
 *   Detect and initialise the external QSPI PSRAM on QMI chip select 1.
 *   Assigns the CS1 function to CONFIG_RP23XX_PSRAM_CS1_GPIO, reads the
 *   device ID to confirm an APS6404-family part is present, switches it to
 *   quad mode and programs the QMI M1 read/write formats so the region at
 *   RP23XX_PSRAM_BASE becomes directly addressable (and writable).
 *
 * Returned Value:
 *   The size of the detected PSRAM in bytes, or 0 if no PSRAM was found.
 *
 ****************************************************************************/

size_t rp23xx_psramconfig(void);

/****************************************************************************
 * Name: rp23xx_psram_size
 *
 * Description:
 *   Return the size in bytes detected by the last rp23xx_psramconfig(), or 0
 *   if PSRAM is absent or has not been configured.
 *
 ****************************************************************************/

size_t rp23xx_psram_size(void);

/****************************************************************************
 * Name: rp23xx_psram_restore
 *
 * Description:
 *   Re-apply the QMI M1 (PSRAM) format and timing registers.  Programming
 *   the flash goes through the bootrom, which reconfigures the shared QMI
 *   interface for chip select 0; this restores the CS1 configuration
 *   afterwards so the memory-mapped PSRAM keeps working across a flash
 *   erase or program.  Runs from RAM and does nothing when no PSRAM was
 *   detected.
 *
 ****************************************************************************/

void rp23xx_psram_restore(void);

#endif /* __ARCH_ARM_SRC_RP23XX_RP23XX_PSRAM_H */
