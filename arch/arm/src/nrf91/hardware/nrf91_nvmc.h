/****************************************************************************
 * arch/arm/src/nrf91/hardware/nrf91_nvmc.h
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

#ifndef __ARCH_ARM_SRC_NRF91_HARDWARE_NRF91_NVMC_H
#define __ARCH_ARM_SRC_NRF91_HARDWARE_NRF91_NVMC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/nrf91_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* NVMC Register Offsets ****************************************************/

/* Registers for the NVMC */

#define NRF91_NVMC_READY_OFFSET                0x400  /* Ready flag */
#define NRF91_NVMC_READYNEXT_OFFSET            0x408  /* Ready flag */
#define NRF91_NVMC_CONFIG_OFFSET               0x504  /* Configuration register */
#define NRF91_NVMC_ERASEALL_OFFSET             0x50c  /* Register for erasing all non-volatile user memory */
#define NRF91_NVMC_ERASEPAGEPARTIALCFG_OFFSET  0x51c  /* Register for partial erase configuration */
#define NRF91_NVMC_ICACHECNF_OFFSET            0x540  /* I-Code cache configuration register */
#define NRF91_NVMC_IHIT_OFFSET                 0x548  /* I-Code cache hit counter. */
#define NRF91_NVMC_IMISS_OFFSET                0x54c  /* I-Code cache miss counter */
#define NRF91_NVMC_CONFIGNS_OFFSET             0x584  /* Non-secure configuration register */
#define NRF91_NVMC_WRITEUICRNS_OFFSET          0x588  /* Non-secure APPROTECT enable register */

/* NVMC Register Addresses **************************************************/

#define NRF91_NVMC_READY                 (NRF91_NVMC_BASE + NRF91_NVMC_READY_OFFSET)
#define NRF91_NVMC_READYNEXT             (NRF91_NVMC_BASE + NRF91_NVMC_READYNEXT_OFFSET)
#define NRF91_NVMC_CONFIG                (NRF91_NVMC_BASE + NRF91_NVMC_CONFIG_OFFSET)
#define NRF91_NVMC_ERASEALL              (NRF91_NVMC_BASE + NRF91_NVMC_ERASEALL_OFFSET)
#define NRF91_NVMC_ERASEPAGEPARTIALCFG   (NRF91_NVMC_BASE + NRF91_NVMC_ERASEPAGEPARTIALCFG_OFFSET)
#define NRF91_NVMC_ICACHECNF             (NRF91_NVMC_BASE + NRF91_NVMC_ICACHECNF_OFFSET)
#define NRF91_NVMC_IHIT                  (NRF91_NVMC_BASE + NRF91_NVMC_IHIT_OFFSET)
#define NRF91_NVMC_IMISS                 (NRF91_NVMC_BASE + NRF91_NVMC_IMISS_OFFSET)
#define NRF91_NVMC_CONFIGNS              (NRF91_NVMC_BASE + NRF91_NVMC_CONFIGNS_OFFSET)
#define NRF91_NVMC_WRITEUICRNS           (NRF91_NVMC_BASE + NRF91_NVMC_WRITEUICRNS_OFFSET)

/* NVMC Register Bitfield Definitions ***************************************/

/* READY Register */

#define NVMC_READY_READY                 (1 << 0) /* NVMC is ready */

/* CONFIG Register */

#define NVMC_CONFIG_SHIFT                (0)
#define NVMC_CONFIG_MASK                 (3 << NVMC_CONFIG_SHIFT)
#define NVMC_CONFIG_REN                  (0 << NVMC_CONFIG_SHIFT) /* Read-only access */
#define NVMC_CONFIG_WEN                  (1 << NVMC_CONFIG_SHIFT) /* Write Enabled */
#define NVMC_CONFIG_EEN                  (2 << NVMC_CONFIG_SHIFT) /* Erase Enabled */
#define NVMC_CONFIG_PEEN                 (4 << NVMC_CONFIG_SHIFT) /* Partial erase enabled */

/* ICACHECNF Register */

#define NVMC_ICACHECNF_CACHEEN           (1 << 0) /* Cache enable */
#define NVMC_ICACHECNF_CACHEPROFEN       (1 << 8) /* Cache profiling enable */

/* WRITEUICRNS Register */

#define NVMC_WRITEUICRNS_SET              (1 << 0) /* Allow non-secure code to set APPROTECT */
#define NVMC_WRITEUICRNS_KEY              (0xAFBE5A7 << 4)

#endif /* __ARCH_ARM_SRC_NRF91_HARDWARE_NRF91_NVMC_H */
