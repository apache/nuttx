/****************************************************************************
 * arch/arm/src/nrf52/hardware/nrf52_nvmc.h
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

#ifndef __ARCH_ARM_SRC_NRF52_HARDWARE_NRF52_NVMC_H
#define __ARCH_ARM_SRC_NRF52_HARDWARE_NRF52_NVMC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/nrf52_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* NVMC Register Offsets ****************************************************/

/* Registers for the NVMC */

#define NRF52_NVMC_READY_OFFSET          0x400  /* Ready flag */
#define NRF52_NVMC_CONFIG_OFFSET         0x504  /* Configuration register */
#define NRF52_NVMC_ERASEPAGE_OFFSET      0x508  /* Register for erasing a page in Code area */
#define NRF52_NVMC_ERASEPCR1_OFFSET      0x508  /* Equivalent to ERASEPAGE */
#define NRF52_NVMC_ERASEALL_OFFSET       0x50c  /* Register for erasing all non-volatile user memory */
#define NRF52_NVMC_ERASEPCR0_OFFSET      0x510  /* Register for erasing a page in Code area. Equiv. ERASEPAGE */
#define NRF52_NVMC_ERASEUICR_OFFSET      0x514  /* Register for erasing User Information Configuration Registers */
#define NRF52_NVMC_ICACHECNF_OFFSET      0x540  /* I-Code cache configuration register */
#define NRF52_NVMC_IHIT_OFFSET           0x548  /* I-Code cache hit counter. */
#define NRF52_NVMC_IMISS_OFFSET          0x54c  /* I-Code cache miss counter */

/* NVMC Register Addresses **************************************************/

#define NRF52_NVMC_READY                 (NRF52_NVMC_BASE + NRF52_NVMC_READY_OFFSET)
#define NRF52_NVMC_CONFIG                (NRF52_NVMC_BASE + NRF52_NVMC_CONFIG_OFFSET)
#define NRF52_NVMC_ERASEPAGE             (NRF52_NVMC_BASE + NRF52_NVMC_ERASEPAGE_OFFSET)
#define NRF52_NVMC_ERASEPCR1             (NRF52_NVMC_BASE + NRF52_NVMC_ERASEPCR1_OFFSET)
#define NRF52_NVMC_ERASEALL              (NRF52_NVMC_BASE + NRF52_NVMC_ERASEALL_OFFSET)
#define NRF52_NVMC_ERASEPCR0             (NRF52_NVMC_BASE + NRF52_NVMC_ERASEPCR0_OFFSET)
#define NRF52_NVMC_ERASEUICR             (NRF52_NVMC_BASE + NRF52_NVMC_ERASEUICR_OFFSET)
#define NRF52_NVMC_ICACHECNF             (NRF52_NVMC_BASE + NRF52_NVMC_ICACHECNF_OFFSET)
#define NRF52_NVMC_IHIT                  (NRF52_NVMC_BASE + NRF52_NVMC_IHIT_OFFSET)
#define NRF52_NVMC_IMISS                 (NRF52_NVMC_BASE + NRF52_NVMC_IMISS_OFFSET)

/* NVMC Register Bitfield Definitions ***************************************/

/* READY Register */

#define NVMC_READY_READY                 (1 << 0) /* NVMC is ready */

/* CONFIG Register */

#define NVMC_CONFIG_SHIFT                (0)
#define NVMC_CONFIG_MASK                 (3 << NVMC_CONFIG_SHIFT)
#define NVMC_CONFIG_REN                  (0 << NVMC_CONFIG_SHIFT) /* Read-only access */
#define NVMC_CONFIG_WEN                  (1 << NVMC_CONFIG_SHIFT) /* Write Enabled */
#define NVMC_CONFIG_EEN                  (2 << NVMC_CONFIG_SHIFT) /* Erase Enabled */

/* ICACHECNF Register */

#define NVMC_ICACHECNF_CACHEEN           (1 << 0) /* Cache enable */
#define NVMC_ICACHECNF_CACHEPROFEN       (1 << 8) /* Cache profiling enable */

/* INTENSET Register */

#endif /* __ARCH_ARM_SRC_NRF52_HARDWARE_NRF52_NVMC_H */
