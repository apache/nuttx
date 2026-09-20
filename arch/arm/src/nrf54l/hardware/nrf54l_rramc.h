/****************************************************************************
 * arch/arm/src/nrf54l/hardware/nrf54l_rramc.h
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

#ifndef __ARCH_ARM_SRC_NRF54L_HARDWARE_NRF54L_RRAMC_H
#define __ARCH_ARM_SRC_NRF54L_HARDWARE_NRF54L_RRAMC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/nrf54l_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define NRF54L_RRAMC_TASKS_WAKEUP_OFFSET         0x000             /* Wake RRAM from low power */
#define NRF54L_RRAMC_TASKS_CLRWRITEBUF_OFFSET    0x004             /* Discard buffered writes */
#define NRF54L_RRAMC_TASKS_COMMITWRITEBUF_OFFSET 0x008             /* Commit buffered writes to RRAM */
#define NRF54L_RRAMC_SUB_WAKEUP_OFFSET           0x080             /* Subscribe to WAKEUP task */
#define NRF54L_RRAMC_SUB_CLRWRITEBUF_OFFSET      0x084             /* Subscribe to CLRWRITEBUF task */
#define NRF54L_RRAMC_SUB_COMMITWRITEBUF_OFFSET   0x088             /* Subscribe to COMMITWRITEBUF task */
#define NRF54L_RRAMC_EVENTS_WOKENUP_OFFSET       0x100             /* RRAM wakeup completed */
#define NRF54L_RRAMC_EVENTS_READY_OFFSET         0x104             /* RRAM ready */
#define NRF54L_RRAMC_EVENTS_READYNEXT_OFFSET     0x108             /* Ready for next write */
#define NRF54L_RRAMC_EVENTS_ACCESSERROR_OFFSET   0x10c             /* RRAM access error */
#define NRF54L_RRAMC_PUB_WOKENUP_OFFSET          0x180             /* Publish WOKENUP event */
#define NRF54L_RRAMC_INTEN_OFFSET                0x300             /* Enable or disable interrupts */
#define NRF54L_RRAMC_INTENSET_OFFSET             0x304             /* Enable interrupts */
#define NRF54L_RRAMC_INTENCLR_OFFSET             0x308             /* Disable interrupts */
#define NRF54L_RRAMC_INTPEND_OFFSET              0x30c             /* Pending interrupts */
#define NRF54L_RRAMC_READY_OFFSET                0x400             /* Ready flag */
#define NRF54L_RRAMC_READYNEXT_OFFSET            0x404             /* Next write ready */
#define NRF54L_RRAMC_ACCESSERRORADDR_OFFSET      0x408             /* Address of access error */
#define NRF54L_RRAMC_WRITEBUFEMPTY_OFFSET        0x418             /* Write buffer empty status */
#define NRF54L_RRAMC_ECC_ERRORADDR_OFFSET        0x420             /* Address of ECC error */
#define NRF54L_RRAMC_CONFIG_OFFSET               0x500             /* Write enable and buffer configuration */
#define NRF54L_RRAMC_READYNEXTTIMEOUT_OFFSET     0x50c             /* Next-write timeout */
#define NRF54L_RRAMC_POWER_CONFIG_OFFSET         0x510             /* Power mode configuration */
#define NRF54L_RRAMC_POWER_STANDBYCONFIG_OFFSET  0x514             /* Standby mode configuration */
#define NRF54L_RRAMC_POWER_LOWPOWERCONFIG_OFFSET 0x518             /* Low-power mode configuration */
#define NRF54L_RRAMC_ERASEALL_OFFSET             0x540             /* Erase all RRAM */
#define NRF54L_RRAMC_REGION_ADDRESS_OFFSET(n)    (0x550 + (n) * 8) /* Region n start address */
#define NRF54L_RRAMC_REGION_CONFIG_OFFSET(n)     (0x554 + (n) * 8) /* Region n configuration */

/* Register addresses *******************************************************/

#define NRF54L_RRAMC_TASKS_WAKEUP          (NRF54L_RRAMC_BASE + NRF54L_RRAMC_TASKS_WAKEUP_OFFSET)
#define NRF54L_RRAMC_TASKS_CLRWRITEBUF     (NRF54L_RRAMC_BASE + NRF54L_RRAMC_TASKS_CLRWRITEBUF_OFFSET)
#define NRF54L_RRAMC_TASKS_COMMITWRITEBUF  (NRF54L_RRAMC_BASE + NRF54L_RRAMC_TASKS_COMMITWRITEBUF_OFFSET)
#define NRF54L_RRAMC_SUB_WAKEUP            (NRF54L_RRAMC_BASE + NRF54L_RRAMC_SUB_WAKEUP_OFFSET)
#define NRF54L_RRAMC_SUB_CLRWRITEBUF       (NRF54L_RRAMC_BASE + NRF54L_RRAMC_SUB_CLRWRITEBUF_OFFSET)
#define NRF54L_RRAMC_SUB_COMMITWRITEBUF    (NRF54L_RRAMC_BASE + NRF54L_RRAMC_SUB_COMMITWRITEBUF_OFFSET)
#define NRF54L_RRAMC_EVENTS_WOKENUP        (NRF54L_RRAMC_BASE + NRF54L_RRAMC_EVENTS_WOKENUP_OFFSET)
#define NRF54L_RRAMC_EVENTS_READY          (NRF54L_RRAMC_BASE + NRF54L_RRAMC_EVENTS_READY_OFFSET)
#define NRF54L_RRAMC_EVENTS_READYNEXT      (NRF54L_RRAMC_BASE + NRF54L_RRAMC_EVENTS_READYNEXT_OFFSET)
#define NRF54L_RRAMC_EVENTS_ACCESSERROR    (NRF54L_RRAMC_BASE + NRF54L_RRAMC_EVENTS_ACCESSERROR_OFFSET)
#define NRF54L_RRAMC_PUB_WOKENUP           (NRF54L_RRAMC_BASE + NRF54L_RRAMC_PUB_WOKENUP_OFFSET)
#define NRF54L_RRAMC_INTEN                 (NRF54L_RRAMC_BASE + NRF54L_RRAMC_INTEN_OFFSET)
#define NRF54L_RRAMC_INTENSET              (NRF54L_RRAMC_BASE + NRF54L_RRAMC_INTENSET_OFFSET)
#define NRF54L_RRAMC_INTENCLR              (NRF54L_RRAMC_BASE + NRF54L_RRAMC_INTENCLR_OFFSET)
#define NRF54L_RRAMC_INTPEND               (NRF54L_RRAMC_BASE + NRF54L_RRAMC_INTPEND_OFFSET)
#define NRF54L_RRAMC_READY                 (NRF54L_RRAMC_BASE + NRF54L_RRAMC_READY_OFFSET)
#define NRF54L_RRAMC_READYNEXT             (NRF54L_RRAMC_BASE + NRF54L_RRAMC_READYNEXT_OFFSET)
#define NRF54L_RRAMC_CONFIG                (NRF54L_RRAMC_BASE + NRF54L_RRAMC_CONFIG_OFFSET)
#define NRF54L_RRAMC_ERASEALL              (NRF54L_RRAMC_BASE + NRF54L_RRAMC_ERASEALL_OFFSET)
#define NRF54L_RRAMC_ACCESSERRORADDR       (NRF54L_RRAMC_BASE + NRF54L_RRAMC_ACCESSERRORADDR_OFFSET)
#define NRF54L_RRAMC_WRITEBUFEMPTY         (NRF54L_RRAMC_BASE + NRF54L_RRAMC_WRITEBUFEMPTY_OFFSET)
#define NRF54L_RRAMC_ECC_ERRORADDR         (NRF54L_RRAMC_BASE + NRF54L_RRAMC_ECC_ERRORADDR_OFFSET)
#define NRF54L_RRAMC_READYNEXTTIMEOUT      (NRF54L_RRAMC_BASE + NRF54L_RRAMC_READYNEXTTIMEOUT_OFFSET)
#define NRF54L_RRAMC_POWER_CONFIG          (NRF54L_RRAMC_BASE + NRF54L_RRAMC_POWER_CONFIG_OFFSET)
#define NRF54L_RRAMC_POWER_STANDBYCONFIG   (NRF54L_RRAMC_BASE + NRF54L_RRAMC_POWER_STANDBYCONFIG_OFFSET)
#define NRF54L_RRAMC_POWER_LOWPOWERCONFIG  (NRF54L_RRAMC_BASE + NRF54L_RRAMC_POWER_LOWPOWERCONFIG_OFFSET)
#define NRF54L_RRAMC_REGION_ADDRESS(n)     (NRF54L_RRAMC_BASE + NRF54L_RRAMC_REGION_ADDRESS_OFFSET(n))
#define NRF54L_RRAMC_REGION_CONFIG(n)      (NRF54L_RRAMC_BASE + NRF54L_RRAMC_REGION_CONFIG_OFFSET(n))

/* Register bit definitions *************************************************/

/* READY Register */

#define RRAMC_READY_READY                 (1 << 0) /* RRAMC is ready */

/* CONFIG Register */

#define RRAMC_CONFIG_WEN                  (1 << 0)
#define RRAMC_CONFIG_WRITEBUFSIZE_SHIFT   (8)
#define RRAMC_CONFIG_WRITEBUFSIZE_MASK    (0x3f << RRAMC_CONFIG_WRITEBUFSIZE_SHIFT)
#define RRAMC_CONFIG_WRITEBUFSIZE_32      (32 << RRAMC_CONFIG_WRITEBUFSIZE_SHIFT)

/* SUBSCRIBE/PUBLISH Registers */

#define RRAMC_SUBPUB_CHIDX_MASK           (0xff)
#define RRAMC_SUBPUB_ENABLE               (0x80000000)

/* INTEN/INTENSET/INTENCLR/INTPEND Registers */

#define RRAMC_INT_WOKENUP                 (1 << 0)
#define RRAMC_INT_READY                   (1 << 1)
#define RRAMC_INT_READYNEXT               (1 << 2)
#define RRAMC_INT_ACCESSERROR             (1 << 3)

/* READYNEXT and BUFSTATUS.WRITEBUFEMPTY Registers */

#define RRAMC_READYNEXT_READY             (1 << 0)
#define RRAMC_WRITEBUFEMPTY_EMPTY         (1 << 0)

/* POWER.CONFIG Register */

#define RRAMC_POWER_ACCESSTIMEOUT_MASK    (0xffff)
#define RRAMC_POWER_POF_ABORT             (1 << 16)

/* POWER.STANDBYCONFIG Register */

#define RRAMC_STANDBY_MODE_MASK           (3 << 0)
#define RRAMC_STANDBY_NORMAL              (0 << 0)
#define RRAMC_STANDBY_POWERDOWN           (2 << 0)
#define RRAMC_STANDBY_STANDBY             (3 << 0)
#ifdef CONFIG_ARCH_CHIP_NRF54L15
#  define RRAMC_STANDBY_NAP               (1 << 0)
#endif

/* POWER.LOWPOWERCONFIG Register */

#define RRAMC_LOWPOWER_MODE_MASK          (3 << 0)
#define RRAMC_LOWPOWER_STANDBY            (1 << 0)
#ifdef CONFIG_ARCH_CHIP_NRF54L15
#  define RRAMC_LOWPOWER_POWERDOWN        (0 << 0)
#  define RRAMC_LOWPOWER_NAP              (2 << 0)
#  define RRAMC_LOWPOWER_POWEROFF         (3 << 0)
#else
#  define RRAMC_LOWPOWER_POWEROFF         (0 << 0)
#endif

/* REGION.CONFIG Register */

#define RRAMC_REGION_READ                 (1 << 0)
#define RRAMC_REGION_WRITE                (1 << 1)
#define RRAMC_REGION_EXECUTE              (1 << 2)
#define RRAMC_REGION_SECURE               (1 << 3)
#define RRAMC_REGION_WRITEONCE            (1 << 12)
#define RRAMC_REGION_LOCK                 (1 << 13)
#define RRAMC_REGION_SIZE_SHIFT           (16)
#ifdef CONFIG_ARCH_CHIP_NRF54L15
#  define RRAMC_REGION_OWNER_SHIFT        (4)
#  define RRAMC_REGION_OWNER_MASK         (0xf << RRAMC_REGION_OWNER_SHIFT)
#  define RRAMC_REGION_SIZE_MASK          (0x1f << RRAMC_REGION_SIZE_SHIFT)
#else
#  define RRAMC_REGION_SIZE_MASK          (0x7f << RRAMC_REGION_SIZE_SHIFT)
#endif

#endif /* __ARCH_ARM_SRC_NRF54L_HARDWARE_NRF54L_RRAMC_H */
