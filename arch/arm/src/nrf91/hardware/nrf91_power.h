/****************************************************************************
 * arch/arm/src/nrf91/hardware/nrf91_power.h
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

#ifndef __ARCH_ARM_SRC_NRF91_HARDWARE_NRF91_POWER_H
#define __ARCH_ARM_SRC_NRF91_HARDWARE_NRF91_POWER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "nrf91_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define NRF91_POWER_TASKS_CONSTLAT_OFFSET     0x078  /* Enable constant latency mode */
#define NRF91_POWER_TASKS_LOWPWR_OFFSET       0x07c  /* Enable low power mode (variable latency) */
#define NRF91_POWER_EVENTS_POFWARN_OFFSET     0x108  /* Power failure warning */
#define NRF91_POWER_EVENTS_SLEEPENTER_OFFSET  0x114  /* CPU entered WFI/WFE sleep */
#define NRF91_POWER_EVENTS_SLEEPEXIT_OFFSET   0x118  /* CPU exited WFI/WFE sleep */
#define NRF91_POWER_INTEN_OFFSET              0x300  /* Enable or disable interrupt */
#define NRF91_POWER_INTENSET_OFFSET           0x304  /* Enable interrupt */
#define NRF91_POWER_INTENCLR_OFFSET           0x308  /* Disable interrupt */
#define NRF91_POWER_RESETREAS_OFFSET          0x400  /* Reset reason */
#define NRF91_POWER_POWERSTATUS_OFFSET        0x440  /* Modem domain power status */
#define NRF91_POWER_GPREGRET_OFFSET           0x51c  /* General purpose retention register */
#define NRF91_POWER_GPREGRET2_OFFSET          0x520  /* General purpose retention register */
#define NRF91_POWER_LTEMODEM_STARTN_OFFSET    0x610  /* Start LTE modem */
#define NRF91_POWER_LTEMODEM_FORCEOFF_OFFSET  0x614  /* Force off LTE modem */

/* Register definitions *****************************************************/

#define NRF91_POWER_TASKS_CONSTLAT     (NRF91_POWER_BASE + NRF91_POWER_TASKS_CONSTLAT_OFFSET)
#define NRF91_POWER_TASKS_LOWPWR       (NRF91_POWER_BASE + NRF91_POWER_TASKS_LOWPWR_OFFSET)
#define NRF91_POWER_EVENTS_POFWARN     (NRF91_POWER_BASE + NRF91_POWER_EVENTS_POFWARN_OFFSET)
#define NRF91_POWER_EVENTS_SLEEPENTER  (NRF91_POWER_BASE + NRF91_POWER_EVENTS_SLEEPENTER_OFFSET)
#define NRF91_POWER_EVENTS_SLEEPEXIT   (NRF91_POWER_BASE + NRF91_POWER_EVENTS_SLEEPEXIT_OFFSET)
#define NRF91_POWER_INTENSET           (NRF91_POWER_BASE + NRF91_POWER_INTENSET_OFFSET)
#define NRF91_POWER_INTENCLR           (NRF91_POWER_BASE + NRF91_POWER_INTENCLR_OFFSET)
#define NRF91_POWER_RESETREAS          (NRF91_POWER_BASE + NRF91_POWER_RESETREAS_OFFSET)
#define NRF91_POWER_POWERSTATUS        (NRF91_POWER_BASE + NRF91_POWER_POWERSTATUS_OFFSET)
#define NRF91_POWER_SYSTEMOFF          (NRF91_POWER_BASE + NRF91_POWER_SYSTEMOFF_OFFSET)
#define NRF91_POWER_POFCON             (NRF91_POWER_BASE + NRF91_POWER_POFCON_OFFSET)
#define NRF91_POWER_GPREGRET           (NRF91_POWER_BASE + NRF91_POWER_GPREGRET_OFFSET)
#define NRF91_POWER_GPREGRET2          (NRF91_POWER_BASE + NRF91_POWER_GPREGRET2_OFFSET)
#define NRF91_POWER_LTEMODEM_STARTN    (NRF91_POWER_BASE + NRF91_POWER_LTEMODEM_STARTN_OFFSET)
#define NRF91_POWER_LTEMODEM_FORCEOFF  (NRF91_POWER_BASE + NRF91_POWER_LTEMODEM_FORCEOFF_OFFSET)

/* Register bit definitions *************************************************/

/* TODO */

#endif /* __ARCH_ARM_SRC_NRF91_HARDWARE_NRF91_POWER_H */
