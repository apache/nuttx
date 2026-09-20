/****************************************************************************
 * arch/arm/src/nrf54l/hardware/nrf54l_power.h
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

#ifndef __ARCH_ARM_SRC_NRF54L_HARDWARE_NRF54L_POWER_H
#define __ARCH_ARM_SRC_NRF54L_HARDWARE_NRF54L_POWER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "nrf54l_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define NRF54L_POWER_TASKS_CONSTLAT_OFFSET     0x000030  /* Enable constant latency mode */
#define NRF54L_POWER_TASKS_LOWPWR_OFFSET       0x000034  /* Enable low power mode */
#define NRF54L_POWER_SUBSCRIBE_CONSTLAT_OFFSET 0x0000b0  /* Subscribe to CONSTLAT */
#define NRF54L_POWER_SUBSCRIBE_LOWPWR_OFFSET   0x0000b4  /* Subscribe to LOWPWR */
#define NRF54L_POWER_EVENTS_POFWARN_OFFSET     0x000130  /* Power failure warning */
#define NRF54L_POWER_EVENTS_SLEEPENTER_OFFSET  0x000134  /* CPU entered WFI/WFE sleep */
#define NRF54L_POWER_EVENTS_SLEEPEXIT_OFFSET   0x000138  /* CPU exited WFI/WFE sleep */
#define NRF54L_POWER_PUBLISH_POFWARN_OFFSET    0x0001b0  /* Publish POFWARN */
#define NRF54L_POWER_PUBLISH_SLEEPENTER_OFFSET 0x0001b4  /* Publish SLEEPENTER */
#define NRF54L_POWER_PUBLISH_SLEEPEXIT_OFFSET  0x0001b8  /* Publish SLEEPEXIT */
#define NRF54L_POWER_INTEN_OFFSET              0x000300  /* Enable or disable interrupt */
#define NRF54L_POWER_INTENSET_OFFSET           0x000304  /* Enable interrupt */
#define NRF54L_POWER_INTENCLR_OFFSET           0x000308  /* Disable interrupt */
#define NRF54L_POWER_GPREGRET0_OFFSET          0x000500  /* General purpose retention register 0 */
#define NRF54L_POWER_GPREGRET1_OFFSET          0x000504  /* General purpose retention register 1 */
#define NRF54L_POWER_CONSTLATSTAT_OFFSET       0x000520  /* Constant latency status */

/* Register definitions *****************************************************/

#define NRF54L_POWER_TASKS_CONSTLAT     (NRF54L_POWER_BASE + NRF54L_POWER_TASKS_CONSTLAT_OFFSET)
#define NRF54L_POWER_TASKS_LOWPWR       (NRF54L_POWER_BASE + NRF54L_POWER_TASKS_LOWPWR_OFFSET)
#define NRF54L_POWER_SUBSCRIBE_CONSTLAT (NRF54L_POWER_BASE + NRF54L_POWER_SUBSCRIBE_CONSTLAT_OFFSET)
#define NRF54L_POWER_SUBSCRIBE_LOWPWR   (NRF54L_POWER_BASE + NRF54L_POWER_SUBSCRIBE_LOWPWR_OFFSET)
#define NRF54L_POWER_EVENTS_POFWARN     (NRF54L_POWER_BASE + NRF54L_POWER_EVENTS_POFWARN_OFFSET)
#define NRF54L_POWER_EVENTS_SLEEPENTER  (NRF54L_POWER_BASE + NRF54L_POWER_EVENTS_SLEEPENTER_OFFSET)
#define NRF54L_POWER_EVENTS_SLEEPEXIT   (NRF54L_POWER_BASE + NRF54L_POWER_EVENTS_SLEEPEXIT_OFFSET)
#define NRF54L_POWER_PUBLISH_POFWARN    (NRF54L_POWER_BASE + NRF54L_POWER_PUBLISH_POFWARN_OFFSET)
#define NRF54L_POWER_PUBLISH_SLEEPENTER (NRF54L_POWER_BASE + NRF54L_POWER_PUBLISH_SLEEPENTER_OFFSET)
#define NRF54L_POWER_PUBLISH_SLEEPEXIT  (NRF54L_POWER_BASE + NRF54L_POWER_PUBLISH_SLEEPEXIT_OFFSET)
#define NRF54L_POWER_INTEN              (NRF54L_POWER_BASE + NRF54L_POWER_INTEN_OFFSET)
#define NRF54L_POWER_INTENSET           (NRF54L_POWER_BASE + NRF54L_POWER_INTENSET_OFFSET)
#define NRF54L_POWER_INTENCLR           (NRF54L_POWER_BASE + NRF54L_POWER_INTENCLR_OFFSET)
#define NRF54L_POWER_GPREGRET0          (NRF54L_POWER_BASE + NRF54L_POWER_GPREGRET0_OFFSET)
#define NRF54L_POWER_GPREGRET1          (NRF54L_POWER_BASE + NRF54L_POWER_GPREGRET1_OFFSET)
#define NRF54L_POWER_CONSTLATSTAT       (NRF54L_POWER_BASE + NRF54L_POWER_CONSTLATSTAT_OFFSET)

/* Register bit definitions *************************************************/

#define POWER_TASKS_CONSTLAT          (1 << 0)
#define POWER_TASKS_LOWPWR            (1 << 0)
#define POWER_EVENTS_POFWARN          (1 << 0)
#define POWER_EVENTS_SLEEPENTER       (1 << 0)
#define POWER_EVENTS_SLEEPEXIT        (1 << 0)

#define POWER_INT_POFWARN             (1 << 12)
#define POWER_INT_SLEEPENTER          (1 << 13)
#define POWER_INT_SLEEPEXIT           (1 << 14)

#define POWER_SUBSCRIBE_CHIDX_MASK    (0xff)
#define POWER_SUBSCRIBE_EN            (1 << 31)
#define POWER_PUBLISH_CHIDX_MASK      (0xff)
#define POWER_PUBLISH_EN              (1 << 31)

#define POWER_GPREGRET_MASK           (0xff)
#define POWER_CONSTLATSTAT_STATUS     (1 << 0)

#endif /* __ARCH_ARM_SRC_NRF54L_HARDWARE_NRF54L_POWER_H */
