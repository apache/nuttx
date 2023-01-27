/****************************************************************************
 * arch/arm/src/nrf53/hardware/nrf53_power.h
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

#ifndef __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_POWER_H
#define __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_POWER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "nrf53_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define NRF53_POWER_TASKS_CONSTLAT_OFFSET     0x000078  /* Enable constant latency mode */
#define NRF53_POWER_TASKS_LOWPWR_OFFSET       0x00007c  /* Enable low power mode (variable latency) */
#define NRF53_POWER_EVENTS_POFWARN_OFFSET     0x000108  /* Power failure warning */
#define NRF53_POWER_EVENTS_SLEEPENTER_OFFSET  0x000114  /* CPU entered WFI/WFE sleep */
#define NRF53_POWER_EVENTS_SLEEPEXIT_OFFSET   0x000118  /* CPU exited WFI/WFE sleep */
#define NRF53_POWER_PUBLISH_POFWARN_OFFSET    0x000188  /* Publish configuration for event POFWARN */
#define NRF53_POWER_PUBLISH_SLEEPENTER_OFFSET 0x000194  /* Publish configuration for event SLEEPENTER */
#define NRF53_POWER_PUBLISH_SLEEPEXIT_OFFSET  0x000198  /* Publish configuration for event SLEEPEXIT */
#define NRF53_POWER_INTEN_OFFSET              0x000300  /* Enable or disable interrupt */
#define NRF53_POWER_INTENSET_OFFSET           0x000304  /* Enable interrupt */
#define NRF53_POWER_INTENCLR_OFFSET           0x000308  /* Disable interrupt */
#define NRF53_POWER_GPREGRET0_OFFSET          0x00051c  /* General purpose retention register 1 */
#define NRF53_POWER_GPREGRET1_OFFSET          0x000520  /* General purpose retention register 2 */

/* Register definitions *****************************************************/

#define NRF53_POWER_TASKS_CONSTLAT     (NRF53_POWER_BASE + NRF53_POWER_TASKS_CONSTLAT_OFFSET)
#define NRF53_POWER_TASKS_LOWPWR       (NRF53_POWER_BASE + NRF53_POWER_TASKS_LOWPWR_OFFSET)
#define NRF53_POWER_EVENTS_POFWARN     (NRF53_POWER_BASE + NRF53_POWER_EVENTS_POFWARN_OFFSET)
#define NRF53_POWER_EVENTS_SLEEPENTER  (NRF53_POWER_BASE + NRF53_POWER_EVENTS_SLEEPENTER_OFFSET)
#define NRF53_POWER_EVENTS_SLEEPEXIT   (NRF53_POWER_BASE + NRF53_POWER_EVENTS_SLEEPEXIT_OFFSET)
#define NRF53_POWER_PUBLISH_POFWARN    (NRF53_POWER_BASE + NRF53_POWER_PUBLISH_POFWARN_OFFSET)
#define NRF53_POWER_PUBLISH_SLEEPENTER (NRF53_POWER_BASE + NRF53_POWER_PUBLISH_SLEEPENTER_OFFSET)
#define NRF53_POWER_PUBLISH_SLEEPEXIT  (NRF53_POWER_BASE + NRF53_POWER_PUBLISH_SLEEPEXIT_OFFSET)
#define NRF53_POWER_INTEN              (NRF53_POWER_BASE + NRF53_POWER_INTEN_OFFSET)
#define NRF53_POWER_INTENSET           (NRF53_POWER_BASE + NRF53_POWER_INTENSET_OFFSET)
#define NRF53_POWER_INTENCLR           (NRF53_POWER_BASE + NRF53_POWER_INTENCLR_OFFSET)
#define NRF53_POWER_GPREGRET1          (NRF53_POWER_BASE + NRF53_POWER_GPREGRET1_OFFSET)
#define NRF53_POWER_GPREGRET2          (NRF53_POWER_BASE + NRF53_POWER_GPREGRET2_OFFSET)

/* Register bit definitions *************************************************/

#define NRF53_POWER_INTENSET_POFWARN         (1 << 2)   /* Read: Enabled */
#define NRF53_POWER_INTENSET_SLEEPENTER      (1 << 5)   /* Read: Enabled */
#define NRF53_POWER_INTENSET_SLEEPEXIT       (1 << 6)   /* Read: Enabled */

#define NRF53_POWER_INTENCLR_POFWARN         (1 << 2)   /* Read: Enabled */
#define NRF53_POWER_INTENCLR_SLEEPENTER      (1 << 5)   /* Read: Enabled */
#define NRF53_POWER_INTENCLR_SLEEPEXIT       (1 << 6)   /* Read: Enabled */

#define NRF53_POWER_SYSTEMOFF_ENABLE         (1 << 0)   /* Enable System OFF mode */

#define NRF53_POWER_GPREGRET1_MASK           (0xff)     /* General purpose retention register 1 */

#define NRF53_POWER_GPREGRET2_GPREGRET_MASK  (0xff)     /* General purpose retention register 2*/

#endif /* __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_POWER_H */
