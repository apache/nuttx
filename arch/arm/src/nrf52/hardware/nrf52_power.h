/****************************************************************************
 * arch/arm/src/nrf52/hardware/nrf52_power.h
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

#ifndef __ARCH_ARM_SRC_NRF52_HARDWARE_NRF52_POWER_H
#define __ARCH_ARM_SRC_NRF52_HARDWARE_NRF52_POWER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "nrf52_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define NRF52_POWER_TASKS_CONSTLAT_OFFSET     0x000078  /* Enable constant latency mode */
#define NRF52_POWER_TASKS_LOWPWR_OFFSET       0x00007c  /* Enable low power mode (variable latency) */
#define NRF52_POWER_EVENTS_POFWARN_OFFSET     0x000108  /* Power failure warning */
#define NRF52_POWER_EVENTS_SLEEPENTER_OFFSET  0x000114  /* CPU entered WFI/WFE sleep */
#define NRF52_POWER_EVENTS_SLEEPEXIT_OFFSET   0x000118  /* CPU exited WFI/WFE sleep */
#define NRF52_POWER_INTENSET_OFFSET           0x000304  /* Enable interrupt */
#define NRF52_POWER_INTENCLR_OFFSET           0x000308  /* Disable interrupt */
#define NRF52_POWER_RESETREAS_OFFSET          0x000400  /* Reset reason */
#define NRF52_POWER_RAMSTATUS_OFFSET          0x000428  /* Deprecated register -  RAM status register */
#define NRF52_POWER_SYSTEMOFF_OFFSET          0x000500  /* System OFF register */
#define NRF52_POWER_POFCON_OFFSET             0x000510  /* Power failure comparator configuration */
#define NRF52_POWER_GPREGRET_OFFSET           0x00051c  /* General purpose retention register */
#define NRF52_POWER_GPREGRET2_OFFSET          0x000520  /* General purpose retention register */
#define NRF52_POWER_RAMON_OFFSET              0x000524  /* Deprecated register -  RAM on/off register (this register is retained) */
#define NRF52_POWER_RAMONB_OFFSET             0x000554  /* Deprecated register -  RAM on/off register (this register is retained) */
#define NRF52_POWER_DCDCEN_OFFSET             0x000578  /* DC/DC enable register */

/* Register definitions *****************************************************/

#define NRF52_POWER_TASKS_CONSTLAT     (NRF52_POWER_BASE + NRF52_POWER_TASKS_CONSTLAT_OFFSET)
#define NRF52_POWER_TASKS_LOWPWR       (NRF52_POWER_BASE + NRF52_POWER_TASKS_LOWPWR_OFFSET)
#define NRF52_POWER_EVENTS_POFWARN     (NRF52_POWER_BASE + NRF52_POWER_EVENTS_POFWARN_OFFSET)
#define NRF52_POWER_EVENTS_SLEEPENTER  (NRF52_POWER_BASE + NRF52_POWER_EVENTS_SLEEPENTER_OFFSET)
#define NRF52_POWER_EVENTS_SLEEPEXIT   (NRF52_POWER_BASE + NRF52_POWER_EVENTS_SLEEPEXIT_OFFSET)
#define NRF52_POWER_INTENSET           (NRF52_POWER_BASE + NRF52_POWER_INTENSET_OFFSET)
#define NRF52_POWER_INTENCLR           (NRF52_POWER_BASE + NRF52_POWER_INTENCLR_OFFSET)
#define NRF52_POWER_RESETREAS          (NRF52_POWER_BASE + NRF52_POWER_RESETREAS_OFFSET)
#define NRF52_POWER_RAMSTATUS          (NRF52_POWER_BASE + NRF52_POWER_RAMSTATUS_OFFSET)
#define NRF52_POWER_SYSTEMOFF          (NRF52_POWER_BASE + NRF52_POWER_SYSTEMOFF_OFFSET)
#define NRF52_POWER_POFCON             (NRF52_POWER_BASE + NRF52_POWER_POFCON_OFFSET)
#define NRF52_POWER_GPREGRET           (NRF52_POWER_BASE + NRF52_POWER_GPREGRET_OFFSET)
#define NRF52_POWER_GPREGRET2          (NRF52_POWER_BASE + NRF52_POWER_GPREGRET2_OFFSET)
#define NRF52_POWER_RAMON              (NRF52_POWER_BASE + NRF52_POWER_RAMON_OFFSET)
#define NRF52_POWER_RAMONB             (NRF52_POWER_BASE + NRF52_POWER_RAMONB_OFFSET)
#define NRF52_POWER_DCDCEN             (NRF52_POWER_BASE + NRF52_POWER_DCDCEN_OFFSET)

/* Register bit definitions *************************************************/

#define NRF52_POWER_INTENSET_POFWARN         (1 << 2)   /* Read: Enabled */
#define NRF52_POWER_INTENSET_SLEEPENTER      (1 << 5)   /* Read: Enabled */
#define NRF52_POWER_INTENSET_SLEEPEXIT       (1 << 6)   /* Read: Enabled */

#define NRF52_POWER_INTENCLR_POFWARN         (1 << 2)   /* Read: Enabled */
#define NRF52_POWER_INTENCLR_SLEEPENTER      (1 << 5)   /* Read: Enabled */
#define NRF52_POWER_INTENCLR_SLEEPEXIT       (1 << 6)   /* Read: Enabled */

#define NRF52_POWER_RESETREAS_RESETPIN       (1 << 0)   /* Detected */
#define NRF52_POWER_RESETREAS_DOG            (1 << 1)   /* Detected */
#define NRF52_POWER_RESETREAS_SREQ           (1 << 2)   /* Detected */
#define NRF52_POWER_RESETREAS_LOCKUP         (1 << 3)   /* Detected */
#define NRF52_POWER_RESETREAS_OFF            (1 << 16)  /* Detected */
#define NRF52_POWER_RESETREAS_LPCOMP         (1 << 17)  /* Detected */
#define NRF52_POWER_RESETREAS_DIF            (1 << 18)  /* Detected */
#define NRF52_POWER_RESETREAS_NFC            (1 << 19)  /* Detected */

#define NRF52_POWER_RAMSTATUS_RAMBLOCK0      (1 << 0)   /* On */
#define NRF52_POWER_RAMSTATUS_RAMBLOCK1      (1 << 1)   /* On */
#define NRF52_POWER_RAMSTATUS_RAMBLOCK2      (1 << 2)   /* On */
#define NRF52_POWER_RAMSTATUS_RAMBLOCK3      (1 << 3)   /* On */

#define NRF52_POWER_SYSTEMOFF_ENABLE         (1 << 0)   /* Enable System OFF mode */

#define NRF52_POWER_POFCON_POF               (1 << 0)   /* Enable */
#define NRF52_POWER_POFCON_THRESHOLD_SHIFT   (1)        /* Power failure comparator threshold setting */

#define NRF52_POWER_POFCON_THRESHOLD_MASK    (0x0f << NRF52_POWER_POFCON_THRESHOLD_SHIFT)
#define NRF52_POWER_POFCON_THRESHOLD_V19     (0x6 << NRF52_POWER_POFCON_THRESHOLD_SHIFT)  /* Set threshold to 1.9 V */
#define NRF52_POWER_POFCON_THRESHOLD_V20     (0x7 << NRF52_POWER_POFCON_THRESHOLD_SHIFT)  /* Set threshold to 2.0 V */
#define NRF52_POWER_POFCON_THRESHOLD_V21     (0x8 << NRF52_POWER_POFCON_THRESHOLD_SHIFT)  /* Set threshold to 2.1 V */
#define NRF52_POWER_POFCON_THRESHOLD_V22     (0x9 << NRF52_POWER_POFCON_THRESHOLD_SHIFT)  /* Set threshold to 2.2 V */
#define NRF52_POWER_POFCON_THRESHOLD_V23     (0xa << NRF52_POWER_POFCON_THRESHOLD_SHIFT)  /* Set threshold to 2.3 V */
#define NRF52_POWER_POFCON_THRESHOLD_V24     (0xb << NRF52_POWER_POFCON_THRESHOLD_SHIFT)  /* Set threshold to 2.4 V */
#define NRF52_POWER_POFCON_THRESHOLD_V27     (0xe << NRF52_POWER_POFCON_THRESHOLD_SHIFT)  /* Set threshold to 2.7 V */
#define NRF52_POWER_POFCON_THRESHOLD_V28     (0xf << NRF52_POWER_POFCON_THRESHOLD_SHIFT)  /* Set threshold to 2.8 V */

#define NRF52_POWER_GPREGRET_MASK            (0xff)     /* General purpose retention register */

#define NRF52_POWER_GPREGRET2_GPREGRET_MASK  (0xff)     /* General purpose retention register */

#define NRF52_POWER_RAMON_ONRAM0             (1 << 0)   /* On */
#define NRF52_POWER_RAMON_ONRAM1             (1 << 1)   /* On */
#define NRF52_POWER_RAMON_OFFRAM0            (1 << 16)  /* On */
#define NRF52_POWER_RAMON_OFFRAM1            (1 << 17)  /* On */

#define NRF52_POWER_RAMONB_ONRAM2            (1 << 0)   /* On */
#define NRF52_POWER_RAMONB_ONRAM3            (1 << 1)   /* On */
#define NRF52_POWER_RAMONB_OFFRAM2           (1 << 16)  /* On */
#define NRF52_POWER_RAMONB_OFFRAM3           (1 << 17)  /* On */

#define NRF52_POWER_DCDCEN_ENABLE            (1 << 0)   /* Enable */

#endif /* __ARCH_ARM_SRC_NRF52_HARDWARE_NRF52_POWER_H */
