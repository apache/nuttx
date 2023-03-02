/****************************************************************************
 * arch/arm/src/nrf52/hardware/nrf52_wdt.h
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

#ifndef __ARCH_ARM_SRC_NRF52_HARDWARE_NRF52_WDT_H
#define __ARCH_ARM_SRC_NRF52_HARDWARE_NRF52_WDT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/nrf52_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* WDT Register Offsets *****************************************************/

#define NRF52_WDT_TASKS_START_OFFSET      0x0000  /* Start the watchdog */
#define NRF52_WDT_EVENTS_TIMEOUT_OFFSET   0x0100  /* Watchdog timeout */
#define NRF52_WDT_INTENSET_OFFSET         0x0304  /* Enable interrupt */
#define NRF52_WDT_INTENCLR_OFFSET         0x0308  /* Disable interrupt */
#define NRF52_WDT_RUNSTATUS_OFFSET        0x0400  /* Run status */
#define NRF52_WDT_REQSTATUS_OFFSET        0x0404  /* Request status */
#define NRF52_WDT_CRV_OFFSET              0x0504  /* Counter reload value */
#define NRF52_WDT_RREN_OFFSET             0x0508  /* Enable register for reload request registers */
#define NRF52_WDT_CONFIG_OFFSET           0x050c  /* Configuration register */
#define NRF52_WDT_RR0_OFFSET              0x0600  /* Reload request 0 */
#define NRF52_WDT_RR1_OFFSET              0x0604  /* Reload request 1 */
#define NRF52_WDT_RR2_OFFSET              0x0608  /* Reload request 2 */
#define NRF52_WDT_RR3_OFFSET              0x060c  /* Reload request 3 */
#define NRF52_WDT_RR4_OFFSET              0x0610  /* Reload request 4 */
#define NRF52_WDT_RR5_OFFSET              0x0614  /* Reload request 5 */
#define NRF52_WDT_RR6_OFFSET              0x0618  /* Reload request 6 */
#define NRF52_WDT_RR7_OFFSET              0x061c  /* Reload request 7 */

/* WDT Register Addresses ***************************************************/

#define NRF52_WDT_TASKS_START             (NRF52_WDT_BASE + NRF52_WDT_TASKS_START_OFFSET)
#define NRF52_WDT_EVENTS_TIMEOUT          (NRF52_WDT_BASE + NRF52_WDT_EVENTS_TIMEOUT_OFFSET)
#define NRF52_WDT_INTENSET                (NRF52_WDT_BASE + NRF52_WDT_INTENSET_OFFSET)
#define NRF52_WDT_INTENCLR                (NRF52_WDT_BASE + NRF52_WDT_INTENCLR_OFFSET)
#define NRF52_WDT_RUNSTATUS               (NRF52_WDT_BASE + NRF52_WDT_RUNSTATUS_OFFSET)
#define NRF52_WDT_REQSTATUS               (NRF52_WDT_BASE + NRF52_WDT_REQSTATUS_OFFSET)
#define NRF52_WDT_CRV                     (NRF52_WDT_BASE + NRF52_WDT_CRV_OFFSET)
#define NRF52_WDT_RREN                    (NRF52_WDT_BASE + NRF52_WDT_RREN_OFFSET)
#define NRF52_WDT_CONFIG                  (NRF52_WDT_BASE + NRF52_WDT_CONFIG_OFFSET)
#define NRF52_WDT_RR0                     (NRF52_WDT_BASE + NRF52_WDT_RR0_OFFSET)
#define NRF52_WDT_RR1                     (NRF52_WDT_BASE + NRF52_WDT_RR1_OFFSET)
#define NRF52_WDT_RR2                     (NRF52_WDT_BASE + NRF52_WDT_RR2_OFFSET)
#define NRF52_WDT_RR3                     (NRF52_WDT_BASE + NRF52_WDT_RR3_OFFSET)
#define NRF52_WDT_RR4                     (NRF52_WDT_BASE + NRF52_WDT_RR4_OFFSET)
#define NRF52_WDT_RR5                     (NRF52_WDT_BASE + NRF52_WDT_RR5_OFFSET)
#define NRF52_WDT_RR6                     (NRF52_WDT_BASE + NRF52_WDT_RR6_OFFSET)
#define NRF52_WDT_RR7                     (NRF52_WDT_BASE + NRF52_WDT_RR7_OFFSET)

/* WDT Register Bitfield Definitions ****************************************/

/* INTENSET/INTENCLR Register */

#define WDT_INT_TIMEOUT        (1 << 0) /* Bit 0: TIMEOUT */

/* REQSTATUS[x] Register */

#define WDT_REQSTATUS_RR(x)    (1 << (x)) /* Bits 0-7: Request status for RR[i] register */

/* RREN[x] Register */

#define WDT_RREN_RR(x)         (1 << (x)) /* Bits 0-7: Enable or disable RR[i] register */

/* CONFIG Register */

#define WDT_CONFIG_SLEEP       (1 << 0) /* Bit 0: */
#define WDT_CONFIG_HALT        (1 << 3) /* Bit 3: */

/* RR[x] Register */

#define WDT_RR_VALUE           (0x6E524635UL) /* Fixed value, don't modify it */

#endif /* __ARCH_ARM_SRC_NRF52_HARDWARE_NRF52_WDT_H */
