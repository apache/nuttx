/***************************************************************************************************
 * arch/arm/src/nrf52/hardware/nrf52_wdt.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author:  Alan Carvalho de Assis <acassis@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ***************************************************************************************************/

#ifndef __ARCH_ARM_SRC_NRF52_HARDWARE_NRF52_WDT_H
#define __ARCH_ARM_SRC_NRF52_HARDWARE_NRF52_WDT_H

/***************************************************************************************************
 * Included Files
 ***************************************************************************************************/

#include <nuttx/config.h>
#include "hardware/nrf52_memorymap.h"

/***************************************************************************************************
 * Pre-processor Definitions
 ***************************************************************************************************/

/* WDT Register Offsets ****************************************************************************/

/* Registers for the WDT function: */

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

/* WDT Register Addresses **************************************************************************/

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

/* WDT Register Bitfield Definitions ***************************************************************/

/* INTENSET/INTENCLR Register */

#define WDT_INT_TIMEOUT        (1 << 0) /* Bit 0: TIMEOUT */

/* REQSTATUS[x] Register */

#define WDT_REQSTATUS_RR(x)    (1 << (x)) /* Bits 0-7: Request status for RR[i] register */

/* RREN[x] Register */

#define WDT_RREN_RR(x)         (1 << (x)) /* Bits 0-7: Enable or disable RR[i] register */

/* CONFIG Register */

#define WDT_CONFIG_SLEEP       (1 << 0) /* Bit 0: */
#define WDT_CONFIG_HALT        (1 << 0) /* Bit 3: */

/* RR[x] Register */

#define WDT_RR_VALUE           (0x6E524635UL) /* Fixed value, don't modify it */

#endif /* __ARCH_ARM_SRC_NRF52_HARDWARE_NRF52_WDT_H */
