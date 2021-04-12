/****************************************************************************
 * arch/arm/src/imxrt/hardware/imxrt_wdog.h
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

#ifndef __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_WDOG_H
#define __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_WDOG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/imxrt_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define IMXRT_WDOG_WCR_OFFSET       0x0000  /* Watchdog control register */
#define IMXRT_WDOG_WSR_OFFSET       0x0002  /* Watchdog service register */
#define IMXRT_WDOG_WRSR_OFFSET      0x0004  /* Watchdog reset status */
#define IMXRT_WDOG_WICR_OFFSET      0x0006  /* Watchdog interrupt control */
#define IMXRT_WDOG_WMCR_OFFSET      0x0008  /* Watchdog misc control */

#define IMXRT_RTWDOG_CS_OFFSET      0x0000  /* Watchdog control and status register */
#define IMXRT_RTWDOG_CNT_OFFSET     0x0004  /* Watchdog counter register */
#define IMXRT_RTWDOG_TOVAL_OFFSET   0x0008  /* Watchdog timeout value register */
#define IMXRT_RTWDOG_WIN_OFFSET     0x000c  /* Watchdog window register */

/* Register addresses *******************************************************/

#define IMXRT_WDOG1_WCR             (IMXRT_WDOG1_BASE + IMXRT_WDOG_WCR_OFFSET)
#define IMXRT_WDOG1_WSR             (IMXRT_WDOG1_BASE + IMXRT_WDOG_WSR_OFFSET)
#define IMXRT_WDOG1_WRSR            (IMXRT_WDOG1_BASE + IMXRT_WDOG_WRSR_OFFSET)
#define IMXRT_WDOG1_WICR            (IMXRT_WDOG1_BASE + IMXRT_WDOG_WICR_OFFSET)
#define IMXRT_WDOG1_WMCR            (IMXRT_WDOG1_BASE + IMXRT_WDOG_WMCR_OFFSET)

#define IMXRT_WDOG2_WCR             (IMXRT_WDOG2_BASE + IMXRT_WDOG_WCR_OFFSET)
#define IMXRT_WDOG2_WSR             (IMXRT_WDOG2_BASE + IMXRT_WDOG_WSR_OFFSET)
#define IMXRT_WDOG2_WRSR            (IMXRT_WDOG2_BASE + IMXRT_WDOG_WRSR_OFFSET)
#define IMXRT_WDOG2_WMCR            (IMXRT_WDOG2_BASE + IMXRT_WDOG_WMCR_OFFSET)

#define IMXRT_RTWDOG_CS             (IMXRT_WDOG3_BASE + IMXRT_RTWDOG_CS_OFFSET)
#define IMXRT_RTWDOG_CNT            (IMXRT_WDOG3_BASE + IMXRT_RTWDOG_CNT_OFFSET)
#define IMXRT_RTWDOG_TOVAL          (IMXRT_WDOG3_BASE + IMXRT_RTWDOG_TOVAL_OFFSET)
#define IMXRT_RTWDOG_WIN            (IMXRT_WDOG3_BASE + IMXRT_RTWDOG_WIN_OFFSET)

/* Register bit definitions *************************************************/

/* Watchdog control and status register */

#define WDOG_WCR_WDZST              (1 << 0)  /* Bit 0:  Watchdog Low Power */
#define WDOG_WCR_WDBG               (1 << 1)  /* Bit 1:  Watchdog DEBUG Enable */
#define WDOG_WCR_WDE                (1 << 2)  /* Bit 2:  Watchdog Enable */
#define WDOG_WCR_WDT                (1 << 3)  /* Bit 3:  WDOG_B Time-out assertion */
#define WDOG_WCR_SRS                (1 << 4)  /* Bit 4:  Software Reset Signal */
#define WDOG_WCR_WDA                (1 << 5)  /* Bit 5:  WDOG_B assertion */
#define WDOG_WCR_SRE                (1 << 6)  /* Bit 6:  Software reset extension */
#define WDOG_WCR_WDW                (1 << 7)  /* Bit 7:  Watchdog Disable for Wait */

#define WDOG_WCR_WT_SHIFT           (8)       /* Bits 8-15: Watchdog time-out value */
#define WDOG_WCR_WT_MASK            (0xff << WDOG_WCR_WT_SHIFT)
#  define WDOG_WCR_WT(n)            ((uint16_t)((n)) << WDOG_WCR_WT_SHIFT)

/* Watchdog reset status */

#define WDOG_WRSR_SFTW              (1 << 0)  /* Bit 0:  Software Reset */
#define WDOG_WRSR_TOUT              (1 << 1)  /* Bit 1:  Timeout */
                                              /* Bits 2-3: reserved */
#define WDOG_WRSR_POR               (1 << 4)  /* Bit 4:  Power on reset */
                                              /* Bits 5-15: Reserved */

/* Watchdog interrupt control */

#define WDOG_WICR_WICT_SHIFT        (0)       /* Bits 0-7: Watchdog Interrupt Count Time-out */
#define WDOG_WICR_WICT_MASK         (0xff << WDOG_WCR_WT_SHIFT)
#  define WDOG_WICR_WICT(n)         ((uint16_t)((n)) << WDOG_WICR_WICT_SHIFT)
                                              /* Bits 8-13: Reserved */
#define WDOG_WICR_WTIS              (1 << 14) /* Bit 14: Watchdog Timer Interrupt Status */
#define WDOG_WICR_WIE               (1 << 15) /* Bit 15: Watchdog Timer Interrupt enable */

/* Watchdog misc control */

#define WDOG_WMCR_PDE               (1 << 0)  /* Bit 0:  Power Down Enable */
                                              /* Bits 1-15: Reserved */

/* RT Watchdog Control and Status Register */

#define RTWDOG_CS_STOP              (1 << 0)  /* Bit 0:  Stop enable */
#define RTWDOG_CS_WAIT              (1 << 1)  /* Bit 1:  Wait enable */
#define RTWDOG_CS_DBG               (1 << 2)  /* Bit 2:  Debug Enable */
#define RTWDOG_CS_TST_SHIFT         (3)       /* Bits 3-4: Enables the fast test mode */
#define RTWDOG_CS_TST_MASK          (0x03 << RTWDOG_CS_TST_SHIFT)
#  define RTWDOG_CS_TST(n)          ((uint32_t)((n)) << RTWDOG_CS_TST_SHIFT)
#define RTWDOG_CS_UPDATE            (1 << 5)  /* Bit 5:  Update */
#define RTWDOG_CS_INT               (1 << 6)  /* Bit 6:  Interrupt */
#define RTWDOG_CS_EN                (1 << 7)  /* Bit 7:  Enable */
#define RTWDOG_CS_CLK_SHIFT         (8)       /* Bits 8-9: Clock */
#define RTWDOG_CS_CLK_MASK          (0x03 << RTWDOG_CS_CLK_SHIFT)
#  define RTWDOG_CS_CLK(n)          ((uint32_t)((n)) << RTWDOG_CS_CLK_SHIFT)
#define RTWDOG_CS_RCS               (1 << 10) /* Bit 10: Reconfiguration Success */
#define RTWDOG_CS_ULK               (1 << 11) /* Bit 11: Unlock status */
#define RTWDOG_CS_PRES              (1 << 12) /* Bit 12: Watchdog prescaler */
#define RTWDOG_CS_CMD32EN           (1 << 13) /* Bit 13: WDOG support for 32-bit */
#define RTWDOG_CS_FLG               (1 << 14) /* Bit 14: Interrupt Flag */
#define RTWDOG_CS_WIN               (1 << 15) /* Bit 15: Watchdog Window */

#define RTWDOG_UPDATE_KEY           (0xd928c520)
#define RTWDOG_REFRESH_KEY          (0xb480a602)

#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_WDOG_H */
