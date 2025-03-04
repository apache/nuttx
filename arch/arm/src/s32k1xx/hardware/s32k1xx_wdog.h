/****************************************************************************
 * arch/arm/src/s32k1xx/hardware/s32k1xx_wdog.h
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

#ifndef __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_WDOG_H
#define __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_WDOG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <hardware/s32k1xx_memorymap.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* WDOG Register Offsets ****************************************************/

#define S32K1XX_WDOG_CS_OFFSET       0x0000  /* Watchdog Control and Status Register */
#define S32K1XX_WDOG_CNT_OFFSET      0x0004  /* Watchdog Counter Register */
#define S32K1XX_WDOG_TOVAL_OFFSET    0x0008  /* Watchdog Timeout Value Register */
#define S32K1XX_WDOG_WIN_OFFSET      0x000c  /* Watchdog Window Register */

/* WDOG Register Addresses **************************************************/

#define S32K1XX_WDOG_CS              (S32K1XX_WDOG_BASE + S32K1XX_WDOG_CS_OFFSET)
#define S32K1XX_WDOG_CNT             (S32K1XX_WDOG_BASE + S32K1XX_WDOG_CNT_OFFSET)
#define S32K1XX_WDOG_TOVAL           (S32K1XX_WDOG_BASE + S32K1XX_WDOG_TOVAL_OFFSET)
#define S32K1XX_WDOG_WIN             (S32K1XX_WDOG_BASE + S32K1XX_WDOG_WIN_OFFSET)

/* WDOG Register Bitfield Definitions ***************************************/

/* Watchdog Control and Status Register */

#define WDOG_CS_STOP                 (1 << 0)  /* Bit 0:  Stop Enable */
#define WDOG_CS_WAIT                 (1 << 1)  /* Bit 1:  Wait Enable */
#define WDOG_CS_DBG                  (1 << 2)  /* Bit 2:  Debug Enable */
#define WDOG_CS_TST_SHIFT            (3)       /* Bits 3-4:  Watchdog test */
#define WDOG_CS_TST_MASK             (3 << WDOG_CS_TST_SHIFT)
#  define WDOG_CS_TST_DISABLE        (0 << WDOG_CS_TST_SHIFT) /* Watchdog test mode disabled */
#  define WDOG_CS_TST_USER           (1 << WDOG_CS_TST_SHIFT) /* Watchdog user mode enabled */
#  define WDOG_CS_TST_LOWBYTE        (2 << WDOG_CS_TST_SHIFT) /* Watchdog low byte test mode */
#  define WDOG_CS_TST_HIGHBYTE       (3 << WDOG_CS_TST_SHIFT) /* Watchdog high byte test mode */

#define WDOG_CS_UPDATE               (1 << 5)  /* Bit 5:  Allow updates */
#define WDOG_CS_INT                  (1 << 6)  /* Bit 6:  Watchdog Interrupt */
#define WDOG_CS_EN                   (1 << 7)  /* Bit 7:  Watchdog Enable */
#define WDOG_CS_CLK_SHIFT            (8)       /* Bits 8-9: Watchdog Clock */
#define WDOG_CS_CLK_MASK             (3 << WDOG_CS_CLK_SHIFT)
#  define WDOG_CS_CLK_BUSCLK         (0 << WDOG_CS_CLK_SHIFT) /* Bus clock */
#  define WDOG_CS_CLK_LPOCLK         (1 << WDOG_CS_CLK_SHIFT) /* LPO clock */
#  define WDOG_CS_CLK_INTCLK         (2 << WDOG_CS_CLK_SHIFT) /* INTCLK (internal clock) */
#  define WDOG_CS_CLK_ERCLK          (3 << WDOG_CS_CLK_SHIFT) /* ERCLK (external reference clock) */

#define WDOG_CS_RCS                  (1 << 10) /* Bit 10: Reconfiguration Success */
#define WDOG_CS_ULK                  (1 << 11) /* Bit 11: Unlock status */
#define WDOG_CS_PRES                 (1 << 12) /* Bit 12: Watchdog prescalr */
#define WDOG_CS_CMD32EN              (1 << 13) /* Bit 13: WDOG support for 32-bit command write */
#define WDOG_CS_FLG                  (1 << 14) /* Bit 14: Watchdog Interrupt Flag */
#define WDOG_CS_WIN                  (1 << 15) /* Bit 15: Watchdog Window */

/* Watchdog Counter Register (16-bit counter value) */

#define WDOG_CNT_CNTLOW_SHIFT        (0)       /* Bits 0-7:  Low byte of the Watchdog Counter */
#define WDOG_CNT_CNTLOW_MASK         (0xff << WDOG_CNT_CNTLOW_SHIFT)
#define WDOG_CNT_CNTHIGH_SHIFT       (8)       /* Bits 8-15: High byte of the Watchdog Counter */
#define WDOG_CNT_CNTHIGH_MASK        (0xff << WDOG_CNT_CNTHIGH_SHIFT)

/* The refresh write sequence can be either:
 *
 * If WDOG_CS[CMD32EN] is 0:
 * 1) Two 16-bit writes: 0xa602, 0xb480
 * 2) Four 8-bit writes: 0xa6, 0x02, 0xb4, 0x80
 * If WDOG_CS[CMD32EN] is 1:
 * 3) One 32-bit write (0xb480a602)
 */

#define WDOG_CNT_REFRESH_HWORD1      0xa602
#define WDOG_CNT_REFRESH_HWORD2      0xb480
#define WDOG_CNT_REFRESH_BYTE1       0xa6
#define WDOG_CNT_REFRESH_BYTE2       0x02
#define WDOG_CNT_REFRESH_BYTE3       0xb4
#define WDOG_CNT_REFRESH_BYTE4       0x80
#define WDOG_CNT_REFRESH_DWORD       0xb480a602

/* Value to unlock the watchdog registers */

#define WDOG_CNT_UNLOCK              0xd928c520

/* Watchdog Timeout Value Register */

#define WDOG_TOVAL_TOVALLOW_SHIFT    (0)       /* Bits 0-7:  Low byte of the timeout value */
#define WDOG_TOVAL_TOVALLOW_MASK     (0xff << WDOG_TOVAL_TOVALLOW_SHIFT)
#define WDOG_TOVAL_TOVALHIGH_SHIFT   (8)       /* Bits 8-15: High byte of the timeout value */
#define WDOG_TOVAL_TOVALHIGH_MASK    (0xff << WDOG_TOVAL_TOVALHIGH_SHIFT)

/* Watchdog Window Register */

#define WDOG_WIN_WINLOW_SHIFT        (0)       /* Bits 0-7:  Low byte of Watchdog Window */
#define WDOG_WIN_WINLOW_MASK         (0xff << WDOG_WIN_WINLOW_SHIFT)
#define WDOG_WIN_WINHIGH_SHIFT       (8)       /* Bits 8-15: High byte of Watchdog Window */
#define WDOG_WIN_WINHIGH_MASK        (0xff << WDOG_WIN_WINHIGH_SHIFT)

#endif /* __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_WDOG_H */
