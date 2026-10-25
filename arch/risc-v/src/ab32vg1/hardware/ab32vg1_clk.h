/****************************************************************************
 * arch/risc-v/src/ab32vg1/hardware/ab32vg1_clk.h
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

#ifndef __ARCH_RISCV_SRC_AB32VG1_HARDWARE_AB32VG1_CLK_H
#define __ARCH_RISCV_SRC_AB32VG1_HARDWARE_AB32VG1_CLK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/ab32vg1_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clock, PLL and watchdog registers.  The clock tree is not covered by the
 * register manual; values follow the RT-Thread BSP (system_ab32vgx.c) and
 * were checked on hardware by measuring TMR0 against the UART bit clock.
 */

#define AB32VG1_CLKCON0         AB32VG1_SFR(0, 0x19)
#define AB32VG1_WDTCON          AB32VG1_SFR(0, 0x1a)
#define AB32VG1_CLKCON1         AB32VG1_SFR(0, 0x1d)
#define AB32VG1_SPI0BAUD        AB32VG1_SFR(0, 0x2c)
#define AB32VG1_PLL0DIV         AB32VG1_SFR(3, 0x23)
#define AB32VG1_PLL0CON         AB32VG1_SFR(3, 0x26)
#define AB32VG1_CLKCON2         AB32VG1_SFR(3, 0x2a)
#define AB32VG1_CLKGAT0         AB32VG1_SFR(3, 0x2c)
#define AB32VG1_CLKCON3         AB32VG1_SFR(3, 0x2f)
#define AB32VG1_CLKGAT1         AB32VG1_SFR(3, 0x3f)

/* CLKCON0 */

#define CLKCON0_SYSCLK_RC2M     (0 << 2)          /* [3:2] system clock */
#define CLKCON0_SYSCLK_SPLL     (2 << 2)
#define CLKCON0_SYSCLK_MASK     (3 << 2)
#define CLKCON0_SPLL_PLL0       (0 << 4)          /* [6:4] SPLL source */
#define CLKCON0_SPLL_XOSC26M    (4 << 4)
#define CLKCON0_SPLL_MASK       (7 << 4)

/* CLKCON1 */

#define CLKCON1_UART_INC        (1 << 14)         /* uart_inc clock on */

/* CLKCON2 */

#define CLKCON2_SPLLDIV_SHIFT   8                 /* [12:8] SPLL divider */
#define CLKCON2_SPLLDIV_MASK    (0x1f << CLKCON2_SPLLDIV_SHIFT)

/* PLL0CON */

#define PLL0CON_VCO_SHIFT       3                 /* [5:3] VCO band */
#define PLL0CON_VCO_MASK        (7 << PLL0CON_VCO_SHIFT)
#define PLL0CON_VCO_HIGH        (1 << PLL0CON_VCO_SHIFT) /* > 206 MHz */
#define PLL0CON_ANALOG_EN       (1 << 6)
#define PLL0CON_LDO_EN          (1 << 12)
#define PLL0CON_SDM_EN          (1 << 18)
#define PLL0CON_DIV_UPDATE      (1 << 20)

/* PLL0 runs at 240 MHz from the 26 MHz crystal */

#define AB32VG1_XOSC_FREQUENCY  26000000
#define AB32VG1_PLL0_FREQUENCY  240000000
#define AB32VG1_PLL0DIV_240M    (240 * 65536 / 26)

/* CLKGAT0 / CLKGAT1 peripheral clock gates */

#define CLKGAT0_TMR0            (1 << 8)
#define CLKGAT0_SD0             (1 << 9)
#define CLKGAT0_UART0           (1 << 10)
#define CLKGAT0_HSUART          (1 << 11)
#define CLKGAT0_SARADC          (1 << 13)
#define CLKGAT0_USB             (1 << 14)
#define CLKGAT0_PORT            (1 << 17)
#define CLKGAT0_SPI0            (1 << 19)
#define CLKGAT0_UART1           (1 << 21)
#define CLKGAT0_TMR1            (1 << 24)
#define CLKGAT0_TMR2            (1 << 25)
#define CLKGAT1_TMR3            (1 << 8)
#define CLKGAT1_TMR4            (1 << 9)
#define CLKGAT1_TMR5            (1 << 10)
#define CLKGAT1_UART2           (1 << 11)
#define CLKGAT1_SPI1            (1 << 12)

/* WDTCON (register manual, chapter 6).  The *_WR fields are write keys. */

#define WDTCON_CLR              0x0000000a        /* Kick the watchdog */
#define WDTCON_DISABLE          0x00000aa0        /* WDTEN=0, RSTEN=0 */
#define WDTCON_RESET_1MS        0x0a000110        /* 1 ms timeout, reset */
#define WDTCON_WDTPND           (1u << 31)

#endif /* __ARCH_RISCV_SRC_AB32VG1_HARDWARE_AB32VG1_CLK_H */
