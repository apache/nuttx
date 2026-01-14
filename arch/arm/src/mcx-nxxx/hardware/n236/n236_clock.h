/****************************************************************************
 * arch/arm/src/mcx-nxxx/hardware/n236/n236_clock.h
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

#ifndef __ARCH_ARM_SRC_MCX_NXXX_HARDWARE_N236_N236_CLOCK_H
#define __ARCH_ARM_SRC_MCX_NXXX_HARDWARE_N236_N236_CLOCK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clock muxes */

#define SYSCON_SYSTICKCLKSEL0   (NXXX_SYSCON0_BASE + 0x260)
#define SYSCON_TRACECLKSEL      (NXXX_SYSCON0_BASE + 0x268)
#define SYSCON_CTIMERCLKSEL0    (NXXX_SYSCON0_BASE + 0x26C)
#define SYSCON_CTIMERCLKSEL1    (NXXX_SYSCON0_BASE + 0x270)
#define SYSCON_CTIMERCLKSEL2    (NXXX_SYSCON0_BASE + 0x274)
#define SYSCON_CTIMERCLKSEL3    (NXXX_SYSCON0_BASE + 0x278)
#define SYSCON_CTIMERCLKSEL4    (NXXX_SYSCON0_BASE + 0x27C)
#define SYSCON_CLKOUTCLKSEL     (NXXX_SYSCON0_BASE + 0x288)
#define SYSCON_ADC0CLKSEL       (NXXX_SYSCON0_BASE + 0x2A4)
#define SYSCON_FCCLKSEL0        (NXXX_SYSCON0_BASE + 0x2B0)
#define SYSCON_FCCLKSEL1        (NXXX_SYSCON0_BASE + 0x2B4)
#define SYSCON_FCCLKSEL2        (NXXX_SYSCON0_BASE + 0x2B8)
#define SYSCON_FCCLKSEL3        (NXXX_SYSCON0_BASE + 0x2BC)
#define SYSCON_FCCLKSEL4        (NXXX_SYSCON0_BASE + 0x2C0)
#define SYSCON_FCCLKSEL5        (NXXX_SYSCON0_BASE + 0x2C4)
#define SYSCON_FCCLKSEL6        (NXXX_SYSCON0_BASE + 0x2C8)
#define SYSCON_FCCLKSEL7        (NXXX_SYSCON0_BASE + 0x2CC)
#define SYSCON_ADC1CLKSEL       (NXXX_SYSCON0_BASE + 0x464)
#define SYSCON_PLLCLKDIVSEL     (NXXX_SYSCON0_BASE + 0x52C)
#define SYSCON_I3C0FCLKSEL      (NXXX_SYSCON0_BASE + 0x530)
#define SYSCON_MICFILFCLKSEL    (NXXX_SYSCON0_BASE + 0x548)
#define SYSCON_FLEXIOCLKSEL     (NXXX_SYSCON0_BASE + 0x560)
#define SYSCON_FLEXCAN0CLKSEL   (NXXX_SYSCON0_BASE + 0x5A0)
#define SYSCON_FLEXCAN1CLKSEL   (NXXX_SYSCON0_BASE + 0x5A8)
#define SYSCON_EWM0CLKSEL       (NXXX_SYSCON0_BASE + 0x5D4)
#define SYSCON_WDT1CLKSEL       (NXXX_SYSCON0_BASE + 0x5D8)
#define SYSCON_OSTIMERCLKSEL    (NXXX_SYSCON0_BASE + 0x5E0)
#define SYSCON_CMP0FCLKSEL      (NXXX_SYSCON0_BASE + 0x5F0)
#define SYSCON_CMP0RRCLKSEL     (NXXX_SYSCON0_BASE + 0x5F8)
#define SYSCON_CMP1FCLKSEL      (NXXX_SYSCON0_BASE + 0x600)
#define SYSCON_CMP1RRCLKSEL     (NXXX_SYSCON0_BASE + 0x608)
#define SYSCON_UTICKCLKSEL      (NXXX_SYSCON0_BASE + 0x878)
#define SYSCON_SAI0CLKSEL       (NXXX_SYSCON0_BASE + 0x880)
#define SYSCON_SAI1CLKSEL       (NXXX_SYSCON0_BASE + 0x884)
#define SYSCON_I3C1FCLKSEL      (NXXX_SYSCON0_BASE + 0xB30)

/* Clock dividers */

#define SYSCON_DIVSYSTICKCLK0   (NXXX_SYSCON0_BASE + 0x300)
#define SYSCON_DIVTRACECLK      (NXXX_SYSCON0_BASE + 0x308)
#define SYSCON_DIVSLOWCLK       (NXXX_SYSCON0_BASE + 0x378)
#define SYSCON_DIVAHBCLK        (NXXX_SYSCON0_BASE + 0x380)
#define SYSCON_DIVCLKOUT        (NXXX_SYSCON0_BASE + 0x384)
#define SYSCON_DIVFROHFCLK      (NXXX_SYSCON0_BASE + 0x388)
#define SYSCON_DIVWDT0CLK       (NXXX_SYSCON0_BASE + 0x38C)
#define SYSCON_DIVADC0CLK       (NXXX_SYSCON0_BASE + 0x394)
#define SYSCON_DIVPLLCLK        (NXXX_SYSCON0_BASE + 0x3C4)
#define SYSCON_DIVCTIMER0CLK    (NXXX_SYSCON0_BASE + 0x3D0)
#define SYSCON_DIVCTIMER1CLK    (NXXX_SYSCON0_BASE + 0x3D4)
#define SYSCON_DIVCTIMER2CLK    (NXXX_SYSCON0_BASE + 0x3D8)
#define SYSCON_DIVCTIMER3CLK    (NXXX_SYSCON0_BASE + 0x3DC)
#define SYSCON_DIVCTIMER4CLK    (NXXX_SYSCON0_BASE + 0x3E0)
#define SYSCON_DIVPLL1CLK0      (NXXX_SYSCON0_BASE + 0x3E4)
#define SYSCON_DIVPLL1CLK1      (NXXX_SYSCON0_BASE + 0x3E8)
#define SYSCON_DIVUTICKCLK      (NXXX_SYSCON0_BASE + 0x3F0)
#define SYSCON_DIVFRG           (NXXX_SYSCON0_BASE + 0x3F4)
#define SYSCON_DIVADC1CLK       (NXXX_SYSCON0_BASE + 0x468)
#define SYSCON_DIVI3C0FCLK      (NXXX_SYSCON0_BASE + 0x540)
#define SYSCON_DIVMICFILFCLK    (NXXX_SYSCON0_BASE + 0x54C)
#define SYSCON_DIVFLEXIOCLK     (NXXX_SYSCON0_BASE + 0x564)
#define SYSCON_DIVFLEXCAN0CLK   (NXXX_SYSCON0_BASE + 0x5A4)
#define SYSCON_DIVFLEXCAN1CLK   (NXXX_SYSCON0_BASE + 0x5AC)
#define SYSCON_DIVWDT1CLK       (NXXX_SYSCON0_BASE + 0x5DC)
#define SYSCON_DIVCMP0FCLK      (NXXX_SYSCON0_BASE + 0x5F4)
#define SYSCON_DIVCMP0RRCLK     (NXXX_SYSCON0_BASE + 0x5FC)
#define SYSCON_DIVCMP1FCLK      (NXXX_SYSCON0_BASE + 0x604)
#define SYSCON_DIVCMP1RRCLK     (NXXX_SYSCON0_BASE + 0x60C)
#define SYSCON_DIVFLEXCOM0CLK   (NXXX_SYSCON0_BASE + 0x850)
#define SYSCON_DIVFLEXCOM1CLK   (NXXX_SYSCON0_BASE + 0x854)
#define SYSCON_DIVFLEXCOM2CLK   (NXXX_SYSCON0_BASE + 0x858)
#define SYSCON_DIVFLEXCOM3CLK   (NXXX_SYSCON0_BASE + 0x85C)
#define SYSCON_DIVFLEXCOM4CLK   (NXXX_SYSCON0_BASE + 0x860)
#define SYSCON_DIVFLEXCOM5CLK   (NXXX_SYSCON0_BASE + 0x864)
#define SYSCON_DIVFLEXCOM6CLK   (NXXX_SYSCON0_BASE + 0x868)
#define SYSCON_DIVFLEXCOM7CLK   (NXXX_SYSCON0_BASE + 0x86C)
#define SYSCON_DIVSAI0CLK       (NXXX_SYSCON0_BASE + 0x888)
#define SYSCON_DIVSAI1CLK       (NXXX_SYSCON0_BASE + 0x88C)
#define SYSCON_DIVI3C1FCLK      (NXXX_SYSCON0_BASE + 0xB40)

#define SYSCON_CLKDIV_DIV_SHIFT (0) /* Bits 0-7: Clock divider value */
#define SYSCON_CLKDIV_DIV_MASK  (0x7 << SYSCON_CLKDIV_DIV_SHIFT)
#define SYSCON_CLKDIV_DIV(n)    (((n) << SYSCON_CLKDIV_DIV_SHIFT) & SYSCON_CLKDIV_DIV_MASK)

#define SYSCON_CLKDIV_RESET     (1 << 29) /* Bit 29: Resets the divider counter */
#define SYSCON_CLKDIV_HALT      (1 << 30) /* Bit 30: Halts the divider counter */
#define SYSCON_CLKDIV_STAB      (1 << 31) /* Bit 31: Divider status flag */

/* Peripheral clock selection */

#define SYSCON_SYSTICKCLK0      PERIPH_CLOCK(SYSCON_SYSTICKCLKSEL0, SYSCON_DIVSYSTICKCLK0)
#define SYSCON_TRACECLK         PERIPH_CLOCK(SYSCON_TRACECLKSEL, SYSCON_DIVTRACECLK)
#define SYSCON_CTIMERCLK0       PERIPH_CLOCK(SYSCON_CTIMERCLKSEL0, SYSCON_DIVCTIMER0CLK)
#define SYSCON_CTIMERCLK1       PERIPH_CLOCK(SYSCON_CTIMERCLKSEL1, SYSCON_DIVCTIMER1CLK)
#define SYSCON_CTIMERCLK2       PERIPH_CLOCK(SYSCON_CTIMERCLKSEL2, SYSCON_DIVCTIMER2CLK)
#define SYSCON_CTIMERCLK3       PERIPH_CLOCK(SYSCON_CTIMERCLKSEL3, SYSCON_DIVCTIMER3CLK)
#define SYSCON_CTIMERCLK4       PERIPH_CLOCK(SYSCON_CTIMERCLKSEL4, SYSCON_DIVCTIMER4CLK)
#define SYSCON_CLKOUTCLK        PERIPH_CLOCK(SYSCON_CLKOUTCLKSEL, SYSCON_DIVCLKOUT)
#define SYSCON_ADC0CLK          PERIPH_CLOCK(SYSCON_ADC0CLKSEL, SYSCON_DIVADC0CLK)
#define SYSCON_FCCLK0           PERIPH_CLOCK(SYSCON_FCCLKSEL0, SYSCON_DIVFLEXCOM0CLK)
#define SYSCON_FCCLK1           PERIPH_CLOCK(SYSCON_FCCLKSEL1, SYSCON_DIVFLEXCOM1CLK)
#define SYSCON_FCCLK2           PERIPH_CLOCK(SYSCON_FCCLKSEL2, SYSCON_DIVFLEXCOM2CLK)
#define SYSCON_FCCLK3           PERIPH_CLOCK(SYSCON_FCCLKSEL3, SYSCON_DIVFLEXCOM3CLK)
#define SYSCON_FCCLK4           PERIPH_CLOCK(SYSCON_FCCLKSEL4, SYSCON_DIVFLEXCOM4CLK)
#define SYSCON_FCCLK5           PERIPH_CLOCK(SYSCON_FCCLKSEL5, SYSCON_DIVFLEXCOM5CLK)
#define SYSCON_FCCLK6           PERIPH_CLOCK(SYSCON_FCCLKSEL6, SYSCON_DIVFLEXCOM6CLK)
#define SYSCON_FCCLK7           PERIPH_CLOCK(SYSCON_FCCLKSEL7, SYSCON_DIVFLEXCOM7CLK)
#define SYSCON_ADC1CLK          PERIPH_CLOCK(SYSCON_ADC1CLKSEL, SYSCON_DIVADC1CLK)
#define SYSCON_PLLCLKDIV        PERIPH_CLOCK(SYSCON_PLLCLKDIVSEL, SYSCON_DIVPLLCLK)
#define SYSCON_I3C0FCLK         PERIPH_CLOCK(SYSCON_I3C0FCLKSEL, SYSCON_DIVI3C0FCLK)
#define SYSCON_MICFILFCLK       PERIPH_CLOCK(SYSCON_MICFILFCLKSEL, SYSCON_DIVMICFILFCLK)
#define SYSCON_FLEXIOCLK        PERIPH_CLOCK(SYSCON_FLEXIOCLKSEL, SYSCON_DIVFLEXIOCLK)
#define SYSCON_FLEXCAN0CLK      PERIPH_CLOCK(SYSCON_FLEXCAN0CLKSEL, SYSCON_DIVFLEXCAN0CLK)
#define SYSCON_FLEXCAN1CLK      PERIPH_CLOCK(SYSCON_FLEXCAN1CLKSEL, SYSCON_DIVFLEXCAN1CLK)
#define SYSCON_EWM0CLK          PERIPH_CLOCK(SYSCON_EWM0CLKSEL, ) /* TODO */
#define SYSCON_WDT1CLK          PERIPH_CLOCK(SYSCON_WDT1CLKSEL, SYSCON_DIVWDT1CLK)
#define SYSCON_OSTIMERCLK       PERIPH_CLOCK(SYSCON_OSTIMERCLKSEL, ) /* TODO */
#define SYSCON_CMP0FCLK         PERIPH_CLOCK(SYSCON_CMP0FCLKSEL, SYSCON_DIVCMP0FCLK)
#define SYSCON_CMP0RRCLK        PERIPH_CLOCK(SYSCON_CMP0RRCLKSEL, SYSCON_DIVCMP0RRCLK)
#define SYSCON_CMP1FCLK         PERIPH_CLOCK(SYSCON_CMP1FCLKSEL, SYSCON_DIVCMP1FCLK)
#define SYSCON_CMP1RRCLK        PERIPH_CLOCK(SYSCON_CMP1RRCLKSEL, SYSCON_DIVCMP1RRCLK)
#define SYSCON_UTICKCLK         PERIPH_CLOCK(SYSCON_UTICKCLKSEL, SYSCON_DIVUTICKCLK)
#define SYSCON_SAI0CLK          PERIPH_CLOCK(SYSCON_SAI0CLKSEL, SYSCON_DIVSAI0CLK)
#define SYSCON_SAI1CLK          PERIPH_CLOCK(SYSCON_SAI1CLKSEL, SYSCON_DIVSAI1CLK)
#define SYSCON_I3C1FCLK         PERIPH_CLOCK(SYSCON_I3C1FCLKSEL, SYSCON_DIVI3C1FCLK)

/* Clock source names */

#define CLK_IN_TO_MAIN_CLK          1   /* clk_in to MAIN_CLK */
#define FRO12M_TO_MAIN_CLK          2   /* FRO_12M to MAIN_CLK */
#define FRO_HF_TO_MAIN_CLK          3   /* FRO_HF to MAIN_CLK */
#define XTAL32K2_TO_MAIN_CLK        4   /* xtal32k[2] to MAIN_CLK */
#define PLL0_TO_MAIN_CLK            5   /* PLL0 to MAIN_CLK */
#define PLL1_TO_MAIN_CLK            6   /* PLL1 to MAIN_CLK */
#define USB_PLL_TO_MAIN_CLK         7   /* USBPLL to MAIN_CLK */
#define NONE_TO_MAIN_CLK            15  /* NONE to MAIN_CLK */

#define SYSTICK_DIV0_TO_SYSTICK0    0   /* SYSTICK_DIV0 to SYSTICK0 */
#define CLK_1M_TO_SYSTICK0          1   /* Clk1MHz to SYSTICK0 */
#define LPOSC_TO_SYSTICK0           2   /* LPOscilla to r to SYSTICK0 */
#define NONE_TO_SYSTICK0            7   /* NONE to SYSTICK0 */

#define TRACE_DIV_TO_TRACE          0   /* TRACE_DIV to TRACE */
#define CLK_1M_TO_TRACE             1   /* Clk1MHz to TRACE */
#define LPOSC_TO_TRACE              2   /* LPOscilla to r to TRACE */
#define NONE_TO_TRACE               7   /* NONE to TRACE */

#define CLK_1M_TO_CTIMER0           0   /* CLK_1M to CTIMER0 */
#define PLL0_TO_CTIMER0             1   /* PLL0 to CTIMER0 */
#define PLL1_CLK0_TO_CTIMER0        2   /* PLL1_clk0 to CTIMER0 */
#define FRO_HF_TO_CTIMER0           3   /* FRO_HF to CTIMER0 */
#define FRO12M_TO_CTIMER0           4   /* FRO12MHz to CTIMER0 */
#define SAI0_MCLK_IN_TO_CTIMER0     5   /* SAI0MCLKIN to CTIMER0 */
#define LPOSC_TO_CTIMER0            6   /* LPOscilla to r to CTIMER0 */
#define SAI1_MCLK_IN_TO_CTIMER0     8   /* SAI1MCLKIN to CTIMER0 */
#define SAI0_TX_BCLK_TO_CTIMER0     9   /* SAI0TX_BCLK to CTIMER0 */
#define SAI0_RX_BCLK_TO_CTIMER0     10  /* SAI0RX_BCLK to CTIMER0 */
#define SAI1_TX_BCLK_TO_CTIMER0     11  /* SAI1TX_BCLK to CTIMER0 */
#define SAI1_RX_BCLK_TO_CTIMER0     12  /* SAI1RX_BCLK to CTIMER0 */
#define NONE_TO_CTIMER0             15  /* NONE to CTIMER0 */

#define CLK_1M_TO_CTIMER1           0   /* CLK_1M to CTIMER1 */
#define PLL0_TO_CTIMER1             1   /* PLL0 to CTIMER1 */
#define PLL1_CLK0_TO_CTIMER1        2   /* PLL1_clk0 to CTIMER1 */
#define FRO_HF_TO_CTIMER1           3   /* FRO_HF to CTIMER1 */
#define FRO12M_TO_CTIMER1           4   /* FRO12MHz to CTIMER1 */
#define SAI0_MCLK_IN_TO_CTIMER1     5   /* SAI0MCLKIN to CTIMER1 */
#define LPOSC_TO_CTIMER1            6   /* LPOscilla to r to CTIMER1 */
#define SAI1_MCLK_IN_TO_CTIMER1     8   /* SAI1MCLKIN to CTIMER1 */
#define SAI0_TX_BCLK_TO_CTIMER1     9   /* SAI0TX_BCLK to CTIMER1 */
#define SAI0_RX_BCLK_TO_CTIMER1     10  /* SAI0RX_BCLK to CTIMER1 */
#define SAI1_TX_BCLK_TO_CTIMER1     11  /* SAI1TX_BCLK to CTIMER1 */
#define SAI1_RX_BCLK_TO_CTIMER1     12  /* SAI1RX_BCLK to CTIMER1 */
#define NONE_TO_CTIMER1             15  /* NONE to CTIMER1 */

#define CLK_1M_TO_CTIMER2           0   /* CLK_1M to CTIMER2 */
#define PLL0_TO_CTIMER2             1   /* PLL0 to CTIMER2 */
#define PLL1_CLK0_TO_CTIMER2        2   /* PLL1_clk0 to CTIMER2 */
#define FRO_HF_TO_CTIMER2           3   /* FRO_HF to CTIMER2 */
#define FRO12M_TO_CTIMER2           4   /* FRO12MHz to CTIMER2 */
#define SAI0_MCLK_IN_TO_CTIMER2     5   /* SAI0MCLKIN to CTIMER2 */
#define LPOSC_TO_CTIMER2            6   /* LPOscilla to r to CTIMER2 */
#define SAI1_MCLK_IN_TO_CTIMER2     8   /* SAI1MCLKIN to CTIMER2 */
#define SAI0_TX_BCLK_TO_CTIMER2     9   /* SAI0TX_BCLK to CTIMER2 */
#define SAI0_RX_BCLK_TO_CTIMER2     10  /* SAI0RX_BCLK to CTIMER2 */
#define SAI1_TX_BCLK_TO_CTIMER2     11  /* SAI1TX_BCLK to CTIMER2 */
#define SAI1_RX_BCLK_TO_CTIMER2     12  /* SAI1RX_BCLK to CTIMER2 */
#define NONE_TO_CTIMER2             15  /* NONE to CTIMER2 */

#define CLK_1M_TO_CTIMER3           0   /* CLK_1M to CTIMER3 */
#define PLL0_TO_CTIMER3             1   /* PLL0 to CTIMER3 */
#define PLL1_CLK0_TO_CTIMER3        2   /* PLL1_clk0 to CTIMER3 */
#define FRO_HF_TO_CTIMER3           3   /* FRO_HF to CTIMER3 */
#define FRO12M_TO_CTIMER3           4   /* FRO12MHz to CTIMER3 */
#define SAI0_MCLK_IN_TO_CTIMER3     5   /* SAI0MCLKIN to CTIMER3 */
#define LPOSC_TO_CTIMER3            6   /* LPOscilla to r to CTIMER3 */
#define SAI1_MCLK_IN_TO_CTIMER3     8   /* SAI1MCLKIN to CTIMER3 */
#define SAI0_TX_BCLK_TO_CTIMER3     9   /* SAI0TX_BCLK to CTIMER3 */
#define SAI0_RX_BCLK_TO_CTIMER3     10  /* SAI0RX_BCLK to CTIMER3 */
#define SAI1_TX_BCLK_TO_CTIMER3     11  /* SAI1TX_BCLK to CTIMER3 */
#define SAI1_RX_BCLK_TO_CTIMER3     12  /* SAI1RX_BCLK to CTIMER3 */
#define NONE_TO_CTIMER3             15  /* NONE to CTIMER3 */

#define CLK_1M_TO_CTIMER4           0   /* CLK_1M to CTIMER4 */
#define PLL0_TO_CTIMER4             1   /* PLL0 to CTIMER4 */
#define PLL1_CLK0_TO_CTIMER4        2   /* PLL1_clk0 to CTIMER4 */
#define FRO_HF_TO_CTIMER4           3   /* FRO_HF to CTIMER4 */
#define FRO12M_TO_CTIMER4           4   /* FRO12MHz to CTIMER4 */
#define SAI0_MCLK_IN_TO_CTIMER4     5   /* SAI0MCLKIN to CTIMER4 */
#define LPOSC_TO_CTIMER4            6   /* LPOscilla to r to CTIMER4 */
#define SAI1_MCLK_IN_TO_CTIMER4     8   /* SAI1MCLKIN to CTIMER4 */
#define SAI0_TX_BCLK_TO_CTIMER4     9   /* SAI0TX_BCLK to CTIMER4 */
#define SAI0_RX_BCLK_TO_CTIMER4     10  /* SAI0RX_BCLK to CTIMER4 */
#define SAI1_TX_BCLK_TO_CTIMER4     11  /* SAI1TX_BCLK to CTIMER4 */
#define SAI1_RX_BCLK_TO_CTIMER4     12  /* SAI1RX_BCLK to CTIMER4 */
#define NONE_TO_CTIMER4             15  /* NONE to CTIMER4 */

#define MAIN_CLK_TO_CLKOUT          0   /* MAIN_CLK to CLKOUT */
#define PLL0_TO_CLKOUT              1   /* PLL0 to CLKOUT */
#define CLK_IN_TO_CLKOUT            2   /* Clk_in to CLKOUT */
#define FRO_HF_TO_CLKOUT            3   /* FRO_HF to CLKOUT */
#define FRO12M_TO_CLKOUT            4   /* FRO12MHz to CLKOUT */
#define PLL1_CLK0_TO_CLKOUT         5   /* PLL1_clk0 to CLKOUT */
#define LPOSC_TO_CLKOUT             6   /* LPOscilla to r to CLKOUT */
#define USB_PLL_TO_CLKOUT           7   /* USB_PLL to CLKOUT */
#define NONE_TO_CLKOUT              15  /* NONE to CLKOUT */

#define PLL0_TO_ADC0                1   /* PLL0 to ADC0 */
#define FRO_HF_TO_ADC0              2   /* FRO_HF to ADC0 */
#define FRO12M_TO_ADC0              3   /* FRO12MHz to ADC0 */
#define CLK_IN_TO_ADC0              4   /* Clk_in to ADC0 */
#define PLL1_CLK0_TO_ADC0           5   /* PLL1_clk0 to ADC0 */
#define USB_PLL_TO_ADC0             6   /* USBPLL to ADC0 */
#define NONE_TO_ADC0                7   /* NONE to ADC0 */

#define PLL_DIV_TO_FLEXCOMM0        1   /* PLL_DIV to FLEXCOMM0 */
#define FRO12M_TO_FLEXCOMM0         2   /* FRO12M to FLEXCOMM0 */
#define FRO_HF_DIV_TO_FLEXCOMM0     3   /* FRO_HF_DIV to FLEXCOMM0 */
#define CLK_1M_TO_FLEXCOMM0         4   /* CLK_1MHz to FLEXCOMM0 */
#define USB_PLL_TO_FLEXCOMM0        5   /* USB_PLL to FLEXCOMM0 */
#define LPOSC_TO_FLEXCOMM0          6   /* LPOscilla to r to FLEXCOMM0 */
#define NONE_TO_FLEXCOMM0           7   /* NONE to FLEXCOMM0 */

#define PLL_DIV_TO_FLEXCOMM1        1   /* PLL_DIV to FLEXCOMM1 */
#define FRO12M_TO_FLEXCOMM1         2   /* FRO12M to FLEXCOMM1 */
#define FRO_HF_DIV_TO_FLEXCOMM1     3   /* FRO_HF_DIV to FLEXCOMM1 */
#define CLK_1M_TO_FLEXCOMM1         4   /* CLK_1MHz to FLEXCOMM1 */
#define USB_PLL_TO_FLEXCOMM1        5   /* USB_PLL to FLEXCOMM1 */
#define LPOSC_TO_FLEXCOMM1          6   /* LPOscilla to r to FLEXCOMM1 */
#define NONE_TO_FLEXCOMM1           7   /* NONE to FLEXCOMM1 */

#define PLL_DIV_TO_FLEXCOMM2        1   /* PLL_DIV to FLEXCOMM2 */
#define FRO12M_TO_FLEXCOMM2         2   /* FRO12M to FLEXCOMM2 */
#define FRO_HF_DIV_TO_FLEXCOMM2     3   /* FRO_HF_DIV to FLEXCOMM2 */
#define CLK_1M_TO_FLEXCOMM2         4   /* CLK_1MHz to FLEXCOMM2 */
#define USB_PLL_TO_FLEXCOMM2        5   /* USB_PLL to FLEXCOMM2 */
#define LPOSC_TO_FLEXCOMM2          6   /* LPOscilla to r to FLEXCOMM2 */
#define NONE_TO_FLEXCOMM2           7   /* NONE to FLEXCOMM2 */

#define PLL_DIV_TO_FLEXCOMM3        1   /* PLL_DIV to FLEXCOMM3 */
#define FRO12M_TO_FLEXCOMM3         2   /* FRO12M to FLEXCOMM3 */
#define FRO_HF_DIV_TO_FLEXCOMM3     3   /* FRO_HF_DIV to FLEXCOMM3 */
#define CLK_1M_TO_FLEXCOMM3         4   /* CLK_1MHz to FLEXCOMM3 */
#define USB_PLL_TO_FLEXCOMM3        5   /* USB_PLL to FLEXCOMM3 */
#define LPOSC_TO_FLEXCOMM3          6   /* LPOscilla to r to FLEXCOMM3 */
#define NONE_TO_FLEXCOMM3           7   /* NONE to FLEXCOMM3 */

#define PLL_DIV_TO_FLEXCOMM4        1   /* PLL_DIV to FLEXCOMM4 */
#define FRO12M_TO_FLEXCOMM4         2   /* FRO12M to FLEXCOMM4 */
#define FRO_HF_DIV_TO_FLEXCOMM4     3   /* FRO_HF_DIV to FLEXCOMM4 */
#define CLK_1M_TO_FLEXCOMM4         4   /* CLK_1MHz to FLEXCOMM4 */
#define USB_PLL_TO_FLEXCOMM4        5   /* USB_PLL to FLEXCOMM4 */
#define LPOSC_TO_FLEXCOMM4          6   /* LPOscilla to r to FLEXCOMM4 */
#define NONE_TO_FLEXCOMM4           7   /* NONE to FLEXCOMM4 */

#define PLL_DIV_TO_FLEXCOMM5        1   /* PLL_DIV to FLEXCOMM5 */
#define FRO12M_TO_FLEXCOMM5         2   /* FRO12M to FLEXCOMM5 */
#define FRO_HF_DIV_TO_FLEXCOMM5     3   /* FRO_HF_DIV to FLEXCOMM5 */
#define CLK_1M_TO_FLEXCOMM5         4   /* CLK_1MHz to FLEXCOMM5 */
#define USB_PLL_TO_FLEXCOMM5        5   /* USB_PLL to FLEXCOMM5 */
#define LPOSC_TO_FLEXCOMM5          6   /* LPOscilla to r to FLEXCOMM5 */
#define NONE_TO_FLEXCOMM5           7   /* NONE to FLEXCOMM5 */

#define PLL_DIV_TO_FLEXCOMM6        1   /* PLL_DIV to FLEXCOMM6 */
#define FRO12M_TO_FLEXCOMM6         2   /* FRO12M to FLEXCOMM6 */
#define FRO_HF_DIV_TO_FLEXCOMM6     3   /* FRO_HF_DIV to FLEXCOMM6 */
#define CLK_1M_TO_FLEXCOMM6         4   /* CLK_1MHz to FLEXCOMM6 */
#define USB_PLL_TO_FLEXCOMM6        5   /* USB_PLL to FLEXCOMM6 */
#define LPOSC_TO_FLEXCOMM6          6   /* LPOscilla to r to FLEXCOMM6 */
#define NONE_TO_FLEXCOMM6           7   /* NONE to FLEXCOMM6 */

#define PLL_DIV_TO_FLEXCOMM7        1   /* PLL_DIV to FLEXCOMM7 */
#define FRO12M_TO_FLEXCOMM7         2   /* FRO12M to FLEXCOMM7 */
#define FRO_HF_DIV_TO_FLEXCOMM7     3   /* FRO_HF_DIV to FLEXCOMM7 */
#define CLK_1M_TO_FLEXCOMM7         4   /* CLK_1MHz to FLEXCOMM7 */
#define USB_PLL_TO_FLEXCOMM7        5   /* USB_PLL to FLEXCOMM7 */
#define LPOSC_TO_FLEXCOMM7          6   /* LPOscilla to r to FLEXCOMM7 */
#define NONE_TO_FLEXCOMM7           7   /* NONE to FLEXCOMM7 */

#define PLL0_TO_ADC1                1   /* PLL0 to ADC1 */
#define FRO_HF_TO_ADC1              2   /* FRO_HF to ADC1 */
#define FRO12M_TO_ADC1              3   /* FRO12M to ADC1 */
#define CLK_IN_TO_ADC1              4   /* clk_in to ADC1 */
#define PLL1_CLK0_TO_ADC1           5   /* PLL1_clk0 to ADC1 */
#define USB_PLL_TO_ADC1             6   /* USBPLL to ADC1 */
#define NONE_TO_ADC1                7   /* NONE to ADC1 */

#define PLL0_TO_PLLCLKDIV           0   /* PLL0 to PLLCLKDIV */
#define PLL1_CLK0_TO_PLLCLKDIV      1   /* pll1_clk0 to PLLCLKDIV */
#define NONE_TO_PLLCLKDIV           1   /* NONE to PLLCLKDIV */

#define PLL0_TO_I3C0FCLK            1   /* PLL0 to I3C0FCLK */
#define FRO_HF_TO_I3C0FCLK          3   /* FRO_HF to I3C0FCLK */
#define CLK_1M_TO_I3C0FCLK          4   /* CLK_1M to I3C0FCLK */
#define PLL1_CLK0_TO_I3C0FCLK       5   /* PLL1_clk0 to I3C0FCLK */
#define USB_PLL_TO_I3C0FCLK         6   /* USBPLL to I3C0FCLK */
#define NONE_TO_I3C0FCLK            7   /* NONE to I3C0FCLK */

#define PLL0_TO_I3C0FCLKSTC         1   /* PLL0 to I3C0FCLKSTC */
#define FRO_HF_TO_I3C0FCLKSTC       3   /* FRO_HF to I3C0FCLKSTC */
#define CLK_1M_TO_I3C0FCLKSTC       4   /* CLK_1M to I3C0FCLKSTC */
#define PLL1_CLK0_TO_I3C0FCLKSTC    5   /* PLL1_clk0 to I3C0FCLKSTC */
#define USB_PLL_TO_I3C0FCLKSTC      6   /* USBPLL to I3C0FCLKSTC */
#define NONE_TO_I3C0FCLKSTC         7   /* NONE to I3C0FCLKSTC */

#define PLL0_TO_I3C0FCLKSLOW        1   /* PLL0 to I3C0FCLKS */
#define FRO_HF_TO_I3C0FCLKSLOW      3   /* FRO_HF to I3C0FCLKS */
#define CLK_1M_TO_I3C0FCLKSLOW      4   /* CLK_1M to I3C0FCLKS */
#define PLL1_CLK0_TO_I3C0FCLKSLOW   5   /* PLL1_clk0 to I3C0FCLKS */
#define USB_PLL_TO_I3C0FCLKSLOW     6   /* USBPLL to I3C0FCLKS */
#define NONE_TO_I3C0FCLKSLOW        7   /* NONE to I3C0FCLKS */

#define FRO12M_TO_MICFILF           0   /* FRO_12M to MICFILF */
#define PLL0_TO_MICFILF             1   /* PLL0 to MICFILF */
#define CLK_IN_TO_MICFILF           2   /* Clk_in to MICFILF */
#define FRO_HF_TO_MICFILF           3   /* FRO_HF to MICFILF */
#define PLL1_CLK0_TO_MICFILF        4   /* PLL1_clk0 to MICFILF */
#define SAI0_MCLK_IN_TO_MICFILF     5   /* SAI0_MCLK to MICFILF */
#define USB_PLL_TO_MICFILF          6   /* USBPLL to MICFILF */
#define SAI1_MCLK_IN_TO_MICFILF     8   /* SAI1_MCLK to MICFILF */
#define NONE_TO_MICFILF             15  /* NONE to MICFILF */

#define PLL0_TO_FLEXIO              1   /* PLL0 to FLEXIO */
#define CLK_IN_TO_FLEXIO            2   /* Clk_in to FLEXIO */
#define FRO_HF_TO_FLEXIO            3   /* FRO_HF to FLEXIO */
#define FRO12M_TO_FLEXIO            4   /* FRO_12M to FLEXIO */
#define PLL1_CLK0_TO_FLEXIO         5   /* pll1_clk0 to FLEXIO */
#define USB_PLL_TO_FLEXIO           6   /* USBPLL to FLEXIO */
#define NONE_TO_FLEXIO              7   /* NONE to FLEXIO */

#define PLL0_TO_FLEXCAN0            1   /* PLL0 to FLEXCAN0 */
#define CLK_IN_TO_FLEXCAN0          2   /* Clk_in to FLEXCAN0 */
#define FRO_HF_TO_FLEXCAN0          3   /* FRO_HF to FLEXCAN0 */
#define PLL1_CLK0_TO_FLEXCAN0       5   /* pll1_clk0 to FLEXCAN0 */
#define USB_PLL_TO_FLEXCAN0         6   /* USBPLL to FLEXCAN0 */
#define NONE_TO_FLEXCAN0            7   /* NONE to FLEXCAN0 */

#define PLL0_TO_FLEXCAN1            1   /* PLL0 to FLEXCAN1 */
#define CLK_IN_TO_FLEXCAN1          2   /* Clk_in to FLEXCAN1 */
#define FRO_HF_TO_FLEXCAN1          3   /* FRO_HF to FLEXCAN1 */
#define PLL1_CLK0_TO_FLEXCAN1       5   /* pll1_clk0 to FLEXCAN1 */
#define USB_PLL_TO_FLEXCAN1         6   /* USBPLL to FLEXCAN1 */
#define NONE_TO_FLEXCAN1            7   /* NONE to FLEXCAN1 */

#define CLK_16K2_TO_EWM0            0   /* clk_16k[2] to EWM0 */
#define XTAL32K2_TO_EWM0            1   /* xtal32k[2] to EWM0 */

#define CLK_16K2_TO_WDT1            0   /* FRO16Kclock2 to WDT1 */
#define FRO_HF_DIV_TO_WDT1          1   /* FRO_HF_DIV to WDT1 */
#define CLK_1M_TO_WDT1              2   /* clk_1m to WDT1 */
#define CLK_1M_2_TO_WDT1            3   /* clk_1m to WDT1 */

#define CLK_16K2_TO_OSTIMER         0   /* clk_16k[2] to OSTIMER */
#define XTAL32K2_TO_OSTIMER         1   /* xtal32k[2] to OSTIMER */
#define CLK_1M_TO_OSTIMER           2   /* clk_1m to OSTIMER */
#define NONE_TO_OSTIMER             3   /* NONE to OSTIMER */

#define PLL0_TO_CMP0F               1   /* PLL0 to CMP0F */
#define FRO_HF_TO_CMP0F             2   /* FRO_HF to CMP0F */
#define FRO12M_TO_CMP0F             3   /* FRO_12M to CMP0F */
#define CLK_IN_TO_CMP0F             4   /* Clk_in to CMP0F */
#define PLL1_CLK0_TO_CMP0F          5   /* PLL1_clk0 to CMP0F */
#define USB_PLL_TO_CMP0F            6   /* USBPLL to CMP0F */
#define NONE_TO_CMP0F               7   /* NONE to CMP0F */

#define PLL0_TO_CMP0RR              1   /* PLL0 to CMP0RR */
#define FRO_HF_TO_CMP0RR            2   /* FRO_HF to CMP0RR */
#define FRO12M_TO_CMP0RR            3   /* FRO_12M to CMP0RR */
#define CLK_IN_TO_CMP0RR            4   /* Clk_in to CMP0RR */
#define PLL1_CLK0_TO_CMP0RR         5   /* PLL1_clk0 to CMP0RR */
#define USB_PLL_TO_CMP0RR           6   /* USBPLL to CMP0RR */
#define NONE_TO_CMP0RR              7   /* NONE to CMP0RR */

#define PLL0_TO_CMP1F               1   /* PLL0 to CMP1F */
#define FRO_HF_TO_CMP1F             2   /* FRO_HF to CMP1F */
#define FRO12M_TO_CMP1F             3   /* FRO_12M to CMP1F */
#define CLK_IN_TO_CMP1F             4   /* Clk_in to CMP1F */
#define PLL1_CLK0_TO_CMP1F          5   /* PLL1_clk0 to CMP1F */
#define USB_PLL_TO_CMP1F            6   /* USBPLL to CMP1F */
#define NONE_TO_CMP1F               7   /* NONE to CMP1F */

#define PLL0_TO_CMP1RR              1   /* PLL0 to CMP1RR */
#define FRO_HF_TO_CMP1RR            2   /* FRO_HF to CMP1RR */
#define FRO12M_TO_CMP1RR            3   /* FRO_12M to CMP1RR */
#define CLK_IN_TO_CMP1RR            4   /* Clk_in to CMP1RR */
#define PLL1_CLK0_TO_CMP1RR         5   /* PLL1_clk0 to CMP1RR */
#define USB_PLL_TO_CMP1RR           6   /* USBPLL to CMP1RR */
#define NONE_TO_CMP1RR              7   /* NONE to CMP1RR */

#define CLK_IN_TO_UTICK             0   /* Clk_in to UTICK */
#define XTAL32K2_TO_UTICK           1   /* xtal32k[2] to UTICK */
#define CLK_1M_TO_UTICK             2   /* clk_1m to UTICK */
#define NONE_TO_UTICK               3   /* NONE to UTICK */

#define PLL0_TO_SAI0                1   /* PLL0 to SAI0 */
#define CLK_IN_TO_SAI0              2   /* Clk_in to SAI0 */
#define FRO_HF_TO_SAI0              3   /* FRO_HF to SAI0 */
#define PLL1_CLK0_TO_SAI0           4   /* PLL1_clk0 to SAI0 */
#define USB_PLL_TO_SAI0             6   /* USBPLL to SAI0 */
#define NONE_TO_SAI0                7   /* NONE to SAI0 */

#define PLL0_TO_SAI1                1   /* PLL0 to SAI1 */
#define CLK_IN_TO_SAI1              2   /* Clk_in to SAI1 */
#define FRO_HF_TO_SAI1              3   /* FRO_HF to SAI1 */
#define PLL1_CLK0_TO_SAI1           4   /* PLL1_clk0 to SAI1 */
#define USB_PLL_TO_SAI1             6   /* USBPLL to SAI1 */
#define NONE_TO_SAI1                7   /* NONE to SAI1 */

#define PLL0_TO_I3C1FCLK            1   /* PLL0 to I3C1FCLK */
#define FRO_HF_TO_I3C1FCLK          3   /* FRO_HF to I3C1FCLK */
#define CLK_1M_TO_I3C1FCLK          4   /* CLK_1M to I3C1FCLK */
#define PLL1_CLK0_TO_I3C1FCLK       5   /* PLL1_clk0 to I3C1FCLK */
#define USB_PLL_TO_I3C1FCLK         6   /* USBPLL to I3C1FCLK */
#define NONE_TO_I3C1FCLK            7   /* NONE to I3C1FCLK */

#define PLL0_TO_I3C1FCLKSTC         1   /* PLL0 to I3C1FCLKSTC */
#define FRO_HF_TO_I3C1FCLKSTC       3   /* FRO_HF to I3C1FCLKSTC */
#define CLK_1M_TO_I3C1FCLKSTC       4   /* CLK_1M to I3C1FCLKSTC */
#define PLL1_CLK0_TO_I3C1FCLKSTC    5   /* PLL1_clk0 to I3C1FCLKSTC */
#define USB_PLL_TO_I3C1FCLKSTC      6   /* USBPLL to I3C1FCLKSTC */
#define NONE_TO_I3C1FCLKSTC         7   /* NONE to I3C1FCLKSTC */

#define PLL0_TO_I3C1FCLKSLOW        1   /* PLL0 to I3C1FCLKS */
#define FRO_HF_TO_I3C1FCLKSLOW      3   /* FRO_HF to I3C1FCLKS */
#define CLK_1M_TO_I3C1FCLKSLOW      4   /* CLK_1M to I3C1FCLKS */
#define PLL1_CLK0_TO_I3C1FCLKSLOW   5   /* PLL1_clk0 to I3C1FCLKS */
#define USB_PLL_TO_I3C1FCLKSLOW     6   /* USBPLL to I3C1FCLKS */
#define NONE_TO_I3C1FCLKSLOW        7   /* NONE to I3C1FCLKS */

/* Clock gate control */

#define AHB_CLK_CTRL0           (NXXX_SYSCON0_BASE + 0x200)
#define AHB_CLK_CTRL1           (NXXX_SYSCON0_BASE + 0x204)
#define AHB_CLK_CTRL2           (NXXX_SYSCON0_BASE + 0x208)
#define AHB_CLK_CTRL3           (NXXX_SYSCON0_BASE + 0x20C)

#define AHB_CLK_CTRL_SET0       (NXXX_SYSCON0_BASE + 0x220)
#define AHB_CLK_CTRL_SET1       (NXXX_SYSCON0_BASE + 0x224)
#define AHB_CLK_CTRL_SET2       (NXXX_SYSCON0_BASE + 0x228)
#define AHB_CLK_CTRL_SET3       (NXXX_SYSCON0_BASE + 0x22C)

#define AHB_CLK_CTRL_CLR0       (NXXX_SYSCON0_BASE + 0x240)
#define AHB_CLK_CTRL_CLR1       (NXXX_SYSCON0_BASE + 0x244)
#define AHB_CLK_CTRL_CLR2       (NXXX_SYSCON0_BASE + 0x248)
#define AHB_CLK_CTRL_CLR3       (NXXX_SYSCON0_BASE + 0x24C)

/* Clock gates in SYSCON */

#define CLOCK_GATE_ROM         CLOCK_GATE(AHB_CLK_CTRL0, 1)     /* Rom */
#define CLOCK_GATE_SRAM1       CLOCK_GATE(AHB_CLK_CTRL0, 2)     /* Sram1 */
#define CLOCK_GATE_SRAM2       CLOCK_GATE(AHB_CLK_CTRL0, 3)     /* Sram2 */
#define CLOCK_GATE_SRAM3       CLOCK_GATE(AHB_CLK_CTRL0, 4)     /* Sram3 */
#define CLOCK_GATE_SRAM4       CLOCK_GATE(AHB_CLK_CTRL0, 5)     /* Sram4 */
#define CLOCK_GATE_SRAM5       CLOCK_GATE(AHB_CLK_CTRL0, 6)     /* Sram5 */
#define CLOCK_GATE_SRAM6       CLOCK_GATE(AHB_CLK_CTRL0, 7)     /* Sram6 */
#define CLOCK_GATE_SRAM7       CLOCK_GATE(AHB_CLK_CTRL0, 8)     /* Sram7 */
#define CLOCK_GATE_FMU         CLOCK_GATE(AHB_CLK_CTRL0, 9)     /* Fmu */
#define CLOCK_GATE_FMC         CLOCK_GATE(AHB_CLK_CTRL0, 10)    /* Fmc */
#define CLOCK_GATE_FLEXSPI     CLOCK_GATE(AHB_CLK_CTRL0, 11)    /* Flexspi */
#define CLOCK_GATE_INPUTMUX0   CLOCK_GATE(AHB_CLK_CTRL0, 12)    /* InputMux0 */
#define CLOCK_GATE_INPUTMUX    CLOCK_GATE(AHB_CLK_CTRL0, 12)    /* InputMux0 */
#define CLOCK_GATE_PORT0       CLOCK_GATE(AHB_CLK_CTRL0, 13)    /* Port0 */
#define CLOCK_GATE_PORT1       CLOCK_GATE(AHB_CLK_CTRL0, 14)    /* Port1 */
#define CLOCK_GATE_PORT2       CLOCK_GATE(AHB_CLK_CTRL0, 15)    /* Port2 */
#define CLOCK_GATE_PORT3       CLOCK_GATE(AHB_CLK_CTRL0, 16)    /* Port3 */
#define CLOCK_GATE_PORT4       CLOCK_GATE(AHB_CLK_CTRL0, 17)    /* Port4 */
#define CLOCK_GATE_GPIO0       CLOCK_GATE(AHB_CLK_CTRL0, 19)    /* Gpio0 */
#define CLOCK_GATE_GPIO1       CLOCK_GATE(AHB_CLK_CTRL0, 20)    /* Gpio1 */
#define CLOCK_GATE_GPIO2       CLOCK_GATE(AHB_CLK_CTRL0, 21)    /* Gpio2 */
#define CLOCK_GATE_GPIO3       CLOCK_GATE(AHB_CLK_CTRL0, 22)    /* Gpio3 */
#define CLOCK_GATE_GPIO4       CLOCK_GATE(AHB_CLK_CTRL0, 23)    /* Gpio4 */
#define CLOCK_GATE_PINT        CLOCK_GATE(AHB_CLK_CTRL0, 25)    /* Pint */
#define CLOCK_GATE_DMA0        CLOCK_GATE(AHB_CLK_CTRL0, 26)    /* Dma0 */
#define CLOCK_GATE_CRC0        CLOCK_GATE(AHB_CLK_CTRL0, 27)    /* Crc */
#define CLOCK_GATE_WWDT0       CLOCK_GATE(AHB_CLK_CTRL0, 28)    /* Wwdt0 */
#define CLOCK_GATE_WWDT1       CLOCK_GATE(AHB_CLK_CTRL0, 29)    /* Wwdt1 */
#define CLOCK_GATE_MAILBOX     CLOCK_GATE(AHB_CLK_CTRL0, 31)    /* Mailbox */
#define CLOCK_GATE_MRT         CLOCK_GATE(AHB_CLK_CTRL1, 0)     /* Mrt */
#define CLOCK_GATE_OSTIMER     CLOCK_GATE(AHB_CLK_CTRL1, 1)     /* OsTimer */
#define CLOCK_GATE_SCT         CLOCK_GATE(AHB_CLK_CTRL1, 2)     /* Sct */
#define CLOCK_GATE_ADC0        CLOCK_GATE(AHB_CLK_CTRL1, 3)     /* Adc0 */
#define CLOCK_GATE_ADC1        CLOCK_GATE(AHB_CLK_CTRL1, 4)     /* Adc1 */
#define CLOCK_GATE_DAC0        CLOCK_GATE(AHB_CLK_CTRL1, 5)     /* Dac0 */
#define CLOCK_GATE_RTC0        CLOCK_GATE(AHB_CLK_CTRL1, 6)     /* Rtc */
#define CLOCK_GATE_EVSIM0      CLOCK_GATE(AHB_CLK_CTRL1, 8)     /* Evsim0 */
#define CLOCK_GATE_EVSIM1      CLOCK_GATE(AHB_CLK_CTRL1, 9)     /* Evsim1 */
#define CLOCK_GATE_UTICK       CLOCK_GATE(AHB_CLK_CTRL1, 10)    /* Utick */
#define CLOCK_GATE_LPFLEXCOMM0 CLOCK_GATE(AHB_CLK_CTRL1, 11)    /* LPFlexComm0 */
#define CLOCK_GATE_LPFLEXCOMM1 CLOCK_GATE(AHB_CLK_CTRL1, 12)    /* LPFlexComm1 */
#define CLOCK_GATE_LPFLEXCOMM2 CLOCK_GATE(AHB_CLK_CTRL1, 13)    /* LPFlexComm2 */
#define CLOCK_GATE_LPFLEXCOMM3 CLOCK_GATE(AHB_CLK_CTRL1, 14)    /* LPFlexComm3 */
#define CLOCK_GATE_LPFLEXCOMM4 CLOCK_GATE(AHB_CLK_CTRL1, 15)    /* LPFlexComm4 */
#define CLOCK_GATE_LPFLEXCOMM5 CLOCK_GATE(AHB_CLK_CTRL1, 16)    /* LPFlexComm5 */
#define CLOCK_GATE_LPFLEXCOMM6 CLOCK_GATE(AHB_CLK_CTRL1, 17)    /* LPFlexComm6 */
#define CLOCK_GATE_LPFLEXCOMM7 CLOCK_GATE(AHB_CLK_CTRL1, 18)    /* LPFlexComm7 */
#define CLOCK_GATE_LPFLEXCOMM8 CLOCK_GATE(AHB_CLK_CTRL1, 19)    /* LPFlexComm8 */
#define CLOCK_GATE_LPFLEXCOMM9 CLOCK_GATE(AHB_CLK_CTRL1, 20)    /* LPFlexComm9 */
#define CLOCK_GATE_LPUART0     CLOCK_GATE(AHB_CLK_CTRL1, 11)    /* LPUart0 */
#define CLOCK_GATE_LPUART1     CLOCK_GATE(AHB_CLK_CTRL1, 12)    /* LPUart1 */
#define CLOCK_GATE_LPUART2     CLOCK_GATE(AHB_CLK_CTRL1, 13)    /* LPUart2 */
#define CLOCK_GATE_LPUART3     CLOCK_GATE(AHB_CLK_CTRL1, 14)    /* LPUart3 */
#define CLOCK_GATE_LPUART4     CLOCK_GATE(AHB_CLK_CTRL1, 15)    /* LPUart4 */
#define CLOCK_GATE_LPUART5     CLOCK_GATE(AHB_CLK_CTRL1, 16)    /* LPUart5 */
#define CLOCK_GATE_LPUART6     CLOCK_GATE(AHB_CLK_CTRL1, 17)    /* LPUart6 */
#define CLOCK_GATE_LPUART7     CLOCK_GATE(AHB_CLK_CTRL1, 18)    /* LPUart7 */
#define CLOCK_GATE_LPUART8     CLOCK_GATE(AHB_CLK_CTRL1, 19)    /* LPUart8 */
#define CLOCK_GATE_LPUART9     CLOCK_GATE(AHB_CLK_CTRL1, 20)    /* LPUart9 */
#define CLOCK_GATE_LPSPI0      CLOCK_GATE(AHB_CLK_CTRL1, 11)    /* LPSpi0 */
#define CLOCK_GATE_LPSPI1      CLOCK_GATE(AHB_CLK_CTRL1, 12)    /* LPSpi1 */
#define CLOCK_GATE_LPSPI2      CLOCK_GATE(AHB_CLK_CTRL1, 13)    /* LPSpi2 */
#define CLOCK_GATE_LPSPI3      CLOCK_GATE(AHB_CLK_CTRL1, 14)    /* LPSpi3 */
#define CLOCK_GATE_LPSPI4      CLOCK_GATE(AHB_CLK_CTRL1, 15)    /* LPSpi4 */
#define CLOCK_GATE_LPSPI5      CLOCK_GATE(AHB_CLK_CTRL1, 16)    /* LPSpi5 */
#define CLOCK_GATE_LPSPI6      CLOCK_GATE(AHB_CLK_CTRL1, 17)    /* LPSpi6 */
#define CLOCK_GATE_LPSPI7      CLOCK_GATE(AHB_CLK_CTRL1, 18)    /* LPSpi7 */
#define CLOCK_GATE_LPSPI8      CLOCK_GATE(AHB_CLK_CTRL1, 19)    /* LPSpi8 */
#define CLOCK_GATE_LPSPI9      CLOCK_GATE(AHB_CLK_CTRL1, 20)    /* LSpi9 */
#define CLOCK_GATE_LPI2C0      CLOCK_GATE(AHB_CLK_CTRL1, 11)    /* LPI2c0 */
#define CLOCK_GATE_LPI2C1      CLOCK_GATE(AHB_CLK_CTRL1, 12)    /* LPI2c1 */
#define CLOCK_GATE_LPI2C2      CLOCK_GATE(AHB_CLK_CTRL1, 13)    /* LPI2c2 */
#define CLOCK_GATE_LPI2C3      CLOCK_GATE(AHB_CLK_CTRL1, 14)    /* LPI2c3 */
#define CLOCK_GATE_LPI2C4      CLOCK_GATE(AHB_CLK_CTRL1, 15)    /* LPI2c4 */
#define CLOCK_GATE_LPI2C5      CLOCK_GATE(AHB_CLK_CTRL1, 16)    /* LPI2c5 */
#define CLOCK_GATE_LPI2C6      CLOCK_GATE(AHB_CLK_CTRL1, 17)    /* LPI2c6 */
#define CLOCK_GATE_LPI2C7      CLOCK_GATE(AHB_CLK_CTRL1, 18)    /* LPI2c7 */
#define CLOCK_GATE_LPI2C8      CLOCK_GATE(AHB_CLK_CTRL1, 19)    /* LPI2c8 */
#define CLOCK_GATE_LPI2C9      CLOCK_GATE(AHB_CLK_CTRL1, 20)    /* LPI2c9 */
#define CLOCK_GATE_MICFIL      CLOCK_GATE(AHB_CLK_CTRL1, 21)    /* Micfil */
#define CLOCK_GATE_TIMER2      CLOCK_GATE(AHB_CLK_CTRL1, 22)    /* Timer2 */
#define CLOCK_GATE_USB0RAM     CLOCK_GATE(AHB_CLK_CTRL1, 23)    /* Usb0Ram */
#define CLOCK_GATE_USB0FSDCD   CLOCK_GATE(AHB_CLK_CTRL1, 24)    /* Usb0FsDcd */
#define CLOCK_GATE_USB0FS      CLOCK_GATE(AHB_CLK_CTRL1, 25)    /* Usb0Fs */
#define CLOCK_GATE_TIMER0      CLOCK_GATE(AHB_CLK_CTRL1, 26)    /* Timer0 */
#define CLOCK_GATE_TIMER1      CLOCK_GATE(AHB_CLK_CTRL1, 27)    /* Timer1 */
#define CLOCK_GATE_PKCRAM      CLOCK_GATE(AHB_CLK_CTRL1, 29)    /* PkcRam */
#define CLOCK_GATE_SMARTDMA    CLOCK_GATE(AHB_CLK_CTRL1, 31)    /* SmartDma */
#define CLOCK_GATE_ESPI        CLOCK_GATE(AHB_CLK_CTRL2, 0)     /* Espi */
#define CLOCK_GATE_DMA1        CLOCK_GATE(AHB_CLK_CTRL2, 1)     /* Dma1 */
#define CLOCK_GATE_ENET        CLOCK_GATE(AHB_CLK_CTRL2, 2)     /* Enet */
#define CLOCK_GATE_USDHC       CLOCK_GATE(AHB_CLK_CTRL2, 3)     /* uSdhc */
#define CLOCK_GATE_FLEXIO      CLOCK_GATE(AHB_CLK_CTRL2, 4)     /* Flexio */
#define CLOCK_GATE_SAI0        CLOCK_GATE(AHB_CLK_CTRL2, 5)     /* Sai0 */
#define CLOCK_GATE_SAI1        CLOCK_GATE(AHB_CLK_CTRL2, 6)     /* Sai1 */
#define CLOCK_GATE_TRO         CLOCK_GATE(AHB_CLK_CTRL2, 7)     /* Tro */
#define CLOCK_GATE_FREQME      CLOCK_GATE(AHB_CLK_CTRL2, 8)     /* Freqme */
#define CLOCK_GATE_TRNG        CLOCK_GATE(AHB_CLK_CTRL2, 13)    /* Trng */
#define CLOCK_GATE_FLEXCAN0    CLOCK_GATE(AHB_CLK_CTRL2, 14)    /* Flexcan0 */
#define CLOCK_GATE_FLEXCAN1    CLOCK_GATE(AHB_CLK_CTRL2, 15)    /* Flexcan1 */
#define CLOCK_GATE_USBHS       CLOCK_GATE(AHB_CLK_CTRL2, 16)    /* UsbHs */
#define CLOCK_GATE_USBHSPHY    CLOCK_GATE(AHB_CLK_CTRL2, 17)    /* UsbHsPhy */
#define CLOCK_GATE_CSS         CLOCK_GATE(AHB_CLK_CTRL2, 18)    /* Css */
#define CLOCK_GATE_POWERQUAD   CLOCK_GATE(AHB_CLK_CTRL2, 19)    /* PowerQuad */
#define CLOCK_GATE_PLULUT      CLOCK_GATE(AHB_CLK_CTRL2, 20)    /* PluLut */
#define CLOCK_GATE_TIMER3      CLOCK_GATE(AHB_CLK_CTRL2, 21)    /* Timer3 */
#define CLOCK_GATE_TIMER4      CLOCK_GATE(AHB_CLK_CTRL2, 22)    /* Timer4 */
#define CLOCK_GATE_PUF         CLOCK_GATE(AHB_CLK_CTRL2, 23)    /* Puf */
#define CLOCK_GATE_PKC         CLOCK_GATE(AHB_CLK_CTRL2, 24)    /* Pkc */
#define CLOCK_GATE_SCG         CLOCK_GATE(AHB_CLK_CTRL2, 26)    /* Scg */
#define CLOCK_GATE_GDET        CLOCK_GATE(AHB_CLK_CTRL2, 29)    /* Gdet */
#define CLOCK_GATE_SM3         CLOCK_GATE(AHB_CLK_CTRL2, 30)    /* Sm3 */
#define CLOCK_GATE_I3C0        CLOCK_GATE(AHB_CLK_CTRL3, 0)     /* I3c0 */
#define CLOCK_GATE_I3C1        CLOCK_GATE(AHB_CLK_CTRL3, 1)     /* I3c1 */
#define CLOCK_GATE_SINC        CLOCK_GATE(AHB_CLK_CTRL3, 2)     /* Sinc */
#define CLOCK_GATE_COOLFLUX    CLOCK_GATE(AHB_CLK_CTRL3, 3)     /* CoolFlux */
#define CLOCK_GATE_QDC0        CLOCK_GATE(AHB_CLK_CTRL3, 4)     /* Qdc0 */
#define CLOCK_GATE_QDC1        CLOCK_GATE(AHB_CLK_CTRL3, 5)     /* Qdc1 */
#define CLOCK_GATE_PWM0        CLOCK_GATE(AHB_CLK_CTRL3, 6)     /* Pwm0 */
#define CLOCK_GATE_PWM1        CLOCK_GATE(AHB_CLK_CTRL3, 7)     /* Pwm1 */
#define CLOCK_GATE_EVTG        CLOCK_GATE(AHB_CLK_CTRL3, 8)     /* Evtg */
#define CLOCK_GATE_DAC1        CLOCK_GATE(AHB_CLK_CTRL3, 11)    /* Dac1 */
#define CLOCK_GATE_DAC2        CLOCK_GATE(AHB_CLK_CTRL3, 12)    /* Dac2 */
#define CLOCK_GATE_OPAMP0      CLOCK_GATE(AHB_CLK_CTRL3, 13)    /* Opamp0 */
#define CLOCK_GATE_OPAMP1      CLOCK_GATE(AHB_CLK_CTRL3, 14)    /* Opamp1 */
#define CLOCK_GATE_OPAMP2      CLOCK_GATE(AHB_CLK_CTRL3, 15)    /* Opamp2 */
#define CLOCK_GATE_CMP2        CLOCK_GATE(AHB_CLK_CTRL3, 18)    /* Cmp2 */
#define CLOCK_GATE_VREF        CLOCK_GATE(AHB_CLK_CTRL3, 19)    /* Vref */
#define CLOCK_GATE_COOLFLUXAPB CLOCK_GATE(AHB_CLK_CTRL3, 20)    /* CoolFluxApb */
#define CLOCK_GATE_NEUTRON     CLOCK_GATE(AHB_CLK_CTRL3, 21)    /* Neutron */
#define CLOCK_GATE_TSI         CLOCK_GATE(AHB_CLK_CTRL3, 22)    /* Tsi */
#define CLOCK_GATE_EWM         CLOCK_GATE(AHB_CLK_CTRL3, 23)    /* Ewm */
#define CLOCK_GATE_EWM0        CLOCK_GATE(AHB_CLK_CTRL3, 23)    /* Ewm */
#define CLOCK_GATE_EIM         CLOCK_GATE(AHB_CLK_CTRL3, 24)    /* Eim */
#define CLOCK_GATE_ERM         CLOCK_GATE(AHB_CLK_CTRL3, 25)    /* Erm */
#define CLOCK_GATE_INTM        CLOCK_GATE(AHB_CLK_CTRL3, 26)    /* Intm */
#define CLOCK_GATE_SEMA42      CLOCK_GATE(AHB_CLK_CTRL3, 27)    /* Sema42 */
#define CLOCK_GATE_PWM0_SM0    CLOCK_GATE(REG_PWM0SUBCTL, 0U)   /* PWM0 SM0 */
#define CLOCK_GATE_PWM0_SM1    CLOCK_GATE(REG_PWM0SUBCTL, 1U)   /* PWM0 SM1 */
#define CLOCK_GATE_PWM0_SM2    CLOCK_GATE(REG_PWM0SUBCTL, 2U)   /* PWM0 SM2 */
#define CLOCK_GATE_PWM0_SM3    CLOCK_GATE(REG_PWM0SUBCTL, 3U)   /* PWM0 SM3 */
#define CLOCK_GATE_PWM1_SM0    CLOCK_GATE(REG_PWM1SUBCTL, 0U)   /* PWM1 SM0 */
#define CLOCK_GATE_PWM1_SM1    CLOCK_GATE(REG_PWM1SUBCTL, 1U)   /* PWM1 SM1 */
#define CLOCK_GATE_PWM1_SM2    CLOCK_GATE(REG_PWM1SUBCTL, 2U)   /* PWM1 SM2 */
#define CLOCK_GATE_PWM1_SM3    CLOCK_GATE(REG_PWM1SUBCTL, 3U)   /* PWM1 SM3 */

/* Miscellaneous SYSCON registers */

/* LPCAC Control */

#define SYSCON_LPCAC_CTRL                        (NXXX_SYSCON0_BASE + 0x824)

#define SYSCON_LPCAC_CTRL_DIS_LPCAC_SHIFT        (0)
#define SYSCON_LPCAC_CTRL_DIS_LPCAC_MASK         (0x01 << SYSCON_LPCAC_CTRL_DIS_LPCAC_SHIFT)
#define SYSCON_LPCAC_CTRL_DIS_LPCAC(x)           (((x) << SYSCON_LPCAC_CTRL_DIS_LPCAC_SHIFT) & SYSCON_LPCAC_CTRL_DIS_LPCAC_MASK)

#define SYSCON_LPCAC_CTRL_CLR_LPCAC_SHIFT        (1)
#define SYSCON_LPCAC_CTRL_CLR_LPCAC_MASK         (0x01 << SYSCON_LPCAC_CTRL_CLR_LPCAC_SHIFT)
#define SYSCON_LPCAC_CTRL_CLR_LPCAC(x)           (((x) << SYSCON_LPCAC_CTRL_CLR_LPCAC_SHIFT) & SYSCON_LPCAC_CTRL_CLR_LPCAC_MASK)

#define SYSCON_LPCAC_CTRL_FRC_NO_ALLOC_SHIFT     (2)
#define SYSCON_LPCAC_CTRL_FRC_NO_ALLOC_MASK      (0x01 << SYSCON_LPCAC_CTRL_FRC_NO_ALLOC_SHIFT)
#define SYSCON_LPCAC_CTRL_FRC_NO_ALLOC(x)        (((x) << SYSCON_LPCAC_CTRL_FRC_NO_ALLOC_SHIFT) & SYSCON_LPCAC_CTRL_FRC_NO_ALLOC_MASK)

#define SYSCON_LPCAC_CTRL_PARITY_MISS_EN_SHIFT   (3)
#define SYSCON_LPCAC_CTRL_PARITY_MISS_EN_MASK    (0x01 << SYSCON_LPCAC_CTRL_PARITY_MISS_EN_SHIFT)
#define SYSCON_LPCAC_CTRL_PARITY_MISS_EN(x)      (((x) << SYSCON_LPCAC_CTRL_PARITY_MISS_EN_SHIFT) & SYSCON_LPCAC_CTRL_PARITY_MISS_EN_MASK)

#define SYSCON_LPCAC_CTRL_DIS_LPCAC_WTBF_SHIFT   (4)
#define SYSCON_LPCAC_CTRL_DIS_LPCAC_WTBF_MASK    (0x01 << SYSCON_LPCAC_CTRL_DIS_LPCAC_WTBF_SHIFT)
#define SYSCON_LPCAC_CTRL_DIS_LPCAC_WTBF(x)      (((x) << SYSCON_LPCAC_CTRL_DIS_LPCAC_WTBF_SHIFT) & SYSCON_LPCAC_CTRL_DIS_LPCAC_WTBF_MASK)

#define SYSCON_LPCAC_CTRL_LIM_LPCAC_WTBF_SHIFT   (5)
#define SYSCON_LPCAC_CTRL_LIM_LPCAC_WTBF_MASK    (0x01 << SYSCON_LPCAC_CTRL_LIM_LPCAC_WTBF_SHIFT)
#define SYSCON_LPCAC_CTRL_LIM_LPCAC_WTBF(x)      (((x) << SYSCON_LPCAC_CTRL_LIM_LPCAC_WTBF_SHIFT) & SYSCON_LPCAC_CTRL_LIM_LPCAC_WTBF_MASK)

#define SYSCON_LPCAC_CTRL_PARITY_FAULT_EN_SHIFT  (6)
#define SYSCON_LPCAC_CTRL_PARITY_FAULT_EN_MASK   (0x01 << SYSCON_LPCAC_CTRL_PARITY_FAULT_EN_SHIFT)
#define SYSCON_LPCAC_CTRL_PARITY_FAULT_EN(x)     (((x) << SYSCON_LPCAC_CTRL_PARITY_FAULT_EN_SHIFT) & SYSCON_LPCAC_CTRL_PARITY_FAULT_EN_MASK)

#define SYSCON_LPCAC_CTRL_LPCAC_XOM_SHIFT        (7)
#define SYSCON_LPCAC_CTRL_LPCAC_XOM_MASK         (0x01 << SYSCON_LPCAC_CTRL_LPCAC_XOM_SHIFT)
#define SYSCON_LPCAC_CTRL_LPCAC_XOM(x)           (((x) << SYSCON_LPCAC_CTRL_LPCAC_XOM_SHIFT) & SYSCON_LPCAC_CTRL_LPCAC_XOM_MASK)

/* RAM ECC Enable Control */

#define SYSCON_ECC_ENABLE_CTRL                      (NXXX_SYSCON0_BASE + 0xe44)

#define SYSCON_ECC_ENABLE_CTRL_RAMA_ECC_ENABLE      (1 << 0) /* RAMA ECC enable */
#define SYSCON_ECC_ENABLE_CTRL_RAMB_RAMX_ECC_ENABLE (1 << 1) /* RAMB and RAMX ECC enable */
#define SYSCON_ECC_ENABLE_CTRL_RAMD_RAMC_ECC_ENABLE (1 << 2) /* RAMD and RAMC ECC enable */

/* Clock Control */

#define SYSCON_CLOCKCTRL                        (NXXX_SYSCON0_BASE + 0xa18)

#define SYSCON_CLOCKCTRL_CLKIN_ENA_FM_USBH_LPT  (1 << 1) /* Enables the clk_in clock for the Frequency Measurement, USB HS and LPTMR0/1 modules. */
#define SYSCON_CLOCKCTRL_FRO1MHZ_ENA            (1 << 2) /* Enables the FRO_1MHz clock for RTC module and for UTICK */
#define SYSCON_CLOCKCTRL_FRO12MHZ_ENA           (1 << 3) /* Enables the FRO_12MHz clock for the Flash, LPTMR0/1, and Frequency Measurement modules */
#define SYSCON_CLOCKCTRL_FRO_HF_ENA             (1 << 4) /* Enables FRO HF clock for the Frequency Measure module */
#define SYSCON_CLOCKCTRL_CLKIN_ENA              (1 << 5) /* Enables clk_in clock for MICFIL, CAN0/1, I3C0/1, SAI0/1, clkout */
#define SYSCON_CLOCKCTRL_FRO1MHZ_CLK_ENA        (1 << 6) /* Enables FRO_1MHz clock for clock muxing in clock gen */

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef enum
{
  CORESYSCLK  = 0,    /* Core / system clock */
  BUSCLK      = 1,    /* AHB Bus clock */
  SYSTICKCLK0 = 2,    /* Systick clock 0 */
  CLOCKOUT    = 3,    /* CLOCKOUT */
  FRO12M      = 4,    /* FRO12M */
  CLK1M       = 5,    /* CLK1M */
  FROHF       = 6,    /* FRO48 / 144 */
  CLK48M      = 7,    /* CLK48M */
  CLK144M     = 8,    /* CLK144M */
  CLK16K0     = 9,    /* CLK16K[0] */
  CLK16K1     = 10,   /* CLK16K[1] */
  CLK16K2     = 11,   /* CLK16K[2] */
  CLK16K3     = 12,   /* CLK16K[3] */
  EXTCLK      = 13,   /* External clock */
  OSC32K0     = 14,   /* OSC32K[0] */
  OSC32K1     = 15,   /* OSC32K[1] */
  OSC32K2     = 16,   /* OSC32K[2] */
  OSC32K3     = 17,   /* OSC32K[3] */
  PLL0OUT     = 18,   /* PLL0 output */
  PLL1OUT     = 19,   /* PLL1 output */
  USBPLLOUT   = 20,   /* USBPLL output */
  LPOSC       = 21,   /* Low power oscillator */
  LDB_PLL_CLK = 38,
} clock_id_e;

#endif /* __ARCH_ARM_SRC_MCX_NXXX_HARDWARE_N236_N236_CLOCK_H */
