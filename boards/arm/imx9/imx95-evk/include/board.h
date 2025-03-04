/****************************************************************************
 * boards/arm/imx9/imx95-evk/include/board.h
 *
 * SPDX-License-Identifier: Apache-2.0
 * SPDX-FileCopyrightText: 2024 NXP
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

#ifndef __BOARDS_ARM_IMX9_IMX95_EVK_INCLUDE_BOARD_H
#define __BOARDS_ARM_IMX9_IMX95_EVK_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BOARD_XTAL_FREQUENCY       24000000

#define BOARD_CPU_FREQUENCY        BOARD_XTAL_FREQUENCY //FIXME

#define LPUART3_CLK                (LPUART3_CLK_ROOT_OSC_24M_CLK | CLOCK_DIV(1))
#define LPI2C6_CLK                 (LPI2C6_CLK_ROOT_OSC_24M_CLK | CLOCK_DIV(1))
#define LPSPI1_CLK                 (LPSPI1_CLK_ROOT_OSC_24M_CLK | CLOCK_DIV(1))

#define FLEXCAN1_CLK               (CAN1_CLK_ROOT_SYS_PLL1_DFS1_DIV2_CLK | CLOCK_DIV(5))

/* This LED is not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.  In that case, the usage by the board port is defined in
 * include/board.h and src/imxrt_autoleds.c. The LED is used to encode
 * OS-related events as follows:
 *
 *   -------------------- ----------------------------- ------- -------
 *   SYMBOL                   Meaning                    LED1    LED2
 *                                                       GREEN   RED
 *   -------------------- ----------------------------- ------- -------
 */

#define LED_STARTED       0  /* NuttX has been started   OFF     OFF   */
#define LED_HEAPALLOCATE  0  /* Heap has been allocated  OFF     OFF   */
#define LED_IRQSENABLED   0  /* Interrupts enabled       OFF     OFF   */
#define LED_STACKCREATED  1  /* Idle stack created       ON      OFF   */
#define LED_INIRQ         2  /* In an interrupt         (No change)    */
#define LED_SIGNAL        2  /* In a signal handler     (No change)    */
#define LED_ASSERTION     2  /* An assertion failed     (No change)    */
#define LED_PANIC         3  /* The system has crashed   OFF     FLASH */
#undef  LED_IDLE             /* Not used                (Not used)     */

/* Default PAD configurations */

#define IOMUX_LPI2C_DEFAULT  (IOMUXC_PAD_OD_ENABLE | IOMUXC_PAD_FSEL_SFAST | IOMUXC_PAD_DSE_X6)
#define IOMUX_LPSPI_DEFAULT  (IOMUXC_PAD_PU_ON     | IOMUXC_PAD_FSEL_FAST  | IOMUXC_PAD_DSE_X6)
#define IOMUX_GPIO_DEFAULT   (IOMUXC_PAD_FSEL_SLOW | IOMUXC_PAD_DSE_X6)

/* UART pin muxings */

#define MUX_LPUART3_RX       IOMUX_CFG(IOMUXC_PAD_GPIO_IO15_LPUART3_RX, 0, IOMUXC_MUX_SION_ON)
#define MUX_LPUART3_TX       IOMUX_CFG(IOMUXC_PAD_GPIO_IO14_LPUART3_TX, IOMUXC_PAD_FSEL_SLOW | IOMUXC_PAD_DSE_X4, 0)

/* FLEXIO to PWM pin muxings */

/* EVK signals
 * GPIO_IO04 -> FLEXIO1_04
 * GPIO_IO05 -> FLEXIO1_05
 * GPIO_IO06 -> FLEXIO1_06
 * GPIO_IO07 -> FLEXIO1_07
 */

#define FLEXIO1_PWM0_MUX IOMUX_CFG(IOMUXC_PAD_GPIO_IO04_FLEXIO1_FLEXIO04, IOMUXC_PAD_FSEL_SFAST | IOMUXC_PAD_DSE_X6, 0)
#define FLEXIO1_PWM1_MUX IOMUX_CFG(IOMUXC_PAD_GPIO_IO05_FLEXIO1_FLEXIO05, IOMUXC_PAD_FSEL_SFAST | IOMUXC_PAD_DSE_X6, 0)
#define FLEXIO1_PWM2_MUX IOMUX_CFG(IOMUXC_PAD_GPIO_IO06_FLEXIO1_FLEXIO06, IOMUXC_PAD_FSEL_SFAST | IOMUXC_PAD_DSE_X6, 0)
#define FLEXIO1_PWM3_MUX IOMUX_CFG(IOMUXC_PAD_GPIO_IO07_FLEXIO1_FLEXIO07, IOMUXC_PAD_FSEL_SFAST | IOMUXC_PAD_DSE_X6, 0)

/* LPI2Cs */

/* TPM3 ch3 to PWM pin GPIO_IO24 muxing */

#define TPM3_PWM3_MUX IOMUX_CFG(IOMUXC_PAD_GPIO_IO24_TPM3_CH3, IOMUXC_PAD_FSEL_SFAST | IOMUXC_PAD_DSE_X6, 0)

/* LPI2Cs */

#define MUX_LPI2C6_SCL       IOMUX_CFG(IOMUXC_PAD_GPIO_IO03_LPI2C6_SCL, IOMUX_LPI2C_DEFAULT, IOMUXC_MUX_SION_ON)
#define MUX_LPI2C6_SDA       IOMUX_CFG(IOMUXC_PAD_GPIO_IO02_LPI2C6_SDA, IOMUX_LPI2C_DEFAULT, IOMUXC_MUX_SION_ON)

/* I2C reset functionality */

#define GPIO_LPI2C1_SCL_RESET  (GPIO_PORT1 | GPIO_PIN0 | GPIO_OUTPUT | GPIO_OUTPUT_ONE)
#define GPIO_LPI2C1_SDA_RESET  (GPIO_PORT1 | GPIO_PIN1 | GPIO_OUTPUT | GPIO_OUTPUT_ONE)

/* LPSPIs */

#define MUX_LPSPI1_SCK       IOMUX_CFG(IOMUXC_PAD_SAI1_TXD0_LPSPI1_SCK,  IOMUX_LPSPI_DEFAULT, IOMUXC_MUX_SION_ON)
#define MUX_LPSPI1_MOSI      IOMUX_CFG(IOMUXC_PAD_SAI1_RXD0_LPSPI1_SOUT, IOMUX_LPSPI_DEFAULT, IOMUXC_MUX_SION_ON)
#define MUX_LPSPI1_MISO      IOMUX_CFG(IOMUXC_PAD_SAI1_TXC_LPSPI1_SIN,  IOMUX_LPSPI_DEFAULT, IOMUXC_MUX_SION_ON)

/* SPI CS */

#define MUX_LPSPI1_CS        IOMUX_CFG(IOMUXC_PAD_SAI1_TXFS_GPIO1_IO11,  IOMUX_GPIO_DEFAULT, IOMUXC_MUX_SION_ON)
#define GPIO_LPSPI1_CS       (GPIO_PORT1 | GPIO_PIN11 | GPIO_OUTPUT | GPIO_OUTPUT_ONE)

/* FlexCAN */

#define GPIO_FLEXCAN1_TX     IOMUX_CFG(IOMUXC_PAD_PDM_CLK_CAN1_TX,  IOMUX_GPIO_DEFAULT, IOMUXC_MUX_SION_ON)
#define GPIO_FLEXCAN1_RX     IOMUX_CFG(IOMUXC_PAD_PDM_BIT_STREAM0_CAN1_RX,  IOMUX_GPIO_DEFAULT, IOMUXC_MUX_SION_ON)

#define GPIO_FLEXCAN2_TX     IOMUX_CFG(IOMUXC_PAD_GPIO_IO25_CAN2_TX,  IOMUX_GPIO_DEFAULT, IOMUXC_MUX_SION_ON)
#define GPIO_FLEXCAN2_RX     IOMUX_CFG(IOMUXC_PAD_GPIO_IO27_CAN2_RX,  IOMUX_GPIO_DEFAULT, IOMUXC_MUX_SION_ON)

/* Set the PLL clocks as follows:
 *
 * - OSC24M       : 24   MHz
 * - ARMPLL_OUT   : 1692 MHz
 * - DRAMPLL      : 933  MHz
 * - SYSPLL1      : 4000 MHz
 * - SYSPLL_PFD0  : 1000 MHz
 * - SYSPLL_PFD1  : 800  MHz
 * - SYSPLL_PFD2  : 625  MHz
 * - AUDIOPLL_OUT : OFF
 * - VIDEOPLL_OUT : OFF
 *
 * After reset all clock sources (OSCPLL) and root clocks (CLOCK_ROOT) are
 * running, but gated (LPCG).
 *
 * By default, all peripheral root clocks are set to the 24 MHz oscillator.
 */

#define ARMPLL_CFG  PLL_CFG(IMX95_ARMPLL_BASE, false, PLL_PARMS(1, 2, 141, 0, 0))
#define DRAMPLL_CFG PLL_CFG(IMX95_DRAMPLL_BASE, true, PLL_PARMS(1, 2, 155, 1, 2))

#define PLL_CFGS \
  { \
    PLL_CFG(IMX95_SYSPLL_BASE,  true, PLL_PARMS(1, 4, 166, 2, 3)), \
  }

#define PFD_CFGS \
  { \
    PFD_CFG(IMX95_SYSPLL_BASE, 0, PFD_PARMS(4, 0, true)), \
    PFD_CFG(IMX95_SYSPLL_BASE, 1, PFD_PARMS(5, 0, true)), \
    PFD_CFG(IMX95_SYSPLL_BASE, 2, PFD_PARMS(6, 2, true)), \
  }

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_IMX9_IMX95_EVK_INCLUDE_BOARD_H */
