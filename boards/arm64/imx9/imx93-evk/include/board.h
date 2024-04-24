/****************************************************************************
 * boards/arm64/imx9/imx93-evk/include/board.h
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

#ifndef __BOARDS_ARM64_IMX9_IMX93_EVK_INCLUDE_BOARD_H
#define __BOARDS_ARM64_IMX9_IMX93_EVK_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Default PAD configurations */

#define IOMUX_LPI2C_DEFAULT  (IOMUXC_PAD_OD_ENABLE | IOMUXC_PAD_FSEL_SFAST | IOMUXC_PAD_DSE_X6)
#define IOMUX_LPSPI_DEFAULT  (IOMUXC_PAD_PU_ON     | IOMUXC_PAD_FSEL_FAST  | IOMUXC_PAD_DSE_X6)
#define IOMUX_GPIO_DEFAULT   (IOMUXC_PAD_FSEL_SLOW | IOMUXC_PAD_DSE_X6)

/* UART pin muxings */

#define MUX_LPUART1_RX       IOMUX_CFG(IOMUXC_PAD_UART1_RXD_LPUART1_RX, 0, IOMUXC_MUX_SION_ON)
#define MUX_LPUART1_TX       IOMUX_CFG(IOMUXC_PAD_UART1_TXD_LPUART1_TX, IOMUXC_PAD_FSEL_SLOW | IOMUXC_PAD_DSE_X4, 0)

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

#define MUX_LPI2C1_SCL       IOMUX_CFG(IOMUXC_PAD_I2C1_SCL_LPI2C1_SCL, IOMUX_LPI2C_DEFAULT, IOMUXC_MUX_SION_ON)
#define MUX_LPI2C1_SDA       IOMUX_CFG(IOMUXC_PAD_I2C1_SDA_LPI2C1_SDA, IOMUX_LPI2C_DEFAULT, IOMUXC_MUX_SION_ON)

/* I2C reset functionality */

#define GPIO_LPI2C1_SCL_RESET  (GPIO_PORT1 | GPIO_PIN0 | GPIO_OUTPUT | GPIO_OUTPUT_ONE)
#define GPIO_LPI2C1_SDA_RESET  (GPIO_PORT1 | GPIO_PIN1 | GPIO_OUTPUT | GPIO_OUTPUT_ONE)

/* LPSPIs */

#define MUX_LPSPI3_SCK       IOMUX_CFG(IOMUXC_PAD_GPIO_IO11_LPSPI3_SCK,  IOMUX_LPSPI_DEFAULT, IOMUXC_MUX_SION_ON)
#define MUX_LPSPI3_MOSI      IOMUX_CFG(IOMUXC_PAD_GPIO_IO10_LPSPI3_SOUT, IOMUX_LPSPI_DEFAULT, IOMUXC_MUX_SION_ON)
#define MUX_LPSPI3_MISO      IOMUX_CFG(IOMUXC_PAD_GPIO_IO09_LPSPI3_SIN,  IOMUX_LPSPI_DEFAULT, IOMUXC_MUX_SION_ON)
#define MUX_LPSPI6_SCK       IOMUX_CFG(IOMUXC_PAD_GPIO_IO03_LPSPI6_SCK,  IOMUX_LPSPI_DEFAULT, IOMUXC_MUX_SION_ON)
#define MUX_LPSPI6_MOSI      IOMUX_CFG(IOMUXC_PAD_GPIO_IO02_LPSPI6_SOUT, IOMUX_LPSPI_DEFAULT, IOMUXC_MUX_SION_ON)
#define MUX_LPSPI6_MISO      IOMUX_CFG(IOMUXC_PAD_GPIO_IO01_LPSPI6_SIN,  IOMUX_LPSPI_DEFAULT, IOMUXC_MUX_SION_ON)

/* SPI CS */

#define MUX_LPSPI3_CS        IOMUX_CFG(IOMUXC_PAD_GPIO_IO08_GPIO2_IO08,  IOMUX_GPIO_DEFAULT, IOMUXC_MUX_SION_ON)
#define GPIO_LPSPI3_CS       (GPIO_PORT2 | GPIO_PIN8 | GPIO_OUTPUT | GPIO_OUTPUT_ONE)
#define MUX_LPSPI6_CS        IOMUX_CFG(IOMUXC_PAD_GPIO_IO00_GPIO2_IO00,  IOMUX_GPIO_DEFAULT, IOMUXC_MUX_SION_ON)
#define GPIO_LPSPI6_CS       (GPIO_PORT2 | GPIO_PIN0 | GPIO_OUTPUT | GPIO_OUTPUT_ONE)

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

#define ARMPLL_CFG  PLL_CFG(IMX9_ARMPLL_BASE, false, PLL_PARMS(1, 2, 141, 0, 0))
#define DRAMPLL_CFG PLL_CFG(IMX9_DRAMPLL_BASE, true, PLL_PARMS(1, 2, 155, 1, 2))

#define PLL_CFGS \
  { \
    PLL_CFG(IMX9_SYSPLL_BASE,  true, PLL_PARMS(1, 4, 166, 2, 3)), \
  }

#define PFD_CFGS \
  { \
    PFD_CFG(IMX9_SYSPLL_BASE, 0, PFD_PARMS(4, 0, true)), \
    PFD_CFG(IMX9_SYSPLL_BASE, 1, PFD_PARMS(5, 0, true)), \
    PFD_CFG(IMX9_SYSPLL_BASE, 2, PFD_PARMS(6, 2, true)), \
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
#endif /* __BOARDS_ARM64_IMX9_IMX93_EVK_INCLUDE_BOARD_H */
