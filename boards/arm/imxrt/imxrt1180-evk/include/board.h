/****************************************************************************
 * boards/arm/imxrt/imxrt1180-evk/include/board.h
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

#ifndef __BOARDS_ARM_IMXRT_IMXRT1180_EVK_INCLUDE_BOARD_H
#define __BOARDS_ARM_IMXRT_IMXRT1180_EVK_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "hardware/rt118x/imxrt118x_iomuxc.h"
#include "hardware/rt118x/imxrt118x_pinmux.h"
#include "imxrt_iomuxc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* Both cores share ARM_PLL at its supported 798 MHz maximum:
 *
 *   M7  = CLOCK_ROOT0 = ARM_PLL / 1 = 798 MHz
 *   M33 = CLOCK_ROOT1 = ARM_PLL / 3 = 266 MHz
 *
 * Both are within the HSRUN maxima of 800 / 300 MHz (IMXRT1180RM Rev. 10
 * Table 128).
 *
 * BOARD_CPU_FREQUENCY is the frequency of the core this image runs on: it
 * drives the SysTick reload value, so it must match the core actually
 * executing or the whole system time base is scaled wrong.
 */

#define BOARD_XTAL_FREQUENCY      24000000

#define BOARD_CCM_DOMAIN_CM33     2
#define BOARD_CCM_DOMAIN_CM7      4

#ifdef CONFIG_ARCH_CHIP_MIMXRT1189CVM8C_CM33
#  define BOARD_CPU_FREQUENCY     266000000
#  define BOARD_CCM_DOMAIN_ID     BOARD_CCM_DOMAIN_CM33
#else
#  define BOARD_CPU_FREQUENCY     798000000
#  define BOARD_CCM_DOMAIN_ID     BOARD_CCM_DOMAIN_CM7
#endif

/* LPUART1 pin configuration.  The MIMXRT1180-EVK routes LPUART1 to
 * GPIO_AON_08 (TX) and GPIO_AON_09 (RX) via the MCU-Link VCOM
 * (verified against schematic SCH-50577_C4: net LPUART1_TXD -> RT1180
 * ball B1).
 *
 * The pad configuration flags are common to peripheral use of the
 * LPUART: no pull, high drive strength.
 */

#define IOMUX_LPUART_DEFAULT (IOMUXC_PAD_PDRV_HIGH | IOMUXC_PAD_PULL_NONE)

#define GPIO_LPUART1_TX  IOMUX_PIN(IOMUXC_PAD_GPIO_AON_08_LPUART1_TX, \
                                   IOMUX_LPUART_DEFAULT, 0)
#define GPIO_LPUART1_RX  IOMUX_PIN(IOMUXC_PAD_GPIO_AON_09_LPUART1_RX, \
                                   IOMUX_LPUART_DEFAULT, IOMUXC_MUX_SION_ON)

/* User LEDs: D6 (green) on GPIO_AD_27 = RGPIO4.27,
 *            D7 (red)   on GPIO_AD_26 = RGPIO4.26.
 */

#define IOMUX_LED_DEFAULT   (IOMUXC_PAD_PDRV_HIGH | IOMUXC_PAD_PULL_NONE)

#define GPIO_LED1           IOMUX_GPIO(IOMUXC_PAD_GPIO_AD_27_GPIO4_IO27,  \
                                       IOMUX_LED_DEFAULT,                \
                                       GPIO_OUTPUT | GPIO_OUTPUT_ZERO |  \
                                       GPIO_PORT4  | GPIO_PIN27)
#define GPIO_LED2           IOMUX_GPIO(IOMUXC_PAD_GPIO_AD_26_GPIO4_IO26,  \
                                       IOMUX_LED_DEFAULT,                \
                                       GPIO_OUTPUT | GPIO_OUTPUT_ZERO |  \
                                       GPIO_PORT4  | GPIO_PIN26)

/* LPI2C2: routed to on-board sensors, ENET PHY ID EEPROM etc.
 *   GPIO_AON_15 -> LPI2C2_SDA
 *   GPIO_AON_16 -> LPI2C2_SCL
 */

#define IOMUX_LPI2C_DEFAULT (IOMUXC_PAD_PDRV_HIGH | IOMUXC_PAD_PULL_UP | \
                             IOMUXC_PAD_ODE_ON)

#define GPIO_LPI2C2_SDA  IOMUX_PIN(IOMUXC_PAD_GPIO_AON_15_LPI2C2_SDA,   \
                                   IOMUX_LPI2C_DEFAULT, IOMUXC_MUX_SION_ON)
#define GPIO_LPI2C2_SCL  IOMUX_PIN(IOMUXC_PAD_GPIO_AON_16_LPI2C2_SCL,   \
                                   IOMUX_LPI2C_DEFAULT, IOMUXC_MUX_SION_ON)

/* LPSPI1: routed to the on-board LPSPI NOR flash (U12 companion).
 *   GPIO_AON_04 -> LPSPI1_SCK
 *   GPIO_AON_05 -> LPSPI1_PCS0
 *   GPIO_AON_06 -> LPSPI1_SDO
 *   GPIO_AON_07 -> LPSPI1_SDI
 */

#define IOMUX_LPSPI_DEFAULT (IOMUXC_PAD_PDRV_HIGH | IOMUXC_PAD_PULL_NONE)

#define GPIO_LPSPI1_SCK  IOMUX_PIN(IOMUXC_PAD_GPIO_AON_04_LPSPI1_SCK,   \
                                   IOMUX_LPSPI_DEFAULT, 0)
#define GPIO_LPSPI1_CS   IOMUX_PIN(IOMUXC_PAD_GPIO_AON_05_LPSPI1_PCS0,  \
                                   IOMUX_LPSPI_DEFAULT, 0)
#define GPIO_LPSPI1_MOSI IOMUX_PIN(IOMUXC_PAD_GPIO_AON_06_LPSPI1_SDO,   \
                                   IOMUX_LPSPI_DEFAULT, 0)
#define GPIO_LPSPI1_MISO IOMUX_PIN(IOMUXC_PAD_GPIO_AON_07_LPSPI1_SDI,   \
                                   IOMUX_LPSPI_DEFAULT, IOMUXC_MUX_SION_ON)

#endif /* __BOARDS_ARM_IMXRT_IMXRT1180_EVK_INCLUDE_BOARD_H */
