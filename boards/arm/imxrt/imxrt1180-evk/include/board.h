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

/* FlexSPI1 Port A is connected to the EVK's 16 MiB W25Q128JW NOR.  These
 * assignments match the MCUXpresso SDK evkmimxrt1180 XIP example.
 */

#define GPIO_FLEXSPI_DQS IOMUX_PIN( \
  IOMUXC_PAD_GPIO_B2_07_FLEXSPI1_BUS2BIT_A_DQS, 0, 0)
#define GPIO_FLEXSPI_SCK IOMUX_PIN( \
  IOMUXC_PAD_GPIO_B2_08_FLEXSPI1_BUS2BIT_A_SCLK, 0, 0)
#define GPIO_FLEXSPI_CS IOMUX_PIN( \
  IOMUXC_PAD_GPIO_B2_09_FLEXSPI1_BUS2BIT_A_SS0_B, 0, 0)
#define GPIO_FLEXSPI_IO0 IOMUX_PIN( \
  IOMUXC_PAD_GPIO_B2_10_FLEXSPI1_BUS2BIT_A_DATA00, 0, 0)
#define GPIO_FLEXSPI_IO1 IOMUX_PIN( \
  IOMUXC_PAD_GPIO_B2_11_FLEXSPI1_BUS2BIT_A_DATA01, 0, 0)
#define GPIO_FLEXSPI_IO2 IOMUX_PIN( \
  IOMUXC_PAD_GPIO_B2_12_FLEXSPI1_BUS2BIT_A_DATA02, 0, 0)
#define GPIO_FLEXSPI_IO3 IOMUX_PIN( \
  IOMUXC_PAD_GPIO_B2_13_FLEXSPI1_BUS2BIT_A_DATA03, 0, 0)

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

#define IOMUX_LPUART_DEFAULT (IOMUXC_PAD_DSE)

#define GPIO_LPUART1_TX  IOMUX_PIN(IOMUXC_PAD_GPIO_AON_08_LPUART1_TX, \
                                   IOMUX_LPUART_DEFAULT, 0)
#define GPIO_LPUART1_RX  IOMUX_PIN(IOMUXC_PAD_GPIO_AON_09_LPUART1_RX, \
                                   IOMUX_LPUART_DEFAULT, IOMUXC_MUX_SION_ON)

/* User LEDs: D6 (green) on GPIO_AD_27 = RGPIO4.27,
 *            D7 (red)   on GPIO_AD_26 = RGPIO4.26.
 */

#define IOMUX_LED_DEFAULT   (IOMUXC_PAD_DSE)

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

#define IOMUX_LPI2C_DEFAULT (IOMUXC_PAD_DSE | IOMUXC_PAD_PUE | \
                             IOMUXC_PAD_PUS | IOMUXC_PAD_ODE)

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

#define IOMUX_LPSPI_DEFAULT (IOMUXC_PAD_DSE)

#define GPIO_LPSPI1_SCK  IOMUX_PIN(IOMUXC_PAD_GPIO_AON_04_LPSPI1_SCK,   \
                                   IOMUX_LPSPI_DEFAULT, 0)
#define GPIO_LPSPI1_CS   IOMUX_PIN(IOMUXC_PAD_GPIO_AON_05_LPSPI1_PCS0,  \
                                   IOMUX_LPSPI_DEFAULT, 0)
#define GPIO_LPSPI1_MOSI IOMUX_PIN(IOMUXC_PAD_GPIO_AON_06_LPSPI1_SDO,   \
                                   IOMUX_LPSPI_DEFAULT, 0)
#define GPIO_LPSPI1_MISO IOMUX_PIN(IOMUXC_PAD_GPIO_AON_07_LPSPI1_SDI,   \
                                   IOMUX_LPSPI_DEFAULT, IOMUXC_MUX_SION_ON)

/* USDHC1: microSD card slot.
 *   GPIO_SD_B1_00 -> USDHC1_CMD
 *   GPIO_SD_B1_01 -> USDHC1_CLK
 *   GPIO_SD_B1_02 -> USDHC1_DATA0
 *   GPIO_SD_B1_03 -> USDHC1_DATA1
 *   GPIO_SD_B1_04 -> USDHC1_DATA2
 *   GPIO_SD_B1_05 -> USDHC1_DATA3
 *   GPIO_AD_15    -> card detect (plain GPIO, active low)
 *   GPIO_AD_14    -> card power enable (active low)
 *   GPIO_AD_34    -> I/O voltage select (low keeps 3.3 V signalling)
 */

#define IOMUX_USDHC1_CLK_DEFAULT  (IOMUXC_PAD_PDRV_HIGH | IOMUXC_PAD_PULL_NONE)
#define IOMUX_USDHC1_CMD_DEFAULT  (IOMUXC_PAD_PDRV_HIGH | IOMUXC_PAD_PULL_UP)
#define IOMUX_USDHC1_DATA_DEFAULT (IOMUXC_PAD_PDRV_HIGH | IOMUXC_PAD_PULL_UP)
#define IOMUX_USDHC1_CD_DEFAULT (IOMUXC_PAD_DSE | IOMUXC_PAD_PUE | \
                                 IOMUXC_PAD_PUS)

#define PIN_USDHC1_CMD   IOMUX_PIN(IOMUXC_PAD_GPIO_SD_B1_00_USDHC1_CMD,   \
                                   IOMUX_USDHC1_CMD_DEFAULT, 0)
#define PIN_USDHC1_DCLK  IOMUX_PIN(IOMUXC_PAD_GPIO_SD_B1_01_USDHC1_CLK,   \
                                   IOMUX_USDHC1_CLK_DEFAULT, 0)
#define PIN_USDHC1_D0    IOMUX_PIN(IOMUXC_PAD_GPIO_SD_B1_02_USDHC1_DATA0, \
                                   IOMUX_USDHC1_DATA_DEFAULT, 0)
#define PIN_USDHC1_D1    IOMUX_PIN(IOMUXC_PAD_GPIO_SD_B1_03_USDHC1_DATA1, \
                                   IOMUX_USDHC1_DATA_DEFAULT, 0)
#define PIN_USDHC1_D2    IOMUX_PIN(IOMUXC_PAD_GPIO_SD_B1_04_USDHC1_DATA2, \
                                   IOMUX_USDHC1_DATA_DEFAULT, 0)
#define PIN_USDHC1_D3    IOMUX_PIN(IOMUXC_PAD_GPIO_SD_B1_05_USDHC1_DATA3, \
                                   IOMUX_USDHC1_DATA_DEFAULT, 0)

#define PIN_USDHC1_CD_GPIO IOMUX_GPIO(IOMUXC_PAD_GPIO_AD_15_GPIO4_IO15, \
                                      IOMUX_USDHC1_CD_DEFAULT,         \
                                      GPIO_INPUT | GPIO_PORT4 | GPIO_PIN15)

#define IOMUX_SDCARD_DEFAULT (IOMUXC_PAD_DSE)

#define GPIO_SD_PWREN    IOMUX_GPIO(IOMUXC_PAD_GPIO_AD_14_GPIO4_IO14, \
                                    IOMUX_SDCARD_DEFAULT,            \
                                    GPIO_OUTPUT | GPIO_OUTPUT_ZERO | \
                                    GPIO_PORT4  | GPIO_PIN14)

#define GPIO_SD1_VSELECT IOMUX_GPIO(IOMUXC_PAD_GPIO_AD_34_GPIO5_IO02, \
                                    IOMUX_SDCARD_DEFAULT,            \
                                    GPIO_OUTPUT | GPIO_OUTPUT_ZERO | \
                                    GPIO_PORT5  | GPIO_PIN2)

/* USDHC clock dividers used by arch/arm/src/imxrt/imxrt_usdhc.c. The
 * uSDHC1 root clock is configured to 198 MHz (SYS_PLL2_PFD2 / 2); see
 * the usdhc1_clk_root entry in imxrt_clockconfig_m7.c.
 */

#define BOARD_USDHC_IDMODE_PRESCALER    USDHC_SYSCTL_SDCLKFS_DIV256
#define BOARD_USDHC_IDMODE_DIVISOR      USDHC_SYSCTL_DVS_DIV(2)

#define BOARD_USDHC_MMCMODE_PRESCALER   USDHC_SYSCTL_SDCLKFS_DIV8
#define BOARD_USDHC_MMCMODE_DIVISOR     USDHC_SYSCTL_DVS_DIV(1)

#define BOARD_USDHC_SD1MODE_PRESCALER   USDHC_SYSCTL_SDCLKFS_DIV8
#define BOARD_USDHC_SD1MODE_DIVISOR     USDHC_SYSCTL_DVS_DIV(1)

#define BOARD_USDHC_SD4MODE_PRESCALER   USDHC_SYSCTL_SDCLKFS_DIV8
#define BOARD_USDHC_SD4MODE_DIVISOR     USDHC_SYSCTL_DVS_DIV(1)

/* SD high speed mode (wide 4-bit, up to 50 MHz): 198 MHz / 4 = 49.5 MHz. */

#define BOARD_USDHC_SD4MODE_HS_PRESCALER USDHC_SYSCTL_SDCLKFS_DIV4
#define BOARD_USDHC_SD4MODE_HS_DIVISOR   USDHC_SYSCTL_DVS_DIV(1)

/* NETC: SPF-50577 C4 sheets 6, 7 and 15-19. */

#define BOARD_NETC_PIN(p) IOMUX_PIN(IOMUXC_PAD_##p, IOMUXC_PAD_PULL_NONE, 0)

#define GPIO_NETC_MDC IOMUX_PIN(IOMUXC_PAD_GPIO_AD_30_NETC_EMDC, \
                               IOMUXC_PAD_DSE | IOMUXC_PAD_PUE, 0)
#define GPIO_NETC_MDIO IOMUX_PIN(IOMUXC_PAD_GPIO_AD_31_NETC_EMDIO, \
                                IOMUXC_PAD_DSE | IOMUXC_PAD_PUE | \
                                IOMUXC_PAD_ODE, 0)

#define BOARD_ENET0_PORT0_PINS \
{ \
  BOARD_NETC_PIN(GPIO_EMC_B2_13_NETC_PINMUX_ETH4_TXD00), \
  BOARD_NETC_PIN(GPIO_EMC_B2_14_NETC_PINMUX_ETH4_TXD01), \
  BOARD_NETC_PIN(GPIO_EMC_B2_15_NETC_PINMUX_ETH4_TX_EN), \
  IOMUX_PIN(IOMUXC_PAD_GPIO_EMC_B2_16_NETC_PINMUX_ETH4_TX_CLK, \
            IOMUXC_PAD_PULL_NONE, IOMUXC_MUX_SION_ON), \
  BOARD_NETC_PIN(GPIO_EMC_B2_17_NETC_PINMUX_ETH4_RXD00), \
  BOARD_NETC_PIN(GPIO_EMC_B2_18_NETC_PINMUX_ETH4_RXD01), \
  BOARD_NETC_PIN(GPIO_EMC_B2_19_NETC_PINMUX_ETH4_RX_DV), \
  BOARD_NETC_PIN(GPIO_EMC_B2_20_NETC_PINMUX_ETH4_RX_ER) \
}

#define BOARD_ENET1_PORT0_PINS \
{ \
  BOARD_NETC_PIN(GPIO_EMC_B2_05_NETC_PINMUX_ETH0_TXD00), \
  BOARD_NETC_PIN(GPIO_EMC_B2_06_NETC_PINMUX_ETH0_TXD01), \
  BOARD_NETC_PIN(GPIO_EMC_B2_07_NETC_PINMUX_ETH0_TX_EN), \
  IOMUX_PIN(IOMUXC_PAD_GPIO_EMC_B2_08_NETC_PINMUX_ETH0_TX_CLK, \
            IOMUXC_PAD_PULL_NONE, IOMUXC_MUX_SION_ON), \
  BOARD_NETC_PIN(GPIO_EMC_B2_09_NETC_PINMUX_ETH0_RXD00), \
  BOARD_NETC_PIN(GPIO_EMC_B2_10_NETC_PINMUX_ETH0_RXD01), \
  BOARD_NETC_PIN(GPIO_EMC_B2_11_NETC_PINMUX_ETH0_RX_DV), \
  BOARD_NETC_PIN(GPIO_EMC_B2_12_NETC_PINMUX_ETH0_RX_ER) \
}

#define BOARD_ENET1_PORT1_PINS \
{ \
  BOARD_NETC_PIN(GPIO_B1_00_NETC_PINMUX_ETH1_TXD00), \
  BOARD_NETC_PIN(GPIO_B1_01_NETC_PINMUX_ETH1_TXD01), \
  BOARD_NETC_PIN(GPIO_B1_02_NETC_PINMUX_ETH1_TX_EN), \
  BOARD_NETC_PIN(GPIO_B1_03_NETC_PINMUX_ETH1_TX_CLK), \
  BOARD_NETC_PIN(GPIO_B1_04_NETC_PINMUX_ETH1_RXD00), \
  BOARD_NETC_PIN(GPIO_B1_05_NETC_PINMUX_ETH1_RXD01), \
  BOARD_NETC_PIN(GPIO_B1_06_NETC_PINMUX_ETH1_RX_DV), \
  BOARD_NETC_PIN(GPIO_B1_07_NETC_PINMUX_ETH1_TXD02), \
  BOARD_NETC_PIN(GPIO_B1_08_NETC_PINMUX_ETH1_TXD03), \
  BOARD_NETC_PIN(GPIO_B1_09_NETC_PINMUX_ETH1_RXD02), \
  BOARD_NETC_PIN(GPIO_B1_10_NETC_PINMUX_ETH1_RXD03), \
  BOARD_NETC_PIN(GPIO_B1_11_NETC_PINMUX_ETH1_RX_CLK) \
}

#define BOARD_ENET1_PORT2_PINS \
{ \
  BOARD_NETC_PIN(GPIO_EMC_B1_13_NETC_PINMUX_ETH2_RX_DV), \
  BOARD_NETC_PIN(GPIO_EMC_B1_16_NETC_PINMUX_ETH2_RXD00), \
  BOARD_NETC_PIN(GPIO_EMC_B1_17_NETC_PINMUX_ETH2_RXD01), \
  BOARD_NETC_PIN(GPIO_EMC_B1_21_NETC_PINMUX_ETH2_RX_CLK), \
  BOARD_NETC_PIN(GPIO_EMC_B1_22_NETC_PINMUX_ETH2_RXD02), \
  BOARD_NETC_PIN(GPIO_EMC_B1_23_NETC_PINMUX_ETH2_RXD03), \
  BOARD_NETC_PIN(GPIO_EMC_B1_24_NETC_PINMUX_ETH2_TXD03), \
  BOARD_NETC_PIN(GPIO_EMC_B1_25_NETC_PINMUX_ETH2_TXD02), \
  BOARD_NETC_PIN(GPIO_EMC_B1_26_NETC_PINMUX_ETH2_TXD01), \
  BOARD_NETC_PIN(GPIO_EMC_B1_27_NETC_PINMUX_ETH2_TXD00), \
  BOARD_NETC_PIN(GPIO_EMC_B1_28_NETC_PINMUX_ETH2_TX_EN), \
  BOARD_NETC_PIN(GPIO_EMC_B1_29_NETC_PINMUX_ETH2_TX_CLK) \
}

#define BOARD_ENET1_PORT3_PINS \
{ \
  BOARD_NETC_PIN(GPIO_EMC_B1_00_NETC_PINMUX_ETH3_TXD03), \
  BOARD_NETC_PIN(GPIO_EMC_B1_01_NETC_PINMUX_ETH3_TXD02), \
  BOARD_NETC_PIN(GPIO_EMC_B1_02_NETC_PINMUX_ETH3_RX_CLK), \
  BOARD_NETC_PIN(GPIO_EMC_B1_03_NETC_PINMUX_ETH3_RXD03), \
  BOARD_NETC_PIN(GPIO_EMC_B1_04_NETC_PINMUX_ETH3_RXD02), \
  BOARD_NETC_PIN(GPIO_EMC_B1_05_NETC_PINMUX_ETH3_TXD00), \
  BOARD_NETC_PIN(GPIO_EMC_B1_06_NETC_PINMUX_ETH3_TXD01), \
  BOARD_NETC_PIN(GPIO_EMC_B1_07_NETC_PINMUX_ETH3_TX_EN), \
  BOARD_NETC_PIN(GPIO_EMC_B1_08_NETC_PINMUX_ETH3_TX_CLK), \
  BOARD_NETC_PIN(GPIO_EMC_B1_09_NETC_PINMUX_ETH3_RXD00), \
  BOARD_NETC_PIN(GPIO_EMC_B1_10_NETC_PINMUX_ETH3_RXD01), \
  BOARD_NETC_PIN(GPIO_EMC_B1_11_NETC_PINMUX_ETH3_RX_DV) \
}

#define IOMUX_NETC_RESET_DEFAULT (IOMUXC_PAD_ODE | IOMUXC_PAD_PULL_DOWN)

#define GPIO_ENET0_PORT0_PHY_RESET \
  IOMUX_GPIO(IOMUXC_PAD_GPIO_AD_13_GPIO4_IO13, IOMUX_NETC_RESET_DEFAULT, \
             GPIO_OUTPUT | GPIO_OUTPUT_ONE | GPIO_PORT4 | GPIO_PIN13)
#define GPIO_ENET1_PORT0_PHY_RESET \
  IOMUX_GPIO(IOMUXC_PAD_GPIO_AD_25_GPIO4_IO25, IOMUX_NETC_RESET_DEFAULT, \
             GPIO_OUTPUT | GPIO_OUTPUT_ONE | GPIO_PORT4 | GPIO_PIN25)
#define GPIO_ENET1_PORT1_PHY_RESET \
  IOMUX_GPIO(IOMUXC_PAD_GPIO_B1_13_GPIO6_IO13, IOMUX_NETC_RESET_DEFAULT, \
             GPIO_OUTPUT | GPIO_OUTPUT_ONE | GPIO_PORT6 | GPIO_PIN13)
#define GPIO_ENET1_PORT2_PHY_RESET \
  IOMUX_GPIO(IOMUXC_PAD_GPIO_AD_28_GPIO4_IO28, IOMUX_NETC_RESET_DEFAULT, \
             GPIO_OUTPUT | GPIO_OUTPUT_ONE | GPIO_PORT4 | GPIO_PIN28)
#define GPIO_ENET1_PORT3_PHY_RESET \
  IOMUX_GPIO(IOMUXC_PAD_GPIO_B2_01_GPIO6_IO15, IOMUX_NETC_RESET_DEFAULT, \
             GPIO_OUTPUT | GPIO_OUTPUT_ONE | GPIO_PORT6 | GPIO_PIN15)

/* Leave PHY0/4 LED1/PHYAD1 straps under control of the external pull-ups. */

#define GPIO_ENET0_PHYAD1 \
  IOMUX_GPIO(IOMUXC_PAD_GPIO_AD_16_GPIO4_IO16, 0, \
             GPIO_INPUT | GPIO_PORT4 | GPIO_PIN16)
#define GPIO_ENET4_PHYAD1 \
  IOMUX_GPIO(IOMUXC_PAD_GPIO_AD_17_GPIO4_IO17, 0, \
             GPIO_INPUT | GPIO_PORT4 | GPIO_PIN17)

#if defined(CONFIG_IMXRT_NETC_ENETC0) && defined(CONFIG_IMXRT_LPUART3)
#  error "ENETC0 PHY reset shares GPIO_AD_13 with LPUART3 TX"
#endif

/* Buttons ******************************************************************/

/* The MIMXRT1180-EVK has one general purpose user button, SW8 ("GPIO INT
 * BUTTON"), on GPIO_AON_04 = RGPIO1.4.  The button shorts the pin to GND
 * when pressed.  The external pull-up (R2154) is not fitted, so the
 * internal pull-up is used.
 */

#define BUTTON_SW8        0
#define NUM_BUTTONS       1

#define BUTTON_SW8_BIT    (1 << BUTTON_SW8)

#define IOMUX_SW_DEFAULT  (IOMUXC_PAD_PUE | IOMUXC_PAD_PUS)

#define GPIO_SW8          (IOMUX_GPIO(IOMUXC_PAD_GPIO_AON_04_GPIO1_IO04, \
                                      IOMUX_SW_DEFAULT,                 \
                                      GPIO_INTERRUPT |                  \
                                      GPIO_PORT1 | GPIO_PIN4) |         \
                           GPIO_INTBOTH_EDGES)

#endif /* __BOARDS_ARM_IMXRT_IMXRT1180_EVK_INCLUDE_BOARD_H */
