/****************************************************************************
 * boards/arm/imxrt/ddcnc-ec300/include/board.h
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

#ifndef __BOARDS_ARM_IMXRT_DDCNC_EC300_INCLUDE_BOARD_H
#define __BOARDS_ARM_IMXRT_DDCNC_EC300_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/* Do not include i.MXRT header files here */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* The Digital Dream EC300 is fitted with a MIMXRT1052CVL5B (industrial
 * grade part), which is rated for a maximum core frequency of 528MHz.
 *
 * Set VDD_SOC to 1.25V
 */

#define IMXRT_VDD_SOC (0x12)

/* Set Arm PLL (PLL1) to  fOut    = (24Mhz * ARM_PLL_DIV_SELECT/2) /
 *                                  ARM_PODF_DIVISOR
 *                        528Mhz  = (24Mhz * ARM_PLL_DIV_SELECT/2) /
 *                                  ARM_PODF_DIVISOR
 *                        ARM_PLL_DIV_SELECT = 88
 *                        ARM_PODF_DIVISOR   = 2
 *                        528Mhz  = (24Mhz * 88/2) / 2
 *
 *     AHB_CLOCK_ROOT             = PLL1fOut / IMXRT_AHB_PODF_DIVIDER
 *     1Hz to 528 MHz             = 528Mhz / IMXRT_ARM_CLOCK_DIVIDER
 *                        IMXRT_ARM_CLOCK_DIVIDER = 1
 *                        528Mhz  = 528Mhz / 1
 *
 *     PRE_PERIPH_CLK_SEL         = PRE_PERIPH_CLK_SEL_PLL1
 *     PERIPH_CLK_SEL             = 1 (0 select PERIPH_CLK2_PODF,
 *                                     1 select PRE_PERIPH_CLK_SEL_PLL1)
 *     PERIPH_CLK                 = 528Mhz
 *
 *     IPG_CLOCK_ROOT             = AHB_CLOCK_ROOT / IMXRT_IPG_PODF_DIVIDER
 *                       IMXRT_IPG_PODF_DIVIDER = 4
 *                       132Mhz = 528Mhz / 4
 *
 *     PERCLK_CLOCK_ROOT          = IPG_CLOCK_ROOT /
 *                                  IMXRT_PERCLK_PODF_DIVIDER
 *                       IMXRT_PERCLK_PODF_DIVIDER = 9
 *                       14.6Mhz  = 132Mhz / 9
 *
 *     SEMC_CLK_ROOT              = 528Mhz / IMXRT_SEMC_PODF_DIVIDER
 *                                           (labeled AIX_PODF in 18.2)
 *                       IMXRT_SEMC_PODF_DIVIDER = 8
 *                       66Mhz    = 528Mhz / 8
 *
 * Set Sys PLL (PLL2) to  fOut    = (24Mhz * (20+(2*(DIV_SELECT)))
 *                        528Mhz  = (24Mhz * (20+(2*(1)))
 *
 * Set USB1 PLL (PLL3) to fOut    = (24Mhz * 20)
 *                         480Mhz = (24Mhz * 20)
 *
 * Set LPSPI PLL3 PFD0 to fOut    = (480Mhz / 12 * 18)
 *                        720Mhz  = (480Mhz / 12 * 18)
 *                         90Mhz  = (720Mhz / LSPI_PODF_DIVIDER)
 *
 * Set LPI2C PLL3 / 8 to   fOut   = (480Mhz / 8)
 *                         60Mhz  = (480Mhz / 8)
 *                         12Mhz  = (60Mhz / LSPI_PODF_DIVIDER)
 *
 * The ENET PLL (PLL6) is configured by the common i.MX RT clock logic to
 * generate the 50MHz RMII reference clock that is driven out on the
 * ENET_REF_CLK pin (GPIO_B1_10) to the LAN8720A PHY.
 */

#define BOARD_XTAL_FREQUENCY       24000000
#define IMXRT_PRE_PERIPH_CLK_SEL   CCM_CBCMR_PRE_PERIPH_CLK_SEL_PLL1
#define IMXRT_PERIPH_CLK_SEL       CCM_CBCDR_PERIPH_CLK_SEL_PRE_PERIPH
#define IMXRT_ARM_PLL_DIV_SELECT   88
#define IMXRT_ARM_PODF_DIVIDER     2
#define IMXRT_AHB_PODF_DIVIDER     1
#define IMXRT_IPG_PODF_DIVIDER     4
#define IMXRT_PERCLK_CLK_SEL       CCM_CSCMR1_PERCLK_CLK_SEL_IPG_CLK_ROOT
#define IMXRT_PERCLK_PODF_DIVIDER  9
#define IMXRT_SEMC_PODF_DIVIDER    8

#define IMXRT_LPSPI_CLK_SELECT     CCM_CBCMR_LPSPI_CLK_SEL_PLL3_PFD0
#define IMXRT_LSPI_PODF_DIVIDER    8

#define IMXRT_LPI2C_CLK_SELECT     CCM_CSCDR2_LPI2C_CLK_SEL_PLL3_60M
#define IMXRT_LSI2C_PODF_DIVIDER   5

#define IMXRT_USDHC1_CLK_SELECT    CCM_CSCMR1_USDHC1_CLK_SEL_PLL2_PFD0
#define IMXRT_USDHC1_PODF_DIVIDER  2

#define IMXRT_SYS_PLL_SELECT       CCM_ANALOG_PLL_SYS_DIV_SELECT_22

#define IMXRT_USB1_PLL_DIV_SELECT  CCM_ANALOG_PLL_USB1_DIV_SELECT_20

#define BOARD_CPU_FREQUENCY \
  (BOARD_XTAL_FREQUENCY * (IMXRT_ARM_PLL_DIV_SELECT / 2)) / IMXRT_ARM_PODF_DIVIDER

/* LED definitions **********************************************************/

/* There are three LED indicators located on the EC300 front panel:
 *
 *   - POWER: Main power supply (hardware controlled)
 *   - IO:    IO power supply (hardware controlled)
 *   - CONN:  Communication status, connected to GPIO_SD_B1_00
 *            (GPIO3_IO00).  Low illuminates.
 *
 * Only the CONN LED is under software control.
 */

/* LED index values for use with board_userled() */

#define BOARD_USERLED     0
#define BOARD_NLEDS       1

/* LED bits for use with board_userled_all() */

#define BOARD_USERLED_BIT (1 << BOARD_USERLED)

/* This LED is not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.  In that case, the usage by the board port is defined in
 * include/board.h and src/imxrt_autoleds.c.  The CONN LED is used to
 * encode OS-related events as follows:
 *
 *   ------------------- ----------------------- ------
 *   SYMBOL              Meaning                 LED
 *   ------------------- ----------------------- ------
 */

#define LED_STARTED       0  /* NuttX has been started  OFF    */
#define LED_HEAPALLOCATE  0  /* Heap has been allocated OFF    */
#define LED_IRQSENABLED   0  /* Interrupts enabled      OFF    */
#define LED_STACKCREATED  1  /* Idle stack created      ON     */
#define LED_INIRQ         2  /* In an interrupt         N/C    */
#define LED_SIGNAL        2  /* In a signal handler     N/C    */
#define LED_ASSERTION     2  /* An assertion failed     N/C    */
#define LED_PANIC         3  /* The system has crashed  FLASH  */
#undef  LED_IDLE             /* Not used                       */

/* Thus if the LED is statically on, NuttX has successfully  booted and is,
 * apparently, running normally.  If the LED is flashing at approximately
 * 2Hz, then a fatal error has been detected and the system has halted.
 */

/* Ethernet *****************************************************************/

/* The EC300 uses a SMSC/Microchip LAN8720A RMII PHY.  The i.MX RT provides
 * the 50MHz reference clock to the PHY (CONFIG_IMXRT_MAC_PROVIDES_TXC).
 *
 * The PHY is strapped to answer at MDIO address 0, but the i.MX RT ENET
 * driver hard-codes address 1 for CONFIG_ETH0_PHY_LAN8720.  So the
 * CONFIG_ETH0_PHY_MULTI run-time PHY selection is used instead, which
 * allows the board to provide the PHY description below.
 */

#define BOARD_ETH0_PHY_LIST           \
  {                                   \
    MII_LAN8720_NAME,                 \
    MII_PHYID1_LAN8720,               \
    MII_PHYID2_LAN8720,               \
    MII_LAN8720_SCSR,                 \
    0,                                \
    0xffff,                           \
    MII_LAN8720_SPSCR_10MBPS,         \
    MII_LAN8720_SPSCR_100MBPS,        \
    MII_LAN8720_SPSCR_DUPLEX,         \
    22,                               \
    0,                                \
    0                                 \
  }

/* ETH Disambiguation *******************************************************/

#define GPIO_ENET_TX_DATA00  (GPIO_ENET_TX_DATA00_1| \
                              IOMUX_ENET_DATA_DEFAULT)                    /* GPIO_B1_07 */
#define GPIO_ENET_TX_DATA01  (GPIO_ENET_TX_DATA01_1| \
                              IOMUX_ENET_DATA_DEFAULT)                    /* GPIO_B1_08 */
#define GPIO_ENET_RX_DATA00  (GPIO_ENET_RX_DATA00_1| \
                              IOMUX_ENET_DATA_DEFAULT)                    /* GPIO_B1_04 */
#define GPIO_ENET_RX_DATA01  (GPIO_ENET_RX_DATA01_1| \
                              IOMUX_ENET_DATA_DEFAULT)                    /* GPIO_B1_05 */
#define GPIO_ENET_MDIO       (GPIO_ENET_MDIO_1|IOMUX_ENET_MDIO_DEFAULT)   /* GPIO_B1_15 */
#define GPIO_ENET_MDC        (GPIO_ENET_MDC_1|IOMUX_ENET_MDC_DEFAULT)     /* GPIO_B1_14 */
#define GPIO_ENET_RX_EN      (GPIO_ENET_RX_EN_1|IOMUX_ENET_EN_DEFAULT)    /* GPIO_B1_06 */
#define GPIO_ENET_RX_ER      (GPIO_ENET_RX_ER_1|IOMUX_ENET_RXERR_DEFAULT) /* GPIO_B1_11 */
#define GPIO_ENET_TX_CLK     (GPIO_ENET_REF_CLK_2|\
                              IOMUX_ENET_TX_CLK_DEFAULT)                  /* GPIO_B1_10 */
#define GPIO_ENET_TX_EN      (GPIO_ENET_TX_EN_1|IOMUX_ENET_EN_DEFAULT)    /* GPIO_B1_09 */

/* PIO Disambiguation *******************************************************/

/* LPUARTs
 *
 * LPUART4 is used as the debug console.  LPUART5 is connected to the
 * MPG port (NVMPG pendant).
 */

#define GPIO_LPUART4_RX   (GPIO_LPUART4_RX_1|IOMUX_UART_DEFAULT) /* GPIO_B1_01 */
#define GPIO_LPUART4_TX   (GPIO_LPUART4_TX_1|IOMUX_UART_DEFAULT) /* GPIO_B1_00 */
#define GPIO_LPUART5_RX   (GPIO_LPUART5_RX_1|IOMUX_UART_DEFAULT) /* GPIO_B1_13 */
#define GPIO_LPUART5_TX   (GPIO_LPUART5_TX_1|IOMUX_UART_DEFAULT) /* GPIO_B1_12 */

#endif /* __BOARDS_ARM_IMXRT_DDCNC_EC300_INCLUDE_BOARD_H */
