/****************************************************************************
 * boards/arm/imxrt/imxrt1050-evk/include/board.h
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

#ifndef __BOARDS_ARM_IMXRT_IMXRT1050_EVK_INCLUDE_BOARD_H
#define __BOARDS_ARM_IMXRT_IMXRT1050_EVK_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/* Do not include i.MXRT header files here */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* Set VDD_SOC to 1.25V */

#define IMXRT_VDD_SOC (0x12)

/* Set Arm PLL (PLL1) to  fOut    = (24Mhz * ARM_PLL_DIV_SELECT/2) /
 *                                  ARM_PODF_DIVISOR
 *                        600Mhz  = (24Mhz * ARM_PLL_DIV_SELECT/2) /
 *                                  ARM_PODF_DIVISOR
 *                        ARM_PLL_DIV_SELECT = 100
 *                        ARM_PODF_DIVISOR   = 2
 *                        600Mhz  = (24Mhz * 100/2) / 2
 *
 *     AHB_CLOCK_ROOT             = PLL1fOut / IMXRT_AHB_PODF_DIVIDER
 *     1Hz to 600 MHz             = 600Mhz / IMXRT_ARM_CLOCK_DIVIDER
 *                        IMXRT_ARM_CLOCK_DIVIDER = 1
 *                        600Mhz  = 600Mhz / 1
 *
 *     PRE_PERIPH_CLK_SEL         = PRE_PERIPH_CLK_SEL_PLL1
 *     PERIPH_CLK_SEL             = 1 (0 select PERIPH_CLK2_PODF,
 *                                     1 select PRE_PERIPH_CLK_SEL_PLL1)
 *     PERIPH_CLK                 = 600Mhz
 *
 *     IPG_CLOCK_ROOT             = AHB_CLOCK_ROOT / IMXRT_IPG_PODF_DIVIDER
 *                       IMXRT_IPG_PODF_DIVIDER = 4
 *                       150Mhz = 600Mhz / 4
 *
 *     PERCLK_CLOCK_ROOT          = IPG_CLOCK_ROOT /
 *                                  IMXRT_PERCLK_PODF_DIVIDER
 *                       IMXRT_PERCLK_PODF_DIVIDER = 9
 *                       16.6Mhz  = 150Mhz / 9
 *
 *     SEMC_CLK_ROOT              = 600Mhz / IMXRT_SEMC_PODF_DIVIDER
 *                                           (labeled AIX_PODF in 18.2)
 *                       IMXRT_SEMC_PODF_DIVIDER = 8
 *                       75Mhz    = 600Mhz / 8
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
 * These clock frequencies can be verified via the CCM_CLKO1 pin and sending
 * the appropriate clock to it with something like;
 *
 *   putreg32( <Clk number> | CCM_CCOSR_CLKO1_EN ,   IMXRT_CCM_CCOSR);
 *   imxrt_config_gpio(GPIO_CCM_CLKO1);
 */

#define BOARD_XTAL_FREQUENCY       24000000
#define IMXRT_PRE_PERIPH_CLK_SEL   CCM_CBCMR_PRE_PERIPH_CLK_SEL_PLL1
#define IMXRT_PERIPH_CLK_SEL       CCM_CBCDR_PERIPH_CLK_SEL_PRE_PERIPH
#define IMXRT_ARM_PLL_DIV_SELECT   100
#define IMXRT_ARM_PODF_DIVIDER     2
#define IMXRT_AHB_PODF_DIVIDER     1
#define IMXRT_IPG_PODF_DIVIDER     4
#define IMXRT_PERCLK_CLK_SEL       CCM_CSCMR1_PERCLK_CLK_SEL_IPG_CLK_ROOT
#define IMXRT_PERCLK_PODF_DIVIDER  9
#define IMXRT_SEMC_PODF_DIVIDER    8
#define IMXRT_LPSPI_CLK_SELECT     CCM_CBCMR_LPSPI_CLK_SEL_PLL3_PFD0
#define IMXRT_LSPI_PODF_DIVIDER    8

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

/* Define this to enable tracing */

#if 0
#  define IMXRT_TRACE_PODF_DIVIDER 1
#  define IMXRT_TRACE_CLK_SELECT   CCM_CBCMR_TRACE_CLK_SEL_PLL2_PFD0
#endif

/* LED definitions **********************************************************/

/* There are four LED status indicators located on the EVK Board.
 *  The functions of these LEDs include:
 *
 *   - Main Power Supply(D3)
 *     Green: DC 5V main supply is normal.
 *     Red:   J2 input voltage is over 5.6V.
 *     Off:   The board is not powered.
 *   - Reset RED LED(D15)
 *   - OpenSDA LED(D16)
 *   - USER LED(D18)
 *
 * Only a single LED, D18, is under software control.
 */

/* LED index values for use with board_userled() */

#define BOARD_USERLED     0
#define BOARD_NLEDS       1

/* LED bits for use with board_userled_all() */

#define BOARD_USERLED_BIT (1 << BOARD_USERLED)

/* This LED is not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.  In that case, the usage by the board port is defined in
 * include/board.h and src/imxrt_autoleds.c. The LED is used to encode
 * OS-related events as follows:
 *
 *   -------------------- ----------------------------- ------
 *   SYMBOL                   Meaning                   LED
 *   -------------------- ----------------------------- ------
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

/* Button definitions *******************************************************/

/* The IMXRT board has one external user button
 *
 * 1. SW8 (IRQ88)   GPIO5-00
 */

#define BUTTON_SW8        0

#define BUTTON_SW8_BIT    (1 << BUTTON_SW8)

/* SDIO *********************************************************************/

/* Pin drive characteristics - drive strength in particular may need tuning
 * for specific boards, but has been checked by scope on the EVKB to make
 * sure shapes are square with minimal ringing.
 */

#define PIN_USDHC1_D0       (GPIO_USDHC1_DATA0 | IOMUX_USDHC1_DATAX_DEFAULT)
#define PIN_USDHC1_D1       (GPIO_USDHC1_DATA1 | IOMUX_USDHC1_DATAX_DEFAULT)
#define PIN_USDHC1_D2       (GPIO_USDHC1_DATA2 | IOMUX_USDHC1_DATAX_DEFAULT)
#define PIN_USDHC1_D3       (GPIO_USDHC1_DATA3 | IOMUX_USDHC1_DATAX_DEFAULT)
#define PIN_USDHC1_DCLK     (GPIO_USDHC1_CLK   | IOMUX_USDHC1_CLK_DEFAULT)
#define PIN_USDHC1_CMD      (GPIO_USDHC1_CMD   | IOMUX_USDHC1_CMD_DEFAULT)
#define PIN_USDHC1_CD       (GPIO_USDHC1_CD_2  | IOMUX_USDHC1_CLK_DEFAULT)

/* 386 KHz for initial inquiry stuff */

#define BOARD_USDHC_IDMODE_PRESCALER    USDHC_SYSCTL_SDCLKFS_DIV256
#define BOARD_USDHC_IDMODE_DIVISOR      USDHC_SYSCTL_DVS_DIV(2)

/* 24.8MHz for other modes */

#define BOARD_USDHC_MMCMODE_PRESCALER   USDHC_SYSCTL_SDCLKFS_DIV8
#define BOARD_USDHC_MMCMODE_DIVISOR     USDHC_SYSCTL_DVS_DIV(1)

#define BOARD_USDHC_SD1MODE_PRESCALER   USDHC_SYSCTL_SDCLKFS_DIV8
#define BOARD_USDHC_SD1MODE_DIVISOR     USDHC_SYSCTL_DVS_DIV(1)

#define BOARD_USDHC_SD4MODE_PRESCALER   USDHC_SYSCTL_SDCLKFS_DIV8
#define BOARD_USDHC_SD4MODE_DIVISOR     USDHC_SYSCTL_DVS_DIV(1)

/* LCD **********************************************************************/

#ifdef CONFIG_IMXRT_LCD
/* LCD controller */

#  define GPIO_LCD_DATA23    (GPIO_LCD_DATA23_1 | IOMUX_LCD_DEFAULT)
#  define GPIO_LCD_DATA22    (GPIO_LCD_DATA22_1 | IOMUX_LCD_DEFAULT)
#  define GPIO_LCD_DATA21    (GPIO_LCD_DATA21_1 | IOMUX_LCD_DEFAULT)
#  define GPIO_LCD_DATA20    (GPIO_LCD_DATA20_1 | IOMUX_LCD_DEFAULT)
#  define GPIO_LCD_DATA19    (GPIO_LCD_DATA19_1 | IOMUX_LCD_DEFAULT)
#  define GPIO_LCD_DATA18    (GPIO_LCD_DATA18_1 | IOMUX_LCD_DEFAULT)
#  define GPIO_LCD_DATA17    (GPIO_LCD_DATA17_1 | IOMUX_LCD_DEFAULT)
#  define GPIO_LCD_DATA16    (GPIO_LCD_DATA16_1 | IOMUX_LCD_DEFAULT)
#  define GPIO_LCD_DATA15    (GPIO_LCD_DATA15_1 | IOMUX_LCD_DEFAULT)
#  define GPIO_LCD_DATA14    (GPIO_LCD_DATA14_1 | IOMUX_LCD_DEFAULT)
#  define GPIO_LCD_DATA13    (GPIO_LCD_DATA13_1 | IOMUX_LCD_DEFAULT)
#  define GPIO_LCD_DATA12    (GPIO_LCD_DATA12_1 | IOMUX_LCD_DEFAULT)
#  define GPIO_LCD_DATA11    (GPIO_LCD_DATA11_1 | IOMUX_LCD_DEFAULT)
#  define GPIO_LCD_DATA10    (GPIO_LCD_DATA10_1 | IOMUX_LCD_DEFAULT)
#  define GPIO_LCD_DATA09    (GPIO_LCD_DATA09_1 | IOMUX_LCD_DEFAULT)
#  define GPIO_LCD_DATA08    (GPIO_LCD_DATA08_1 | IOMUX_LCD_DEFAULT)
#  define GPIO_LCD_DATA07    (GPIO_LCD_DATA07_1 | IOMUX_LCD_DEFAULT)
#  define GPIO_LCD_DATA06    (GPIO_LCD_DATA06_1 | IOMUX_LCD_DEFAULT)
#  define GPIO_LCD_DATA05    (GPIO_LCD_DATA05_1 | IOMUX_LCD_DEFAULT)
#  define GPIO_LCD_DATA04    (GPIO_LCD_DATA04_1 | IOMUX_LCD_DEFAULT)
#  define GPIO_LCD_DATA03    (GPIO_LCD_DATA03_1 | IOMUX_LCD_DEFAULT)
#  define GPIO_LCD_DATA02    (GPIO_LCD_DATA02_1 | IOMUX_LCD_DEFAULT)
#  define GPIO_LCD_DATA01    (GPIO_LCD_DATA01_1 | IOMUX_LCD_DEFAULT)
#  define GPIO_LCD_DATA00    (GPIO_LCD_DATA00_1 | IOMUX_LCD_DEFAULT)
#  define GPIO_LCD_ENABLE    (GPIO_LCD_ENABLE_1 | IOMUX_LCD_DEFAULT)
#  define GPIO_LCD_HSYNC     (GPIO_LCD_HSYNC_1 | IOMUX_LCD_DEFAULT)
#  define GPIO_LCD_VSYNC     (GPIO_LCD_VSYNC_1 | IOMUX_LCD_DEFAULT)
#  define GPIO_LCD_CLK       (GPIO_LCD_CLK_1 | IOMUX_LCD_DEFAULT)
#endif

/* ETH Disambiguation *******************************************************/

#define GPIO_ENET_TX_DATA00  (GPIO_ENET_TX_DATA00_1| \
                              IOMUX_ENET_DATA_DEFAULT)                    /* GPIO_B1_07 */
#define GPIO_ENET_TX_DATA01  (GPIO_ENET_TX_DATA01_1| \
                              IOMUX_ENET_DATA_DEFAULT)                    /* GPIO_B1_08 */
#define GPIO_ENET_RX_DATA00  (GPIO_ENET_RX_DATA00_1| \
                              IOMUX_ENET_DATA_DEFAULT)                    /* GPIO_B1_04 */
#define GPIO_ENET_RX_DATA01  (GPIO_ENET_RX_DATA01_1| \
                              IOMUX_ENET_DATA_DEFAULT)                    /* GPIO_B1_05 */
#define GPIO_ENET_MDIO       (GPIO_ENET_MDIO_3|IOMUX_ENET_MDIO_DEFAULT)   /* GPIO_EMC_41 */
#define GPIO_ENET_MDC        (GPIO_ENET_MDC_3|IOMUX_ENET_MDC_DEFAULT)     /* GPIO_EMC_40 */
#define GPIO_ENET_RX_EN      (GPIO_ENET_RX_EN_1|IOMUX_ENET_EN_DEFAULT)    /* GPIO_B1_06 */
#define GPIO_ENET_RX_ER      (GPIO_ENET_RX_ER_1|IOMUX_ENET_RXERR_DEFAULT) /* GPIO_B1_11 */
#define GPIO_ENET_TX_CLK     (GPIO_ENET_REF_CLK_2|\
                              IOMUX_ENET_TX_CLK_DEFAULT)                  /* GPIO_B1_10 */
#define GPIO_ENET_TX_EN      (GPIO_ENET_TX_EN_1|IOMUX_ENET_EN_DEFAULT)    /* GPIO_B1_09 */

/* PIO Disambiguation *******************************************************/

/* LPUARTs
 *
 * Virtual console port provided by OpenSDA on UART1 and
 * Arduino RS-232 Shield on UART3.
 *
 */

#define GPIO_LPUART1_RX   (GPIO_LPUART1_RX_1|IOMUX_UART_DEFAULT) /* GPIO_AD_B0_13 */
#define GPIO_LPUART1_TX   (GPIO_LPUART1_TX_1|IOMUX_UART_DEFAULT) /* GPIO_AD_B0_12 */
#define GPIO_LPUART3_RX   (GPIO_LPUART3_RX_1|IOMUX_UART_DEFAULT) /* GPIO_AD_B1_07 */
#define GPIO_LPUART3_TX   (GPIO_LPUART3_TX_1|IOMUX_UART_DEFAULT) /* GPIO_AD_B1_06 */

/* LPI2Cs
 *
 * Arduino Connector LPI2C1 and audio/gyro IO on LPI2C3.
 */

#define GPIO_LPI2C1_SDA   (GPIO_LPI2C1_SDA_2|IOMUX_LPI2C_DEFAULT) /* GPIO_AD_B1_01 */
#define GPIO_LPI2C1_SCL   (GPIO_LPI2C1_SCL_2|IOMUX_LPI2C_DEFAULT) /* GPIO_AD_B1_00 */
#define GPIO_LPI2C3_SDA   (GPIO_LPI2C3_SDA_2|IOMUX_LPI2C_DEFAULT) /* GPIO_AD_B1_01 */
#define GPIO_LPI2C3_SCL   (GPIO_LPI2C3_SCL_2|IOMUX_LPI2C_DEFAULT) /* GPIO_AD_B1_00 */

/* LPSPI
 *
 * Arduino Connector
 *
 *   J24 D09   GPIO_AD_B0_02  LPSPI3_SDI
 *   J24 D14   GPIO_AD_B0_01  LPSPI3_SDO
 *   J24 D15   GPIO_AD_B0_00  LPSPI3_SCK
 */

#define GPIO_LPSPI3_SCK   (GPIO_LPSPI3_SCK_2|IOMUX_LPSPI_DEFAULT) /* GPIO_AD_B0_00 */
#define GPIO_LPSPI3_MISO  (GPIO_LPSPI3_SDI_2|IOMUX_LPSPI_DEFAULT) /* GPIO_AD_B0_02 */
#define GPIO_LPSPI3_MOSI  (GPIO_LPSPI3_SDO_2|IOMUX_LPSPI_DEFAULT) /* GPIO_AD_B0_01 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

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

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_IMXRT_IMXRT1050_EVK_INCLUDE_BOARD_H */
