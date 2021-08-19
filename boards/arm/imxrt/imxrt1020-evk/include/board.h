/****************************************************************************
 * boards/arm/imxrt/imxrt1020-evk/include/board.h
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

#ifndef __BOARDS_ARM_IMXRT1020_EVK_INCLUDE_BOARD_H
#define __BOARDS_ARM_IMXRT1020_EVK_INCLUDE_BOARD_H

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

/* Set Sys PLL (PLL2) to  fOut    = (24Mhz * SYS_PLL_DIV_SELECT) /
 *                                   ARM_PODF_DIVISOR
 *                        528Mhz  = (24Mhz * SYS_PLL_DIV_SELECT) /
 *                                   ARM_PODF_DIVISOR
 *                        SYS_PLL_DIV_SELECT = 22
 *                        SYS_PODF_DIVISOR   = 1
 *                        528Mhz  = (24Mhz * 22) / 1
 *
 *     AHB_CLOCK_ROOT             = PLL6fOut / IMXRT_AHB_PODF_DIVIDER
 *     1Hz to 500 MHz             = MHz / IMXRT_ARM_CLOCK_DIVIDER
 *                        IMXRT_ARM_CLOCK_DIVIDER = 1
 *                        500Mhz  = 500Mhz / 1
 *
 *     PRE_PERIPH_CLK_SEL         = PRE_PERIPH_CLK_SEL_PLL6
 *     PERIPH_CLK                 = 500Mhz
 *
 *     IPG_CLOCK_ROOT             = AHB_CLOCK_ROOT / IMXRT_IPG_PODF_DIVIDER
 *                       IMXRT_IPG_PODF_DIVIDER = 4
 *                       125Mhz = 500Mhz / 4
 *
 *     PERCLK_CLOCK_ROOT          = IPG_CLOCK_ROOT /
 *                                  IMXRT_PERCLK_PODF_DIVIDER
 *                       IMXRT_PERCLK_PODF_DIVIDER = 2
 *                        62.5Mhz = 125Mhz / 2
 *
 *     SEMC_CLK_ROOT              = PERIPH_CLK / IMXRT_SEMC_PODF_DIVIDER
 *                       IMXRT_SEMC_PODF_DIVIDER = 4
 *                       125Mhz   = 500 / 4
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
#define BOARD_CPU_FREQUENCY        500000000U

#define IMXRT_PRE_PERIPH_CLK_SEL   CCM_CBCMR_PRE_PERIPH_CLK_SEL_PLL6
#define IMXRT_PERIPH_CLK_SEL       CCM_CBCDR_PERIPH_CLK_SEL_PRE_PERIPH
#define IMXRT_ARM_PODF_DIVIDER     1
#define IMXRT_AHB_PODF_DIVIDER     1
#define IMXRT_IPG_PODF_DIVIDER     4
#define IMXRT_PERCLK_CLK_SEL       CCM_CSCMR1_PERCLK_CLK_SEL_IPG_CLK_ROOT
#define IMXRT_PERCLK_PODF_DIVIDER  2
#define IMXRT_SEMC_PODF_DIVIDER    4

#define IMXRT_LPSPI_CLK_SELECT     CCM_CBCMR_LPSPI_CLK_SEL_PLL3_PFD0
#define IMXRT_LSPI_PODF_DIVIDER    8

#define IMXRT_LPI2C_CLK_SELECT     CCM_CSCDR2_LPI2C_CLK_SEL_PLL3_60M
#define IMXRT_LSI2C_PODF_DIVIDER   5

#define IMXRT_USDHC1_CLK_SELECT    CCM_CSCMR1_USDHC1_CLK_SEL_PLL2_PFD0
#define IMXRT_USDHC1_PODF_DIVIDER  1
#define IMXRT_USDHC2_CLK_SELECT    CCM_CSCMR1_USDHC2_CLK_SEL_PLL2_PFD0
#define IMXRT_USDHC2_PODF_DIVIDER  4

#define IMXRT_SYS_PLL_DIV_SELECT   CCM_ANALOG_PLL_SYS_DIV_SELECT_22
#define IMXRT_USB1_PLL_DIV_SELECT  CCM_ANALOG_PLL_USB1_DIV_SELECT_20
#define IMXRT_AUDIO_PLL_DIV_SELECT (45)

/* Define this to enable tracing */

#if 0
#  define IMXRT_TRACE_PODF_DIVIDER 1
#  define IMXRT_TRACE_CLK_SELECT   CCM_CBCMR_TRACE_CLK_SEL_PLL2_PFD0
#endif

/* LED definitions **********************************************************/

/* There is one user accessible LED status indicator located on the 1020-EVK.
 * The function of the LEDs include:
 *
 * D3: Power (Green) & Overpower (Red)
 * D5: User LED (Green) GPIO_AD_B0_05
 * D15: RST LED (Red)
 *
 * This LED is not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.  In that case, the usage by the board port is defined in
 * include/board.h and src/imxrt_autoleds.c. The LED is used to encode
 * OS-related events as documented in board.h
 *
 *   -------------------- ----------------------------- --------
 *   SYMBOL                   Meaning                   USERLED
 *   -------------------- ----------------------------- --------
 */

#define LED_STARTED       0  /* NuttX has been started  OFF    */
#define LED_HEAPALLOCATE  1  /* Heap has been allocated OFF    */
#define LED_IRQSENABLED   2  /* Interrupts enabled      OFF    */
#define LED_STACKCREATED  3  /* Idle stack created      ON     */
#define LED_INIRQ         4  /* In an interrupt         N/C    */
#define LED_SIGNAL        5  /* In a signal handler     N/C    */
#define LED_ASSERTION     6  /* An assertion failed     N/C    */
#define LED_PANIC         7  /* The system has crashed  FLASH  */
#undef  LED_IDLE             /* Not used                       */

/* The intention is that if the LED is statically on, NuttX has successfully
 * booted and is, apparently, running normally.  If the LED is flashing at
 * approximately 2Hz, then a fatal error has been detected and the system has
 * halted.
 */

/* LED index values for use with board_userled() */

#define BOARD_USERLED      0
#define BOARD_NLEDS        1

/* LED bits for use with board_userled_all() */

#define BOARD_USERLED_BIT  (1 << BOARD_USERLED)

/* Which device the SD card appears on */

#define BOARD_USDHC_SD_ID               (0)

/* Button definitions *******************************************************/

/* The IMXRT board has three external buttons
 *
 * 1. SW2 (IRQ88, ONOFF)  Not on a GPIO, No muxing
 * 2. SW3 (IRQ88, POR)    Not on a GPIO, No muxing
 * 2. SW4 (IRQ88, USER)   Wakeup, GPIO5-0
 */

#define BUTTON_WAKE       0
#define BUTTON_WAKE_BIT    (1 << BUTTON_WAKE)

/* SDIO - Used for both Port 1 & 2 ******************************************/

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

/* Pinning ******************************************************************/

/* Make sure these entries match to allow interrupts to be present */

#define GPIO_ENET_GRP       IMXRT_GPIO1_16_31_IRQ

#ifndef GPIO_ENET_GRP
#  ifdef CONFIG_IMXRT_ENET
#    error GPIO_ENET_IRQ Host IRQ not defined!
#  endif
#endif

#define GPIO_ENET_MDIO       GPIO_ENET_MDIO_1 | IOMUX_ENET_MDIO_DEFAULT
#define GPIO_ENET_MDC        GPIO_ENET_MDC_1 | IOMUX_ENET_MDC_DEFAULT
#define GPIO_ENET_RX_EN      GPIO_ENET_RX_EN_1 | IOMUX_ENET_EN_DEFAULT /* AKA CRS_DV */
#define GPIO_ENET_TX_EN      GPIO_ENET_TX_EN_1 | IOMUX_ENET_EN_DEFAULT
#define GPIO_ENET_TX_CLK     GPIO_ENET_REF_CLK_2 | IOMUX_ENET_TX_CLK_DEFAULT
#define GPIO_ENET_RX_DATA00  GPIO_ENET_RX_DATA00_2 | IOMUX_ENET_DATA_DEFAULT
#define GPIO_ENET_RX_DATA01  GPIO_ENET_RX_DATA01_2 | IOMUX_ENET_DATA_DEFAULT
#define GPIO_ENET_TX_DATA00  GPIO_ENET_TX_DATA00_2 | IOMUX_ENET_DATA_DEFAULT
#define GPIO_ENET_TX_DATA01  GPIO_ENET_TX_DATA01_2 | IOMUX_ENET_DATA_DEFAULT

/* LPI2Cs *******************************************************************/

#define GPIO_LPI2C1_SDA   GPIO_LPI2C1_SDA_1|IOMUX_I2C_DEFAULT /* AD_B1_15 */
#define GPIO_LPI2C1_SCL   GPIO_LPI2C1_SCL_1|IOMUX_I2C_DEFAULT /* AD_B1_14 */
#define GPIO_LPI2C4_SDA   GPIO_LPI2C4_SDA_1|IOMUX_I2C_DEFAULT /* SD_B1_02 */
#define GPIO_LPI2C4_SCL   GPIO_LPI2C4_SCL_1|IOMUX_I2C_DEFAULT /* SD_B1_03 */

/* LPSPI ********************************************************************/

#define GPIO_LPSPI1_MOSI GPIO_LPSPI1_SDO_1|IOMUX_LPSPI_DEFAULT  /* AD_B0_12 */
#define GPIO_LPSPI1_MISO GPIO_LPSPI1_SDI_1|IOMUX_LPSPI_DEFAULT  /* AD_B0_13 */
#define GPIO_LPSPI1_SCK  GPIO_LPSPI1_SCK_1|IOMUX_LPSPI_DEFAULT  /* AD_B0_10 */
#define GPIO_LPSPI1_PCS  GPIO_LPSPI1_PCS0_1|IOMUX_LPSPI_DEFAULT /* AD_B0_11 */

/* LPUARTS Disambiguation ***************************************************/

#define GPIO_LPUART1_RX   GPIO_LPUART1_RX_1|IOMUX_UART_DEFAULT /* AD_B0_07 */
#define GPIO_LPUART1_TX   GPIO_LPUART1_TX_1|IOMUX_UART_DEFAULT /* AD_B0_06 */
#define GPIO_LPUART2_RX   GPIO_LPUART2_RX_2|IOMUX_UART_DEFAULT /* AD_B1_09 */
#define GPIO_LPUART2_TX   GPIO_LPUART2_TX_2|IOMUX_UART_DEFAULT /* AD_B1_08 */

/* SDIO *********************************************************************/

#define PIN_USDHC1_D0       (GPIO_USDHC1_DATA0_1 | IOMUX_USDHC1_DATAX_DEFAULT) /* SD_B0_04 */
#define PIN_USDHC1_D1       (GPIO_USDHC1_DATA1_1 | IOMUX_USDHC1_DATAX_DEFAULT) /* SD_B0_05 */
#define PIN_USDHC1_D2       (GPIO_USDHC1_DATA2_1 | IOMUX_USDHC1_DATAX_DEFAULT) /* SD_B0_00 */
#define PIN_USDHC1_D3       (GPIO_USDHC1_DATA3_1 | IOMUX_USDHC1_DATAX_DEFAULT) /* SD_B0_01 */
#define PIN_USDHC1_DCLK     (GPIO_USDHC1_CLK_1   | IOMUX_USDHC1_CLK_DEFAULT)   /* SD_B0_03 */
#define PIN_USDHC1_CMD      (GPIO_USDHC1_CMD_1   | IOMUX_USDHC1_CMD_DEFAULT)   /* SD_B0_02 */

/* N.B. This is not using a USDHC CD_B input but a regular GPIO.  The
 * post-fix _GPIO enables GPIO testing logic in the USDHC driver.
 */

#define PIN_USDHC1_CD_GPIO  (IOMUX_VSD_DEFAULT | \
                             GPIO_PORT3 | GPIO_PIN19 )                        /* SD_B0_06 */

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

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_IMXRT1020_EVK_INCLUDE_BOARD_H */
