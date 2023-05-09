/****************************************************************************
 * boards/arm/imxrt/imxrt1170-evk/include/board.h
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

#ifndef __BOARDS_ARM_IMXRT_IMXRT1170_EVK_INCLUDE_BOARD_H
#define __BOARDS_ARM_IMXRT_IMXRT1170_EVK_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/* Do not include i.MXRT header files here. */

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
 *     PRECLK_CLOCK_ROOT          = IPG_CLOCK_ROOT /
 *                                  IMXRT_PERCLK_PODF_DIVIDER
 *                       IMXRT_PERCLK_PODF_DIVIDER = 9
 *                       16.6Mhz  = 150Mhz / 9
 *
 *     SEMC_CLK_ROOT              = 600Mhz / IMXRT_SEMC_PODF_DIVIDER
 *                                  (labeled AIX_PODF in 18.2)
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

#define IMXRT_LPI2C_CLK_SELECT     CCM_CSCDR2_LPI2C_CLK_SEL_PLL3_60M
#define IMXRT_LSI2C_PODF_DIVIDER   5

#define IMXRT_CAN_CLK_SELECT       CCM_CSCMR2_CAN_CLK_SEL_PLL3_SW_80
#define IMXRT_CAN_PODF_DIVIDER     1

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

/* There are five LED status indicators located on the EVK Board.
 * The functions of these LEDs include:
 *
 *   - Main Power Supply (D16)
 *     Green: DC 5V main supply is normal.
 *     Red:   J43 input voltage is over 5.6V.
 *     Off:   The board is not powered.
 *   - Reset LED - Red (D7)
 *   - OpenSDA LED - Red (D5)
 *   - USER LED 1 - Green (D6)
 *   - USER LED 2 - Red (D34)
 *
 * Only two LEDs, D6 & D34, are under software control.
 */

/* LED index values for use with board_userled() */

#define BOARD_USERLED1    0
#define BOARD_USERLED2    1

#define BOARD_NLEDS       2

/* LED bits for use with board_userled_all() */

#define BOARD_USERLED1_BIT (1 << BOARD_USERLED1)
#define BOARD_USERLED2_BIT (1 << BOARD_USERLED2)

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

/* Thus if the LED is statically on, NuttX has successfully  booted and is,
 * apparently, running normally.  If the LED is flashing at approximately
 * 2Hz, then a fatal error has been detected and the system has halted.
 */

/* Button definitions *******************************************************/

/* The IMXRT1170-EVK board has seven (sets of) switches or buttons:
 *
 *   - External boot configuration switches (SW1)
 *   - External boot configuration switches (SW2)
 *   - MCU reset button (SW3)
 *   - System Reset Button / POR_BUTTON (SW4)
 *   - 5V power switch (SW5)
 *   - CPU ONOFF Button (SW6)
 *   - CPU Wakeup Button / USER_BUTTON (SW7)
 *
 * Only one button, SW7, can be used for user input.
 */

#define BUTTON_SW7        0
#define NUM_BUTTONS       1

#define BUTTON_SW7_BIT    (1 << BUTTON_SW7)

/* SDIO *********************************************************************/

/* Pin drive characteristics - drive strength in particular may need tuning
 * for specific boards.  Settings have been copied from i.MX RT 1060 EVK.
 */

#define PIN_USDHC1_D0     (GPIO_USDHC1_DATA0_1 | IOMUX_USDHC1_DATAX_DEFAULT)
#define PIN_USDHC1_D1     (GPIO_USDHC1_DATA1_1 | IOMUX_USDHC1_DATAX_DEFAULT)
#define PIN_USDHC1_D2     (GPIO_USDHC1_DATA2_1 | IOMUX_USDHC1_DATAX_DEFAULT)
#define PIN_USDHC1_D3     (GPIO_USDHC1_DATA3_1 | IOMUX_USDHC1_DATAX_DEFAULT)
#define PIN_USDHC1_DCLK   (GPIO_USDHC1_CLK_1   | IOMUX_USDHC1_CLK_DEFAULT)
#define PIN_USDHC1_CMD    (GPIO_USDHC1_CMD_1   | IOMUX_USDHC1_CMD_DEFAULT)
#define PIN_USDHC1_CD     (GPIO_USDHC1_CD_2    | IOMUX_USDHC1_CLK_DEFAULT)

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

/* ETH Disambiguation *******************************************************/

/* TO DO: Check & fix */

#define GPIO_ENET_TX_DATA00  (GPIO_ENET_TX_DATA0_1| \
                              IOMUX_ENET_DATA_DEFAULT)                    /* GPIO_B1_07 */
#define GPIO_ENET_TX_DATA01  (GPIO_ENET_TX_DATA1_1| \
                              IOMUX_ENET_DATA_DEFAULT)                    /* GPIO_B1_08 */
#define GPIO_ENET_RX_DATA00  (GPIO_ENET_RX_DATA0_1| \
                              IOMUX_ENET_DATA_DEFAULT)                    /* GPIO_B1_04 */
#define GPIO_ENET_RX_DATA01  (GPIO_ENET_RX_DATA1_1| \
                              IOMUX_ENET_DATA_DEFAULT)                    /* GPIO_B1_05 */
#define GPIO_ENET_MDIO       (GPIO_ENET_MDIO_2|IOMUX_ENET_MDIO_DEFAULT)   /* GPIO_EMC_41 */
#define GPIO_ENET_MDC        (GPIO_ENET_MDC_2|IOMUX_ENET_MDC_DEFAULT)     /* GPIO_EMC_40 */
#define GPIO_ENET_RX_EN      (GPIO_ENET_RX_EN_1|IOMUX_ENET_EN_DEFAULT)    /* GPIO_B1_06 */
#define GPIO_ENET_RX_ER      (GPIO_ENET_RX_ER_1|IOMUX_ENET_RXERR_DEFAULT) /* GPIO_B1_11 */
#define GPIO_ENET_TX_CLK     (GPIO_ENET_REF_CLK_2|\
                              IOMUX_ENET_TX_CLK_DEFAULT)                  /* GPIO_B1_10 */
#define GPIO_ENET_TX_EN      (GPIO_ENET_TX_EN_1|IOMUX_ENET_EN_DEFAULT)    /* GPIO_B1_09 */

#if 0
#define GPIO_ENET_INT        (GPIO_OUTPUT | GPIO_OUTPUT_ZERO | \
                              GPIO_PORT9 | GPIO_PIN11)                    /* GPIO_AD_12 */
#define GPIO_ENET_NRST       (GPIO_OUTPUT | GPIO_OUTPUT_ZERO | \
                              GPIO_PORT12 | GPIO_PIN12)                   /* GPIO_LPSR_12 */
#endif

/* PIO Disambiguation *******************************************************/

/* LPUARTs
 *
 * Virtual console port provided by OpenSDA on UART1 and
 * Arduino RS-232 Shield on UART3.
 *
 */

#define GPIO_LPUART1_RX      (GPIO_LPUART1_RX_1|IOMUX_UART_DEFAULT|PADMUX_SION) /* GPIO_AD_25 */
#define GPIO_LPUART1_TX      (GPIO_LPUART1_TX_1|IOMUX_UART_DEFAULT|PADMUX_SION) /* GPIO_AD_24 */
#define GPIO_LPUART3_RX      (GPIO_LPUART3_RX_1|IOMUX_UART_DEFAULT)             /* GPIO_AD_B1_07 */
#define GPIO_LPUART3_TX      (GPIO_LPUART3_TX_1|IOMUX_UART_DEFAULT)             /* GPIO_AD_B1_06 */

/* LPI2Cs
 *
 * Arduino Connector LPI2C1 and audio/gyro IO on LPI2C5.
 */

#define GPIO_LPI2C5_SDA      (GPIO_LPI2C5_SDA_1|IOMUX_LPI2C_DEFAULT) /* GPIO_LPSR_04 */
#define GPIO_LPI2C5_SCL      (GPIO_LPI2C5_SCL_1|IOMUX_LPI2C_DEFAULT) /* GPIO_LPSR_05 */
#define GPIO_LPI2C6_SDA      (GPIO_LPI2C6_SDA_1|IOMUX_LPI2C_DEFAULT) /* GPIO_LPSR_06 */
#define GPIO_LPI2C6_SCL      (GPIO_LPI2C6_SCL_1|IOMUX_LPI2C_DEFAULT) /* GPIO_LPSR_07 */

/* LPSPI
 *
 * Arduino Connector Install J63-J66
 *
 *   J26 D5   GPIO_LPSR_12  LPSPI6_SDI
 *   J26 D7   GPIO_LPSR_11  LPSPI6_SDO
 *   J26 D9   GPIO_LPSR_10  LPSPI6_SCK
 *   J26 D11  GPIO_LPSR_09  LPSPI6_CS0
 */

#define GPIO_LPSPI6_SCK      (GPIO_LPSPI6_SCK_1|IOMUX_LPSPI_DEFAULT)
#define GPIO_LPSPI6_MISO     (GPIO_LPSPI6_SDI_1|IOMUX_LPSPI_DEFAULT)
#define GPIO_LPSPI6_MOSI     (GPIO_LPSPI6_SDO_1|IOMUX_LPSPI_DEFAULT)

/* FlexCAN */

#define GPIO_FLEXCAN2_TX     (GPIO_FLEXCAN2_TX_3|IOMUX_CAN_DEFAULT)
#define GPIO_FLEXCAN2_RX     (GPIO_FLEXCAN2_RX_3|IOMUX_CAN_DEFAULT)

#define GPIO_FLEXCAN3_TX     (GPIO_FLEXCAN3_TX_2|IOMUX_CAN_DEFAULT)
#define GPIO_FLEXCAN3_RX     (GPIO_FLEXCAN3_RX_2|IOMUX_CAN_DEFAULT)

/* FLEXSPI */

#define GPIO_FLEXSPI_DQS     (GPIO_FLEXSPIA_DQS_2|IOMUX_FLEXSPI_DEFAULT)
#define GPIO_FLEXSPI_CS      (GPIO_FLEXSPIA_SS0_2|IOMUX_FLEXSPI_DEFAULT)
#define GPIO_FLEXSPI_IO0     (GPIO_FLEXSPIA_DATA00_2|IOMUX_FLEXSPI_DEFAULT)
#define GPIO_FLEXSPI_IO1     (GPIO_FLEXSPIA_DATA01_2|IOMUX_FLEXSPI_DEFAULT)
#define GPIO_FLEXSPI_IO2     (GPIO_FLEXSPIA_DATA02_2|IOMUX_FLEXSPI_DEFAULT)
#define GPIO_FLEXSPI_IO3     (GPIO_FLEXSPIA_DATA03_2|IOMUX_FLEXSPI_DEFAULT)
#define GPIO_FLEXSPI_SCK     (GPIO_FLEXSPIA_SCLK_2|IOMUX_FLEXSPI_DEFAULT)

/* FLEXSPI2 */

#define GPIO_FLEXSPI2_CS      (GPIO_FLEXSPI2_A_SS0_B_1|IOMUX_FLEXSPI_DEFAULT)
#define GPIO_FLEXSPI2_IO0     (GPIO_FLEXSPI2_A_DATA0_1|IOMUX_FLEXSPI_DEFAULT) /* SOUT */
#define GPIO_FLEXSPI2_IO1     (GPIO_FLEXSPI2_A_DATA1_1|IOMUX_FLEXSPI_DEFAULT) /* SIN */
#define GPIO_FLEXSPI2_SCK     (GPIO_FLEXSPI2_A_SCLK_1|IOMUX_FLEXSPI_CLK_DEFAULT)

/* SDRAM */

#define GPIO_SDRAM_CS  (GPIO_OUTPUT | GPIO_OUTPUT_ONE | IOMUX_GOUT_DEFAULT | \
                     GPIO_PORT1 | GPIO_PIN29)  /* GPIO_EMC_B1_29 */
#define GPIO_SDRAM_CLK  (GPIO_OUTPUT | GPIO_OUTPUT_ONE | IOMUX_GOUT_DEFAULT | \
                     GPIO_PORT1 | GPIO_PIN26)  /* GPIO_EMC_B1_26 */

#define GPIO_FLEXSPI2_SCK_IO (GPIO_OUTPUT | GPIO_OUTPUT_ONE | IOMUX_GOUT_DEFAULT | \
                     GPIO_PORT2 | GPIO_PIN20)  /* GPIO_EMC_B2_20 */
#define GPIO_FLEXSPI2_CS_IO (GPIO_OUTPUT | GPIO_OUTPUT_ONE | IOMUX_GOUT_DEFAULT | \
                     GPIO_PORT2 | GPIO_PIN21)  /* GPIO_EMC_B2_21 */
#define GPIO_FLEXSPI2_D1_IO (GPIO_OUTPUT | GPIO_OUTPUT_ONE | IOMUX_GOUT_DEFAULT | \
                     GPIO_PORT2 | GPIO_PIN24)  /* GPIO_EMC_B2_14 */

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
#endif /* __BOARDS_ARM_IMXRT_IMXRT1170_EVK_INCLUDE_BOARD_H */
