/****************************************************************************
 * boards/arm/lpc43xx/lpc4330-xplorer/include/board.h
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

#ifndef __BOARDS_ARM_LPC43XX_LPC4330_XPLORER_INCLUDE_BOARD_H
#define __BOARDS_ARM_LPC43XX_LPC4330_XPLORER_INCLUDE__BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdbool.h>

#if defined(CONFIG_ARCH_IRQBUTTONS) && defined(CONFIG_LPC43_GPIO_IRQ)
#  include <nuttx/irq.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* NOTE:  The following definitions require lpc43_cgu.h.  It is not included
 * here because the including C file may not have that file in its include
 * path.
 *
 * The Xplorer board has four crystals on board:
 *
 *     Y1 - RTC 32.768 MHz oscillator input,
 *     Y2 - 24.576 MHz input to the UDA 1380 audio codec,
 *     Y3 - 12.000 MHz LPC43xx crystal oscillator input
 *     Y4 - 50 MHz input for Ethernet
 */

#define BOARD_XTAL_FREQUENCY        (12000000)  /* XTAL oscillator frequency (Y3) */
#define BOARD_RTCCLK_FREQUENCY      (32768)     /* RTC oscillator frequency (Y1) */
#define BOARD_INTRCOSC_FREQUENCY    (4000000)   /* Internal RC oscillator frequency */

/* Integer and direct modes are supported:
 *
 * In integer mode (Fclkout < 156000000):
 *    Fclkin  = BOARD_XTAL_FREQUENCY
 *    Fclkout = Msel * FClkin / Nsel
 *    Fcco    = 2 * Psel * Fclkout
 * In direct mode (Fclkout > 156000000):
 *    Fclkin  = BOARD_XTAL_FREQUENCY
 *    Fclkout = Msel * FClkin / Nsel
 *    Fcco    = Fclkout
 */

#ifdef CONFIG_LPC43_72MHz

/* NOTE:  At 72MHz, the calibrated value of CONFIG_BOARD_LOOPSPERMSEC was
 * determined to be:
 *
 *    CONFIG_BOARD_LOOPSPERMSEC=7191
 *
 * executing from SRAM.
 */

/* Final clocking (Integer mode with no ramp-up)
 *
 *    Fclkout = 6 * 12MHz / 1  = 72MHz
 *    Fcco    = 2 * 2 * 72MHz = 216MHz
 */

#  define BOARD_PLL_MSEL            (6)         /* Msel = 6 */
#  define BOARD_PLL_NSEL            (1)         /* Nsel = 1 */
#  define BOARD_PLL_PSEL            (2)         /* Psel = 2 */

#  define BOARD_FCLKOUT_FREQUENCY   (72000000)  /* 6 * 12,000,000 / 1 */
#  define BOARD_FCCO_FREQUENCY      (244000000) /* 2 * 2 * Fclkout */

#else

/* NOTE:  At 72MHz, the calibrated value of CONFIG_BOARD_LOOPSPERMSEC was
 * determined to be:
 *
 *    CONFIG_BOARD_LOOPSPERMSEC=18535
 *
 * executing from SRAM.
 */

/* Intermediate ramp-up clocking (Integer mode). If BOARD_PLL_RAMP_MSEL
 * is not defined, there will be no ramp-up.
 *
 *    Fclkout = 9 * 12MHz / 1  = 108MHz
 *    Fcco    = 2 * 1 * 108MHz = 216MHz
 */

#  define BOARD_PLL_RAMP_MSEL       (9)         /* Msel = 9 */
#  define BOARD_PLL_RAMP_NSEL       (1)         /* Nsel = 1 */
#  define BOARD_PLL_RAMP_PSEL       (1)         /* Psel = 1 */

#  define BOARD_RAMP_FCLKOUT        (108000000) /* 9 * 12,000,000 / 1 */
#  define BOARD_RAMP_FCCO           (216000000) /* 2 * 1 * Fclkout */

/* Final clocking (Direct mode).
 *
 *    Fclkout = 17 * 12MHz / 1 = 204MHz
 *    Fcco    = Fclockout      = 204MHz
 */

#  define BOARD_PLL_MSEL            (17)        /* Msel = 17 */
#  define BOARD_PLL_NSEL            (1)         /* Nsel = 1 */

#  define BOARD_FCLKOUT_FREQUENCY   (204000000) /* 17 * 12,000,000 / 1 */
#  define BOARD_FCCO_FREQUENCY      (204000000) /* Fclockout */

#endif

/* This is the clock setup we configure for:
 *
 * SYSCLK = BOARD_OSCCLK_FREQUENCY = 12MHz  -> Main oscillator for source
 * PLL0CLK = (2 * 20 * SYSCLK) / 1 = 480MHz -> multipler=20, pre-divider=1
 * CCLCK = 480MHz / 6 = 80MHz               -> divider = 6
 */

#define LPC43_CCLK                  BOARD_FCLKOUT_FREQUENCY

/* APB Clocking */

#if defined(CONFIG_LPC43_BUS) || defined(CONFIG_LPC43_MCPWM) || \
    defined(CONFIG_LPC43_I2C0) || defined(CONFIG_LPC43_I2S0) || \
    defined(CONFIG_LPC43_I2S1)  || defined(CONFIG_LPC43_CAN1)
#  define BOARD_ABP1_CLKSRC         BASE_APB_CLKSEL_XTAL
#  define BOARD_ABP1_FREQUENCY      BOARD_XTAL_FREQUENCY
#endif

#if defined(CONFIG_LPC43_BUS) || defined(CONFIG_LPC43_I2C1) || \
    defined(CONFIG_LPC43_DAC) || defined(CONFIG_LPC43_ADC0) || \
    defined(CONFIG_LPC43_ADC1)  || defined(CONFIG_LPC43_CAN0)
#  define BOARD_ABP3_CLKSRC         BASE_APB_CLKSEL_XTAL
#  define BOARD_ABP3_FREQUENCY      BOARD_XTAL_FREQUENCY
#endif

/* SSP Clocking */

#define BOARD_IDIVA_DIVIDER         (2)
#define BOARD_IDIVA_CLKSRC          IDIVA_CLKSEL_PLL1
#define BOARD_IDIVA_FREQUENCY       (BOARD_FCLKOUT_FREQUENCY/BOARD_IDIVA_DIVIDER)

#define BOARD_SSP0_CLKSRC           BASE_SSP0_CLKSEL_IDIVA
#define BOARD_SSP0_BASEFREQ         BOARD_IDIVA_FREQUENCY

#define BOARD_SSP1_CLKSRC           BASE_SSP1_CLKSEL_IDIVA
#define BOARD_SSP1_BASEFREQ         BOARD_IDIVA_FREQUENCY

/* USB0 *********************************************************************/

/* Settings needed in lpc43_cpu.c */

#define BOARD_USB0_CLKSRC           PLL0USB_CLKSEL_XTAL
#define BOARD_USB0_MDIV             0x06167ffa /* Table 149 datsheet, valid for 12Mhz Fclkin */
#define BOARD_USB0_NP_DIV           0x00302062 /* Table 149 datsheet, valid for 12Mhz Fclkin */

/* SPIFI clocking ***********************************************************/

/* The SPIFI will receive clocking from a divider per the settings provided
 * in this file.  The NuttX code will configure PLL1 as the input clock
 * for the selected divider
 */

#undef  BOARD_SPIFI_PLL1                        /* No division */
#undef  BOARD_SPIFI_DIVA                        /* Supports division by 1-4 */
#undef  BOARD_SPIFI_DIVB                        /* Supports division by 1-16 */
#undef  BOARD_SPIFI_DIVC                        /* Supports division by 1-16 */
#undef  BOARD_SPIFI_DIVD                        /* Supports division by 1-16 */
#undef  BOARD_SPIFI_DIVE                        /* Supports division by 1-256 */

#if BOARD_FCLKOUT_FREQUENCY < 20000000
#  define BOARD_SPIFI_PLL1          1           /* Use PLL1 directly */
#else
#  define BOARD_SPIFI_DIVB          1           /* Use IDIVB */
#endif

/* We need to configure the divider so that its output is as close to the
 * desired SCLK value.  The peak data transfer rate will be about half of
 * this frequency in bytes per second.
 */

#if BOARD_FCLKOUT_FREQUENCY < 20000000
#  define BOARD_SPIFI_FREQUENCY     BOARD_FCLKOUT_FREQUENCY  /* 72Mhz? */
#else
#  define BOARD_SPIFI_DIVIDER       (14)        /* 204MHz / 14 = 14.57MHz */
#  define BOARD_SPIFI_FREQUENCY     (102000000) /* 204MHz / 14 = 14.57MHz */
#endif

/* SD/MMC or SDIO interface *************************************************/

#define BOARD_SDMMC_CEIL(a,b)    (((a) + (b) - 1) / (b))

/* For LPC4330 family there is no predivider for the clock */

#define BOARD_SDMMC_FREQUENCY    BOARD_MAIN_CLK

/* Mode-dependent function clock division
 *
 * NOTE:  Clock division is 2*n. For example, value of 0 means divide by
 * 2 * 0 = 0 (no division, bypass), value of 1 means divide by 2 * 1 = 2,
 * value of 255 means divide by 2 * 255 = 510, and so on.
 *
 * SD/MMC logic will write the value ((clkdiv + 1) >> 1) as the divisor.
 * So an odd value calculated below will be moved up to next higher divider
 * value.
 * So the value 3 will cause 2 to be written as the divider value and the
 * effective divider will be 4.
 *
 * NOTE: The SDIO function clock to the interface can be up to 52 MHZ.
 *       See UM10503 Section 22.2.
 */

#define BOARD_CLKDIV_INIT       BOARD_SDMMC_CEIL(BOARD_SDMMC_FREQUENCY, 400000)
#define BOARD_CLKDIV_MMCXFR     BOARD_SDMMC_CEIL(BOARD_SDMMC_FREQUENCY, 20000000)
#define BOARD_CLKDIV_SDWIDEXFR  BOARD_SDMMC_CEIL(BOARD_SDMMC_FREQUENCY, 25000000)
#define BOARD_CLKDIV_SDXFR      BOARD_SDMMC_CEIL(BOARD_SDMMC_FREQUENCY, 25000000)

/* UART clocking ************************************************************/

/* Configure all U[S]ARTs to use the XTAL input frequency */

#define BOARD_USART0_CLKSRC         BASE_USART0_CLKSEL_XTAL
#define BOARD_USART0_BASEFREQ       BOARD_XTAL_FREQUENCY

#define BOARD_UART1_CLKSRC          BASE_UART1_CLKSEL_XTAL
#define BOARD_UART1_BASEFREQ        BOARD_XTAL_FREQUENCY

#define BOARD_USART2_CLKSRC         BASE_USART2_CLKSEL_XTAL
#define BOARD_USART2_BASEFREQ       BOARD_XTAL_FREQUENCY

#define BOARD_USART3_CLKSRC         BASE_USART3_CLKSEL_XTAL
#define BOARD_USART3_BASEFREQ       BOARD_XTAL_FREQUENCY

/* Clocking *****************************************************************/

/* The LPC4330-Xplorer has 2 user-controllable LEDs labeled D2 an D3 in the
 * schematic and on but referred to has LED1 and LED2 here, respectively.
 *
 *  LED1   D2  GPIO1[12]
 *  LED2   D3  GPIO1[11]
 *
 * LEDs are pulled high to a low output illuminates the LED.
 *
 * LED index values for use with board_userled()
 */

#define BOARD_LED1          0
#define BOARD_LED2          1
#define BOARD_NLEDS         2

/* LED bits for use with board_userled_all() */

#define BOARD_LED1_BIT      (1 << BOARD_LED1)
#define BOARD_LED2_BIT      (1 << BOARD_LED2)

/* If CONFIG_ARCH_LEDS is defined, the LEDs will be controlled as follows
 * for NuttX debug functionality (where NC means "No Change"). If
 * CONFIG_ARCH_LEDS is not defined, then the LEDs are completely under
 * control of the application.  The following interfaces are then available
 * for application control of the LEDs:
 *
 *  uint32_t board_userled_initialize(void);
 *  void board_userled(int led, bool ledon);
 *  void board_userled_all(uint32_t ledset);
 */

                               /* LED1   LED2   LED1   LED2 */
#define LED_STARTED         0  /* OFF    OFF     -      -   */
#define LED_HEAPALLOCATE    1  /* ON     OFF     -      -   */
#define LED_IRQSENABLED     1  /* ON     OFF     -      -   */
#define LED_STACKCREATED    1  /* ON     OFF     -      -   */
#define LED_INIRQ           2  /* NC     ON      NC     OFF */
#define LED_SIGNAL          2  /* NC     ON      NC     OFF */
#define LED_ASSERTION       2  /* NC     ON      NC     OFF */
#define LED_PANIC           2  /* NC     ON      NC     OFF */

/* UART Pins ****************************************************************/

/* The LPC4330 Xplorer does not have RS-232 drivers or serial connectors on
 * board. USART0 and UART1 are available on J8 as follows:
 *
 *   ------ ------ -----------------------
 *   SIGNAL J8 PIN   LPC4330FET100 PIN
 *                   (TFBGA100 package)
 *   ------ ------ -----------------------
 *   U0_TXD pin  9  F6  P6_4  U0_TXD=Alt 2
 *   U0_RXD pin 10  F9  P6_5  U0_RXD=Alt 2
 *   U1_TXD pin 13  H8  P1_13 U1_TXD=Alt 1
 *   U1_RXD pin 14  J8  P1_14 U1_RXD=Alt 1
 *   ------ ------ -----------------------
 *
 * The following definitions must be provided so that the LPC43 serial
 * driver can set up the U[S]ART for the serial console properly (see the
 * file arch/arc/src/lpc43xx/lpc4310203050_pinconf.h for more info).
 */

#define PINCONF_U0_TXD      PINCONF_U0_TXD_3
#define PINCONF_U0_RXD      PINCONF_U0_RXD_3
#define PINCONF_U0_DIR      PINCONF_U0_DIR_3

#define PINCONF_U1_TXD      PINCONF_U1_TXD_1
#define PINCONF_U1_RXD      PINCONF_U1_RXD_1

#define PINCONF_U2_TXD      PINCONF_U2_TXD_1
#define PINCONF_U2_RXD      PINCONF_U2_RXD_1
#define PINCONF_U2_DIR      PINCONF_U2_DIR_1

#define PINCONF_U3_TXD      PINCONF_U3_TXD_2
#define PINCONF_U3_RXD      PINCONF_U3_RXD_2
#define PINCONF_U3_DIR      PINCONF_U3_DIR_2

/* Ethernet */

#define PINCONF_ENET_RX_DV  PINCONF_ENET_RX_DV_2
#define PINCONF_ENET_RESET  PINCONF_GPIO0p4
#define GPIO_ENET_RESET     (GPIO_MODE_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT0 | GPIO_PIN4)
#define PINCONF_ENET_MDC    PINCONF_ENET_MDC_3
#define PINCONF_ENET_TX_EN  PINCONF_ENET_TX_EN_1

#endif /* __BOARDS_ARM_LPC43XX_LPC4330_XPLORER_INCLUDE_BOARD_H */
