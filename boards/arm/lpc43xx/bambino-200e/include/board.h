/****************************************************************************
 * boards/arm/lpc43xx/bambino-200e/include/board.h
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           Alan Carvalho de Assis acassis@gmail.com
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __BOARDS_ARM_LPC43XX_BAMBINO_200E_INCLUDE_BOARD_H
#define __BOARDS_ARM_LPC43XX_BAMBINO_200E_INCLUDE_BOARD_H

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
 * The Bambino-200e board has three crystals on board:
 *
 *     Y1 - RTC 32.768 MHz oscillator input,
 *     Y2 - 12.000 MHz LPC43xx crystal oscillator input
 *     Y3 - 25 MHz input for Ethernet KSZ8031 PHY
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

#define BOARD_MAIN_CLK              BOARD_FCCO_FREQUENCY   /* Main clock frequency */

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

/* SDIO Clocking */

#define BOARD_SDIO_CLKSRC           BASE_SDIO_CLKSEL_PLL1

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

/* SD/MMC or SDIO interface
 *
 * NOTE: The SDIO function clock to the interface can be up to 50 MHZ.
 * Example:  BOARD_MAIN_CLK=220MHz, CLKDIV=5, Finput=44MHz.
 */

#define BOARD_SDMMC_MAXFREQ      50000000
#define BOARD_SDMMC_CEIL(a,b)    (((a) + (b) - 1) / (b))

#define BOARD_SDMMC_CLKDIV       (1)            /* No source clock divider */
#define BOARD_SDMMC_FREQUENCY    (BOARD_MAIN_CLK / BOARD_SDMMC_CLKDIV)

/* Mode-dependent function clock division
 *
 * Example:  BOARD_SDMMC_FREQUENCY=44MHz
 *           BOARD_CLKDIV_INIT=110,    Fsdmmc=400KHz  (400KHz max)
 *           BOARD_CLKDIV_MMCXFR=4[3], Fsdmmc=11Mhz   (20MHz max) See NOTE:
 *           BOARD_CLKDIV_SDWIDEXFR=2, Fsdmmc=22MHz   (25MHz max)
 *           BOARD_CLKDIV_SDXFR=2,     Fsdmmc=22MHz   (25MHz max)
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

/* The Bambino 200E has 2 user-controllable LEDs labeled LED1 and LED2 in the
 * schematic and on bus referred to has GPIO3[7] and GPIO5[5], respectively.
 *
 *  LED1   GPIO3[7]
 *  LED2   GPIO5[5]
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

/* The Bambino 200E does not have RS-232 drivers or serial connectors on
 * board. UART1 and USART2 are availables on Socket 5 and 10, recpectively:
 *
 *   ------  ----------  -----------------------
 *   SIGNAL  Socket/Pin    LPC4330FBD144 PIN
 *   ------  ----------  -----------------------
 *   U1_TXD  s:5  / p:4  63   P5_6  U1_TXD=Alt 1
 *   U1_RXD  s:5  / p:5  61   P1_14 U1_RXD=Alt 1
 *   U2_TXD  s:10 / p:4  104  P2_10 U2_TXD=Alt 1
 *   U2_RXD  s:10 / p:5  105  P2_11 U2_RXD=Alt 1
 *   ------  ----------  -----------------------
 *
 * The following definitions must be provided so that the LPC43 serial
 * driver can set up the U[S]ART for the serial console properly (see the
 * file arch/arc/src/lpc43xx/lpc4310203050_pinconf.h for more info).
 */

#define PINCONF_U0_TXD      PINCONF_U0_TXD_3
#define PINCONF_U0_RXD      PINCONF_U0_RXD_3
#define PINCONF_U0_DIR      PINCONF_U0_DIR_3

#define PINCONF_U1_TXD      PINCONF_U1_TXD_5
#define PINCONF_U1_RXD      PINCONF_U1_RXD_1

#define PINCONF_U2_TXD      PINCONF_U2_TXD_2
#define PINCONF_U2_RXD      PINCONF_U2_RXD_2
#define PINCONF_U2_DIR      PINCONF_U2_DIR_2

#define PINCONF_U3_TXD      PINCONF_U3_TXD_2
#define PINCONF_U3_RXD      PINCONF_U3_RXD_2
#define PINCONF_U3_DIR      PINCONF_U3_DIR_2

/* SPI Pins *****************************************************************/

/* The Bambino 200E has SPI peripheral pins reserved for SPIFI.
 * SSP0 and SSP1 are available on Socket 1 and 10, respectively:
 *
 *   ---------  ----------  -----------------
 *    SIGNAL    Socket/Pin  LPC4330FBD144 PIN
 *   ---------  ----------  -----------------
 *   SSP0_SCK   s:1  / p:9      112  P3_0
 *   SSP0_SSEL  s:1  / p:6      38   P1_0
 *   SSP0_MISO  s:1  / p:8      42   P1_1
 *   SSP0_MOSI  s:1  / p:7      43   P1_2
 *   SSP1_SCK   s:10 / p:9      120  PF_4
 *   SSP1_SSEL  s:10 / p:6      48   P1_5
 *   SSP1_MISO  s:10 / p:8      44   P1_3
 *   SSP1_MOSI  s:10 / p:7      47   P1_4
 *   ---------  ----------  -----------------
 *
 * The following definitions must be provided so that the LPC43 serial
 * driver can set up the SPI ports properly (see the
 * file arch/arm/src/lpc43xx/lpc4310203050_pinconf.h for more info).
 */

#define PINCONF_SSP0_SCK  PINCONF_SSP0_SCK_3
#define PINCONF_SSP0_SSEL PINCONF_SSP0_SSEL_3
#define PINCONF_SSP0_MISO PINCONF_SSP0_MISO_3
#define PINCONF_SSP0_MOSI PINCONF_SSP0_MOSI_3

#define PINCONF_SSP1_SCK  PINCONF_SSP1_SCK_1
#define PINCONF_SSP1_SSEL PINCONF_SSP1_SSEL_3
#define PINCONF_SSP1_MISO PINCONF_SSP1_MISO_3
#define PINCONF_SSP1_MOSI PINCONF_SSP1_MOSI_3

/* Ethernet */

#define PINCONF_ENET_RX_DV  PINCONF_ENET_RX_DV_2
#define PINCONF_ENET_RESET  PINCONF_GPIO0p4
#define GPIO_ENET_RESET     (GPIO_MODE_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT0 | GPIO_PIN4)
#define PINCONF_ENET_MDC    PINCONF_ENET_MDC_3
#define PINCONF_ENET_TX_EN  PINCONF_ENET_TX_EN_1

/* SD/MMC pinout */

#define GPIO_SD_CARD_DET_N         PINCONF_SD_CD_1
#define GPIO_SD_D0                 PINCONF_SD_DAT0_1
#define GPIO_SD_D1                 PINCONF_SD_DAT1_1
#define GPIO_SD_D2                 PINCONF_SD_DAT2_1
#define GPIO_SD_D3                 PINCONF_SD_DAT3_1
#define GPIO_SD_CMD                PINCONF_SD_CMD_1
#define GPIO_SD_CLK                CLKCONF_SD_CLK_2

#endif /* __BOARDS_ARM_LPC43XX_BAMBINO_200E_INCLUDE_BOARD_H */
