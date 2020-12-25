/****************************************************************************
 * boards/arm/lpc43xx/lpc4357-link2/include/board.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __BOARDS_ARM_LPC43XX_LPC4357_LINK2_INCLUDE_BOARD_H
#define __BOARDS_ARM_LPC43XX_LPC4357_LINK2_INCLUDE_BOARD_H

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

#define LPC43_CCLK                  BOARD_FCLKOUT_FREQUENCY

#if defined(CONFIG_LPC43_BUS) || defined(CONFIG_LPC43_MCPWM) || defined(CONFIG_LPC43_I2C0) || defined(CONFIG_LPC43_I2S0) || defined(CONFIG_LPC43_I2S1)  || defined(CONFIG_LPC43_CAN1)
#  define BOARD_ABP1_CLKSRC         BASE_APB_CLKSEL_XTAL
#  define BOARD_ABP1_FREQUENCY      BOARD_XTAL_FREQUENCY
#endif

#if defined(CONFIG_LPC43_BUS) || defined(CONFIG_LPC43_I2C1) || defined(CONFIG_LPC43_DAC) || defined(CONFIG_LPC43_ADC0) || defined(CONFIG_LPC43_ADC1)  || defined(CONFIG_LPC43_CAN0)
#  define BOARD_ABP3_CLKSRC         BASE_APB_CLKSEL_XTAL
#  define BOARD_ABP3_FREQUENCY      BOARD_XTAL_FREQUENCY
#endif

#define BOARD_IDIVA_DIVIDER         (2)
#define BOARD_IDIVA_CLKSRC          IDIVA_CLKSEL_PLL1
#define BOARD_IDIVA_FREQUENCY       (BOARD_FCLKOUT_FREQUENCY/BOARD_IDIVA_DIVIDER)

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

#if CONFIG_SPIFI_LIBRARY
#  define SPIFI_DEVICE_ALL                0        /**< Enables all devices in family */
#  define SPIFI_DEVICE_S25FL016K          0        /**< Enables Spansion S25FL016K device */
#  define SPIFI_DEVICE_S25FL032P          0        /**< Enables Spansion S25FL032P device */
#  define SPIFI_DEVICE_S25FL064P          0        /**< Enables Spansion S25FL064P device */
#  define SPIFI_DEVICE_S25FL129P_64K      0        /**< Enables Spansion S25FL129P (64K block) device */
#  define SPIFI_DEVICE_S25FL129P_256K     0        /**< Enables Spansion S25FL129P (256K block) device */
#  define SPIFI_DEVICE_S25FL164K          0        /**< Enables Spansion S25FL164K device */
#  define SPIFI_DEVICE_S25FL256S_64K      0        /**< Enables Spansion S25FL256S (64K block) device */
#  define SPIFI_DEVICE_S25FL256S_256K     0        /**< Enables Spansion S25FL256S (256K block) device */
#  define SPIFI_DEVICE_S25FL512S          0        /**< Enables Spansion S25FL512S device */
#  define SPIFI_DEVICE_MX25L1635E         0        /**< Enables Macronix MX25L1635E device */
#  define SPIFI_DEVICE_MX25L3235E         0        /**< Enables Macronix MX25L3235E device */
#  define SPIFI_DEVICE_MX25L8035E         0        /**< Enables Macronix MX25L8035E device */
#  define SPIFI_DEVICE_MX25L6435E         0        /**< Enables Macronix MX25L6435E device */
#  define SPIFI_DEVICE_W25Q32FV           0        /**< Enables Winbond W25Q32FV device */
#  define SPIFI_DEVICE_W25Q64FV           0        /**< Enables Winbond W25Q32V device */
#  define SPIFI_DEVICE_W25Q80BV           1        /**< Enables Winbond W25Q80BV device */
#  define SPIFI_DEVICE_REQUENCY_DIVIDER   2        /* PLL1 clock divider */
#endif

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

/* SSP clocking *************************************************************/

/* BOARD_SSPX_BASEFREQ may be further divided by 2-254 to get the SSP clock.
 * If we want a usable range of 400KHz to 25MHz for the SSP, then:
 *
 * 1. SSPCLK must be greater than (2*25MHz) = 50MHz, and
 * 2. SSPCLK must be less than (254*400Khz) = 101.6MHz.
 *
 */

#define BOARD_SSP0_CLKSRC           BASE_SSP0_CLKSEL_IDIVA
#define BOARD_SSP0_BASEFREQ         BOARD_IDIVA_FREQUENCY

#define BOARD_SSP1_CLKSRC           BASE_SSP1_CLKSEL_IDIVA
#define BOARD_SSP1_BASEFREQ         BOARD_IDIVA_FREQUENCY

/* Clocking *****************************************************************/

/* LED1   K2  GPIO0[8]
 *
 * LED index values for use with board_userled()
 */

#define BOARD_LED                   0
#define BOARD_NLEDS                 1

/* LED bits for use with board_userled_all() */

#define BOARD_LED_BIT               (1 << BOARD_LED)

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

                                       /* LED      */
#define LED_STARTED                 0  /* OFF      */
#define LED_HEAPALLOCATE            0  /* OFF      */
#define LED_IRQSENABLED             0  /* OFF      */
#define LED_STACKCREATED            1  /* ON       */
#define LED_INIRQ                   2  /* NC       */
#define LED_SIGNAL                  2  /* NC       */
#define LED_ASSERTION               2  /* NC       */
#define LED_PANIC                   3  /* Flashing */

/* UART Pins ****************************************************************/

/* The following definitions must be provided so that the LPC43 serial
 * driver can set up the U[S]ART for the serial console properly (see the
 * file arch/arc/src/lpc43xx/lpc43*_pinconf.h for more info).
 */

#define PINCONF_U0_TXD              PINCONF_U0_TXD_3
#define PINCONF_U0_RXD              PINCONF_U0_RXD_3
#define PINCONF_U0_DIR              PINCONF_U0_DIR_3

#define PINCONF_U1_TXD              PINCONF_U1_TXD_1
#define PINCONF_U1_RXD              PINCONF_U1_RXD_1

#define PINCONF_U2_TXD              PINCONF_U2_TXD_2
#define PINCONF_U2_RXD              PINCONF_U2_RXD_2
#define PINCONF_U2_DIR              PINCONF_U2_DIR_2

#define PINCONF_U3_TXD              PINCONF_U3_TXD_2
#define PINCONF_U3_RXD              PINCONF_U3_RXD_2
#define PINCONF_U3_DIR              PINCONF_U3_DIR_2

/* I2C1 pins, not really accessible on the board */

#define PINCONF_I2C1_SCL            PINCONF_I2C1_SCL_1
#define PINCONF_I2C1_SDA            PINCONF_I2C1_SDA_1

/* SSP1 pins */

#define PINCONF_SSP1_MISO           PINCONF_SSP1_MISO_3
#define PINCONF_SSP1_MOSI           PINCONF_SSP1_MOSI_3
#define PINCONF_SSP1_SCK            PINCONF_SSP1_SCK_1
#define PINCONF_SSP1_SSEL           PINCONF_SSP1_SSEL_1

#endif /* __BOARDS_ARM_LPC43XX_LPC4357_LINK2_INCLUDE_BOARD_H */
