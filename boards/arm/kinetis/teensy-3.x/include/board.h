/****************************************************************************
 * boards/arm/kinetis/teensy-3.x/include/board.h
 * include/arch/board/board.h
 *
 *   Copyright (C) 2015-2017 Gregory Nutt. All rights reserved.
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

#ifndef __BOARDS_ARM_KINETIS_TEENSY_3X_INCLUDE_BOARD_H
#define __BOARDS_ARM_KINETIS_TEENSY_3X_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
# include <stdint.h>
# include <stdbool.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* The teensy-3.1 has a 16MHz crystal on board */

#undef  BOARD_EXTCLOCK                /* Crystal */
#define BOARD_EXTAL_LP                /* Low Power, as opposed to Hi Gain */

/* BOARD_FRDIV is MCG_C1_FRDIV_DIV512 from kinetis_mcg.h.
 * According to the k20 reference manual, when transitioning MCG clock modes
 * to FLL Bypassed External the C1 divider must be set so that the FLL clock
 * is between 31.25 and 39.0625 khz.
 * For teensy-3.x that works out to a divider of 512.
 */

#define BOARD_FRDIV          MCG_C1_FRDIV_DIV512

#define BOARD_EXTAL_FREQ     16000000 /* 16MHz crystal frequency (REFCLK) */
#define BOARD_XTAL32_FREQ    32768    /* 32KHz RTC Oscillator (not populated) */

/* PLL Configuration.
 * NOTE: Only even frequency crystals are supported that will produce a 2MHz
 * reference clock to the PLL.
 * The rated speed for the MK20DX256VLH7 is 72MHz and 50MHz for the
 * MK20DX128VLH5, but according to the PJRC website, both can be overclocked
 * at 96MHz
 *
 * MK20DX128VLH5 Rated Frequency 50MHz (selecting 48Mhz to use USB)
 *
 *   PLL Input frequency:   PLLIN  = REFCLK/PRDIV = 16MHz/8 = 2MHz
 *   PLL Output frequency:  PLLOUT = PLLIN*VDIV   = 2Mhz*24  = 48MHz
 *   MCG Frequency:         PLLOUT = 48MHz
 *
 * MK20DX256VLH7 Rated Frequency 72MHz
 *
 *   PLL Input frequency:   PLLIN  = REFCLK/PRDIV = 16MHz/8 = 2MHz
 *   PLL Output frequency:  PLLOUT = PLLIN*VDIV   = 2Mhz*36  = 72MHz
 *   MCG Frequency:         PLLOUT = 72MHz
 *
 * Board can be overclocked at 96MHz (per PJRC.com)
 *   PLL Input frequency:   PLLIN  = REFCLK/PRDIV = 16MHz/8 = 2MHz
 *   PLL Output frequency:  PLLOUT = PLLIN*VDIV   = MHz*48 = 96MHz
 *   MCG Frequency:         PLLOUT = 96MHz
 */

#if defined(CONFIG_TEENSY_3X_OVERCLOCK)
/* PLL Configuration */

#  define BOARD_PRDIV        8    /* PLL External Reference Divider */
#  define BOARD_VDIV         48   /* PLL VCO Divider (frequency multiplier) */

/* SIM CLKDIV1 dividers */

#  define BOARD_OUTDIV1      1     /* Core        = MCG, 96MHz */
#  define BOARD_OUTDIV2      2     /* Bus         = MCG/2, 48MHz */
#  define BOARD_OUTDIV3      0     /* N/A         = No  OUTDIV3 */
#  define BOARD_OUTDIV4      4     /* Flash clock = MCG/4, 24MHz */

#elif defined(CONFIG_ARCH_CHIP_MK20DX256VLH7)

/* PLL Configuration */

#  define BOARD_PRDIV        8     /* PLL External Reference Divider */
#  define BOARD_VDIV         36    /* PLL VCO Divider (frequency multiplier) */

/* SIM CLKDIV1 dividers */

#  define BOARD_OUTDIV1      1     /* Core        = MCG, 72MHz */
#  define BOARD_OUTDIV2      2     /* Bus         = MCG/2, 36MHz */
#  define BOARD_OUTDIV3      0     /* N/A         = No  OUTDIV3 */
#  define BOARD_OUTDIV4      3     /* Flash clock = MCG/3, 72MHz */

#elif defined(CONFIG_ARCH_CHIP_MK20DX128VLH5)
/* PLL Configuration */

#  define BOARD_PRDIV        8     /* PLL External Reference Divider */
#  define BOARD_VDIV         24    /* PLL VCO Divider (frequency multiplier) */

/* SIM CLKDIV1 dividers */

#  define BOARD_OUTDIV1      1     /* Core        = MCG, 48MHz */
#  define BOARD_OUTDIV2      1     /* Bus         = MCG/1, 48MHz */
#  define BOARD_OUTDIV3      0     /* N/A         = No  OUTDIV3 */
#  define BOARD_OUTDIV4      2     /* Flash clock = MCG/2, 24MHz */
#endif

#define BOARD_PLLIN_FREQ     (BOARD_EXTAL_FREQ / BOARD_PRDIV)
#define BOARD_PLLOUT_FREQ    (BOARD_PLLIN_FREQ * BOARD_VDIV)
#define BOARD_MCG_FREQ       BOARD_PLLOUT_FREQ

#define BOARD_CORECLK_FREQ   (BOARD_MCG_FREQ / BOARD_OUTDIV1)
#define BOARD_BUS_FREQ       (BOARD_MCG_FREQ / BOARD_OUTDIV2)
#define BOARD_FLEXBUS_FREQ   (BOARD_MCG_FREQ / BOARD_OUTDIV3)
#define BOARD_FLASHCLK_FREQ  (BOARD_MCG_FREQ / BOARD_OUTDIV4)

/* Use MCGPLLCLK as the output SIM_SOPT2 MUX selected by
 * SIM_SOPT2[PLLFLLSEL]
 */

#define BOARD_SOPT2_PLLFLLSEL        SIM_SOPT2_PLLFLLSEL_MCGPLLCLK
#define BOARD_SOPT2_FREQ             BOARD_MCG_FREQ

  /* Divider output clock = Divider input clock × [ (USBFRAC+1) / (USBDIV+1) ]
   *     SIM_CLKDIV2_FREQ = BOARD_SOPT2_FREQ × [ (USBFRAC+1) / (USBDIV+1) ]
   */

#if BOARD_SOPT2_FREQ == 96000000
  /* USBFRAC/USBDIV = 1/2 of 96Mhz clock = 48MHz */

#  define BOARD_SIM_CLKDIV2_USBFRAC  1
#  define BOARD_SIM_CLKDIV2_USBDIV   2
#elif BOARD_SOPT2_FREQ == 72000000
  /* USBFRAC/USBDIV = 2/3 of 72Mhz clock = 48MHz */

#  define BOARD_SIM_CLKDIV2_USBFRAC  2
#  define BOARD_SIM_CLKDIV2_USBDIV   3
#elif BOARD_SOPT2_FREQ == 48000000
  /* USBFRAC/USBDIV = 1/1 of 48Mhz clock = 48MHz */

#  define BOARD_SIM_CLKDIV2_USBFRAC  1
#  define BOARD_SIM_CLKDIV2_USBDIV   1
#endif

#define BOARD_SIM_CLKDIV2_FREQ       (BOARD_SOPT2_FREQ / \
                                      BOARD_SIM_CLKDIV2_USBDIV * \
                                      BOARD_SIM_CLKDIV2_USBFRAC)

/* Use the output of SIM_SOPT2[PLLFLLSEL] as the USB clock source */

#define BOARD_USB_CLKSRC              SIM_SOPT2_USBSRC
#define BOARD_USB_FREQ                BOARD_SIM_CLKDIV2_FREQ

/* Allow USBOTG-FS Controller to Read from FLASH */

#define BOARD_USB_FLASHACCESS

/* PWM Configuration */

/* FTM0 Channels */

#define GPIO_FTM0_CH0OUT PIN_FTM0_CH0_2  /* Pin 22: PTC1 */
#define GPIO_FTM0_CH1OUT PIN_FTM0_CH1_2  /* Pin 23: PTC2 */
#define GPIO_FTM0_CH2OUT PIN_FTM0_CH2_2  /* Pin  9: PTC3 */
#define GPIO_FTM0_CH3OUT PIN_FTM0_CH3    /* Pin 10: PTC4 */
#define GPIO_FTM0_CH4OUT PIN_FTM0_CH4    /* Pin  6: PTD4 */
#define GPIO_FTM0_CH5OUT PIN_FTM0_CH5_2  /* Pin 20: PTD5 */
#define GPIO_FTM0_CH6OUT PIN_FTM0_CH6_2  /* Pin 21: PTD6 */
#define GPIO_FTM0_CH7OUT PIN_FTM0_CH7_2  /* Pin  5: PTD7 */

/* FTM1 Channels */

#define GPIO_FTM1_CH0OUT PIN_FTM1_CH0_1  /* Pin  3: PTA12 */
#define GPIO_FTM1_CH1OUT PIN_FTM1_CH1_1  /* Pin  4: PTA13 */

/* FTM2 Channels */

#define GPIO_FTM2_CH0OUT PIN_FTM2_CH0    /* Pin 25: PTB18 */
#define GPIO_FTM2_CH1OUT PIN_FTM2_CH1    /* Pin 32: PTB19 */

/* LED definitions **********************************************************/

/* A single LED is available driven by PTC5.
 * The LED is grounded so bringing PTC5 high will illuminate the LED.
 */

/* LED index values for use with board_userled() */

#define BOARD_LED                    0
#define BOARD_NLEDS                  1

/* LED bits for use with board_userled_all() */

#define BOARD_LED_BIT                (1 << BOARD_LED)

/* When CONFIG_ARCH_LEDS is defined in the NuttX configuration, NuttX will
 * control the LED as defined below.
 * Thus if the LED is statically on, NuttX has successfully booted and is,
 * apparently, running normally.
 * If the LED is flashing at approximately 2Hz, then a fatal error has been
 * detected and the system has halted.
 */

#define LED_STARTED                  0 /* STATUS LED=OFF */
#define LED_HEAPALLOCATE             0 /* STATUS LED=OFF */
#define LED_IRQSENABLED              0 /* STATUS LED=OFF */
#define LED_STACKCREATED             1 /* STATUS LED=ON */
#define LED_INIRQ                    2 /* STATUS LED=no change */
#define LED_SIGNAL                   2 /* STATUS LED=no change */
#define LED_ASSERTION                3 /* STATUS LED=no change */
#define LED_PANIC                    3 /* STATUS LED=flashing */

/* Button definitions *******************************************************/

/* The teensy-3.1 board has no standard GPIO contact buttons */

/* Alternative pin resolution ***********************************************/

/* The K20 has three UARTs with pin availability as follows:
 *
 *   --------- ------ ----------- -------------------------
 *   UART      PORT   BOARD       PJRC PINOUT DESCRIPTION
 *   FUNCTION         LABEL
 *   --------- ------ ----------- -------------------------
 *   UART0_RX  PTA1   (See above) MINI54TAN / Bootloader
 *             PTB16  Pin 0       RX1 / Touch
 *             PTD6   Pin 21 / A7 RX1 / CS / PWM
 *   UART0_TX  PTA2   (See above) MINI54TAN / Bootloader
 *             PTB17  Pin 1       TX1 / Touch
 *             PTD7   Pin 5       TX1 / PWM
 *   --------- ------ ----------- -------------------------
 *   UART1_RX  PTC3   Pin 9       RX2 / CS / PWM
 *             PTE1   Pad 26      (Pad on back of board)
 *   UART1_TX  PTC4   Pin 10      TX2 / CS / PWM
 *             PTE0   Pad 31      (Pad on back of board)
 *   --------- ------ ----------- -------------------------
 *   UART2_RX  PTD2   Pin 7       RX3 / DOUT
 *   UART2_TX  PTD3   Pin 8       TX3 / DIN
 *   --------- ------ ----------- -------------------------
 *
 * The default serial console is UART0 on pins 0 (RX) and 1 (TX).
 */

#ifdef CONFIG_KINETIS_UART0
#  define PIN_UART0_RX  PIN_UART0_RX_2
#  define PIN_UART0_TX  PIN_UART0_TX_2
#endif

#ifdef CONFIG_KINETIS_UART1
#  define PIN_UART0_RX  PIN_UART1_RX_1
#  define PIN_UART0_TX  PIN_UART1_TX_1
#endif

#ifdef CONFIG_KINETIS_I2C0
#ifdef CONFIG_TEENSY_3X_I2C_ALT_PINS
#  define PIN_I2C0_SCL      (PIN_I2C0_SCL_1 | PIN_ALT2_OPENDRAIN | PIN_ALT2_SLOW)
#  define PIN_I2C0_SDA      (PIN_I2C0_SDA_1 | PIN_ALT2_OPENDRAIN | PIN_ALT2_SLOW)
#else
#  define PIN_I2C0_SCL      (PIN_I2C0_SCL_2 | PIN_ALT2_OPENDRAIN | PIN_ALT2_SLOW)
#  define PIN_I2C0_SDA      (PIN_I2C0_SDA_2 | PIN_ALT2_OPENDRAIN | PIN_ALT2_SLOW)
#endif
#endif

/* REVISIT: Added only for clean compilation with I2C1 enabled. */

#ifdef CONFIG_KINETIS_I2C1
#ifdef CONFIG_TEENSY_3X_I2C_ALT_PINS
#  define PIN_I2C1_SCL      (PIN_I2C1_SCL_1 | PIN_ALT2_OPENDRAIN | PIN_ALT2_SLOW)
#  define PIN_I2C1_SDA      (PIN_I2C1_SDA_1 | PIN_ALT2_OPENDRAIN | PIN_ALT2_SLOW)
#else
#  define PIN_I2C1_SCL      (PIN_I2C1_SCL_2 | PIN_ALT2_OPENDRAIN | PIN_ALT2_SLOW)
#  define PIN_I2C1_SDA      (PIN_I2C1_SDA_2 | PIN_ALT2_OPENDRAIN | PIN_ALT2_SLOW)
#endif
#endif

#endif /* __BOARDS_ARM_KINETIS_TEENSY_3X_INCLUDE_BOARD_H */
