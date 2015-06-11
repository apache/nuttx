/************************************************************************************
 * configs/teensy-3.x/include/board.h
 * include/arch/board/board.h
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
 ************************************************************************************/

#ifndef __CONFIGS_TEENSY_3X_INCLUDE_BOARD_H
#define __CONFIGS_TEENSY_3X_INCLUDE_BOARD_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
# include <stdint.h>
# include <stdbool.h>
#endif

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Clocking *************************************************************************/
/* The teensy-3.1 has a 16MHz crystal on board */

#undef  BOARD_EXTCLOCK                      /* Crystal */
#define BOARD_EXTAL_FREQ     16000000       /* 16MHz crystal frequency (REFCLK) */
#define BOARD_XTAL32_FREQ    32768          /* 32KHz RTC Oscillator (not populated) */

/* PLL Configuration.  NOTE: Only even frequency crystals are supported that will
 * produce a 2MHz reference clock to the PLL.  The rated speed for the MK20DX256VLH7
 * is 72MHz and 50MHz for the MK20DX128VLH5, but according to the PJRC website,
 * both can be overclocked at 96MHz
 *
 * MK20DX128VLH5 Rated Frequency 50MHz
 *
 *   PLL Input frequency:   PLLIN  = REFCLK/PRDIV = 16MHz/8 = 2MHz
 *   PLL Output frequency:  PLLOUT = PLLIN*VDIV   = 2Mhz*25  = 50MHz
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
 *   PLL Output frequency:  PLLOUT = PLLIN*VDIV   = Mhz*48 = 96MHz
 *   MCG Frequency:         PLLOUT = 96MHz
 */

#if defined(CONFIG_TEENSY_3X_OVERCLOCK)
/* PLL Configuration */

#  define BOARD_PRDIV        8              /* PLL External Reference Divider */
#  define BOARD_VDIV         48             /* PLL VCO Divider (frequency multiplier) */

/* SIM CLKDIV1 dividers */

#  define BOARD_OUTDIV1      1              /* Core        = MCG, 96MHz */
#  define BOARD_OUTDIV2      2              /* Bus         = MCG/2, 48MHz */
#  define BOARD_OUTDIV3      2              /* FlexBus     = MCG/2, 48MHz */
#  define BOARD_OUTDIV4      4              /* Flash clock = MCG/4, 24MHz */

#elif defined(CONFIG_ARCH_CHIP_MK20DX256VLH7)

/* PLL Configuration */

#  define BOARD_PRDIV        8              /* PLL External Reference Divider */
#  define BOARD_VDIV         36             /* PLL VCO Divider (frequency multiplier) */

/* SIM CLKDIV1 dividers */

#  define BOARD_OUTDIV1      1              /* Core        = MCG, 72MHz */
#  define BOARD_OUTDIV2      2              /* Bus         = MCG/2, 36MHz */
#  define BOARD_OUTDIV3      2              /* FlexBus     = MCG/2, 36MHz */
#  define BOARD_OUTDIV4      3              /* Flash clock = MCG/3, 72MHz */

#elif defined(CONFIG_ARCH_CHIP_MK20DX128VLH5)
/* PLL Configuration */

#  define BOARD_PRDIV        8              /* PLL External Reference Divider */
#  define BOARD_VDIV         25             /* PLL VCO Divider (frequency multiplier) */

/* SIM CLKDIV1 dividers */

#  define BOARD_OUTDIV1      1              /* Core        = MCG, 50MHz */
#  define BOARD_OUTDIV2      1              /* Bus         = MCG/1, 50MHz */
#  define BOARD_OUTDIV3      1              /* FlexBus     = MCG/1, 20MHz */
#  define BOARD_OUTDIV4      2              /* Flash clock = MCG/2, 25MHz */
#endif

#define BOARD_PLLIN_FREQ     (BOARD_EXTAL_FREQ / BOARD_PRDIV)
#define BOARD_PLLOUT_FREQ    (BOARD_PLLIN_FREQ * BOARD_VDIV)
#define BOARD_MCG_FREQ       BOARD_PLLOUT_FREQ

#define BOARD_CORECLK_FREQ   (BOARD_MCG_FREQ / BOARD_OUTDIV1)
#define BOARD_BUS_FREQ       (BOARD_MCG_FREQ / BOARD_OUTDIV2)
#define BOARD_FLEXBUS_FREQ   (BOARD_MCG_FREQ / BOARD_OUTDIV3)
#define BOARD_FLASHCLK_FREQ  (BOARD_MCG_FREQ / BOARD_OUTDIV4)

/* LED definitions ******************************************************************/
/* A single LED is available driven by PTC5.  The LED is grounded so bringing PTC5
 * high will illuminate the LED.
 */

/* LED index values for use with kinetis_setled() */

#define BOARD_LED                    0
#define BOARD_NLEDS                  1

/* LED bits for use with kinetis_setleds() */

#define BOARD_LED_BIT                (1 << BOARD_LED)

/* When CONFIG_ARCH_LEDS is defined in the NuttX configuration, NuttX will
 * control the LED as defined below.  Thus if the LED is statically on, NuttX has
 * successfully booted and is, apparently, running normally.  If the LED is
 * flashing at approximately 2Hz, then a fatal error has been detected and the
 * system has halted.
 */

#define LED_STARTED                  0 /* STATUS LED=OFF */
#define LED_HEAPALLOCATE             0 /* STATUS LED=OFF */
#define LED_IRQSENABLED              0 /* STATUS LED=OFF */
#define LED_STACKCREATED             1 /* STATUS LED=ON */
#define LED_INIRQ                    2 /* STATUS LED=no change */
#define LED_SIGNAL                   2 /* STATUS LED=no change */
#define LED_ASSERTION                2 /* STATUS LED=no change */
#define LED_PANIC                    3 /* STATUS LED=flashing */

/* Button definitions ***************************************************************/
/* The teensy-3.1 board has no standard GPIO contact buttons */

/* Alternative pin resolution *******************************************************/
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

/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/
/************************************************************************************
 * Name: kinetis_boardinitialize
 *
 * Description:
 *   All STM32 architectures must provide the following entry point.  This entry point
 *   is called early in the initialization -- after all memory has been configured
 *   and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

void kinetis_boardinitialize(void);

/************************************************************************************
 * Name:  kinetis_ledinit, kinetis_setled, and kinetis_setleds
 *
 * Description:
 *   If CONFIG_ARCH_LEDS is defined, then NuttX will control the on-board LED.  If
 *   CONFIG_ARCH_LEDS is not defined, then the following interfaces are available to
 *   control the LEDs from user applications.
 *
 ************************************************************************************/

#ifndef CONFIG_ARCH_LEDS
void kinetis_ledinit(void);
void kinetis_setled(int led, bool ledon);
void kinetis_setleds(uint8_t ledset);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif  /* __CONFIGS_TEENSY_3X_INCLUDE_BOARD_H */
