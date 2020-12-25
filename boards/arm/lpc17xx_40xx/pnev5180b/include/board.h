/****************************************************************************
 * boards/arm/lpc17xx_40xx/pnev5180b/include/board.h
 * include/arch/board/board.h
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Michael Jung <mijung@gmx.net>
 *
 * Based on boards/zkit-arm-1769/include/board.h
 *   Copyright (C) 2013 Zilogic Systems. All rights reserved.
 *   Author: BabuSubashChandar <code@zilogic.com>
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

#ifndef __BOARDS_ARM_LPC17XX_40XX_PNEV5180B_INCLUDE_BOARD_H
#define __BOARDS_ARM_LPC17XX_40XX_PNEV5180B_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>

#if defined(CONFIG_ARCH_IRQBUTTONS) && defined(CONFIG_LPC17_40_GPIOIRQ)
#  include <nuttx/irq.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* NOTE:  The following definitions require lpc17_40_syscon.h.  It is not
 * included here because the including C file may not have that file in its
 * include path.
 */

#define BOARD_XTAL_FREQUENCY        (12000000)            /* XTAL oscillator frequency */
#define BOARD_OSCCLK_FREQUENCY      BOARD_XTAL_FREQUENCY  /* Main oscillator frequency */
#define BOARD_RTCCLK_FREQUENCY      (32768)               /* RTC oscillator frequency */
#define BOARD_INTRCOSC_FREQUENCY    (4000000)             /* Internal RC oscillator frequency */

/* This is the clock setup we configure for:
 *
 *   SYSCLK = BOARD_OSCCLK_FREQUENCY = 12MHz  -> Select Main oscillator for
 *                                               source
 *   PLL0CLK = (2 * 20 * SYSCLK) / 1 = 480MHz -> PLL0 multipler=20,
 *                                               pre-divider=1
 *   CCLCK = 480MHz / 4 = 120MHz              -> CCLK divider = 4
 */

#define LPC17_40_CCLK                 120000000 /* 120Mhz */

/* Select the main oscillator as the frequency source.  SYSCLK is then the
 * frequency of the main oscillator.
 */

#undef CONFIG_LPC17_40_MAINOSC
#define CONFIG_LPC17_40_MAINOSC       1
#define BOARD_SCS_VALUE            SYSCON_SCS_OSCEN

/* Select the main oscillator and CCLK divider. The output of the divider is
 * CCLK. The input to the divider (PLLCLK) will be determined by the PLL
 * output.
 */

#define BOARD_CCLKCFG_DIVIDER      4
#define BOARD_CCLKCFG_VALUE        ((BOARD_CCLKCFG_DIVIDER-1) << SYSCON_CCLKCFG_SHIFT)

/* PLL0.  PLL0 is used to generate the CPU clock divider input (PLLCLK).
 *
 *  Source clock:               Main oscillator
 *  PLL0 Multiplier value (M):  20
 *  PLL0 Pre-divider value (N): 1
 *
 *  PLL0CLK = (2 * 20 * SYSCLK) / 1 = 480MHz
 */

#undef CONFIG_LPC17_40_PLL0
#define CONFIG_LPC17_40_PLL0          1
#define BOARD_CLKSRCSEL_VALUE      SYSCON_CLKSRCSEL_MAIN

#define BOARD_PLL0CFG_MSEL         20
#define BOARD_PLL0CFG_NSEL         1
#define BOARD_PLL0CFG_VALUE \
  (((BOARD_PLL0CFG_MSEL-1) << SYSCON_PLL0CFG_MSEL_SHIFT) | \
   ((BOARD_PLL0CFG_NSEL-1) << SYSCON_PLL0CFG_NSEL_SHIFT))

/* PLL1 -- Not used. */

#undef CONFIG_LPC17_40_PLL1
#define BOARD_PLL1CFG_MSEL         36
#define BOARD_PLL1CFG_NSEL         1
#define BOARD_PLL1CFG_VALUE \
  (((BOARD_PLL1CFG_MSEL-1) << SYSCON_PLL1CFG_MSEL_SHIFT) | \
   ((BOARD_PLL1CFG_NSEL-1) << SYSCON_PLL1CFG_NSEL_SHIFT))

/* USB divider.  This divider is used when PLL1 is not enabled to get the
 * USB clock from PLL0:
 *
 *  USBCLK = PLL0CLK / 10 = 48MHz
 */

#define BOARD_USBCLKCFG_VALUE      SYSCON_USBCLKCFG_DIV10

/* FLASH Configuration */

#undef  CONFIG_LP17_FLASH
#define CONFIG_LP17_FLASH          1
#define BOARD_FLASHCFG_VALUE       0x0000403a

/* LED definitions **********************************************************/

/* If CONFIG_ARCH_LEDs is defined, then NuttX will control the LEDs on the
 * PNEV5180B board.  The following definitions describe how NuttX controls
 * the LEDs:
 */

                             /* LD201  LD200  LD202  LD203                 */

                             /* RED    ORANGE BLUE   GREEN                 */
#define LED_STARTED       0  /* ON     ON     ON     ON                    */
#define LED_HEAPALLOCATE  1  /* OFF    OFF    OFF    ON                    */
#define LED_IRQSENABLED   2  /* OFF    OFF    ON     OFF                   */
#define LED_STACKCREATED  3  /* OFF    OFF    OFF    OFF                   */
#define LED_INIRQ         4  /* OFF    OFF    OFF    ON     (momentary)    */
#define LED_SIGNAL        5  /* OFF    OFF    ON     OFF    (momentary)    */
#define LED_ASSERTION     6  /* OFF    ON     OFF    OFF    (momentary)    */
#define LED_PANIC         7  /* ON     OFF    OFF    OFF    (1Hz flashing) */

/* Alternate pin selections *************************************************/

/* Pin Description                      On Board       Connector
 *  -------------------------------- ---------------- -------------
 * P0.2/TXD0/AD0.7                    TX               J201
 * P0.3/RXD0/AD0.6                    RX
 * P0.22/RTS1/TD1                     LD200            ORANGE LED
 * P0.15/TXD1/SCK0/SCK                PN5180-SCK
 * P0.16/RXD1/SSEL0/SSEL              PN5180-SSEL      PN5180
 * P0.17/CTS1/MISO0/MISO              PN5180-MISO
 * P0.18/DCD1/M0SI0/MOSI              PN5180-MOSI
 * P0.19/DSR1/SDA1                    EEPROM           (Not Assembled)
 * P0.20/DTR1/SCL1                    EEPROM
 * P0.21/RI1/RD1                      PN5180-AUX2      PN5180
 * P0.29/USB_D+                       USB-D+           USB
 * P0.30/USB_D-                       USB-D-
 * P2.0/PWM1.1/TXD1                   LD201            RED LED
 * P2.5/PWM1.6/DTR1/TRACEDATA0        PN5180-nPN_RST
 * P2.9/USB_CONNECT/RXD2              USB_CONNECT      USB
 * P2.11/nEINT1/I2STX_CLK             PN5180-BUSY      PN5180
 * P2.12/nEINT2/I2STX_WS              PN5180-IRQ
 * P3.25/MAT0.0/PWM1.2                LD203            GREEN LED
 * P3.26/STCLK/MAT0.1/PWM1.3          LD202            BLUE LED
 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

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

/****************************************************************************
 * Name: lpc17_40_boardinitialize
 *
 * Description:
 *   All LPC17xx/LPC40xx architectures must provide the following entry
 *   point. This entry point is called early in the initialization -- after
 *   all memory has been configured and mapped but before any devices have
 *   been initialized.
 *
 ****************************************************************************/

void lpc17_40_boardinitialize(void);

/****************************************************************************
 * Name: lpc17_40_led
 *
 * Description:
 *   Once the system has booted, these functions can be used to control LED 1
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void lpc17_40_led(int lednum, int state);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_LPC17XX_40XX_PNEV5180_INCLUDE_BOARD_H */
