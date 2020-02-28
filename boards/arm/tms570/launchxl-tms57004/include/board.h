/****************************************************************************
 * boards/arm/tms570/launchxl-tms57004/include/board.h
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

#ifndef __BOARDS_ARM_TMS570_LAUNCHXL_TMS57004_INCLUDE_BOARD_H
#define __BOARDS_ARM_TMS570_LAUNCHXL_TMS57004_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <stdbool.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* The LaunchXL-TMS57004 has a 16 MHz external crystal. */

#define BOARD_FCLKIN_FREQUENCY 16000000  /* 16 MHz crystal frequency */

/* The maximum frequency for the TMS570LS0432PZ is 80 MHz.
 *
 * REFCLKDIV controls input clock divider:
 *
 *  NR = REFCLKDIV+1
 *  Fintclk = Fclkin / NR
 *
 * PLLMUL controls multipler on divided input clock (Fintclk):
 *
 *  Non-modulated:
 *    NF = (PLLMUL + 256) / 256
 *  Modulated:
 *    NF = (PLLMUL + MULMOD + 256) / 256
 *
 *  Foutputclk = Fintclk x NF (150MHz - 550MHz)
 *
 * ODPLL controls internal PLL output divider:
 *
 *   OD = ODPLL+1
 *   Fpostodclk = Foutputclock / OD
 *
 * Final divisor, R, controls PLL output:
 *
 *   R = PLLDIV + 1
 *   Fpllclock = Fpostodclk / R
 *
 * Or:
 *
 *   Fpllclock = = (Fclkin / NR) x NF / OD / R
 *
 * In this case, we have:
 *
 *   Fclkin = 16,000,000
 *   NR     = 6   (REFCLKDIV=5)
 *   NF     = 120 (PLLMUL = 119 * 256)
 *   OD     = 1   (ODPLL = 0)
 *   R      = 2   (PLLDIV=1)
 *
 * Then:
 *
 *   Fintclk      = 16 MHz / 6      = 2.667 MHz
 *   Foutputclock = 2.667 MHz * 120 = 320 MHz
 *   Fpostodclock = 320 MHz / 2     = 160 MHz
 *   Fpllclock    = 160 MHz / 2     = 80 MHz
 */

#define BOARD_PLL_NR           6   /* REFCLKDIV = 5 */
#define BOARD_PLL_NF           120 /* PLLMUL = 119 * 256 */
#define BOARD_PLL_OD           2   /* ODPLL = 1 */
#define BOARD_PLL_R            2   /* PLLDIV = 1 */
#define BOARD_PLL_FREQUENCY    80000000

/* Clock Sources / Dividers
 *
 * GCLK and HCLK are both driven by PLL1.
 * VCLK is driven by HCLK  (optionally by HCLK/2)
 * RTICLK source is VCLK/2 (optionally from VCLK)
 */

#define BOARD_VCLK_DIVIDER     1
#define BOARD_VCLK2_DIVIDER    1
#define BOARD_RTICLK_DIVIDER   2

/* Resulting frequencies:
 *
 * GCLK and HCLK are both driven by PLL1.
 * VCLK is driven by HCLK  (optionally by HCLK/2)
 * RTICLK source is VCLK/2 (optionally from VCLK)
 */

#define BOARD_GCLK_FREQUENCY   BOARD_PLL_FREQUENCY
#define BOARD_HCLK_FREQUENCY   BOARD_PLL_FREQUENCY
#define BOARD_VCLK_FREQUENCY   (BOARD_HCLK_FREQUENCY / BOARD_VCLK_DIVIDER)
#define BOARD_RTICLK_FREQUENCY (BOARD_VCLK_FREQUENCY / BOARD_RTICLK_DIVIDER)

/* FLASH wait states */

#define BOARD_ASWAIT           0   /* No address setup wait states */
#define BOARD_RWAIT            1   /* One read access wait state */
#define BOARD_EWAIT            4   /* Four wait states for EEPROM access */

/* PIN Multiplexor Initializer **********************************************/

/* You may specify one alternative from each set (the first is the default
 * and, hence, could probably be omitted):
 *
 *  1. {GIOA0, SPI3nCS3}
 *  2. {GIOA1, SPI3nCS2}
 *  3. {GIOA2, SPI3nCS1}
 *  4. {GIOA3, SPI2nCS3}
 *  5. {GIOA4, SPI2nCS2}
 *  6. {GIOA5, EXTCLKIN}
 *  7. {GIOA6, SPI2nCS1, N2HET31}
 *  8. {GIOA7, N2HET29}
 *  9. {MIBSPI1nCS2, N2HET20, N2HET19}
 * 10. {SPI3CLK, EQEPA}
 * 11. {SPI3nENA, EQEPB}
 * 12. {SPI3nCS0, EQEPI}}
 * 13. {MIBSPI1nCS3, N2HET26}
 * 14. {ADEVT, N2HET28}
 * 15. {MIBSPI1nENA, N2HET23, NHET30}
 * 16. {MIBSPI1nCS1, EQEPS, N2HET17}
 */

#define BOARD_PINMUX_INITIALIZER \
  PINMUX_GIOA0_PIN, \
  PINMUX_GIOA1_PIN, \
  PINMUX_GIOA2_PIN, \
  PINMUX_GIOA3_PIN, \
  PINMUX_GIOA4_PIN, \
  PINMUX_GIOA5_PIN, \
  PINMUX_GIOA6_PIN, \
  PINMUX_GIOA7_PIN, \
  PINMUX_MIBSPI1NCS2_PIN, \
  PINMUX_SPI3CLK_PIN, \
  PINMUX_SPI3NENA_PIN, \
  PINMUX_SPI3NCS0_PIN, \
  PINMUX_MIBSPI1NCS3_PIN, \
  PINMUX_ADEVT_PIN, \
  PINMUX_MIBSPI1NENA_PIN, \
  PINMUX_MIBSPI1NCS1_PIN

/* LED definitions **********************************************************/

/* LEDs
 *
 * The launchpad has several LEDs:
 *
 *   - LEd D1 (white) that connects to the USB +5V supply,
 *   - LED D10 (red) that connects to the TMS570's NERROR pin,
 *   - D5 (blue), D6 (blue), and D8 (blue) connect to the XDS100 FT2322,
 *   - D7 (blue) connects to the XSD100 CPLD, and
 *   - Two white, user LEDs labeled D12 that connects to the NHET08
 *     pin and D11 that connects to GIOA2.
 *
 * NHET08 is one of 32 N2HET pins than can be available to the user if not
 * used by N2HET.
 * This implementation, however, uses only the single LED driven by GIOA2.
 * That LED is tied to ground and illuminated with a high level output value.
 */

/* LED index values for use with board_userled() */

#define BOARD_LED_D11        0
#define BOARD_NLEDS          1

/* LED bits for use with board_userled_all() */

#define BOARD_LED_D11_BIT    (1 << BOARD_LED_D11)

/* This LED is not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.  In that case, the usage by the board port is defined in
 * include/board.h and src/tms570_autoleds.c. The LED is used to encode
 * OS-related events as follows:
 */

/* These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.  In that case, the usage by the board port is defined in
 * include/board.h and src/sam_autoleds.c. The LEDs are used to encode
 * OS-related events as follows:
 *
 *   ---------------------- ---------------------------- ------
 *   SYMBOL                 Meaning                      LED
 *   ---------------------- ---------------------------- ------
 */

#define LED_STARTED         0 /* NuttX has been started  OFF      */
#define LED_HEAPALLOCATE    0 /* Heap has been allocated OFF      */
#define LED_IRQSENABLED     0 /* Interrupts enabled      OFF      */
#define LED_STACKCREATED    1 /* Idle stack created      ON       */
#define LED_INIRQ           2 /* In an interrupt         N/C      */
#define LED_SIGNAL          2 /* In a signal handler     N/C      */
#define LED_ASSERTION       2 /* An assertion failed     N/C      */
#define LED_PANIC           3 /* The system has crashed  FLASH    */
#undef  LED_IDLE              /* MCU is is sleep mode    Not used */

/* Thus if the LED is statically on, NuttX has successfully  booted and is,
 * apparently, running normally.  If the LED is flashing at approximately
 * 2Hz, then a fatal error has been detected and the system has halted.
 */

/* Button definitions *******************************************************/

/* Buttons
 *
 *  The launchpad has three mechanical buttons.
 *  Two of these are reset buttons:
 *  One  button is labeled PORRST performs a power-on reset and one labeled
 *  RST performs an MCU reset.
 *  Only one button is available for general software usage.
 *  That button is labeled GIOA7 and is, obviously, sensed on GIOA7.
 *
 * GIOA7 is tied to ground, but will be pulled high if the GIOA7 button is
 * depressed.
 */

#define BUTTON_GIOA7        0
#define NUM_BUTTONS         1

#define BUTTON_GIOA7_BIT    (1 << BUTTON_GIOA7)

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
#endif /* __BOARDS_ARM_TMS570_LAUNCHXL_TMS57004_INCLUDE_BOARD_H */
