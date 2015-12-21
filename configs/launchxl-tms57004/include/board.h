/************************************************************************************
 * configs/launchxl-tms57004/include/board.h
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

#ifndef __CONFIGS_LAUNCHXL_TMS57004_INCLUDE_BOARD_H
#define __CONFIGS_LAUNCHXL_TMS57004_INCLUDE_BOARD_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <stdbool.h>
#endif

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Clocking *************************************************************************/
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

#define BOARD_PLL_NR     6   /* REFCLKDIV = 5 */
#define BOARD_PLL_NF     120 /* PLLMUL = 119 * 256 */
#define BOARD_PLL_OD     2   /* ODPLL = 1 */
#define BOARD_PLL_R      2   /* PLLDIV = 1 */

/* FLASH wait states */

#define BOARD_ASWAIT     0   /* No address setup wait states */
#define BOARD_RWAIT      1   /* One read access wait state */
#define BOARD_EWAIT      4   /* Four wait states for EEPROM access */

/* PIN Multiplexor Initializer ******************************************************/

/* You may specify one alternative from each set (the first is the default and,
 * hence, could probably be omitted):
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

/* LED definitions ******************************************************************/

/* Button definitions ***************************************************************/

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

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif  /* __CONFIGS_LAUNCHXL_TMS57004_INCLUDE_BOARD_H */
