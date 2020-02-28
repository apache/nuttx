/****************************************************************************
 * boards/arm/nuc1xx/nutiny-nuc120/include/board.h
 * include/arch/board/board.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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

#ifndef __BOARDS_ARM_NUC1XX_NUTINY_NUC12_INCLUDE_BOARD_H
#define __BOARDS_ARM_NUC1XX_NUTINY_NUC12_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* Crystal frequencies */

#define BOARD_XTALHI_FREQUENCY 12000000
#define BOARD_XTALLO_FREQUENCY 32768

/* PLL: The PLL must be 48MHz x N times when using USB
 *
 * FOUT = FIN x (NF/NR) x (1 / NO)
 * FIN  = Input reference clock frequency
 * NF   = Feedback divider
 *      = (FB_DV + 2)
 * NR   = Input divider
 *      = (IN_DV + 2)
 * NO   = 1 if OUT_DV == 0
 *        2 if OUT_DV == 1 or 2
 *        4 if OUT_DV == 3
 *
 * FOUT = 12000000 x 48 / 3 / 4
 *      = 48MHz
 */

#define BOARD_PLL_FIN    BOARD_XTALHI_FREQUENCY
#define BOARD_PLL_FB_DV  46
#define BOARD_PLL_NF     (BOARD_PLL_FB_DV+2)
#define BOARD_PLL_IN_DV  1
#define BOARD_PLL_NR     (BOARD_PLL_IN_DV+2)
#define BOARD_PLL_OUT_DV 3
#define BOARD_PLL_NO     4

#define BOARD_PLL_FOUT \
  (BOARD_PLL_FIN * BOARD_PLL_NF / BOARD_PLL_NR / BOARD_PLL_NO)

/* HCLK. FOUT is the HCLK source clock. */

#define BOARD_HCLK_N     1
#define BOARD_HCLK_FREQUENCY (BOARD_PLL_FOUT / BOARD_HCLK_N)

/* USB. FOUT is the source.  The USB CLK must be 48MHz */

#define BOARD_USB_N     1
#define BOARD_USB_FREQUENCY (BOARD_PLL_FOUT / BOARD_USB_N)

/* LED definitions **********************************************************/

/* The NuTiny has a single green LED that can be controlled from software.
 * This LED is connected to PIN17.
 * It is pulled high so a low value will illuminate the LED.
 */

#define BOARD_NLEDS       1

/* If CONFIG_ARCH_LEDs is defined, then NuttX will control the LED on board
 * the NuTiny.
 * The following definitions describe how NuttX controls the LEDs:
 *
 *   SYMBOL                Meaning                 LED state
 *                                                 Initially all LED is OFF
 *   -------------------  -----------------------  ------------- ------------
 *   LED_STARTED          NuttX has been started   LED ON
 *   LED_HEAPALLOCATE     Heap has been allocated  LED ON
 *   LED_IRQSENABLED      Interrupts enabled       LED ON
 *   LED_STACKCREATED     Idle stack created       LED ON
 *   LED_INIRQ            In an interrupt          LED should glow
 *   LED_SIGNAL           In a signal handler      LED might glow
 *   LED_ASSERTION        An assertion failed      LED ON while handling the
 *                                                        assertion
 *   LED_PANIC            The system has crashed   LED Blinking at 2Hz
 *   LED_IDLE             NUC1XX is is sleep mode   (Optional, not used)
 */

#define LED_STARTED       0
#define LED_HEAPALLOCATE  0
#define LED_IRQSENABLED   0
#define LED_STACKCREATED  0
#define LED_INIRQ         0
#define LED_SIGNAL        0
#define LED_ASSERTION     0
#define LED_PANIC         0

/* Button definitions *******************************************************/

/* The NuTiny has no buttons */

#define NUM_BUTTONS        0

#endif /* __BOARDS_ARM_NUC1XX_NUTINY_NUC12_INCLUDE_BOARD_H */
