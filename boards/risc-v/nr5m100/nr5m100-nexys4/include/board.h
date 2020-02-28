/****************************************************************************
 * boards/risc-v/nr5m100/nr5m100-nexys4/include/board.h
 * include/arch/board/board.h
 *
 *   Copyright (C) 2012-2013, 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 *   Modified for NR5M100:
 *
 *   Copyright (C) 2016 Ken Pettit. All rights reserved.
 *   Author: Ken Pettit <pettitkd@gmail.com>
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

#ifndef __BOARDS_RISC-V_NR5M100_NR5M100_NEXYS4_INCLUDE_BOARD_H
#define __BOARDS_RISC-V_NR5M100_NR5M100_NEXYS4_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

#include "nr5.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* The NR5M100_Nexys4 system uses a 100 MHz main oscillator */

#define NR5_HCLK_FREQUENCY      100000000ul
#define NR5_SCLK_FREQUENCY      100000000ul

/* Timer Frequencies are the same as the HCLK frequency */

#define NR5_TIMER_FREQUENCY     NR5_HCLK_FREQUENCY

/* LED definitions **********************************************************/

/* If CONFIG_ARCH_LEDS is not defined, then the user can control
 * the LEDs in any way.
 * The following definitions are used to access individual LEDs.
 */

/* LED index values for use with stm32_setled() */

#if 0
#define BOARD_LED1        0
#define BOARD_LED2        1
#define BOARD_LED3        2
#define BOARD_LED4        3
#endif
#define BOARD_NLEDS       0

#if 1
#define BOARD_LED_GREEN   BOARD_LED1
#define BOARD_LED_ORANGE  BOARD_LED2
#define BOARD_LED_RED     BOARD_LED3
#define BOARD_LED_BLUE    BOARD_LED4

/* LED bits for use with nr5_setleds() */

#define BOARD_LED1_BIT    (1 << BOARD_LED1)
#define BOARD_LED2_BIT    (1 << BOARD_LED2)
#define BOARD_LED3_BIT    (1 << BOARD_LED3)
#define BOARD_LED4_BIT    (1 << BOARD_LED4)

/* If CONFIG_ARCH_LEDs is defined, then NuttX will control the 4 LEDs
 * on the board.
 * The following definitions describe how NuttX controls the LEDs:
 */

#define LED_STARTED       0  /* LED1 */
#define LED_HEAPALLOCATE  1  /* LED2 */
#define LED_IRQSENABLED   2  /* LED1 + LED2 */
#define LED_STACKCREATED  3  /* LED3 */
#define LED_INIRQ         4  /* LED1 + LED3 */
#define LED_SIGNAL        5  /* LED2 + LED3 */
#define LED_ASSERTION     6  /* LED1 + LED2 + LED3 */
#define LED_PANIC         7  /* N/C  + N/C  + N/C + LED4 */

/* Button definitions *******************************************************/

#define BUTTON_USER        0

#define NUM_BUTTONS        5

#define BUTTON_USER_BIT    (1 << BUTTON_USER)

#endif /* 0 */

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

/****************************************************************************
 * Name:  nr5_ledinit, nr5_setled, and nr5_setleds
 *
 * Description:
 *   If CONFIG_ARCH_LEDS is defined,
 *   then NuttX will control the on-board LEDs.
 *   If CONFIG_ARCH_LEDS is not defined,
 *   then the following interfacesare available to
 *   control the LEDs from user applications.
 *
 ****************************************************************************/

#ifndef CONFIG_ARCH_LEDS
void nr5_ledinit(void);
void nr5_setled(int led, bool ledon);
void nr5_setleds(uint8_t ledset);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_RISC-V_NR5M100_NR5M100_NEXYS4_INCLUDE_BOARD_H */
