/****************************************************************************
 * boards/arm/s32k1xx/s32k118evb/include/board.h
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
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

#ifndef __BOARDS_ARM_STM32F4DISCOVERY_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32F4DISCOVERY_INCLUDE_BOARD_H

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

/* The S32K118EVB is fitted with a 40MHz Crystal */

#define BOARD_XTAL_FREQUENCY 40000000

/* The S32K118 will run at 48MHz *.

/* LED definitions **********************************************************/

/* The S32K118EVB has one RGB LED:
 *
 *   RedLED   PTD16 (FTM0CH1)
 *   BlueLED  PTE8  (FTM0CH6)
 *   GreenLED PTD15 (FTM0 CH0)
 *
 * If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in
 * any way.  The following definitions are used to access individual RGB
 * components.
 *
 * The RGB components could, alternatively be controlled through PWM using
 * the common RGB LED driver.
 */

/* LED index values for use with board_userled() */

#define BOARD_RED          0
#define BOARD_BLUE         1
#define BOARD_GREEN        2
#define BOARD_NLEDS        3

/* LED bits for use with board_userled_all() */

#define BOARD_REDLED_BIT   (1 << BOARD_RED)
#define BOARD_GREENLED_BIT (1 << BOARD_BLUE)
#define BOARD_BLUELED_BIT  (1 << BOARD_GREEN)

/* If CONFIG_ARCH_LEDs is defined, then NuttX will control the LEDs on board
 * the s32k118evb.  The following definitions describe how NuttX controls the
 * LEDs:
 */
                                /* RED      GREEN    BLUE */
#define LED_STARTED         0   /* OFF      OFF      OFF  */
#define LED_HEAPALLOCATE    0   /* OFF      OFF      OFF  */
#define LED_IRQSENABLED     0   /* OFF      OFF      OFF  */
#define LED_STACKCREATED    1   /* OFF      ON       OFF  */
#define LED_INIRQ           2   /* OFF      N/C      ON   */
#define LED_SIGNAL          2   /* OFF      N/C      ON   */
#define LED_ASSERTION       2   /* OFF      N/C      ON   */
#define LED_PANIC           3   /* Flashing OFF      N/C  */

/* Button definitions *******************************************************/

/* The S32K118EVB supports two buttons:
 *
 *   SW2  PTD3
 *   SW3  PTD5
 */

#define BUTTON_SW2         0
#define BUTTON_SW3         1
#define NUM_BUTTONS        2

#define BUTTON_SW2_BIT    (1 << BUTTON_SW2)
#define BUTTON_SW3_BIT    (1 << BUTTON_SW3)

/* Alternate function pin selections ****************************************/


/* DMA Channel/Stream Selections ********************************************/


#endif  /* __BOARDS_ARM_STM32F4DISCOVERY_INCLUDE_BOARD_H */
