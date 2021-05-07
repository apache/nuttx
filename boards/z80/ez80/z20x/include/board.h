/****************************************************************************
 * boards/z80/ez80/z20x/include/board.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __BOARDS_Z80_EZ80_Z20X_INCLUDE_BOARD_H
#define __BOARDS_Z80_EZ80_Z20X_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking
 *
 * The z20x CPU board has a 20Mhz crystal. The eZ80F92 has no PLL; the
 * system clock frequency is equal to the crystal frequency of 20MHz, the
 * maximum for the eZ80F92 part.
 */

#define EZ80_SYS_CLK_FREQ           20000000

/* LEDs */

/* The D3 GREEN LED is driven by an eZ80 GPI/O pin.  However, it has some
 * additional properties:
 *
 * 1. On input, it will be '1' if the I/O expansion board is present.
 * 2. Setting it to an output of '0' will generate a system reset.
 * 3. Setting it to an output of '1' will not only illuminate the LED
 *    take the card out of reset and enable power to the SD card slot.
 *
 * As a consequence, the GREEN LED will not be illuminated if SD card
 * support or SPI is disabled.  The only effect of CONFIG_ARCH_LEDS is that
 * the GREEN LED will turned off in the event of a crash.
 */

#define LED_STARTED                 0
#define LED_HEAPALLOCATE            0
#define LED_IRQSENABLED             0
#define LED_STACKCREATED            0
#define LED_IDLE                    0
#define LED_INIRQ                   0
#define LED_ASSERTION               0
#define LED_SIGNAL                  0
#define LED_PANIC                   1

/* Button definitions
 * The z20x CPU board has no on-board buttons that can be sensed by the
 * eZ80.
 */

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __BOARDS_Z80_EZ80_Z20X_INCLUDE_BOARD_H */
