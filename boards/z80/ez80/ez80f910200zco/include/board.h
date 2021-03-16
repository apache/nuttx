/****************************************************************************
 * boards/z80/ez80/ez80f910200zco/include/board.h
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

#ifndef __BOARDS_Z80_EZ80_EZ80F910200ZCO_INCLUDE_BOARD_H
#define __BOARDS_Z80_EZ80_EZ80F910200ZCO_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking */

#define EZ80_SYS_CLK_FREQ           50000000

/* LED pattern definitions                 ON                OFF            */

#define LED_STARTED                 0  /*  '0'               N/A            */
#define LED_HEAPALLOCATE            1  /*  'H'               N/A            */
#define LED_IRQSENABLED             2  /*  'E'               N/A            */
#define LED_STACKCREATED            3  /*  'C'               N/A            */
#define LED_IDLE                    4  /*  'R'               N/A            */
#define LED_INIRQ                   5  /*  (ignored)        (ignored)       */
#define LED_ASSERTION               6  /*  'A'              (previous)      */
#define LED_SIGNAL                  7  /*  'S'              (previous)      */
#define LED_PANIC                   8  /*  '*'              (previous)      */

/* Button definitions */

#define BUTTON_PB0                 0x01 /* PB0: SW1 Bit 0 of GPIO Port B    */
#define BUTTON_PB1                 0x02 /* PB1: SW2 Bit 1 of GPIO Port B    */
#define BUTTON_PB2                 0x04 /* PB2: SW3 Bit 2 of GPIO Port B    */

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

#endif /* __BOARDS_Z80_EZ80_EZ80F910200ZCO_INCLUDE__BOARD_H */
