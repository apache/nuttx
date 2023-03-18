/****************************************************************************
 * boards/avr/atmega/mega1284p-xplained/include/board.h
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

#ifndef __BOARDS_AVR_ATMEGA_MEGA1284P_XPLAINED_INCLUDE_BOARD_H
#define __BOARDS_AVR_ATMEGA_MEGA1284P_XPLAINED_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Clocking *****************************************************************/

/* Assume default CLKDIV8 fuse setting is overridden to CLKDIV1 */

#define BOARD_XTAL_FREQ        11059200         /* 11.0592MHz crystal */
#define BOARD_CPU_CLOCK        BOARD_XTAL_FREQ  /* F_CPU = 16MHz */

/* LED definitions **********************************************************/

/* The Mega1284p-Xplained has 4 LEDs connected to PB0-PB3 */

#define LED_STARTED                0 /* OFF     ON  (never happens) */
#define LED_HEAPALLOCATE           0 /* OFF     ON  (never happens) */
#define LED_IRQSENABLED            0 /* OFF     ON  (never happens) */
#define LED_STACKCREATED           1 /* ON      ON  (never happens) */
#define LED_INIRQ                  2 /* OFF     NC  (momentary) */
#define LED_SIGNAL                 2 /* OFF     NC  (momentary) */
#define LED_ASSERTION              2 /* OFF     NC  (momentary) */
#define LED_PANIC                  0 /* OFF     ON  (1Hz flashing) */

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_AVR_ATMEGA_MEGA1284P_XPLAINED_INCLUDE_BOARD_H */
