/****************************************************************************
 * boards/arm/lc823450/lc823450-xgevk/include/board.h
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

#ifndef __BOARDS_ARM_LC823450_LC823450_XGEVK_INCLUDE_BOARD_H
#define __BOARDS_ARM_LC823450_LC823450_XGEVK_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LED_STARTED       0  /* N/A */
#define LED_HEAPALLOCATE  1  /* N/A */
#define LED_IRQSENABLED   2  /* N/A */
#define LED_STACKCREATED  3  /* N/A */
#define LED_INIRQ         4  /* N/A */
#define LED_SIGNAL        5  /* N/A */
#define LED_ASSERTION     6  /* N/A */
#define LED_PANIC         7  /* N/A */
#define LED_CPU0          8  /* LED0 (D9) */
#define LED_CPU1          9  /* LED1 (D10) */

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

extern unsigned int XT1OSC_CLK;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

void up_init_default_mux(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_LC823450_LC823450_XGEVK_INCLUDE_BOARD_H */
