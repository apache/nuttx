/****************************************************************************
 * boards/renesas/sh1/us7032evb1/include/board.h
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

#ifndef __BOARDS_RENESAS_SH1_US7032EVB1_INCLUDE_BOARD_H
#define __BOARDS_RENESAS_SH1_US7032EVB1_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

#define SH1_CLOCK            20000000  /* 20 MHz */

/* LED definitions **********************************************************/

/* The SH1_LPEVB only a single LED controlled by either port A, pin 15, or
 * port B, pin 15 (selectable via JP8).
 */

#define LED_STARTED          0
#define LED_HEAPALLOCATE     1
#define LED_IRQSENABLED      1
#define LED_STACKCREATED     1
#define LED_INIRQ            0
#define LED_SIGNAL           0
#define LED_ASSERTION        0
#define LED_PANIC            1

/* Button definitions *******************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __BOARDS_RENESAS_SH1_US7032EVB1_INCLUDE_BOARD_H */
