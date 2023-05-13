/****************************************************************************
 * boards/risc-v/k210/maix-bit/include/board.h
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

#ifndef __BOARDS_RISCV_K210_MAIX_BIT_INCLUDE_BOARD_H
#define __BOARDS_RISCV_K210_MAIX_BIT_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

#include "k210.h"

#include "k210_fpioa.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BOARD_LED_PAD     14 /* Connected to red led */

/* Map pad 14 to gpiohs io 0 */

#define BOARD_LED_IO_FUNC K210_IO_FUNC_GPIOHS0
#define BOARD_LED_IO      0

#define LED_STARTED       0  /* N/C */
#define LED_HEAPALLOCATE  1  /* N/C */
#define LED_IRQSENABLED   2  /* N/C */
#define LED_STACKCREATED  3  /* N/C */
#define LED_INIRQ         4  /* N/C */
#define LED_SIGNAL        5  /* N/C */
#define LED_ASSERTION     6  /* N/C */
#define LED_PANIC         7  /* blink */

/* GPIO pins used by the GPIO Subsystem */

#define BOARD_NGPIOOUT    2 /* Amount of GPIO Output pins */
#define BOARD_NGPIOINT    0 /* Amount of GPIO Input */

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

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
 * Name: k210_boardinitialize
 ****************************************************************************/

void k210_boardinitialize(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_RISC-V_K210_MAIX_BIT_INCLUDE_BOARD_H */
