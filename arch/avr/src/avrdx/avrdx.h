/****************************************************************************
 * arch/avr/src/avrdx/avrdx.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __ARCH_AVR_SRC_AVRDX_AVRDX_H
#define __ARCH_AVR_SRC_AVRDX_AVRDX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "avrdx_config.h"

#include <stdint.h>
#include <stdbool.h>
#include "avrdx_iodefs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Base address of PORTn peripheral. Not all chip variants have all
 * I/O ports but the gaps are not in order (as in all chips have eg. PORT F,
 * but some don't have eg. PORT B.) To make the array addressable with
 * direct A == 0, B == 1 etc. conversion regardless of the chip, missing
 * ports are not skipped
 */

#define AVRDX_PORT(n) (*(PORT_t *) (0x0400 + n * 0x20))

/* Same thing as above but doesn't return PORT_t. Needed for compile-time
 * assignments into program memory. (Which avr-gcc cannot handle.)
 */

#define AVRDX_PORT_ADDR(n) (0x0400 + n * 0x20)

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
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: up_clkinit
 *
 * Description:
 *   Initialize clock/PLL settings per the definitions in the board.h file.
 *
 ****************************************************************************/

void up_clkinitialize(void);

/****************************************************************************
 * Name: avrdx_current_freq_per
 *
 * Description:
 *   Calculate and return current f_per
 *
 * Assumptions:
 *   Main clock source is internal oscillator
 *
 ****************************************************************************/

uint32_t avrdx_current_freq_per(void);

/****************************************************************************
 * Name: up_consoleinit
 *
 * Description:
 *   Initialize a console for debug output.  This function is called very
 *   early in the initialization sequence to configure the serial console
 *   uart (only).
 *
 ****************************************************************************/

void up_consoleinit(void);

/****************************************************************************
 * Name: avrdx_boardinitialize
 *
 * Description:
 *   This function must be provided by the board-specific logic in the
 *   directory boards/avr/avrdx/<board-name>/src.
 *
 ****************************************************************************/

void avrdx_boardinitialize(void);

/****************************************************************************
 * Name: avrdx_up_initialize
 *
 * Description:
 *   Extension of up_initialize for AVR DA/DB cores
 ****************************************************************************/

void avrdx_up_initialize(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_AVR_SRC_AVRDX_AVRDX_H */
