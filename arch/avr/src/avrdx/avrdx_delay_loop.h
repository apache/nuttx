/****************************************************************************
 * arch/avr/src/avrdx/avrdx_delay_loop.h
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

#ifndef __ARCH_AVR_ETC_ETC_H
#define __ARCH_AVR_ETC_ETC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
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
 * Name: avrdx_delay_loop
 *
 * Description:
 *   Spins in a loop to cause a delay. On chips with AVRxt cores, it takes:
 *   (3 + count * 6 + call) cycles where count is the parameter and call
 *   is the number of cycles needed for instruction that jumps into this
 *   function (2 cycles for RCALL or 3 cycles for CALL, depends on what
 *   the compiler/linker does.)
 *
 * Input Parameters:
 *   Loop count - uint32_t (registers r22:r25)
 *
 * Return value: none
 *
 * Assumptions/Limitations:
 *   - only called from avrdx_udelay
 *   - count is not zero
 *
 ****************************************************************************/

void avrdx_delay_loop(uint32_t count);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */

#endif
