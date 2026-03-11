/****************************************************************************
 * arch/avr/src/avrdx/avrdx_swint.h
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

#ifndef __ARCH_AVR_SRC_AVRDX_AVRDX_SWINT_H
#define __ARCH_AVR_SRC_AVRDX_AVRDX_SWINT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

#if (CONFIG_AVRDX_SWINT_MAX_VECTORS <= 8)
typedef uint8_t avrdx_swint_size_t;
#elif (CONFIG_AVRDX_SWINT_MAX_VECTORS <= 16)
typedef uint16_t avrdx_swint_size_t;
#elif (CONFIG_AVRDX_SWINT_MAX_VECTORS <= 24)
typedef uint24_t avrdx_swint_size_t;
#elif (CONFIG_AVRDX_SWINT_MAX_VECTORS <= 32)
typedef uint32_t avrdx_swint_size_t;
#else
#  error Kconfig should allow 32 vectors at most
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: avrdx_swint_allocate
 *
 * Description:
 *   Allocates software interrupt vector for the caller and returns it,
 *   if any vector is available
 *
 * Input Parameters:
 *   Handler - to be executed in interrupt context when triggered
 *   Opaque arg argument - passed to the handler
 *
 * Returned Value:
 *   Software interrupt vector number to be used in avrdx_swint_raise
 *   or zero if no vectors are available. The number is single bit mask
 *   so it can be quickly OR'ed with others. Treat it as opaque value though.
 *
 ****************************************************************************/

avrdx_swint_size_t avrdx_swint_allocate(xcpt_t handler, void *arg);

/****************************************************************************
 * Name: avrdx_swint_raise
 *
 * Description:
 *   Raises software interrupt with given vector number. Note that this
 *   function is designed to be runnable from code that runs outside
 *   of NuttX - this mainly means so called zero latency interrupts.
 *
 * Input Parameters:
 *   vector - number that identifies which software interrupt was raised
 *   (use return value from avrdx_swint_allocate.)
 *
 ****************************************************************************/

void avrdx_swint_raise(avrdx_swint_size_t vector);

/****************************************************************************
 * Name: avrdx_swint_initialize
 *
 * Description:
 *   Initialize hardware related to software interrupts.
 *
 ****************************************************************************/

int avrdx_swint_initialize(void);

#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */

#endif
