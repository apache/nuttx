/****************************************************************************
 * arch/arm/src/rtl8720c/ameba_vectors.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"
#include "arm_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

extern uint32_t __stack;
#define IDLE_STACK      ((unsigned int)&__stack - 4)
#ifndef ARMV8M_PERIPHERAL_INTERRUPTS
#  error ARMV8M_PERIPHERAL_INTERRUPTS must be defined to the number of I/O interrupts to be supported
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Chip-specific entrypoint */

extern void ram_start(void);

/* Common exception entrypoint */

extern void exception_common(void);

/****************************************************************************
 * Public data
 ****************************************************************************/

/* The v7m vector table consists of an array of function pointers,
 * with the first
 * slot (vector zero) used to hold the initial stack pointer.
 *
 * As all exceptions (interrupts) are routed via exception_common,
 * we just need to
 * fill this array with pointers to it.
 *
 * Note that the [ ... ] designated initialiser is a GCC extension.
 */

const unsigned int _vectors[] locate_data(".vectors") \
                    aligned_data(0x100) =
{
  /* Initial stack */

  IDLE_STACK,

  /* Reset exception handler */

  (unsigned int) &ram_start,

  /* Vectors 2 - n point directly at the generic handler */

  [2 ...(15 + ARMV8M_PERIPHERAL_INTERRUPTS)] = (unsigned int)
                                               &exception_common
};

