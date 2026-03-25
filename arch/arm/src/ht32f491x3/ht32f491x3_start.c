/****************************************************************************
 * arch/arm/src/ht32f491x3/ht32f491x3_start.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/init.h>
#include <arch/irq.h>

#include "arm_internal.h"

#include "ht32f491x3_lowputc.h"
#include "ht32f491x3_serial.h"
#include "ht32f491x3_start.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FEATURES
#  define showprogress(c) arm_lowputc(c)
#else
#  define showprogress(c)
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

const uintptr_t g_idle_topstack = (uintptr_t)_END_BSS +
  CONFIG_IDLETHREAD_STACKSIZE;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void weak_function ht32f491x3_clockconfig(void)
{
}

void __start(void) __attribute__((section(".entry")));
void __start(void)
{
  const uint32_t *src;
  uint32_t *dest;

  up_irq_disable();

  /* Let the board tune CRM before the early console computes its divisors.
   */

  ht32f491x3_clockconfig();
  ht32f491x3_lowsetup();
  showprogress('A');

  for (dest = (uint32_t *)_START_BSS; dest < (uint32_t *)_END_BSS; )
    {
      *dest++ = 0;
    }

  showprogress('B');

  src  = (const uint32_t *)_DATA_INIT;
  dest = (uint32_t *)_START_DATA;

  while (dest < (uint32_t *)_END_DATA)
    {
      *dest++ = *src++;
    }

  showprogress('C');

  arm_fpuconfig();
  showprogress('D');

#ifdef USE_EARLYSERIALINIT
  arm_earlyserialinit();
#endif
  showprogress('E');

  ht32f491x3_boardinitialize();
  showprogress('F');

  showprogress('\r');
  showprogress('\n');

  nx_start();

  for (; ; );
}
