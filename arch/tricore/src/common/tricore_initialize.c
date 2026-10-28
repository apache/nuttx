/****************************************************************************
 * arch/tricore/src/common/tricore_initialize.c
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

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/init.h>

#include "tricore_internal.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

volatile uintptr_t *g_current_regs[CONFIG_SMP_NCPUS];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if defined(CONFIG_STACK_COLORATION) && CONFIG_ARCH_INTERRUPTSTACK > 15
static inline void up_color_intstack(void)
{
  uint32_t *ptr = (uint32_t *)g_intstackalloc;
  ssize_t size;

  for (size = (CONFIG_ARCH_INTERRUPTSTACK & ~15);
       size > 0;
       size -= sizeof(uint32_t))
    {
      *ptr++ = INTSTACK_COLOR;
    }
}
#else
#  define up_color_intstack()
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_initialize(void)
{
  up_color_intstack();

  aurix_earlyserialinit();
  aurix_serialinit();
}

void tricore_start(int current_cpu)
{
  (void)current_cpu;
  nx_start();
}
