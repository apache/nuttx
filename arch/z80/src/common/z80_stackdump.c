/****************************************************************************
 * arch/z80/src/common/z80_stackdump.c
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
#include <debug.h>

#include "sched/sched.h"
#include "z80_internal.h"

#ifdef CONFIG_ARCH_STACKDUMP

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: z80_stackdump
 ****************************************************************************/

void z80_stackdump(void)
{
  FAR struct tcb_s *rtcb = running_task();
  uintptr_t sp = up_getsp();
  uintptr_t stack_base = (uintptr_t)rtcb->stack_base_ptr;
  uintptr_t stack_size = (uintptr_t)rtcb->adj_stack_size;
  uintptr_t stack;
  uintptr_t stack_top;

  _alert("stack_base: %06x\n", stack_base);
  _alert("stack_size: %06x\n", stack_size);
  _alert("sp:         %06x\n", sp);

  if (sp >= stack_base && sp < stack_base + stack_size)
    {
      stack = sp;
    }
  else
    {
      _alert("ERROR: Stack pointer is not within allocated stack\n");
      stack = stack_base;
    }

  stack_top = stack_base + stack_size;

  /* Flush any buffered SYSLOG data to avoid overwrite */

  syslog_flush();

  for (stack = stack & ~(8 * sizeof(chipreg_t) - 1);
       stack < (stack_top & ~(8 * sizeof(chipreg_t) - 1));
       stack += 8 * sizeof(chipreg_t))
    {
      FAR chipreg_t *ptr = (FAR chipreg_t *)stack;
      _alert("%06x: %06x %06x %06x %06x %06x %06x %06x %06x\n",
             stack, ptr[0], ptr[1], ptr[2], ptr[3],
             ptr[4], ptr[5], ptr[6], ptr[7]);
    }
}

#endif /* CONFIG_ARCH_STACKDUMP */
