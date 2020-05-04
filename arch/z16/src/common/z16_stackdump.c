/****************************************************************************
 * arch/z16/src/common/z16_stackdump.c
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

#include <debug.h>

#include "chip.h"
#include "sched/sched.h"
#include "z16_internal.h"

#ifdef CONFIG_ARCH_STACKDUMP

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: z16_getsp
 ****************************************************************************/

/* To be provided */

/****************************************************************************
 * Name: z16_stackdump
 ****************************************************************************/

static void z16_stackdump(void)
{
  struct tcb_s *rtcb = this_task();
  chipreg_t sp = z16_getsp();
  chipreg_t stack_base = (chipreg_t)rtcb->adj_stack_ptr;
  chipreg_t stack_size = (chipreg_t)rtcb->adj_stack_size;
  chipreg_t stack;

  _alert("stack_base: %08x\n", stack_base);
  _alert("stack_size: %08x\n", stack_size);
  _alert("sp:         %08x\n", sp);

  if (sp >= stack_base || sp < stack_base - stack_size)
    {
      _err("ERROR: Stack pointer is not within allocated stack\n");
      stack = stack_base - stack_size;
    }
  else
    {
      stack = sp;
    }

  for (stack = stack & ~0x0f;
       stack < stack_base;
       stack += 8 * sizeof(chipreg_t))
    {
      chipreg_t *ptr = (chipreg_t *)stack;
      _alert("%08x: %08x %08x %08x %08x %08x %08x %08x %08x\n",
            stack, ptr[0], ptr[1], ptr[2], ptr[3],
            ptr[4], ptr[5], ptr[6], ptr[7]);
    }
}

#endif /* CONFIG_ARCH_STACKDUMP */
