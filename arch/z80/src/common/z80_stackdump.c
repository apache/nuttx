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

#include "z80_arch.h"
#include "sched/sched.h"
#include "z80_internal.h"

#ifdef CONFIG_ARCH_STACKDUMP

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_stackdump
 ****************************************************************************/

void up_stackdump(void)
{
  FAR struct tcb_s *rtcb = this_task();
  uintptr_t sp = z80_getsp();
  uintptr_t stack_base = (uintptr_t)rtcb->adj_stack_ptr;
  uintptr_t stack_size = (uintptr_t)rtcb->adj_stack_size;
  uintptr_t stack;

  _alert("stack_base: %06x\n", stack_base);
  _alert("stack_size: %06x\n", stack_size);
  _alert("sp:         %06x\n", sp);

  if (sp >= stack_base || sp < stack_base - stack_size)
    {
      _alert("ERROR: Stack pointer is not within allocated stack\n");
      stack = stack_base - stack_size;
    }
  else
    {
      stack = sp;
    }

  for (stack = stack & ~0x0f; stack < stack_base; stack += 16)
    {
      FAR uint8_t *ptr = (FAR uint8_t *)stack;

      _alert("%06x: %02x %02x %02x %02x %02x %02x %02x %02x  ",
             "%02x %02x %02x %02x %02x %02x %02x %02x\n",
             stack,
             ptr[0],  ptr[1],  ptr[2],  ptr[3],
             ptr[4],  ptr[5],  ptr[6],  ptr[7],
             ptr[8],  ptr[9],  ptr[10], ptr[11],
             ptr[12], ptr[13], ptr[14], ptr[15]);
    }
}

#endif /* CONFIG_ARCH_STACKDUMP */
