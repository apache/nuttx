/****************************************************************************
 * arch/z80/src/common/up_stackdump.c
 *
 *   Copyright (C) 2007-2009, 2016, 2019 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <debug.h>

#include "up_arch.h"
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
      FAR uint8_t *ptr = (FAR uint8_t*)stack;

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
