/************************************************************
 * common/up_assert.c
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
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
 ************************************************************/

/************************************************************
 * Included Files
 ************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdlib.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "up_arch.h"
#include "os_internal.h"
#include "up_internal.h"

/************************************************************
 * Definitions
 ************************************************************/

/* Output debug info if stack dump is selected -- even if 
 * debug is not selected.
 */

#ifdef CONFIG_ARCH_STACKDUMP
# undef  lldbg
# define lldbg lib_lowprintf
#endif

/************************************************************
 * Private Data
 ************************************************************/

/************************************************************
 * Private Functions
 ************************************************************/

/************************************************************
 * Name: up_getsp
 ************************************************************/

/* I don't know if the builtin to get SP is enabled */

static inline uint32 up_getsp(void)
{
  uint32 sp;
  __asm__
  (
    "\tmov %0, sp\n\t"
    : "=r"(sp)
  );
  return sp;
}

/************************************************************
 * Name: up_stackdump
 ************************************************************/

#ifdef CONFIG_ARCH_STACKDUMP
static void up_stackdump(void)
{
  _TCB *rtcb        = (_TCB*)g_readytorun.head;
  uint32 sp         = up_getsp();
  uint32 stack_base;
  uint32 stack_size;

  if (rtcb->pid == 0)
    {
      stack_base = g_heapbase - 4;
      stack_size = CONFIG_PROC_STACK_SIZE;
    }
  else
    {
      stack_base = (uint32)rtcb->adj_stack_ptr;
      stack_size = (uint32)rtcb->adj_stack_size;
    }

  lldbg("stack_base: %08x\n", stack_base);
  lldbg("stack_size: %08x\n", stack_size);
  lldbg("sp:         %08x\n", sp);

  if (sp >= stack_base || sp < stack_base - stack_size)
    {
      lldbg("ERROR: Stack pointer is not within allocated stack\n");
      return;
    }
  else
    {
      uint32 stack = sp & ~0x1f;

      for (stack = sp & ~0x1f; stack < stack_base; stack += 32)
        {
          uint32 *ptr = (uint32*)stack;
          lldbg("%08x: %08x %08x %08x %08x %08x %08x %08x %08x\n",
                 stack, ptr[0], ptr[1], ptr[2], ptr[3],
                 ptr[4], ptr[5], ptr[6], ptr[7]);
        }
    }

  if (current_regs)
    {
      int regs;

      for (regs = REG_R0; regs <= REG_R15; regs += 8)
        {
          uint32 *ptr = (uint32*)&current_regs[regs];
          lldbg("R%d: %08x %08x %08x %08x %08x %08x %08x %08x\n",
                 regs, ptr[0], ptr[1], ptr[2], ptr[3],
                 ptr[4], ptr[5], ptr[6], ptr[7]);
        }
      lldbg("CPSR: %08x\n", current_regs[REG_CPSR]);
    }
}
#else
# define up_stackdump()
#endif

/************************************************************
 * Name: _up_assert
 ************************************************************/

static void _up_assert(int errorcode) /* __attribute__ ((noreturn)) */
{
  /* Are we in an interrupt handler or the idle task? */

  if (current_regs || ((_TCB*)g_readytorun.head)->pid == 0)
    {
       (void)irqsave();
        for(;;)
          {
#ifdef CONFIG_ARCH_LEDS
            up_ledon(LED_PANIC);
            up_mdelay(250);
            up_ledoff(LED_PANIC);
            up_mdelay(250);
#endif
          }
    }
  else
    {
      exit(errorcode);
    }
}

/************************************************************
 * Public Funtions
 ************************************************************/

/************************************************************
 * Name: up_assert
 ************************************************************/

void up_assert(const ubyte *filename, int lineno)
{
#if CONFIG_TASK_NAME_SIZE > 0
  _TCB *rtcb = (_TCB*)g_readytorun.head;
#endif

  up_ledon(LED_ASSERTION);
#if CONFIG_TASK_NAME_SIZE > 0
  lldbg("Assertion failed at file:%s line: %d task: %s\n",
        filename, lineno, rtcb->name);
#else
  lldbg("Assertion failed at file:%s line: %d\n",
        filename, lineno);
#endif
  up_stackdump();
  _up_assert(EXIT_FAILURE);
}

/************************************************************
 * Name: up_assert_code
 ************************************************************/

void up_assert_code(const ubyte *filename, int lineno, int errorcode)
{
#if CONFIG_TASK_NAME_SIZE > 0
  _TCB *rtcb = (_TCB*)g_readytorun.head;
#endif

  up_ledon(LED_ASSERTION);
#if CONFIG_TASK_NAME_SIZE > 0
  lldbg("Assertion failed at file:%s line: %d task: %s error code: %d\n",
        filename, lineno, rtcb->name, errorcode);
#else
  lldbg("Assertion failed at file:%s line: %d error code: %d\n",
        filename, lineno, errorcode);
#endif
  up_stackdump();
  _up_assert(errorcode);
}
