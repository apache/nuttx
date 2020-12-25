/****************************************************************************
 * boards/arm/cxd56xx/common/src/cxd56_crashdump.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: David Sidrane <david_s5@nscdg.com>
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <debug.h>
#include <time.h>

#include <nuttx/kmalloc.h>

#include <arch/chip/backuplog.h>
#include <arch/chip/crashdump.h>
#include "cxd56_wdt.h"

#include "arm_arch.h"
#include "arm_internal.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_reset_on_crash
 *
 * Description:
 *   System reboot. This API can be called from the interrupt handler.
 *
 ****************************************************************************/

#if defined(CONFIG_CXD56_RESET_ON_CRASH)
static int nmi_handler(int irq, FAR void *context, FAR void *arg)
{
  return 0;
}

static void board_reset_on_crash(void)
{
  /* Overwrite NMI handler to the empty dummy function,
   * because of preventing PANIC() from occurring again.
   */

  irq_attach(CXD56_IRQ_NMI, nmi_handler, NULL);

  /* Output the reset signal by self watchdog timer,
   * so then the system cpu resets the system.
   */

  putreg32(WDOGLOCK_UNLOCK_KEY, CXD56_WDT_WDOGLOCK);
  putreg32(WDOGITCR_ENABLE, CXD56_WDT_WDOGITCR);
  putreg32(WDOGITOP_WDOGRES, CXD56_WDT_WDOGITOP);

  __asm volatile ("dsb");
  for (; ; );
}
#endif /* CONFIG_CXD56_RESET_ON_CRASH */

/****************************************************************************
 * Name: copy_reverse
 ****************************************************************************/

static void copy_reverse(stack_word_t *dest, stack_word_t *src, int size)
{
  while (size--)
    {
      *dest++ = *src--;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_crashdump
 ****************************************************************************/

void board_crashdump(uintptr_t currentsp, FAR void *tcb,
                     FAR const char *filename, int lineno)
{
  FAR struct tcb_s *rtcb;
  fullcontext_t *pdump;

  enter_critical_section();

  rtcb = (FAR struct tcb_s *)tcb;
#ifdef CONFIG_CXD56_BACKUPLOG
  pdump = up_backuplog_alloc("crash", sizeof(fullcontext_t));
#else
  pdump = kmm_malloc(sizeof(fullcontext_t));
#endif
  if (!pdump)
    {
      goto exit;
    }

  /* Zero out everything */

  memset(pdump, 0, sizeof(fullcontext_t));

  /* Save Info */

  clock_gettime(CLOCK_REALTIME, &pdump->info.ts);
  pdump->info.lineno = lineno;

  if (filename)
    {
      int offset = 0;
      unsigned int len = strlen((char *)filename) + 1;

      if (len > sizeof(pdump->info.filename))
        {
          offset = len - sizeof(pdump->info.filename);
        }

      strncpy(pdump->info.filename, (char *)&filename[offset],
              sizeof(pdump->info.filename));
    }

  /* Save the value of the pointer for current_regs as debugging info.
   * It should be NULL in case of an ASSERT and will aid in cross
   * checking the validity of system memory at the time of the
   * fault.
   */

  pdump->info.current_regs = (uintptr_t)CURRENT_REGS;

  /* Save Context */

#if CONFIG_TASK_NAME_SIZE > 0
  strncpy(pdump->info.name, rtcb->name, CONFIG_TASK_NAME_SIZE);
#endif

  pdump->info.pid = rtcb->pid;

  /* If  current_regs is not NULL then we are in an interrupt context
   * and the user context is in current_regs else we are running in
   * the users context
   */

  if (CURRENT_REGS)
    {
#if CONFIG_ARCH_INTERRUPTSTACK > 3
      pdump->info.stacks.interrupt.sp = currentsp;
#endif
      pdump->info.flags |= (REGS_PRESENT | USERSTACK_PRESENT | \
                            INTSTACK_PRESENT);
      memcpy(pdump->info.regs, (void *)CURRENT_REGS,
             sizeof(pdump->info.regs));
      pdump->info.stacks.user.sp = pdump->info.regs[REG_R13];
    }
  else
    {
      /* users context */

      pdump->info.flags |= USERSTACK_PRESENT;
      pdump->info.stacks.user.sp = currentsp;
    }

  pdump->info.stacks.user.top = (uint32_t)rtcb->adj_stack_ptr;
  pdump->info.stacks.user.size = (uint32_t)rtcb->adj_stack_size;

#if CONFIG_ARCH_INTERRUPTSTACK > 3
  /* Get the limits on the interrupt stack memory */

  pdump->info.stacks.interrupt.top = (uint32_t)&g_intstackbase;
  pdump->info.stacks.interrupt.size  = (CONFIG_ARCH_INTERRUPTSTACK & ~3);

  /* If In interrupt Context save the interrupt stack data centered
   * about the interrupt stack pointer
   */

  if ((pdump->info.flags & INTSTACK_PRESENT) != 0)
    {
      stack_word_t *ps = (stack_word_t *) pdump->info.stacks.interrupt.sp;
      copy_reverse(pdump->istack, &ps[ARRAYSIZE(pdump->istack) / 2],
                   ARRAYSIZE(pdump->istack));
    }

  /* Is it Invalid? */

  if (!(pdump->info.stacks.interrupt.sp <= pdump->info.stacks.interrupt.top
      && pdump->info.stacks.interrupt.sp > pdump->info.stacks.interrupt.top
       - pdump->info.stacks.interrupt.size))
    {
      pdump->info.flags |= INVALID_INTSTACK_PTR;
    }

#endif
  /* If In interrupt context or User save the user stack data centered
   * about the user stack pointer
   */

  if ((pdump->info.flags & USERSTACK_PRESENT) != 0)
    {
      stack_word_t *ps = (stack_word_t *) pdump->info.stacks.user.sp;
      copy_reverse(pdump->ustack, &ps[ARRAYSIZE(pdump->ustack) / 2],
                   ARRAYSIZE(pdump->ustack));
    }

  /* Is it Invalid? */

  if (!(pdump->info.stacks.user.sp <= pdump->info.stacks.user.top &&
        pdump->info.stacks.user.sp > pdump->info.stacks.user.top -
        pdump->info.stacks.user.size))
    {
      pdump->info.flags |= INVALID_USERSTACK_PTR;
    }

exit:
#if defined(CONFIG_CXD56_RESET_ON_CRASH)
  board_reset_on_crash();
#endif
  return;
}
