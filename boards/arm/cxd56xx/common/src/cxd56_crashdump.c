/****************************************************************************
 * boards/arm/cxd56xx/common/src/cxd56_crashdump.c
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <time.h>

#include <nuttx/kmalloc.h>

#include <arch/chip/backuplog.h>
#include <arch/chip/crashdump.h>
#include "cxd56_wdt.h"
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
static int nmi_handler(int irq, void *context, void *arg)
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

void board_crashdump(uintptr_t currentsp, void *tcb,
                     const char *filename, int lineno)
{
  struct tcb_s *rtcb;
  fullcontext_t *pdump;

  enter_critical_section();

  rtcb = (struct tcb_s *)tcb;
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

      strlcpy(pdump->info.filename, (char *)&filename[offset],
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
  strlcpy(pdump->info.name, rtcb->name, sizeof(pdump->info.name));
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

  pdump->info.stacks.user.top = (uint32_t)rtcb->stack_base_ptr +
                                          rtcb->adj_stack_size;
  pdump->info.stacks.user.size = (uint32_t)rtcb->adj_stack_size;

#if CONFIG_ARCH_INTERRUPTSTACK > 3
  /* Get the limits on the interrupt stack memory */

#ifdef CONFIG_SMP
  pdump->info.stacks.interrupt.top = (uint32_t)arm_intstack_top();
#else
  pdump->info.stacks.interrupt.top = (uint32_t)g_intstacktop;
#endif
  pdump->info.stacks.interrupt.size = (CONFIG_ARCH_INTERRUPTSTACK & ~3);

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
