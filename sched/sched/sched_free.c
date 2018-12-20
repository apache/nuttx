/****************************************************************************
 * sched/sched/sched_free.c
 *
 *   Copyright (C) 2007, 2009, 2012-2013, 2015-2016, 2018 Gregory Nutt. All
 *     rights reserved.
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

#include <queue.h>
#include <assert.h>

#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/arch.h>
#include <nuttx/wqueue.h>

#include "sched/sched.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sched_ufree and sched_kfree
 *
 * Description:
 *   These function performs deallocations that the operating system may
 *   need to make.  This special interface to free is used in handling
 *   corner cases where the operating system may have to perform
 *   deallocations from within an interrupt handler.
 *
 ****************************************************************************/

void sched_ufree(FAR void *address)
{
#ifdef CONFIG_BUILD_KERNEL
  /* REVISIT:  It is not safe to defer user allocation in the kernel mode
   * build.  Why?  Because the correct user context is in place now but
   * will not be in place when the deferred de-allocation is performed.  In
   * order to make this work, we would need to do something like:  (1) move
   * g_delayed_kufree into the group structure, then traverse the groups to
   * collect garbage on a group-by-group basis.
   */

  DEBUGASSERT(!up_interrupt_context());
  kumm_free(address);

#else
  /* Check if this is an attempt to deallocate memory from an exception
   * handler.  If this function is called from the IDLE task, then we
   * must have exclusive access to the memory manager to do this.
   */

  if (up_interrupt_context() || kumm_trysemaphore() != 0)
    {
      irqstate_t flags;

      /* Yes.. Make sure that this is not a attempt to free kernel memory
       * using the user deallocator.
       */

      flags = enter_critical_section();
#if (defined(CONFIG_BUILD_PROTECTED) || defined(CONFIG_BUILD_KERNEL)) && \
     defined(CONFIG_MM_KERNEL_HEAP)
      DEBUGASSERT(!kmm_heapmember(address));
#endif

      /* Delay the deallocation until a more appropriate time. */

      sq_addlast((FAR sq_entry_t *)address,
                 (FAR sq_queue_t *)&g_delayed_kufree);

      /* Signal the worker thread that is has some clean up to do */

      sched_signal_free();
      leave_critical_section(flags);
    }
  else
    {
      /* No.. just deallocate the memory now. */

      kumm_free(address);
      kumm_givesemaphore();
    }
#endif
}

#ifdef CONFIG_MM_KERNEL_HEAP
void sched_kfree(FAR void *address)
{
  irqstate_t flags;

  /* Check if this is an attempt to deallocate memory from an exception
   * handler.  If this function is called from the IDLE task, then we
   * must have exclusive access to the memory manager to do this.
   */

  if (up_interrupt_context() || kmm_trysemaphore() != 0)
    {
      /* Yes.. Make sure that this is not a attempt to free user memory
       * using the kernel deallocator.
       */

      flags = enter_critical_section();
      DEBUGASSERT(kmm_heapmember(address));

      /* Delay the deallocation until a more appropriate time. */

      sq_addlast((FAR sq_entry_t *)address,
                 (FAR sq_queue_t *)&g_delayed_kfree);

      /* Signal the worker thread that is has some clean up to do */

      sched_signal_free();
      leave_critical_section(flags);
    }
  else
    {
      /* No.. just deallocate the memory now. */

      kmm_free(address);
      kmm_givesemaphore();
    }
}
#endif

/****************************************************************************
 * Name: sched_signal_free
 *
 * Description:
 *   Signal the worker thread that is has some clean up to do.
 *
 ****************************************************************************/

void sched_signal_free(void)
{
#ifdef CONFIG_SCHED_WORKQUEUE
  /* Signal the worker thread that is has some clean up to do */

  work_signal(LPWORK);
#endif
}
