/****************************************************************************
 * sched/sched/sched_garbage.c
 *
 *   Copyright (C) 2009, 2011, 2013, 2016 Gregory Nutt. All rights reserved.
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
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>

#include "sched/sched.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sched_kucleanup
 *
 * Description:
 *   Clean-up deferred de-allocations of user memory
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void sched_kucleanup(void)
{
#ifdef CONFIG_BUILD_KERNEL
  /* REVISIT:  It is not safe to defer user allocation in the kernel mode
   * build.  Why?  Because the correct user context will not be in place
   * when these deferred de-allocations are performed.  In order to make
   * this work, we would need to do something like:  (1) move
   * g_delayed_kufree into the group structure, then traverse the groups to
   * collect garbage on a group-by-group basis.
   */

#else
  irqstate_t flags;
  FAR void *address;

  /* Test if the delayed deallocation queue is empty.  No special protection
   * is needed because this is an atomic test.
   */

  while (g_delayed_kufree.head)
    {
      /* Remove the first delayed deallocation.  This is not atomic and so
       * we must disable interrupts around the queue operation.
       */

      flags = enter_critical_section();
      address = (FAR void *)sq_remfirst((FAR sq_queue_t *)&g_delayed_kufree);
      leave_critical_section(flags);

      /* The address should always be non-NULL since that was checked in the
       * 'while' condition above.
       */

      if (address)
        {
          /* Return the memory to the user heap */

          kumm_free(address);
        }
    }
#endif
}

/****************************************************************************
 * Name: sched_have_kugarbage
 *
 * Description:
 *   Return TRUE if there is user heap garbage to be collected.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   TRUE if there is kernel heap garbage to be collected.
 *
 ****************************************************************************/

#ifndef CONFIG_BUILD_KERNEL
static inline bool sched_have_kugarbage(void)
{
  return (g_delayed_kufree.head != NULL);
}
#else
#  define sched_have_kugarbage() false
#endif

/****************************************************************************
 * Name: sched_kcleanup
 *
 * Description:
 *   Clean-up deferred de-allocations of kernel memory
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if (defined(CONFIG_BUILD_PROTECTED) || defined(CONFIG_BUILD_KERNEL)) && \
     defined(CONFIG_MM_KERNEL_HEAP)
static inline void sched_kcleanup(void)
{
  irqstate_t flags;
  FAR void *address;

  /* Test if the delayed deallocation queue is empty.  No special protection
   * is needed because this is an atomic test.
   */

  while (g_delayed_kfree.head)
    {
      /* Remove the first delayed deallocation.  This is not atomic and so
       * we must disable interrupts around the queue operation.
       */

      flags = enter_critical_section();
      address = (FAR void *)sq_remfirst((FAR sq_queue_t *)&g_delayed_kfree);
      leave_critical_section(flags);

      /* The address should always be non-NULL since that was checked in the
       * 'while' condition above.
       */

      if (address)
        {
          /* Return the memory to the kernel heap */

          kmm_free(address);
        }
    }
}
#else
#  define sched_kcleanup()
#endif

/****************************************************************************
 * Name: sched_have_kgarbage
 *
 * Description:
 *   Return TRUE if there is kernal heap garbage to be collected.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   TRUE if there is kernel heap garbage to be collected.
 *
 ****************************************************************************/

#if (defined(CONFIG_BUILD_PROTECTED) || defined(CONFIG_BUILD_KERNEL)) && \
     defined(CONFIG_MM_KERNEL_HEAP)
static inline bool sched_have_kgarbage(void)
{
  return (g_delayed_kfree.head != NULL);
}
#else
#  define sched_have_kgarbage() false
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sched_garbage_collection
 *
 * Description:
 *   Clean-up memory de-allocations that we queued because they could not
 *   be freed in that execution context (for example, if the memory was freed
 *   from an interrupt handler).
 *
 *   This logic may be called from the worker thread (see work_thread.c).
 *   If, however, CONFIG_SCHED_WORKQUEUE is not defined, then this logic will
 *   be called from the IDLE thread.  It is less optimal for the garbage
 *   collection to be called from the IDLE thread because it runs at a very
 *   low priority and could cause false memory out conditions.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sched_garbage_collection(void)
{
  /* Handle deferred deallocations for the kernel heap */

  sched_kcleanup();

  /* Handle deferred deallocations for the user heap */

  sched_kucleanup();

  /* Handle the architecure-specific garbage collection */

  up_sched_garbage_collection();
}

/****************************************************************************
 * Name: sched_have_garbage
 *
 * Description:
 *   Return TRUE if there is garbage to be collected.
 *
 *   Is is not a good idea for the IDLE threads to take the KMM semaphore.
 *   That can cause the IDLE thread to take processing time from higher
 *   priority tasks.  The IDLE threads will only take the KMM semaphore if
 *   there is garbage to be collected.
 *
 *   Certainly there is a race condition involved in sampling the garbage
 *   state.  The looping nature of the IDLE loops should catch any missed
 *   garbage from the test on the next time arround.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   TRUE if there is garbage to be collected.
 *
 ****************************************************************************/

bool sched_have_garbage(void)
{
  return (sched_have_kgarbage() || sched_have_kugarbage() ||
          up_sched_have_garbage());
}
