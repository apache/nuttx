/****************************************************************************
 * mm/iob/iob_alloc.c
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

#include <assert.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/sched.h>
#include <nuttx/mm/iob.h>

#include "iob.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: iob_alloc_committed
 *
 * Description:
 *   Allocate an I/O buffer by taking the buffer at the head of the committed
 *   list.
 *
 ****************************************************************************/

static FAR struct iob_s *iob_alloc_committed(void)
{
  FAR struct iob_s *iob = NULL;
  irqstate_t flags;

  /* We don't know what context we are called from so we use extreme measures
   * to protect the committed list:  We disable interrupts very briefly.
   */

  flags = enter_critical_section();

  /* Take the I/O buffer from the head of the committed list */

  iob = g_iob_committed;
  if (iob != NULL)
    {
      /* Remove the I/O buffer from the committed list */

      g_iob_committed = iob->io_flink;

      /* Put the I/O buffer in a known state */

      iob->io_flink  = NULL; /* Not in a chain */
      iob->io_len    = 0;    /* Length of the data in the entry */
      iob->io_offset = 0;    /* Offset to the beginning of data */
      iob->io_pktlen = 0;    /* Total length of the packet */
    }

  leave_critical_section(flags);
  return iob;
}

/****************************************************************************
 * Name: iob_allocwait
 *
 * Description:
 *   Allocate an I/O buffer, waiting if necessary.  This function cannot be
 *   called from any interrupt level logic.
 *
 ****************************************************************************/

static FAR struct iob_s *iob_allocwait(bool throttled, unsigned int timeout)
{
  FAR struct iob_s *iob;
  irqstate_t flags;
  FAR sem_t *sem;
  int ret = OK;

#if CONFIG_IOB_THROTTLE > 0
  /* Select the semaphore count to check. */

  sem = (throttled ? &g_throttle_sem : &g_iob_sem);
#else
  sem = &g_iob_sem;
#endif

  /* The following must be atomic; interrupt must be disabled so that there
   * is no conflict with interrupt level I/O buffer allocations.  This is
   * not as bad as it sounds because interrupts will be re-enabled while
   * we are waiting for I/O buffers to become free.
   */

  flags = enter_critical_section();

  /* Try to get an I/O buffer.  If successful, the semaphore count will be
   * decremented atomically.
   */

  iob = iob_tryalloc(throttled);
  while (ret == OK && iob == NULL)
    {
      /* If not successful, then the semaphore count was less than or equal
       * to zero (meaning that there are no free buffers).  We need to wait
       * for an I/O buffer to be released and placed in the committed
       * list.
       */

      if (timeout == UINT_MAX)
        {
          ret = nxsem_wait_uninterruptible(sem);
        }
      else
        {
          ret = nxsem_tickwait_uninterruptible(sem, MSEC2TICK(timeout));
        }

      if (ret >= 0)
        {
          /* When we wake up from wait successfully, an I/O buffer was
           * freed and we hold a count for one IOB.
           */

          iob = iob_alloc_committed();
          if (iob == NULL)
            {
              /* We need release our count so that it is available to
               * iob_tryalloc(), perhaps allowing another thread to take our
               * count.  In that event, iob_tryalloc() will fail above and
               * we will have to wait again.
               */

              nxsem_post(sem);
              iob = iob_tryalloc(throttled);
            }

          /* REVISIT: I think this logic should be moved inside of
           * iob_alloc_committed, so that it can exist inside of the critical
           * section along with all other sem count changes.
           */

#if CONFIG_IOB_THROTTLE > 0
          else
            {
              if (throttled)
                {
                  g_iob_sem.semcount--;
                }
              else
                {
                  g_throttle_sem.semcount--;
                }
            }
#endif
        }
    }

  leave_critical_section(flags);
  return iob;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: iob_timedalloc
 *
 * Description:
 *  Allocate an I/O buffer by taking the buffer at the head of the free list.
 *  This wait will be terminated when the specified timeout expires.
 *
 * Input Parameters:
 *   throttled  - An indication of the IOB allocation is "throttled"
 *   timeout    - Timeout value in milliseconds.
 *
 ****************************************************************************/

FAR struct iob_s *iob_timedalloc(bool throttled, unsigned int timeout)
{
  /* Were we called from the interrupt level? */

  if (up_interrupt_context() || sched_idletask() || timeout == 0)
    {
      /* Yes, then try to allocate an I/O buffer without waiting */

      return iob_tryalloc(throttled);
    }
  else
    {
      /* Then allocate an I/O buffer, waiting as necessary */

      return iob_allocwait(throttled, timeout);
    }
}

/****************************************************************************
 * Name: iob_alloc
 *
 * Description:
 *  Allocate an I/O buffer by taking the buffer at the head of the free list.
 *
 ****************************************************************************/

FAR struct iob_s *iob_alloc(bool throttled)
{
  return iob_timedalloc(throttled, UINT_MAX);
}

/****************************************************************************
 * Name: iob_tryalloc
 *
 * Description:
 *   Try to allocate an I/O buffer by taking the buffer at the head of the
 *   free list without waiting for a buffer to become free.
 *
 ****************************************************************************/

FAR struct iob_s *iob_tryalloc(bool throttled)
{
  FAR struct iob_s *iob;
  irqstate_t flags;
#if CONFIG_IOB_THROTTLE > 0
  FAR sem_t *sem;
#endif

#if CONFIG_IOB_THROTTLE > 0
  /* Select the semaphore count to check. */

  sem = (throttled ? &g_throttle_sem : &g_iob_sem);
#endif

  /* We don't know what context we are called from so we use extreme measures
   * to protect the free list:  We disable interrupts very briefly.
   */

  flags = enter_critical_section();

#if CONFIG_IOB_THROTTLE > 0
  /* If there are free I/O buffers for this allocation */

  if (sem->semcount > 0 ||
      (throttled && g_iob_sem.semcount - CONFIG_IOB_THROTTLE > 0))
#endif
    {
      /* Take the I/O buffer from the head of the free list */

      iob = g_iob_freelist;
      if (iob != NULL)
        {
          /* Remove the I/O buffer from the free list and decrement the
           * counting semaphore(s) that tracks the number of available
           * IOBs.
           */

          g_iob_freelist = iob->io_flink;

          /* Take a semaphore count.  Note that we cannot do this in
           * in the orthodox way by calling nxsem_wait() or nxsem_trywait()
           * because this function may be called from an interrupt
           * handler. Fortunately we know at at least one free buffer
           * so a simple decrement is all that is needed.
           */

          g_iob_sem.semcount--;
          DEBUGASSERT(g_iob_sem.semcount >= 0);

#if CONFIG_IOB_THROTTLE > 0
          /* The throttle semaphore is a little more complicated because
           * it can be negative!  Decrementing is still safe, however.
           *
           * Note: usually g_throttle_sem.semcount >= -CONFIG_IOB_THROTTLE.
           * But it can be smaller than that if there are blocking threads.
           */

          g_throttle_sem.semcount--;
#endif

          leave_critical_section(flags);

          /* Put the I/O buffer in a known state */

          iob->io_flink  = NULL; /* Not in a chain */
          iob->io_len    = 0;    /* Length of the data in the entry */
          iob->io_offset = 0;    /* Offset to the beginning of data */
          iob->io_pktlen = 0;    /* Total length of the packet */
          return iob;
        }
    }

  leave_critical_section(flags);
  return NULL;
}
