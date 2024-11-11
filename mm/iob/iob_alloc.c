/****************************************************************************
 * mm/iob/iob_alloc.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
#ifdef CONFIG_IOB_ALLOC
#  include <nuttx/kmalloc.h>
#endif
#include <nuttx/mm/iob.h>

#include "iob.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static clock_t iob_allocwait_gettimeout(clock_t start, unsigned int timeout)
{
  sclock_t tick;

  tick = clock_systime_ticks() - start;
  if (tick >= MSEC2TICK(timeout))
    {
      tick = 0;
    }
  else
    {
      tick = MSEC2TICK(timeout) - tick;
    }

  return tick;
}

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

  flags = spin_lock_irqsave(&g_iob_lock);

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

  spin_unlock_irqrestore(&g_iob_lock, flags);
  return iob;
}

static FAR struct iob_s *iob_tryalloc_internal(bool throttled)
{
  FAR struct iob_s *iob;
#if CONFIG_IOB_THROTTLE > 0
  int16_t count;
#endif

#if CONFIG_IOB_THROTTLE > 0
  /* Select the count to check. */

  count = (throttled ? g_throttle_count : g_iob_count);
#endif

  /* We don't know what context we are called from so we use extreme measures
   * to protect the free list:  We disable interrupts very briefly.
   */

#if CONFIG_IOB_THROTTLE > 0
  /* If there are free I/O buffers for this allocation */

  if (count > 0)
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

          g_iob_count--;
          DEBUGASSERT(g_iob_count >= 0);

#if CONFIG_IOB_THROTTLE > 0
          /* The throttle semaphore is used to throttle the number of
           * free buffers that are available.  It is used to prevent
           * the overrunning of the free buffer list. Please note that
           * it can only be decremented to zero, which indicates no
           * throttled buffers are available.
           */

          if (g_throttle_count > 0)
            {
              g_throttle_count--;
            }
#endif

          /* Put the I/O buffer in a known state */

          iob->io_flink  = NULL; /* Not in a chain */
          iob->io_len    = 0;    /* Length of the data in the entry */
          iob->io_offset = 0;    /* Offset to the beginning of data */
          iob->io_pktlen = 0;    /* Total length of the packet */
          return iob;
        }
    }

  return NULL;
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
  FAR volatile int16_t *count;
  irqstate_t flags;
  FAR sem_t *sem;
  clock_t start;
  int ret = OK;

#if CONFIG_IOB_THROTTLE > 0
  /* Select the semaphore count to check. */

  count = (throttled ? &g_throttle_count : &g_iob_count);
  sem = (throttled ? &g_throttle_sem : &g_iob_sem);
#else
  count = &g_iob_count;
  sem = &g_iob_sem;
#endif

  /* The following must be atomic; interrupt must be disabled so that there
   * is no conflict with interrupt level I/O buffer allocations.  This is
   * not as bad as it sounds because interrupts will be re-enabled while
   * we are waiting for I/O buffers to become free.
   */

  flags = spin_lock_irqsave(&g_iob_lock);

  /* Try to get an I/O buffer.  If successful, the semaphore count will be
   * decremented atomically.
   */

  iob   = iob_tryalloc_internal(throttled);
  if (iob == NULL)
    {
      /* If not successful, then the semaphore count was less than or equal
       * to zero (meaning that there are no free buffers).  We need to wait
       * for an I/O buffer to be released and placed in the committed
       * list.
       */

      (*count)--;

      spin_unlock_irqrestore(&g_iob_lock, flags);

      if (timeout == UINT_MAX)
        {
          ret = nxsem_wait_uninterruptible(sem);
        }
      else
        {
          start = clock_systime_ticks();
          ret = nxsem_tickwait_uninterruptible(sem,
                                   iob_allocwait_gettimeout(start, timeout));
        }

      if (ret >= 0)
        {
          /* When we wake up from wait successfully, an I/O buffer was
           * freed and we hold a count for one IOB.
           */

          iob = iob_alloc_committed();
          DEBUGASSERT(iob != NULL);
        }

      return iob;
    }

  spin_unlock_irqrestore(&g_iob_lock, flags);
  return iob;
}

#ifdef CONFIG_IOB_ALLOC
/****************************************************************************
 * Name: iob_free_dynamic
 *
 * Description:
 *   Dummy free callback function, do nothing.
 *
 * Input Parameters:
 *   data -
 *
 ****************************************************************************/

static void iob_free_dynamic(FAR void *data)
{
}
#endif

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

  /* We don't know what context we are called from so we use extreme measures
   * to protect the free list:  We disable interrupts very briefly.
   */

  flags = spin_lock_irqsave(&g_iob_lock);
  iob = iob_tryalloc_internal(throttled);
  spin_unlock_irqrestore(&g_iob_lock, flags);
  return iob;
}

#ifdef CONFIG_IOB_ALLOC

/****************************************************************************
 * Name: iob_alloc_dynamic
 *
 * Description:
 *   Allocate an I/O buffer and playload from heap
 *
 * Input Parameters:
 *   size    - The size of the io_data that is allocated.
 *
 *             +---------+
 *             |   IOB   |
 *             | io_data |--+
 *             | buffer  |<-+
 *             +---------+
 *
 ****************************************************************************/

FAR struct iob_s *iob_alloc_dynamic(uint16_t size)
{
  FAR struct iob_s *iob;
  size_t alignsize;

  alignsize = ROUNDUP(sizeof(struct iob_s), CONFIG_IOB_ALIGNMENT) + size;

  iob = kmm_memalign(CONFIG_IOB_ALIGNMENT, alignsize);
  if (iob)
    {
      iob->io_flink   = NULL;             /* Not in a chain */
      iob->io_len     = 0;                /* Length of the data in the entry */
      iob->io_offset  = 0;                /* Offset to the beginning of data */
      iob->io_bufsize = size;             /* Total length of the iob buffer */
      iob->io_pktlen  = 0;                /* Total length of the packet */
      iob->io_free    = iob_free_dynamic; /* Customer free callback */
      iob->io_data    = (FAR uint8_t *)ROUNDUP((uintptr_t)(iob + 1),
                                               CONFIG_IOB_ALIGNMENT);
    }

  return iob;
}

/****************************************************************************
 * Name: iob_alloc_with_data
 *
 * Description:
 *   Allocate an I/O buffer from heap and attach the external payload
 *
 * Input Parameters:
 *   data    - Make io_data point to a specific address, the caller is
 *             responsible for the memory management. The caller should
 *             ensure that the memory is not freed before the iob is freed.
 *
 *             +---------+  +-->+--------+
 *             |   IOB   |  |   |  data  |
 *             | io_data |--+   +--------+
 *             +---------+
 *
 *   size    - The size of the data parameter
 *   free_cb - Notify the caller when the iob is freed. The caller can
 *             perform additional operations on the data before it is freed.
 *             The free_cb is called when the iob is freed.
 *
 ****************************************************************************/

FAR struct iob_s *iob_alloc_with_data(FAR void *data, uint16_t size,
                                      iob_free_cb_t free_cb)
{
  FAR struct iob_s *iob;

  DEBUGASSERT(free_cb != NULL);

  iob = kmm_malloc(sizeof(struct iob_s));
  if (iob)
    {
      iob->io_flink   = NULL;    /* Not in a chain */
      iob->io_len     = 0;       /* Length of the data in the entry */
      iob->io_offset  = 0;       /* Offset to the beginning of data */
      iob->io_bufsize = size;    /* Total length of the iob buffer */
      iob->io_pktlen  = 0;       /* Total length of the packet */
      iob->io_free    = free_cb; /* Customer free callback */
      iob->io_data    = data;
    }

  return iob;
}
#endif
