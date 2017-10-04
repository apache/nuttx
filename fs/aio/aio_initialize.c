/****************************************************************************
 * fs/aio/aio_initialize.c
 *
 *   Copyright (C) 2014, 2017 Gregory Nutt. All rights reserved.
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

#include <semaphore.h>
#include <assert.h>
#include <errno.h>
#include <queue.h>

#include <nuttx/sched.h>

#include "aio/aio.h"

#ifdef CONFIG_FS_AIO

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This is an array of pre-allocated AIO containers */

static struct aio_container_s g_aioc_alloc[CONFIG_FS_NAIOC];

/* This is a list of free AIO containers */

static dq_queue_t g_aioc_free;

/* This counting semaphore tracks the number of free AIO containers */

static sem_t g_aioc_freesem;

/* This binary semaphore supports exclusive access to the list of pending
 * asynchronous I/O.  g_aio_holder and a_aio_count support the reentrant
 * lock.
 */

static sem_t g_aio_exclsem;
static pid_t g_aio_holder;
static uint16_t g_aio_count;

/****************************************************************************
 * Public Data
 ****************************************************************************/
/* This is a list of pending asynchronous I/O.  The user must hold the
 * lock on this list in order to access the list.
 */

dq_queue_t g_aio_pending;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: aio_initialize
 *
 * Description:
 *   Perform one-time initialization of the asynchronous I/O sub-system
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void aio_initialize(void)
{
  int i;

  /* Initialize counting semaphores */

  (void)nxsem_init(&g_aioc_freesem, 0, CONFIG_FS_NAIOC);
  (void)nxsem_init(&g_aio_exclsem, 0, 1);

  g_aio_holder = INVALID_PROCESS_ID;

  /* Initialize the container queues */

  dq_init(&g_aioc_free);
  dq_init(&g_aio_pending);

  /* Add all of the pre-allocated AIO containers to the free list */

  for (i = 0; i < CONFIG_FS_NAIOC; i++)
    {
      /* Add the container to the free list */

      dq_addlast(&g_aioc_alloc[i].aioc_link, &g_aioc_free);
    }
}

/****************************************************************************
 * Name: aio_lock/aio_unlock
 *
 * Description:
 *   Take/give the lock on the pending asynchronous I/O list
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void aio_lock(void)
{
  pid_t me = getpid();

  /* Does this thread already hold the semaphore? */

  if (g_aio_holder == me)
    {
      /* Yes, just increment the counts held */

      DEBUGASSERT(g_aio_count > 0 && g_aio_count < UINT16_MAX);
      g_aio_count++;
    }
  else
    {
      int ret;

      /* No.. take the semaphore */

      do
        {
          ret = nxsem_wait(&g_aio_exclsem);

          /* The only case that an error should occur here is if the wait
           * was awakened by a signal.
           */

          DEBUGASSERT(ret == OK || ret == -EINTR);
        }
      while (ret == -EINTR);

      /* And mark it as ours */

      g_aio_holder = me;
      g_aio_count  = 1;
    }
}

void aio_unlock(void)
{
  DEBUGASSERT(g_aio_holder == getpid() && g_aio_count > 0);

  /* Would decrementing the count release the lock? */

  if (g_aio_count <= 1)
    {
      /* Yes.. that we will no longer be the holder */

      g_aio_holder = INVALID_PROCESS_ID;
      g_aio_count  = 0;
      nxsem_post(&g_aio_exclsem);
    }
  else
    {
      /* Otherwise, just decrement the count.  We still hold the lock. */

      g_aio_count--;
    }
}

/****************************************************************************
 * Name: aioc_alloc
 *
 * Description:
 *   Allocate a new AIO container by taking the next, pre-allocated
 *   container from the free list.  This function will wait until
 *   aioc_free() is called in the event that there is no free container
 *   available in the free list.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   A reference to the allocated AIO container.  This allocation never
 *   fails because the logic will wait in the event that there is no free
 *   container.
 *
 ****************************************************************************/

FAR struct aio_container_s *aioc_alloc(void)
{
  FAR struct aio_container_s *aioc;
  int ret;

  /* Take a count from semaphore, thus guaranteeing that we have an AIO
   * container set aside for us.
   */

  do
    {
      /* Take the semaphore (perhaps waiting) */

      ret = nxsem_wait(&g_aioc_freesem);

      /* The only case that an error should occur here is if the wait was
       * awakened by a signal.
       */

      DEBUGASSERT(ret == OK || ret == -EINTR);
    }
  while (ret == -EINTR);

  /* Get our AIO container */

  aio_lock();
  aioc = (FAR struct aio_container_s *)dq_remfirst(&g_aioc_free);
  aio_unlock();

  DEBUGASSERT(aioc);
  return aioc;
}

/****************************************************************************
 * Name: aioc_free
 *
 * Description:
 *   Free an AIO container by returning it to the free list and, perhaps,
 *   awakening any threads waiting for that resource
 *
 * Input Parameters:
 *   aioc - The AIO container to be free
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void aioc_free(FAR struct aio_container_s *aioc)
{
  DEBUGASSERT(aioc);

  /* Return the container to the free list */

  aio_lock();
  dq_addlast(&aioc->aioc_link, &g_aioc_free);
  aio_unlock();

  /* The post the counting semaphore, announcing the availability of the
   * free AIO container.
   */

  nxsem_post(&g_aioc_freesem);
}

#endif /* CONFIG_FS_AIO */
