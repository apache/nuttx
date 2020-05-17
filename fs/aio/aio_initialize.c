/****************************************************************************
 * fs/aio/aio_initialize.c
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
#include <queue.h>

#include <nuttx/sched.h>
#include <nuttx/semaphore.h>

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

  nxsem_init(&g_aioc_freesem, 0, CONFIG_FS_NAIOC);
  nxsem_set_protocol(&g_aioc_freesem, SEM_PRIO_NONE);
  nxsem_init(&g_aio_exclsem, 0, 1);

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
 *   aio_lock() return -ECANCELED if the calling thread is canceled.
 *
 ****************************************************************************/

int aio_lock(void)
{
  pid_t me = getpid();
  int ret = OK;

  /* Does this thread already hold the semaphore? */

  if (g_aio_holder == me)
    {
      /* Yes, just increment the counts held */

      DEBUGASSERT(g_aio_count > 0 && g_aio_count < UINT16_MAX);
      g_aio_count++;
    }
  else
    {
      ret = nxsem_wait_uninterruptible(&g_aio_exclsem);
      if (ret >= 0)
        {
          /* And mark it as ours */

          g_aio_holder = me;
          g_aio_count  = 1;
        }
    }

  return ret;
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
  FAR struct aio_container_s *aioc = NULL;
  int ret;

  /* Take a count from semaphore, thus guaranteeing that we have an AIO
   * container set aside for us.
   */

  ret = nxsem_wait_uninterruptible(&g_aioc_freesem);
  if (ret < 0)
    {
      return NULL;
    }

  /* Get our AIO container */

  ret = aio_lock();
  if (ret >= 0)
    {
      aioc = (FAR struct aio_container_s *)dq_remfirst(&g_aioc_free);
      aio_unlock();

      DEBUGASSERT(aioc);
    }

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
  int ret;

  DEBUGASSERT(aioc);

  /* Return the container to the free list */

  do
    {
      ret = aio_lock();

      /* The only possible error should be if we were awakened only by
       * thread cancellation.
       */

      DEBUGASSERT(ret == OK || ret == -ECANCELED);
    }
  while (ret < 0);

  dq_addlast(&aioc->aioc_link, &g_aioc_free);
  aio_unlock();

  /* The post the counting semaphore, announcing the availability of the
   * free AIO container.
   */

  nxsem_post(&g_aioc_freesem);
}

#endif /* CONFIG_FS_AIO */
