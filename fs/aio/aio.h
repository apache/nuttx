/****************************************************************************
 * fs/aio/aio.h
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

#ifndef __FS_AIO_AIO_H
#define __FS_AIO_AIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <string.h>
#include <aio.h>
#include <queue.h>

#include <nuttx/wqueue.h>
#include <nuttx/net/net.h>

#ifdef CONFIG_FS_AIO

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Number of pre-allocated AIO Control block containers */

#ifndef CONFIG_FS_NAIOC
#  define CONFIG_FS_NAIOC 8
#endif

#undef AIO_HAVE_PSOCK

#ifdef CONFIG_NET_TCP
#  define AIO_HAVE_PSOCK
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure contains one AIO control block and appends information
 * needed by the logic running on the worker thread.  These structures are
 * pre-allocated, the number pre-allocated controlled by CONFIG_FS_NAIOC.
 */

struct file;
struct aio_container_s
{
  dq_entry_t aioc_link;            /* Supports a doubly linked list */
  FAR struct aiocb *aioc_aiocbp;   /* The contained AIO control block */
  union
  {
    FAR struct file *aioc_filep;   /* File structure to use with the I/O */
#ifdef AIO_HAVE_PSOCK
    FAR struct socket *aioc_psock; /* Socket structure to use with the I/O */
#endif
    FAR void *ptr;                 /* Generic pointer to FAR data */
  } u;
  struct work_s aioc_work;         /* Used to defer I/O to the work thread */
  pid_t aioc_pid;                  /* ID of the waiting task */
#ifdef CONFIG_PRIORITY_INHERITANCE
  uint8_t aioc_prio;               /* Priority of the waiting task */
#endif
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/* This is a list of pending asynchronous I/O.  The user must hold the
 * lock on this list in order to access the list.
 */

EXTERN dq_queue_t g_aio_pending;

/****************************************************************************
 * Public Function Prototypes
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

void aio_initialize(void);

/****************************************************************************
 * Name: aio_lock/aio_unlock
 *
 * Description:
 *   Take/give the lock on the pending asynchronous I/O list.  These locks
 *   are implemented with re-entrant semaphores -- deadlocks will not occur
 *   if the lock is taken multiple times on the same thread.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   aio_lock() return -ECANCELED if the calling thread is canceled.
 *
 ****************************************************************************/

int aio_lock(void);
void aio_unlock(void);

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

FAR struct aio_container_s *aioc_alloc(void);

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

void aioc_free(FAR struct aio_container_s *aioc);

/****************************************************************************
 * Name: aio_contain
 *
 * Description:
 *   Create and initialize a container for the provided AIO control block
 *
 * Input Parameters:
 *   aiocbp - The AIO control block pointer
 *
 * Returned Value:
 *   A reference to the new AIO control block container.   This function
 *   will not fail but will wait if necessary for the resources to perform
 *   this operation.  NULL will be returned on certain errors with the
 *   errno value already set appropriately.
 *
 ****************************************************************************/

FAR struct aio_container_s *aio_contain(FAR struct aiocb *aiocbp);

/****************************************************************************
 * Name: aioc_decant
 *
 * Description:
 *   Remove the AIO control block from the container and free all resources
 *   used by the container.
 *
 * Input Parameters:
 *   aioc - Pointer to the AIO control block container
 *
 * Returned Value:
 *   A pointer to the no-longer contained AIO control block.
 *
 ****************************************************************************/

FAR struct aiocb *aioc_decant(FAR struct aio_container_s *aioc);

/****************************************************************************
 * Name: aio_queue
 *
 * Description:
 *   Schedule the asynchronous I/O on the low priority work queue
 *
 * Input Parameters:
 *   arg - Worker argument.  In this case, a pointer to an instance of
 *     struct aiocb cast to void *.
 *
 * Returned Value:
 *   Zero (OK) on success.  Otherwise, -1 is returned and the errno is set
 *   appropriately.
 *
 ****************************************************************************/

int aio_queue(FAR struct aio_container_s *aioc, worker_t worker);

/****************************************************************************
 * Name: aio_signal
 *
 * Description:
 *   Signal the client that an I/O has completed.
 *
 * Input Parameters:
 *   pid    - ID of the task to signal
 *   aiocbp - Pointer to the asynchronous I/O state structure that includes
 *            information about how to signal the client
 *
 * Returned Value:
 *   Zero (OK) if the client was successfully signalled.  Otherwise, a
 *   negated errno value is returned.
 *
 ****************************************************************************/

int aio_signal(pid_t pid, FAR struct aiocb *aiocbp);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* CONFIG_FS_AIO */
#endif /* __FS_AIO_AIO_H */
