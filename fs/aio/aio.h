/****************************************************************************
 * fs/aio/aio.h
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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

#undef AIO_HAVE_FILEP
#undef AIO_HAVE_PSOCK

#if CONFIG_NFILE_DESCRIPTORS > 0
#  define AIO_HAVE_FILEP
#endif

#if defined(CONFIG_NET_TCP) && CONFIG_NSOCKET_DESCRIPTORS > 0
#  define AIO_HAVE_PSOCK
#endif

#if !defined(AIO_HAVE_FILEP) && !defined(AIO_HAVE_PSOCK)
#  error AIO needs file and/or socket descriptors
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
#ifdef AIO_HAVE_FILEP
    FAR struct file *aioc_filep;   /* File structure to use with the I/O */
#endif
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
 *   None
 *
 ****************************************************************************/

void aio_lock(void);
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

#endif /* CONFIG_FS_AIO */
#endif /* __FS_AIO_AIO_H */
