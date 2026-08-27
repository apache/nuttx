/****************************************************************************
 * libs/libc/wqueue/wqueue.h
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

#ifndef __LIBS_LIBC_WQUEUE_WQUEUE_H
#define __LIBS_LIBC_WQUEUE_WQUEUE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>
#include <sys/types.h>

#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/list.h>
#include <nuttx/wqueue.h>

#if defined(CONFIG_LIBC_USRWORK) && !defined(__KERNEL__)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* Forward reference */

struct usr_wqueue_s;

/* This structure describes one user-mode worker thread. */

struct usr_worker_s
{
  pid_t                    tid;        /* Worker thread ID */
  FAR struct work_s       *work;       /* Work currently being processed */
  FAR struct usr_wqueue_s *wqueue;     /* Parent work queue */
  sem_t                    wait;       /* Wait for the current work */
  uint16_t                 wait_count; /* Number of synchronous waiters */
};

/* This structure defines the state of one user-mode work queue. */

struct usr_wqueue_s
{
  struct list_node        q;        /* The queue of pending work */
  mutex_t                 lock;     /* Exclusive access to the queue */
  sem_t                   wake;     /* Wake-up semaphore */
  FAR struct usr_worker_s *worker;  /* Worker thread state array */
  int                     nthreads; /* Number of worker threads */
  bool                    exit;     /* Request worker thread exit */
  bool                    dynamic;  /* Dynamically allocated queue */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* The state of the user mode work queue */

extern struct usr_wqueue_s g_usrwork;

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: work_remove
 *
 * Description:
 *   Remove work from a user-mode work queue.  The caller must hold
 *   wqueue->lock, and work must be queued on wqueue.
 *
 * Returned Value:
 *   true if removing work changed the head of the queue; otherwise false.
 *
 ****************************************************************************/

static inline_function bool
work_remove(FAR struct usr_wqueue_s *wqueue,
            FAR struct work_s *work)
{
  FAR struct work_s *head;

  head = list_first_entry(&wqueue->q, struct work_s, node);

  work->worker = NULL;
  list_delete(&work->node);

  return head == work;
}

static inline_function void work_wake(FAR struct usr_wqueue_s *wqueue)
{
  int semcount;

  /* Keep enough wake tokens for the worker pool while bounding stale
   * tokens when workers are already running.
   */

  nxsem_get_value(&wqueue->wake, &semcount);
  if (semcount < wqueue->nthreads)
    {
      nxsem_post(&wqueue->wake);
    }
}

#endif /* CONFIG_LIBC_USRWORK && !__KERNEL__ */
#endif /* __LIBS_LIBC_WQUEUE_WQUEUE_H */
