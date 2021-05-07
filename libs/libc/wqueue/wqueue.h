/****************************************************************************
 * libs/libc/wqueue/wqueue.h
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

#ifndef __LIBC_WQUEUE_WQUEUE_H
#define __LIBC_WQUEUE_WQUEUE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <semaphore.h>
#include <pthread.h>

#include <nuttx/wqueue.h>

#if defined(CONFIG_LIB_USRWORK) && !defined(__KERNEL__)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* This structure defines the state of one user-modework queue. */

struct usr_wqueue_s
{
  struct dq_queue_s q;      /* The queue of pending work */
  pid_t             pid;    /* The task ID of the worker thread(s) */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* The state of the user mode work queue */

extern struct usr_wqueue_s g_usrwork;

/* This semaphore/mutex supports exclusive access to the user-mode work
 * queue
 */

#ifdef CONFIG_BUILD_PROTECTED
extern sem_t g_usrsem;
#else
extern pthread_mutex_t g_usrmutex;
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: work_lock
 *
 * Description:
 *   Lock the user-mode work queue.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success, a negated errno on failure.  This error may be
 *   reported:
 *
 *   -EINTR - Wait was interrupted by a signal
 *
 ****************************************************************************/

int work_lock(void);

/****************************************************************************
 * Name: work_unlock
 *
 * Description:
 *   Unlock the user-mode work queue.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void work_unlock(void);

#endif /* CONFIG_LIB_USRWORK && !__KERNEL__*/
#endif /* __LIBC_WQUEUE_WQUEUE_H */
