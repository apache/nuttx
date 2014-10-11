/****************************************************************************
 * sched/libc/wqueue.h
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
  uint32_t          delay;  /* Delay between polling cycles (ticks) */
  struct dq_queue_s q;      /* The queue of pending work */
  pid_t             pid;    /* The task ID of the worker thread(s) */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* The state of the user mode work queue */

extern struct usr_wqueue_s g_usrwork;

/* This semaphore/mutex supports exclusive access to the user-mode work queue */

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
 * Input parameters:
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
 * Input parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void work_unlock(void);

#endif /* CONFIG_LIB_USRWORK && !__KERNEL__*/
#endif /* __LIBC_WQUEUE_WQUEUE_H */
