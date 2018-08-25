/****************************************************************************
 * sched/wqueue/wqueue.h
 *
 *   Copyright (C) 2014, 2016, 2018 Gregory Nutt. All rights reserved.
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

#ifndef __SCHED_WQUEUE_WQUEUE_H
#define __SCHED_WQUEUE_WQUEUE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdbool.h>
#include <queue.h>

#include <nuttx/clock.h>

#ifdef CONFIG_SCHED_WORKQUEUE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Kkernel thread names */

#define HPWORKNAME "hpwork"
#define LPWORKNAME "lpwork"

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/
/* This represents one worker */

struct kworker_s
{
  pid_t             pid;    /* The task ID of the worker thread */
  volatile bool     busy;   /* True: Worker is not available */
};

/* This structure defines the state of one kernel-mode work queue */

struct kwork_wqueue_s
{
  struct dq_queue_s q;         /* The queue of pending work */
  struct kworker_s  worker[1]; /* Describes a worker thread */
};

/* This structure defines the state of one high-priority work queue.  This
 * structure must be cast-compatible with kwork_wqueue_s.
 */

#ifdef CONFIG_SCHED_HPWORK
struct hp_wqueue_s
{
  struct dq_queue_s q;         /* The queue of pending work */

  /* Describes each thread in the high priority queue's thread pool */

  struct kworker_s  worker[CONFIG_SCHED_HPNTHREADS];
};
#endif

/* This structure defines the state of one low-priority work queue.  This
 * structure must be cast compatible with kwork_wqueue_s
 */

#ifdef CONFIG_SCHED_LPWORK
struct lp_wqueue_s
{
  struct dq_queue_s q;      /* The queue of pending work */

  /* Describes each thread in the low priority queue's thread pool */

  struct kworker_s  worker[CONFIG_SCHED_LPNTHREADS];
};
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef CONFIG_SCHED_HPWORK
/* The state of the kernel mode, high priority work queue. */

extern struct hp_wqueue_s g_hpwork;
#endif

#ifdef CONFIG_SCHED_LPWORK
/* The state of the kernel mode, low priority work queue(s). */

extern struct lp_wqueue_s g_lpwork;
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: work_hpstart
 *
 * Description:
 *   Start the high-priority, kernel-mode work queue.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The task ID of the worker thread is returned on success.  A negated
 *   errno value is returned on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SCHED_HPWORK
int work_hpstart(void);
#endif

/****************************************************************************
 * Name: work_lpstart
 *
 * Description:
 *   Start the low-priority, kernel-mode worker thread(s)
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The task ID of the worker thread is returned on success.  A negated
 *   errno value is returned on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SCHED_LPWORK
int work_lpstart(void);
#endif

/****************************************************************************
 * Name: work_process
 *
 * Description:
 *   This is the logic that performs actions placed on any work list.  This
 *   logic is the common underlying logic to all work queues.  This logic is
 *   part of the internal implementation of each work queue; it should not
 *   be called from application level logic.
 *
 * Input Parameters:
 *   wqueue - Describes the work queue to be processed
 *   wndx   - The worker thread index
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void work_process(FAR struct kwork_wqueue_s *wqueue, int wndx);

#endif /* CONFIG_SCHED_WORKQUEUE */
#endif /* __SCHED_WQUEUE_WQUEUE_H */
