/****************************************************************************
 * sched/task/task.h
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

#ifndef __SCHED_TASK_TASK_H
#define __SCHED_TASK_TASK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <sys/types.h>
#include <stdbool.h>

#include <nuttx/sched.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

struct tcb_s; /* Forward reference */

/* Task start-up */

void nxtask_start(void);
int nxtask_setup_stackargs(FAR struct tcb_s *tcb,
                           FAR const char *name,
                           FAR char * const argv[]);
int nxtask_setup_scheduler(FAR struct tcb_s *tcb, int priority,
                           start_t start, main_t main, uint8_t ttype);
#if CONFIG_TASK_NAME_SIZE > 0
void nxtask_setup_name(FAR struct tcb_s *tcb, FAR const char *name);
#else
#  define nxtask_setup_name(tcb, name)
#endif

/* Task exit */

int  nxtask_exit(void);
int  nxtask_terminate(pid_t pid);
void nxtask_exithook(FAR struct tcb_s *tcb, int status);
void nxtask_recover(FAR struct tcb_s *tcb);

/* Cancellation points */

bool nxnotify_cancellation(FAR struct tcb_s *tcb);

/* The end of a process cancels a thread that is in a system call only when
 * the call is over (nxtask_kcancel_notify()).  The architecture must end
 * the call in the context of the thread for that.
 */

#if defined(CONFIG_LIB_SYSCALL) && \
    defined(CONFIG_ARCH_HAVE_SYSCALL_THREAD_RETURN)
#  define HAVE_TASK_KCANCEL 1

/* Values of tcb->kcancel */

#  define TASK_KCANCEL       (1 << 0) /* Ends when the system call is over */
#  define TASK_KCANCEL_UWAIT (1 << 1) /* Waits for a mutex of user code */
#  define TASK_KCANCEL_EXIT  (1 << 2) /* In its own end, as _exit() */

bool nxtask_kcancel_notify(FAR struct tcb_s *tcb);
void nxtask_kcancel(void);
bool nxtask_kcancel_wait(FAR struct tcb_s *tcb, FAR sem_t *sem);

/* A thread that was canceled in a system call does not start a wait that
 * the cancellation ends.  Not once it ends: its end must wait for locks.
 */

#  define nxtask_kcanceled(tcb) \
     (((tcb)->kcancel & TASK_KCANCEL) != 0 && \
      (atomic_read(&(tcb)->shared_flags) & TCB_FLAG_EXIT_PROCESSING) == 0)

/* Such a thread waits for a mutex of user code: the wait ends */

#  define nxtask_kcancel_uwait(tcb) \
     (((tcb)->kcancel & (TASK_KCANCEL | TASK_KCANCEL_UWAIT)) == \
      (TASK_KCANCEL | TASK_KCANCEL_UWAIT))

/* The calling thread starts its own end.  The end of its process leaves it
 * to that end (nxtask_kcancel_notify()).
 */

#  define nxtask_kcancel_exit(tcb) ((tcb)->kcancel |= TASK_KCANCEL_EXIT)
#  define nxtask_kcancel_exiting(tcb) \
     (((tcb)->kcancel & TASK_KCANCEL_EXIT) != 0)
#else
#  define nxtask_kcancel_notify(tcb) false
#  define nxtask_kcancel()
#  define nxtask_kcancel_wait(tcb, sem) false
#  define nxtask_kcanceled(tcb) false
#  define nxtask_kcancel_uwait(tcb) false
#  define nxtask_kcancel_exit(tcb)
#  define nxtask_kcancel_exiting(tcb) false
#endif

/* Task Join */

#ifndef CONFIG_DISABLE_PTHREAD
void nxtask_joininit(FAR struct tcb_s *tcb);
void nxtask_joindestroy(FAR struct tcb_s *tcb);
#else
#  define nxtask_joininit(tcb)
#  define nxtask_joindestroy(tcb)
#endif

#endif /* __SCHED_TASK_TASK_H */
