/****************************************************************************
 * sched/pthread/pthread.h
 *
 *   Copyright (C) 2007-2009, 2011, 2013-2014, 2019 Gregory Nutt. All rights
 *     reserved.
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

#ifndef __SCHED_PTHREAD_PTHREAD_H
#define __SCHED_PTHREAD_PTHREAD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>
#include <sched.h>

#include <nuttx/compiler.h>

/****************************************************************************
 * Public Type Declarations
 ****************************************************************************/

/* The following defines an entry in the pthread logic's local data set.
 * Note that this structure is used to implemented a singly linked list.
 * This structure is used (instead of, say, a binary search tree) because
 * the data set will be searched using the pid as a key -- a process IDs will
 * always be created in a montonically increasing fashion.
 */

struct join_s
{
  FAR struct join_s *next;       /* Implements link list */
  uint8_t        crefs;          /* Reference count */
  bool           started;        /* true: pthread started. */
  bool           detached;       /* true: pthread_detached'ed */
  bool           terminated;     /* true: detach'ed+exit'ed */
  pthread_t      thread;         /* Includes pid */
  sem_t          exit_sem;       /* Implements join */
  sem_t          data_sem;       /* Implements join */
  pthread_addr_t exit_value;     /* Returned data */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

struct pthread_tcb_s; /* Forward reference */
struct task_group_s;  /* Forward reference */

void weak_function pthread_initialize(void);
int pthread_schedsetup(FAR struct pthread_tcb_s *tcb, int priority,
                       start_t start, pthread_startroutine_t entry);

#ifdef CONFIG_PTHREAD_CLEANUP
void pthread_cleanup_popall(FAR struct pthread_tcb_s *tcb);
#endif

int pthread_completejoin(pid_t pid, FAR void *exit_value);
void pthread_destroyjoin(FAR struct task_group_s *group,
                         FAR struct join_s *pjoin);
FAR struct join_s *pthread_findjoininfo(FAR struct task_group_s *group,
                                        pid_t pid);
void pthread_release(FAR struct task_group_s *group);

int pthread_sem_take(FAR sem_t *sem, FAR const struct timespec *abs_timeout,
                     bool intr);
#ifdef CONFIG_PTHREAD_MUTEX_UNSAFE
int pthread_sem_trytake(sem_t *sem);
#endif
int pthread_sem_give(sem_t *sem);

#ifndef CONFIG_PTHREAD_MUTEX_UNSAFE
int pthread_mutex_take(FAR struct pthread_mutex_s *mutex,
                       FAR const struct timespec *abs_timeout, bool intr);
int pthread_mutex_trytake(FAR struct pthread_mutex_s *mutex);
int pthread_mutex_give(FAR struct pthread_mutex_s *mutex);
void pthread_mutex_inconsistent(FAR struct pthread_tcb_s *tcb);
#else
#  define pthread_mutex_take(m,abs_timeout,i)  pthread_sem_take(&(m)->sem,(abs_timeout),(i))
#  define pthread_mutex_trytake(m)             pthread_sem_trytake(&(m)->sem)
#  define pthread_mutex_give(m)                pthread_sem_give(&(m)->sem)
#endif

#ifdef CONFIG_PTHREAD_MUTEX_TYPES
int pthread_mutexattr_verifytype(int type);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __SCHED_PTHREAD_PTHREAD_H */
