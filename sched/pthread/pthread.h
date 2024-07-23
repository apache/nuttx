/****************************************************************************
 * sched/pthread/pthread.h
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

#include <nuttx/compiler.h>
#include <nuttx/semaphore.h>
#include <nuttx/sched.h>

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

int pthread_setup_scheduler(FAR struct pthread_tcb_s *tcb, int priority,
                            start_t start, pthread_startroutine_t entry);

int pthread_completejoin(pid_t pid, FAR void *exit_value);
void pthread_destroyjoin(FAR struct task_group_s *group,
                         FAR struct task_join_s *pjoin);
int pthread_findjoininfo(FAR struct task_group_s *group, pid_t pid,
                         FAR struct task_join_s **join, bool create);
void pthread_release(FAR struct task_group_s *group);

int pthread_sem_take(FAR sem_t *sem, FAR const struct timespec *abs_timeout);
#ifdef CONFIG_PTHREAD_MUTEX_UNSAFE
int pthread_sem_trytake(FAR sem_t *sem);
#endif
int pthread_sem_give(FAR sem_t *sem);

#ifndef CONFIG_PTHREAD_MUTEX_UNSAFE
int pthread_mutex_take(FAR struct pthread_mutex_s *mutex,
                       FAR const struct timespec *abs_timeout);
int pthread_mutex_trytake(FAR struct pthread_mutex_s *mutex);
int pthread_mutex_give(FAR struct pthread_mutex_s *mutex);
void pthread_mutex_inconsistent(FAR struct tcb_s *tcb);
#else
#  define pthread_mutex_take(m,abs_timeout) pthread_sem_take(&(m)->sem,(abs_timeout))
#  define pthread_mutex_trytake(m)          pthread_sem_trytake(&(m)->sem)
#  define pthread_mutex_give(m)             pthread_sem_give(&(m)->sem)
#endif

#ifdef CONFIG_PTHREAD_MUTEX_TYPES
int pthread_mutexattr_verifytype(int type);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __SCHED_PTHREAD_PTHREAD_H */
