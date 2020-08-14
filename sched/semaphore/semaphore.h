/****************************************************************************
 * sched/semaphore/semaphore.h
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

#ifndef __SCHED_SEMAPHORE_SEMAPHORE_H
#define __SCHED_SEMAPHORE_SEMAPHORE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <nuttx/semaphore.h>

#include <stdint.h>
#include <stdbool.h>
#include <sched.h>
#include <queue.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/* Common semaphore logic */

#ifdef CONFIG_PRIORITY_INHERITANCE
void nxsem_initialize(void);
#else
#  define nxsem_initialize()
#endif

/* Wake up a thread that is waiting on semaphore */

void nxsem_wait_irq(FAR struct tcb_s *wtcb, int errcode);

/* Handle semaphore timer expiration */

void nxsem_timeout(wdparm_t pid);

/* Recover semaphore resources with a task or thread is destroyed  */

void nxsem_recover(FAR struct tcb_s *tcb);

/* Special logic needed only by priority inheritance to manage collections of
 * holders of semaphores.
 */

#ifdef CONFIG_PRIORITY_INHERITANCE
void nxsem_initialize_holders(void);
void nxsem_destroyholder(FAR sem_t *sem);
void nxsem_add_holder(FAR sem_t *sem);
void nxsem_add_holder_tcb(FAR struct tcb_s *htcb, FAR sem_t *sem);
void nxsem_boost_priority(FAR sem_t *sem);
void nxsem_release_holder(FAR sem_t *sem);
void nxsem_restore_baseprio(FAR struct tcb_s *stcb, FAR sem_t *sem);
void nxsem_canceled(FAR struct tcb_s *stcb, FAR sem_t *sem);
#else
#  define nxsem_initialize_holders()
#  define nxsem_destroyholder(sem)
#  define nxsem_add_holder(sem)
#  define nxsem_add_holder_tcb(htcb,sem)
#  define nxsem_boost_priority(sem)
#  define nxsem_release_holder(sem)
#  define nxsem_restore_baseprio(stcb,sem)
#  define nxsem_canceled(stcb,sem)
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __SCHED_SEMAPHORE_SEMAPHORE_H */
