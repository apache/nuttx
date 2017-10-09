/****************************************************************************
 * sched/semaphore/semaphore.h
 *
 *   Copyright (C) 2007, 2009-2016 Gregory Nutt. All rights reserved.
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

#ifndef __SCHED_SEMAPHORE_SEMAPHORE_H
#define __SCHED_SEMAPHORE_SEMAPHORE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>
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

void nxsem_timeout(int argc, wdparm_t pid);

/* Recover semaphore resources with a task or thread is destroyed  */

void nxsem_recover(FAR struct tcb_s *tcb);

/* Special logic needed only by priority inheritance to manage collections of
 * holders of semaphores.
 */

#ifdef CONFIG_PRIORITY_INHERITANCE
void nxsem_initholders(void);
void nxsem_destroyholder(FAR sem_t *sem);
void nxsem_addholder(FAR sem_t *sem);
void nxsem_addholder_tcb(FAR struct tcb_s *htcb, FAR sem_t *sem);
void nxsem_boostpriority(FAR sem_t *sem);
void nxsem_releaseholder(FAR sem_t *sem);
void nxsem_restorebaseprio(FAR struct tcb_s *stcb, FAR sem_t *sem);
#  ifndef CONFIG_DISABLE_SIGNALS
void nxsem_canceled(FAR struct tcb_s *stcb, FAR sem_t *sem);
#  else
#    define nxsem_canceled(stcb, sem)
#  endif
#else
#  define nxsem_initholders()
#  define nxsem_destroyholder(sem)
#  define nxsem_addholder(sem)
#  define nxsem_addholder_tcb(htcb,sem)
#  define nxsem_boostpriority(sem)
#  define nxsem_releaseholder(sem)
#  define nxsem_restorebaseprio(stcb,sem)
#  define nxsem_canceled(stcb,sem)
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __SCHED_SEMAPHORE_SEMAPHORE_H */
