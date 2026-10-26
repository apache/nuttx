/****************************************************************************
 * sched/pthread/pthread_mutexinconsistent.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <pthread.h>
#include <sched.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/sched.h>
#include <nuttx/semaphore.h>
#include <nuttx/pthread.h>
#include <nuttx/atomic.h>

#ifndef CONFIG_BUILD_FLAT
#  include <nuttx/addrenv.h>
#  include <nuttx/arch.h>
#  include <nuttx/mm/mm.h>
#  include <nuttx/pgalloc.h>
#  include <nuttx/userspace.h>
#endif

#include "pthread/pthread.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_PTHREAD_MUTEX_TYPES
#  define mutex_get_holder(m)  nxrmutex_get_holder(m)
#  define mutex_reset(m)       nxrmutex_reset(m)
#else
#  define mutex_get_holder(m)  nxmutex_get_holder(m)
#  define mutex_reset(m)       nxmutex_reset(m)
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifndef CONFIG_BUILD_FLAT
static bool pthread_mutex_accessible(FAR struct tcb_s *tcb,
                                     FAR const void *ptr, size_t size)
{
  uintptr_t start = (uintptr_t)ptr;
  uintptr_t end = start + size - 1;

  if ((tcb->flags & TCB_FLAG_TTYPE_MASK) == TCB_FLAG_TTYPE_KERNEL)
    {
      return true;
    }

  if (start == 0 || end < start)
    {
      return false;
    }

#ifdef CONFIG_BUILD_KERNEL
  if (!uaccess_ok(ptr, size) || tcb->addrenv_own == NULL ||
      (start < ARCH_TEXT_VEND && end >= CONFIG_ARCH_TEXT_VBASE))
    {
      return false;
    }

  /* A user virtual address need not have a backing page.  In particular,
   * do not fault again while recovering a task which has already faulted.
   */

  start = MM_PGALIGNDOWN(start);
  end = MM_PGALIGNDOWN(end);
  for (; ; start += MM_PGSIZE)
    {
      if (up_addrenv_find_page(&tcb->addrenv_own->addrenv, start) == 0)
        {
          return false;
        }

      if (start == end)
        {
          return true;
        }
    }
#else
  /* Protected builds share an address space, but user lists must still
   * not lead recovery into kernel memory.
   */

  return (start >= USERSPACE->us_datastart &&
          end < USERSPACE->us_dataend) ||
         (start >= USERSPACE->us_bssstart &&
          end < USERSPACE->us_bssend) ||
         (umm_heapmember((FAR void *)start) &&
          umm_heapmember((FAR void *)end));
#endif
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_mutex_inconsistent
 *
 * Description:
 *   This function is called when a pthread is terminated via either
 *   pthread_exit() or pthread_cancel().  It will check for any mutexes
 *   held by exiting thread.  It will mark them as inconsistent and
 *   then wake up the highest priority waiter for the mutex.  That
 *   instance of pthread_mutex_lock() will then return EOWNERDEAD.
 *
 * Input Parameters:
 *   tcb -- a reference to the TCB of the exiting pthread.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void pthread_mutex_inconsistent(FAR struct tcb_s *tcb)
{
  FAR struct tls_info_s *tls = tcb->stack_alloc_ptr;
  FAR struct pthread_mutex_s *mutex;

#ifndef CONFIG_BUILD_FLAT
  if (!pthread_mutex_accessible(tcb, tls, sizeof(*tls)))
    {
      return;
    }
#endif

  /* Remove and process each mutex held by this task */

  while (tls->tl_mhead != NULL)
    {
      /* Remove the mutex from the TCB list */

      mutex = tls->tl_mhead;
#ifndef CONFIG_BUILD_FLAT
      if (((uintptr_t)mutex & (sizeof(uintptr_t) - 1)) != 0 ||
          !pthread_mutex_accessible(tcb, mutex, sizeof(*mutex)))
        {
          tls->tl_mhead = NULL;
          break;
        }
#endif

      if ((mutex->flags & _PTHREAD_MFLAGS_ROBUST) == 0 ||
          mutex_get_holder(&mutex->mutex) != tcb->pid)
        {
          tls->tl_mhead = NULL;
          break;
        }

      tls->tl_mhead = mutex->flink;
      mutex->flink  = NULL;

      /* Mark the mutex as INCONSISTENT and wake up any waiting thread */

      mutex->flags |= _PTHREAD_MFLAGS_INCONSISTENT;
      mutex_reset(&mutex->mutex);
    }
}
