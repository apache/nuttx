/****************************************************************************
 * sched/pthread/pthread_keydelete.c
 *
 *   Copyright (C) 2007, 2009, 2018 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sched.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>

#include "sched/sched.h"
#include "pthread/pthread.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_key_delete
 *
 * Description:
 *   This POSIX function deletes a thread-specific data key
 *   previously returned by pthread_key_create().
 *
 * Input Parameters:
 *   key - the key to delete
 *
 * Returned Value:
 *   Returns zero (OK) on success.  EINVAL may be returned if an invalid
 *   key is received.
 *
 * POSIX Compatibility:
 *
 ****************************************************************************/

int pthread_key_delete(pthread_key_t key)
{
#if CONFIG_NPTHREAD_KEYS > 0
  FAR struct tcb_s *rtcb = this_task();
  FAR struct task_group_s *group = rtcb->group;
  pthread_keyset_t mask;
  irqstate_t flags;
  int ret = EINVAL;

  DEBUGASSERT((unsigned)key < PTHREAD_KEYS_MAX && group != NULL);
  if ((unsigned)key < PTHREAD_KEYS_MAX)
    {
      /* This is done in a critical section here to avoid concurrent
       * modification of the group keyset.
       */

      mask  = (1 << key);
      flags = spin_lock_irqsave();

      DEBUGASSERT((group->tg_keyset & mask) != 0);
      group->tg_keyset &= ~mask;
      spin_unlock_irqrestore(flags);

      ret = OK;
    }

  return ret;
#else
  return ENOSYS;
#endif
}

