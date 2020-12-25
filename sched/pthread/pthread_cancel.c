/****************************************************************************
 * sched/pthread/pthread_cancel.c
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

#include <sys/types.h>
#include <unistd.h>
#include <pthread.h>
#include <errno.h>

#include "sched/sched.h"
#include "task/task.h"
#include "pthread/pthread.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int pthread_cancel(pthread_t thread)
{
  FAR struct tcb_s *tcb;

  /* First, make sure that the handle references a valid thread */

  if (thread == 0)
    {
      /* pid == 0 is the IDLE task (in a single CPU configuration).  Callers
       * cannot cancel the IDLE task.
       */

      return ESRCH;
    }

  tcb = nxsched_get_tcb((pid_t)thread);
  if (tcb == NULL)
    {
      /* The pid does not correspond to any known thread.  The thread
       * has probably already exited.
       */

      return ESRCH;
    }

  /* Only pthreads should use this interface */

  DEBUGASSERT((tcb->flags & TCB_FLAG_TTYPE_MASK) ==
               TCB_FLAG_TTYPE_PTHREAD);

  /* Notify the target if the non-cancelable or deferred cancellation set */

  if (nxnotify_cancellation(tcb))
    {
      return OK;
    }

  /* Otherwise, perform the asyncrhonous cancellation */

  /* Check to see if the ID refers to ourselves.. this would be the
   * same as pthread_exit(PTHREAD_CANCELED).
   */

  if (tcb == this_task())
    {
      pthread_exit(PTHREAD_CANCELED);
    }

#ifdef CONFIG_PTHREAD_CLEANUP
  /* Perform any stack pthread clean-up callbacks.
   *
   * REVISIT: In this case, the clean-up callback will execute on the
   * thread of the caller of pthread cancel, not on the thread of
   * the thread-to-be-canceled.  This is a problem when deferred
   * cancellation is not supported because, for example, the clean-up
   * function will be unable to unlock its own mutexes.
   */

  pthread_cleanup_popall(tcb);
#endif

  /* Complete pending join operations */

  pthread_completejoin((pid_t)thread, PTHREAD_CANCELED);

#ifndef CONFIG_PTHREAD_MUTEX_UNSAFE
  /* Recover any mutexes still held by the canceled thread */

  pthread_mutex_inconsistent(tcb);
#endif

  /* Then let nxtask_terminate do the real work */

  return nxtask_terminate((pid_t)thread, false);
}
