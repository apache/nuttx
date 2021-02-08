/****************************************************************************
 * sched/signal/sig_procmask.c
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

#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <sched.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/wdog.h>
#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>

#include "sched/sched.h"
#include "signal/signal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsig_procmask
 *
 * Description:
 *   This function allows the calling process to examine and/or change its
 *   signal mask.  If the 'set' is not NULL, then it points to a set of
 *   signals to be used to change the currently blocked set.  The value of
 *   'how' indicates the manner in which the set is changed.
 *
 *   If there any pending unblocked signals after the call to
 *   nxsig_procmask(), those signals will be delivered before
 *    nxsig_procmask() returns.
 *
 *   If nxsig_procmask() fails, the signal mask of the process is not changed
 *   by this function call.
 *
 *   This is an internal OS interface.  It is functionally equivalent to
 *   sigprocmask() except that it does not modify the errno value.
 *
 * Input Parameters:
 *   how - How the signal mask will be changed:
 *         SIG_BLOCK   - The resulting set is the union of the current set
 *                       and the signal set pointed to by 'set'.
 *         SIG_UNBLOCK - The resulting set is the intersection of the current
 *                       set and the complement of the signal set pointed to
 *                       by 'set'.
 *         SIG_SETMASK - The resulting set is the signal set pointed to by
 *                       'set'.
 *   set  - Location of the new signal mask
 *   oset - Location to store the old signal mask
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *
 *     EINVAL - The 'how' argument is invalid.
 *
 ****************************************************************************/

int nxsig_procmask(int how, FAR const sigset_t *set, FAR sigset_t *oset)
{
  FAR struct tcb_s *rtcb = this_task();
  irqstate_t flags;
  int        ret = OK;

  sched_lock();

  /* Return the old signal mask if requested */

  if (oset != NULL)
    {
      *oset = rtcb->sigprocmask;
    }

  /* Modify the current signal mask if so requested */

  if (set != NULL)
    {
      /* Some of these operations are non-atomic.  We need to protect
       * ourselves from attempts to process signals from interrupts
       */

      flags = enter_critical_section();

      /* Okay, determine what we are supposed to do */

      switch (how)
        {
          /* The resulting set is the union of the current set and the
           * signal set pointed to by set.
           */

          case SIG_BLOCK:
            rtcb->sigprocmask |= *set;
            break;

          /* The resulting set is the intersection of the current set and
           * the complement of the signal set pointed to by set.
           */

          case SIG_UNBLOCK:
            rtcb->sigprocmask &= ~(*set);
            break;

          /* The resulting set is the signal set pointed to by set. */

          case SIG_SETMASK:
            rtcb->sigprocmask = *set;
            break;

          default:
            ret = -EINVAL;
            break;
        }

      leave_critical_section(flags);

      /* Now, process any pending signals that were just unmasked */

      nxsig_unmask_pendingsignal();
    }

  sched_unlock();
  return ret;
}

/****************************************************************************
 * Name: sigprocmask
 *
 * Description:
 *   This function allows the calling process to examine and/or change its
 *   signal mask.  If the 'set' is not NULL, then it points to a set of
 *   signals to be used to change the currently blocked set.  The value of
 *   'how' indicates the manner in which the set is changed.
 *
 *   If there any pending unblocked signals after the call to sigprocmask(),
 *   those signals will be delivered before sigprocmask() returns.
 *
 *   If sigprocmask() fails, the signal mask of the process is not changed
 *   by this function call.
 *
 * Input Parameters:
 *   how - How the signal mask will be changed:
 *         SIG_BLOCK   - The resulting set is the union of the current set
 *                       and the signal set pointed to by 'set'.
 *         SIG_UNBLOCK - The resulting set is the intersection of the current
 *                       set and the complement of the signal set pointed to
 *                       by 'set'.
 *         SIG_SETMASK - The resulting set is the signal set pointed to by
 *                       'set'.
 *   set  - Location of the new signal mask
 *   oset - Location to store the old signal mask
 *
 * Returned Value:
 *   This function will return 0 (OK) on success or -1 (ERROR) if how is
 *   invalid.  In the latter case, the errno variable will be set to EINVAL.
 *
 ****************************************************************************/

int sigprocmask(int how, FAR const sigset_t *set, FAR sigset_t *oset)
{
  int ret;

  /* Let nxsig_procmask do all of the work */

  ret = nxsig_procmask(how, set, oset);
  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  return ret;
}
