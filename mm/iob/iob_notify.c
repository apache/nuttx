/****************************************************************************
 * mm/iob/iob_notify.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <sched.h>
#include <assert.h>

#include <nuttx/irq.h>
#include <nuttx/signal.h>
#include <nuttx/semaphore.h>
#include <nuttx/mm/iob.h>

#include "iob.h"

#ifdef CONFIG_IOB_NOTIFIER

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This is the saved information for one IOB notification */

struct iob_notify_s
{
  bool waiter;    /* True if the waiter information is valid */
  uint8_t signo;  /* The signal number to use when notifying the PID */
  pid_t pid;      /* The PID to be notified when an IOB is available */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This is a an array of threads waiting for an IOB.  When an IOB becomes
 * available, *all* of the waiters in this thread will be signaled and the
 * entry will be marked invalid.  If there are multiple waiters then only
 * the highest priority thread will get the IOB.  Lower priority threads
 * will need to call iob_notify once again.
 */

static struct iob_notify_s g_iob_notify[CONFIG_IOB_NWAITERS];

/* This semaphore is used as mutex to enforce mutually exlusive access to
 * the g_io_notify[] array.
 */

static sem_t g_notify_sem = SEM_INITIALIZER(1);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: iob_notify_setup
 *
 * Description:
 *   Set up to notify the specified PID with the provided signal number.
 *
 *   NOTE: To avoid race conditions, the caller should set the sigprocmask
 *   to block signal delivery.  The signal will be delivered once the
 *   signal is removed from the sigprocmask.
 *
 * Input Parameters:
 *   pid   - The PID to be notified.  If a zero value is provided, then the
 *           PID of the calling thread will be used.
 *   signo - The signal number to use with the notification.
 *
 * Returned Value:
 *   > 0   - There are already free IOBs and this this number of free IOBs
 *           (CAUTION:  This value is volatile).  No signal notification
 *           will be provided.
 *   == 0  - There are no free IOBs.  A signal will be sent to 'pid' when
 *           at least one IOB is available.  That IOB is *not* reserved for
 *           the caller.  Hence, due to race conditions, it could be taken
 *           by some other task.  In that event, the caller should call
 *           sig_notify again.
 *   < 0   - An unexpected error occurred and no signal will be sent.  The
 *           returned value is a negated errno value that indicates the
 *           nature of the failure.
 *
 ****************************************************************************/

int iob_notify_setup(int pid, int signo)
{
  int ret;
  int i;

  /* If the pid is zero, then use the pid of the calling thread */

  if (pid <= 0)
    {
      pid = getpid();
    }

  /* Get exclusive access to the notifier array */

  ret = nxsem_wait(&g_notify_sem);
  if (ret < 0)
    {
      return ret;
    }

  /* If there are already free IOBs, then just return the number of free
   * IOBs.
   */

  ret = iob_navail();
  if (ret <= 0)
    {
      /* Find a free entry in the g_iob_notify[] array */

      ret = -ENOSPC;
      for (i = 0; i < CONFIG_IOB_NWAITERS; i++)
        {
          if (!g_iob_notify[i].waiter)
            {
              g_iob_notify[i].waiter =  true;
              g_iob_notify[i].pid    =  pid;
              g_iob_notify[i].signo  =  signo;

              ret = OK;
              break;
            }
        }
    }

  (void)nxsem_post(&g_notify_sem);
  return ret;
}

/****************************************************************************
 * Name: iob_notify_teardown
 *
 * Description:
 *   Eliminate an IOB notification previously setup by iob_notify_setup().
 *   This function should only be called if the notification should be
 *   aborted prior to the notification.  The notification will automatically
 *   be torn down after the signal is sent.
 *
 * Input Parameters:
 *   pid   - The PID whose notification will be torn down.  If a zero value
 *           is provided, then the PID of the calling thread will be used.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int iob_notify_teardown(int pid, int signo)
{
  int ret;
  int i;

  /* If the pid is zero, then use the pid of the calling thread */

  if (pid <= 0)
    {
      pid = getpid();
    }

  /* Get exclusive access to the notifier array */

  ret = nxsem_wait(&g_notify_sem);
  if (ret < 0)
    {
      return ret;
    }

  /* Find the entry matching this PID in the g_iob_notify[] array.  We
   * assume that there is only one.
   */

  ret = -ENOENT;
  for (i = 0; i < CONFIG_IOB_NWAITERS; i++)
    {
      if (g_iob_notify[i].waiter && g_iob_notify[i].pid ==  pid)
        {
          g_iob_notify[i].waiter =  false;
          ret = OK;
          break;
        }
    }

  (void)nxsem_post(&g_notify_sem);
  return ret;
}

/****************************************************************************
 * Name: iob_notify_signal
 *
 * Description:
 *   An IOB has become available.  Signal all threads waiting for an IOB
 *   that an IOB is available.
 *
 *   When an IOB becomes available, *all* of the waiters in this thread will
 *   be signaled.  If there are multiple waiters then only the highest
 *   priority thread will get the IOB.  Lower priority threads will need to
 *   call iob_notify once again.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void iob_notify_signal(void)
{
  int ret;
  int i;

  /* Get exclusive access to the notifier array */

  ret = nxsem_wait(&g_notify_sem);
  while (ret < 0)
    {
      DEBUGASSERT(ret == -EINTR || ret == -ECANCELED);
    }

  /* Don't let any newly started threads block this thread until all of
   * the notifications and been sent.
   */

  sched_lock();

  /* Find the entry matching this PID in the g_iob_notify[] array.  We
   * assume that there is only one.
   */

  for (i = 0; i < CONFIG_IOB_NWAITERS; i++)
    {
      if (g_iob_notify[i].waiter)
        {
          /* Signal the waiter and free the entry */

          (void)nxsig_kill(g_iob_notify[i].pid, g_iob_notify[i].signo);
          g_iob_notify[i].waiter =  false;
        }
    }

  sched_unlock();
  (void)nxsem_post(&g_notify_sem);
}

#endif /* CONFIG_IOB_NOTIFIER */
