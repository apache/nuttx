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
  FAR struct iob_notify_s *flink; /* Supports a singly linked list */
  uint8_t signo;                  /* The signal number for notification */
  pid_t pid;                      /* The PID to be notified */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This is a statically allocated pool of notification structures */

static struct iob_notify_s g_iobnotify_pool[CONFIG_IOB_NWAITERS];

/* This is a list of free notification structures */

static FAR struct iob_notify_s *g_iobnotify_free;

/* This is a singly linked list pending notifications.  When an IOB becomes
 * available, *all* of the waiters in this list will be signaled and the
 * entry will be freed.  If there are multiple waiters then only the highest
 * priority thread will get the IOB.  Lower priority threads will need to
 * call iob_notify_setup() once again.
 */

static FAR struct iob_notify_s *g_iobnotify_pending;

/* This semaphore is used as mutex to enforce mutually exclusive access to
 * the IOB notification structures.
 */

static sem_t g_iobnotify_sem;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: iob_notify_alloc
 *
 * Description:
 *   Allocate a notification structure by removing it from the free list.
 *
 ****************************************************************************/

static FAR struct iob_notify_s *iob_notify_alloc(void)
{
  FAR struct iob_notify_s *notify;

  notify = g_iobnotify_free;
  if (notify != NULL)
    {
      g_iobnotify_free = notify->flink;
      notify->flink    = NULL;
    }

  return notify;
}

/****************************************************************************
 * Name: iob_notify_free
 *
 * Description:
 *   Free a notification structure by returning it to the head of the free
 *   list.
 *
 ****************************************************************************/

static void iob_notify_free(FAR struct iob_notify_s *notify)
{
  notify->flink    = g_iobnotify_free;
  g_iobnotify_free = notify;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: iob_notify_initialize
 *
 * Description:
 *   Set up the notification structure for normal operation.
 *
 ****************************************************************************/

void iob_notify_initialize(void)
{
  int i;

  /* Add each notification structure to the free list */

  for (i = 0; i < CONFIG_IOB_NWAITERS; i++)
    {
      FAR struct iob_notify_s *notify = &g_iobnotify_pool[i];

      /* Add the pre-allocated notification to the head of the free list */

      notify->flink    = g_iobnotify_free;
      g_iobnotify_free = notify;
    }

  /* Initialize the semaphore that enforces mutually exclusive access to the
   * notification data structures.
   */

  nxsem_init(&g_iobnotify_sem, 0, 1);
}

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

  /* If the pid is zero, then use the pid of the calling thread */

  if (pid <= 0)
    {
      pid = getpid();
    }

  /* Get exclusive access to the notifier data structures */

  ret = nxsem_wait(&g_iobnotify_sem);
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
      /* Allocate a new notification */

      FAR struct iob_notify_s *notify = iob_notify_alloc();
      if (notify == NULL)
        {
          ret = -ENOSPC;
        }
      else
        {
          /* Initialize the notification structure */

          notify->pid         =  pid;
          notify->signo       =  signo;

          /* And add it to the head of the pending list */

          notify->flink       = g_iobnotify_pending;
          g_iobnotify_pending = notify;
          ret                 = OK;
        }
    }

  (void)nxsem_post(&g_iobnotify_sem);
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
  FAR struct iob_notify_s *notify;
  FAR struct iob_notify_s *prev;
  int ret;

  /* If the pid is zero, then use the pid of the calling thread */

  if (pid <= 0)
    {
      pid = getpid();
    }

  /* Get exclusive access to the notifier data structures */

  ret = nxsem_wait(&g_iobnotify_sem);
  if (ret < 0)
    {
      return ret;
    }

  /* Find the entry matching this PID in the g_iobnotify_pending list.  We
   * assume that there is only one.
   */

  ret = -ENOENT;
  for (prev = NULL, notify = g_iobnotify_pending;
       notify != NULL;
       prev = notify, notify = notify->flink)
    {
      /* Is this the one we were looking for? */

      if (notify->pid ==  pid)
        {
          /* Yes, remove it from the pending list */

          if (prev == NULL)
            {
              g_iobnotify_pending = notify->flink;
            }
          else
            {
              prev->flink = notify->flink;
            }

          /* And add it to the free list */

          iob_notify_free(notify);

          ret = OK;
          break;
        }
    }

  (void)nxsem_post(&g_iobnotify_sem);
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
  FAR struct iob_notify_s *notify;
  int ret;

  /* Get exclusive access to the notifier data structure */

  ret = nxsem_wait(&g_iobnotify_sem);
  while (ret < 0)
    {
      DEBUGASSERT(ret == -EINTR || ret == -ECANCELED);
    }

  /* Don't let any newly started threads block this thread until all of
   * the notifications and been sent.
   */

  sched_lock();

  /* Process the notification at the head of the pending list until the
   * pending list is empty  */

  while ((notify = g_iobnotify_pending) != NULL)
    {
      /* Remove the notification from the pending list */

      g_iobnotify_pending = notify->flink;

      /* Signal the waiter */

      (void)nxsig_kill(notify->pid, notify->signo);

      /* Free the notification by returning it to the free list */

      iob_notify_free(notify);
    }

  sched_unlock();
  (void)nxsem_post(&g_iobnotify_sem);
}

#endif /* CONFIG_IOB_NOTIFIER */
