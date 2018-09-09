/****************************************************************************
 * sched/signal/sig_notifier.c
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

#include "signal/signal.h"

#ifdef CONFIG_SIG_NOTIFIER

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This is the saved information for one notification */

struct nxsig_notifier_s
{
  FAR struct nxsig_notifier_s *flink; /* Supports a singly linked list */
  FAR void *qualifier;                /* Event qualifier value */
  uint8_t signo;                      /* The signal number for notification */
  uint8_t evtype;                     /* See enum nxsig_evtype_e */
  pid_t pid;                          /* The PID to be notified */
  int16_t key;                        /* Unique ID for this notification */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This is a statically allocated pool of notification structures */

static struct nxsig_notifier_s g_notifier_pool[CONFIG_SIG_NOTIFIER_NWAITERS];

/* This is a list of free notification structures */

static FAR struct nxsig_notifier_s *g_notifier_free;

/* This is a singly linked list of pending notifications.  When an event
 * occurs available, *all* of the waiters for that event in this list will
 * be signaled and the entry will be freed.  If there are multiple waiters
 * for some resource, then only the highest priority thread will get the
 * resource.  Lower priority threads will need to call nxsig_notifier_setup()
 * once again.
 */

static FAR struct nxsig_notifier_s *g_notifier_pending;

/* This semaphore is used as mutex to enforce mutually exclusive access to
 * the notification data structures.
 */

static sem_t g_notifier_sem;

/* Used for lookup key generation */

static uint16_t g_notifier_key;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsig_notifier_alloc
 *
 * Description:
 *   Allocate a notification structure by removing it from the free list.
 *
 ****************************************************************************/

static FAR struct nxsig_notifier_s *nxsig_notifier_alloc(void)
{
  FAR struct nxsig_notifier_s *notifier;

  notifier = g_notifier_free;
  if (notifier != NULL)
    {
      g_notifier_free = notifier->flink;
      notifier->flink = NULL;
    }

  return notifier;
}

/****************************************************************************
 * Name: nxsig_notifier_free
 *
 * Description:
 *   Free a notification structure by returning it to the head of the free
 *   list.
 *
 ****************************************************************************/

static void nxsig_notifier_free(FAR struct nxsig_notifier_s *notifier)
{
  notifier->flink = g_notifier_free;
  g_notifier_free = notifier;
}

/****************************************************************************
 * Name: nxsig_notifier_find
 *
 * Description:
 *   Given a unique key for notification, find the corresponding notification
 *   structure in the pending notification list.
 *
 ****************************************************************************/

static FAR struct nxsig_notifier_s *
  nxsig_notifier_find(int16_t key, FAR struct nxsig_notifier_s **pprev)
{
  FAR struct nxsig_notifier_s *notifier;
  FAR struct nxsig_notifier_s *prev;

  /* Find the entry matching this key in the g_notifier_pending list. */

  for (prev = NULL, notifier = g_notifier_pending;
       notifier != NULL;
       prev = notifier, notifier = notifier->flink)
    {
      /* Is this the one we were looking for? */

      if (notifier->key ==  key)
        {
          /* Return the previous entry if so requested */

          if (pprev != NULL)
            {
              *pprev = prev;
            }

          return notifier;
        }
    }

  return NULL;
}

/****************************************************************************
 * Name: nxsig_notifier_key
 *
 * Description:
 *   Generate a unique key for this notification.
 *
 ****************************************************************************/

static int16_t nxsig_notifier_key(void)
{
  int16_t key;

  /* Loop until a unique key is generated */

  do
    {
      if (g_notifier_key >= INT16_MAX)
        {
          g_notifier_key = 0;
        }

      key = (int16_t)++g_notifier_key;
    }
  while (nxsig_notifier_find(key, NULL) != NULL);

  return key;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsig_notifier_initialize
 *
 * Description:
 *   Set up the notification structure for normal operation.
 *
 ****************************************************************************/

void nxsig_notifier_initialize(void)
{
  int i;

  /* Add each notification structure to the free list */

  for (i = 0; i < CONFIG_SIG_NOTIFIER_NWAITERS; i++)
    {
      FAR struct nxsig_notifier_s *notifier = &g_notifier_pool[i];

      /* Add the pre-allocated notification to the head of the free list */

      notifier->flink    = g_notifier_free;
      g_notifier_free = notifier;
    }

  /* Initialize the semaphore that enforces mutually exclusive access to the
   * notification data structures.
   */

  nxsem_init(&g_notifier_sem, 0, 1);
}

/****************************************************************************
 * Name: nxsig_notifier_setup
 *
 * Description:
 *   Set up to notify the specified PID with the provided signal number.
 *
 *   NOTE: To avoid race conditions, the caller should set the sigprocmask
 *   to block signal delivery.  The signal will be delivered once the
 *   signal is removed from the sigprocmask.
 *
 *   NOTE: If sigwaitinfo() or sigtimedwait() are used to catch the signal
 *   then then qualifier value may be recovered in the sival_ptr value of
 *   the struct siginfo instance.
 *
 * Input Parameters:
 *   pid       - The PID to be notified.  If a zero value is provided,
 *               then the PID of the calling thread will be used.
 *   signo     - The signal number to use with the notification.
 *   evtype    - The event type.
 *   qualifier - Event qualifier to distinguish different cases of the
 *               generic event type.
 *
 * Returned Value:
 *   > 0   - The key which may be used later in a call to
 *           nxsig_notifier_teardown().
 *   == 0  - Not used (reserved for wrapper functions).
 *   < 0   - An unexpected error occurred and no signal will be sent.  The
 *           returned value is a negated errno value that indicates the
 *           nature of the failure.
 *
 ****************************************************************************/

int nxsig_notifier_setup(int pid, int signo, enum nxsig_evtype_e evtype,
                         FAR void *qualifier)
{
  int ret;

  /* If the 'pid' is zero, then use the PID of the calling thread */

  if (pid <= 0)
    {
      pid = getpid();
    }

  /* Get exclusive access to the notifier data structures */

  ret = nxsem_wait(&g_notifier_sem);
  if (ret < 0)
    {
      return ret;
    }

  /* Allocate a new notification */

  FAR struct nxsig_notifier_s *notifier = nxsig_notifier_alloc();
  if (notifier == NULL)
    {
      ret = -ENOSPC;
    }
  else
    {
      /* Generate a unique key for this notification */

      int16_t key         = nxsig_notifier_key();

      /* Initialize the notification structure */

      notifier->pid       =  pid;
      notifier->signo     =  signo;
      notifier->evtype    =  evtype;
      notifier->key       =  key;
      notifier->qualifier =  qualifier;

      /* And add it to the head of the pending list */

      notifier->flink     = g_notifier_pending;
      g_notifier_pending  = notifier;
      ret                 = key;
    }

  (void)nxsem_post(&g_notifier_sem);
  return ret;
}

/****************************************************************************
 * Name: nxsig_notifier_teardown
 *
 * Description:
 *   Eliminate a notification previously setup by nxsig_notifier_setup().
 *   This function should only be called if the notification should be
 *   aborted prior to the notification.  The notification will automatically
 *   be torn down after the signal is sent.
 *
 * Input Parameters:
 *   key - The key value returned from a previous call to
 *         nxsig_notifier_setup().
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int nxsig_notifier_teardown(int key)
{
  FAR struct nxsig_notifier_s *notifier;
  FAR struct nxsig_notifier_s *prev;
  int ret;

  /* Get exclusive access to the notifier data structures */

  ret = nxsem_wait(&g_notifier_sem);
  if (ret < 0)
    {
      return ret;
    }

  /* Find the entry matching this PID in the g_notifier_pending list.  We
   * assume that there is only one.
   */

  notifier = nxsig_notifier_find(key, &prev);
  if (notifier == NULL)
    {
      /* There is no notification with this key in the pending list */

      ret = -ENOENT;
    }
  else
    {
      /* Found it!  Remove the notification from the pending list */

      if (prev == NULL)
        {
          g_notifier_pending = notifier->flink;
        }
      else
        {
          prev->flink = notifier->flink;
        }

      /* And add it to the free list */

      nxsig_notifier_free(notifier);
      ret = OK;
    }

  (void)nxsem_post(&g_notifier_sem);
  return ret;
}

/****************************************************************************
 * Name: nxsig_notifier_signal
 *
 * Description:
 *   An event has just occurred.  Signal all threads waiting for that event.
 *
 *   When an IOB becomes available, *all* of the waiters in this thread will
 *   be signaled.  If there are multiple waiters for a resource then only
 *   the highest priority thread will get the resource.  Lower priority
 *   threads will need to call nxsig_notify once again.
 *
 *   NOTE: If sigwaitinfo() or sigtimedwait() are used to catch the signal
 *   then then qualifier value may be obtained in the sival_ptr value of
 *   the struct siginfo instance.
 *
 * Input Parameters:
 *   evtype   - The type of the event that just occurred.
 *   qualifier - Event qualifier to distinguish different cases of the
 *               generic event type.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void nxsig_notifier_signal(enum nxsig_evtype_e evtype,
                           FAR void *qualifier)
{
  FAR struct nxsig_notifier_s *notifier;
  FAR struct nxsig_notifier_s *prev;
  FAR struct nxsig_notifier_s *next;
  int ret;

  /* Get exclusive access to the notifier data structure */

  ret = nxsem_wait(&g_notifier_sem);
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

  /* Find the entry matching this key in the g_notifier_pending list. */

  for (prev = NULL, notifier = g_notifier_pending;
       notifier != NULL; 
       notifier = next)
    {
#ifdef CONFIG_CAN_PASS_STRUCTS
      union sigval value;
#endif

      /* Set up for the next time through the loop (in case the entry is
       * removed from the list).
       */

      next = notifier->flink;

      /* Check if this is the a notification request for the event that
       * just occurred.
       */

      if (notifier->evtype == evtype && notifier->qualifier == qualifier)
        {
          /* Yes.. Remove the notification from the pending list */

          if (prev == NULL)
            {
              g_notifier_pending = next;
            }
          else
            {
              prev->flink = next;
            }

          /* Signal the waiter */

#ifdef CONFIG_CAN_PASS_STRUCTS
          value.sival_ptr = notifier->qualifier;
          (void)nxsig_queue(notifier->pid, notifier->signo, value);
#else
          (void)nxsig_queue(notifier->pid, notifier->signo,
                            notifier->qualifier);
#endif
          /* Free the notification by returning it to the free list */

          nxsig_notifier_free(notifier);
        }
      else
        {
          /* The entry was not removed, the current entry will be the
           * next previous entry.
           */

          prev = notifier;
        }
    }

  sched_unlock();
  (void)nxsem_post(&g_notifier_sem);
}

#endif /* CONFIG_SIG_NOTIFIER */
