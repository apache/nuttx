/****************************************************************************
 * sched/wqueue/kwork_notifier.c
 *
 *   Copyright (C) 2018-2019 Gregory Nutt. All rights reserved.
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
#include <string.h>
#include <sched.h>
#include <assert.h>

#include <nuttx/kmalloc.h>
#include <nuttx/semaphore.h>
#include <nuttx/wqueue.h>

#include "wqueue/wqueue.h"

#ifdef CONFIG_WQUEUE_NOTIFIER

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This is a doubly linked list of pending notifications.  When an event
 * occurs available, *all* of the waiters for that event in this list will
 * be notified and the entry will be freed.  If there are multiple waiters
 * for some resource, then only the first to execute thread will get the
 * resource.  Lower priority threads will need to call work_notifier_setup()
 * once again.
 */

static dq_queue_t g_notifier_pending;

/* This semaphore is used as mutex to enforce mutually exclusive access to
 * the notification data structures.
 */

static sem_t g_notifier_sem = SEM_INITIALIZER(1);

/* Used for lookup key generation */

static uint16_t g_notifier_key;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: work_notifier_find
 *
 * Description:
 *   Given a unique key for notification, find the corresponding notification
 *   structure in the pending notification list.
 *
 ****************************************************************************/

static FAR struct work_notifier_entry_s *work_notifier_find(int16_t key)
{
  FAR struct work_notifier_entry_s *notifier;
  FAR dq_entry_t *entry;

  /* Find the entry matching this key in the g_notifier_pending list. */

  for (entry  = dq_peek(&g_notifier_pending);
       entry != NULL;
       entry  = dq_next(entry))
    {
      notifier = (FAR struct work_notifier_entry_s *)entry;

      /* Is this the one we were looking for? */

      if (notifier->key ==  key)
        {
          /* Yes.. return a reference to it */

          return notifier;
        }
    }

  return NULL;
}

/****************************************************************************
 * Name: work_notifier_key
 *
 * Description:
 *   Generate a unique key for this notification.
 *
 ****************************************************************************/

static int16_t work_notifier_key(void)
{
  int16_t key;

  /* Loop until a unique key is generated.  Range 1-INT16_MAX. */

  do
    {
      if (g_notifier_key >= INT16_MAX)
        {
          g_notifier_key = 0;
        }

      key = (int16_t)++g_notifier_key;
    }
  while (work_notifier_find(key) != NULL);

  return key;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: work_notifier_setup
 *
 * Description:
 *   Set up to provide a notification when an event occurs.
 *
 * Input Parameters:
 *   info - Describes the work notification.
 *
 * Returned Value:
 *   > 0   - The key which may be used later in a call to
 *           work_notifier_teardown().
 *   == 0  - Not used (reserved for wrapper functions).
 *   < 0   - An unexpected error occurred and no notification will be sent.
 *           The returned value is a negated errno value that indicates the
 *           nature of the failure.
 *
 ****************************************************************************/

int work_notifier_setup(FAR struct work_notifier_s *info)
{
  FAR struct work_notifier_entry_s *notifier;
  int ret;

  DEBUGASSERT(info != NULL && info->worker != NULL);
  DEBUGASSERT(info->qid == HPWORK || info->qid == LPWORK);

  /* Get exclusive access to the notifier data structures */

  ret = nxsem_wait(&g_notifier_sem);
  if (ret < 0)
    {
      return ret;
    }

  /* Allocate a new notification entry */

  notifier = kmm_malloc(sizeof(struct work_notifier_entry_s));
  if (notifier == NULL)
    {
      ret = -ENOMEM;
    }
  else
    {
      /* Duplicate the notification info */

      memcpy(&notifier->info, info, sizeof(struct work_notifier_s));

      /* Generate a unique key for this notification */

      notifier->key = work_notifier_key();

      /* Add the notification to the tail of the pending list
       *
       * REVISIT:  Work will be processed in FIFO order.  Perhaps
       * we should should consider saving the notification is the
       * order of the caller's execution priority so that the
       * notifications executed in a saner order?
       */

      dq_addlast((FAR dq_entry_t *)notifier, &g_notifier_pending);
      ret = notifier->key;
    }

  (void)nxsem_post(&g_notifier_sem);
  return ret;
}

/****************************************************************************
 * Name: work_notifier_teardown
 *
 * Description:
 *   Eliminate a notification previously setup by work_notifier_setup().
 *   This function should only be called if the notification should be
 *   aborted prior to the notification.  The notification will automatically
 *   be torn down after the notification executes.
 *
 * Input Parameters:
 *   key - The key value returned from a previous call to
 *         work_notifier_setup().
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int work_notifier_teardown(int key)
{
  FAR struct work_notifier_entry_s *notifier;
  int ret;

  DEBUGASSERT(key > 0 && key <= INT16_MAX);

  /* Get exclusive access to the notifier data structures */

  ret = nxsem_wait(&g_notifier_sem);
  if (ret < 0)
    {
      return ret;
    }

  /* Find the entry matching this PID in the g_notifier_pending list.  We
   * assume that there is only one.
   */

  notifier = work_notifier_find(key);
  if (notifier == NULL)
    {
      /* There is no notification with this key in the pending list */

      ret = -ENOENT;
    }
  else
    {
      /* Found it!  Remove the notification from the pending list */

      dq_rem((FAR dq_entry_t *)notifier, &g_notifier_pending);

      /* Free the notification */

      kmm_free(notifier);
      ret = OK;
    }

  (void)nxsem_post(&g_notifier_sem);
  return ret;
}

/****************************************************************************
 * Name: work_notifier_signal
 *
 * Description:
 *   An event has just occurred.  Notify all threads waiting for that event.
 *
 *   When an event of interest occurs, *all* of the workers waiting for this
 *   event will be executed.  If there are multiple workers for a resource
 *   then only the first to execute will get the resource.  Others will
 *   need to call work_notifier_setup() once again.
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

void work_notifier_signal(enum work_evtype_e evtype,
                          FAR void *qualifier)
{
  FAR struct work_notifier_entry_s *notifier;
  FAR dq_entry_t *entry;
  FAR dq_entry_t *next;
  int ret;

  /* Get exclusive access to the notifier data structure */

  do
    {
      ret = nxsem_wait(&g_notifier_sem);
      DEBUGASSERT(ret >= 0 || ret == -EINTR || ret == -ECANCELED);
    }
  while (ret < 0);

  /* Don't let any newly started threads block this thread until all of
   * the notifications and been sent.
   */

  sched_lock();

  /* Process the notification at the head of the pending list until the
   * pending list is empty
   */

  for (entry = dq_peek(&g_notifier_pending);
       entry != NULL;
       entry = next)
    {
      FAR struct work_notifier_s *info;

      /* Set up for the next time through the loop (in case the entry is
       * removed from the list).
       */

      next = entry->flink;

      /* Set up some convenience pointers */

      notifier = (FAR struct work_notifier_entry_s *)entry;
      info     = &notifier->info;

      /* Check if this is the a notification request for the event that
       * just occurred.
       */

      if (info->evtype == evtype && info->qualifier == qualifier)
        {
          /* Yes.. Remove the notification from the pending list */

          dq_rem((FAR dq_entry_t *)notifier, &g_notifier_pending);

          /* Schedule the work.  The entire notifier entry is passed as an
           * argument to the work function because that function is
           * responsible for freeing the allocated memory.
           */

          (void)work_queue(info->qid, &notifier->work, info->worker,
                           entry, 0);
        }
    }

  sched_unlock();
  (void)nxsem_post(&g_notifier_sem);
}

#endif /* CONFIG_WQUEUE_NOTIFIER */
