/****************************************************************************
 * sched/wqueue/kwork_notifier.c
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
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <sched.h>
#include <assert.h>

#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>

#include "wqueue/wqueue.h"

#ifdef CONFIG_WQUEUE_NOTIFIER

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes one notification list entry.  It is cast-
 * compatible with struct work_notifier_s.  This structure is an allocated
 * container for the user notification data.   It is allocated because it
 * must persist until the work is executed.
 */

struct work_notifier_entry_s
{
  /* This must appear at the beginning of the structure.  A reference to
   * the struct work_notifier_entry_s instance must be cast-compatible with
   * struct dq_entry_s.
   */

  struct work_s work;           /* Used for scheduling the work */

  /* User notification information */

  struct work_notifier_s info;  /* The notification info */

  /* Additional payload needed to manage the notification */

  uint32_t key;                 /* Unique ID for the notification */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This is a doubly linked list of free notifications. */

static dq_queue_t g_notifier_free;

/* This is a doubly linked list of pending notifications.  When an event
 * occurs available, *all* of the waiters for that event in this list will
 * be notified and the entry will be freed.  If there are multiple waiters
 * for some resource, then only the first to execute thread will get the
 * resource.  Lower priority threads will need to call work_notifier_setup()
 * once again.
 */

static dq_queue_t g_notifier_pending;

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

static FAR struct work_notifier_entry_s *work_notifier_find(uint32_t key)
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

      if (notifier->key == key)
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

static uint32_t work_notifier_key(void)
{
  static uint32_t notifier_key;

  if (++notifier_key == 0)
    {
      notifier_key = 1;
    }

  return notifier_key;
}

/****************************************************************************
 * Name: work_notifier_worker
 *
 * Description:
 *   Forward to the real worker and free the notification.
 *
 ****************************************************************************/

static void work_notifier_worker(FAR void *arg)
{
  FAR struct work_notifier_entry_s *notifier =
    (FAR struct work_notifier_entry_s *)arg;
  irqstate_t flags;

  /* Forward to the real worker */

  notifier->info.worker(notifier->info.arg);

  /* Disable interrupts very briefly. */

  flags = enter_critical_section();

  /* Put the notification to the free list */

  dq_addlast((FAR dq_entry_t *)notifier, &g_notifier_free);

  leave_critical_section(flags);
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
  irqstate_t flags;
  int ret;

  DEBUGASSERT(info != NULL && info->worker != NULL);
  DEBUGASSERT(info->qid == HPWORK || info->qid == LPWORK);

  /* Disable interrupts very briefly. */

  flags = enter_critical_section();

  /* Try to get the entry from the free list */

  notifier = (FAR struct work_notifier_entry_s *)
    dq_remfirst(&g_notifier_free);

  leave_critical_section(flags);

  if (notifier == NULL)
    {
      /* Allocate a new notification entry */

      notifier = kmm_malloc(sizeof(struct work_notifier_entry_s));
    }

  if (notifier == NULL)
    {
      ret = -ENOMEM;
    }
  else
    {
      /* Initialize the work structure */

      memset(&notifier->work, 0, sizeof(notifier->work));

      /* Duplicate the notification info */

      memcpy(&notifier->info, info, sizeof(struct work_notifier_s));

      /* Disable interrupts very briefly. */

      flags = enter_critical_section();

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

      leave_critical_section(flags);
    }

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
  irqstate_t flags;
  int ret = OK;

  /* Disable interrupts very briefly. */

  flags = enter_critical_section();

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

      /* Put the notification to the free list */

      dq_addlast((FAR dq_entry_t *)notifier, &g_notifier_free);
    }

  leave_critical_section(flags);
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
  irqstate_t flags;

  /* Don't let any newly started threads block this thread until all of
   * the notifications and been sent.
   */

  flags = enter_critical_section();

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

          work_queue(info->qid, &notifier->work,
                     work_notifier_worker, entry, 0);
        }
    }

  leave_critical_section(flags);
}

#endif /* CONFIG_WQUEUE_NOTIFIER */
