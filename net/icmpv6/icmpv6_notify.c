/****************************************************************************
 * net/icmpv6/icmpv6_notify.c
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

#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <netinet/in.h>

#include <nuttx/irq.h>
#include <nuttx/net/net.h>

#include "icmpv6/icmpv6.h"

#ifdef CONFIG_NET_ICMPv6_NEIGHBOR

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* List of tasks waiting for Neighbor Discover events */

static struct icmpv6_notify_s *g_icmpv6_waiters;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: icmpv6_wait_setup
 *
 * Description:
 *   Called BEFORE an Neighbor Solicitation is sent.  This function sets up
 *   the Neighbor Advertisement timeout before the Neighbor Solicitation
 *   is sent so that there is no race condition when icmpv6_wait() is called.
 *
 * Assumptions:
 *   This function is called from icmpv6_neighbor() and executes in the
 *   normal tasking environment.
 *
 ****************************************************************************/

void icmpv6_wait_setup(const net_ipv6addr_t ipaddr,
                       FAR struct icmpv6_notify_s *notify)
{
  irqstate_t flags;

  /* Initialize the wait structure */

  net_ipv6addr_copy(notify->nt_ipaddr, ipaddr);
  notify->nt_result = -ETIMEDOUT;

  /* This semaphore is used for signaling and, hence, should not have
   * priority inheritance enabled.
   */

  nxsem_init(&notify->nt_sem, 0, 0);
  nxsem_set_protocol(&notify->nt_sem, SEM_PRIO_NONE);

  /* Add the wait structure to the list with interrupts disabled */

  flags             = enter_critical_section();
  notify->nt_flink  = g_icmpv6_waiters;
  g_icmpv6_waiters  = notify;
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: icmpv6_wait_cancel
 *
 * Description:
 *   Cancel any wait set after icmpv6_wait_setup is called but before
 *   icmpv6_wait()is called (icmpv6_wait() will automatically cancel the
 *   wait).
 *
 * Assumptions:
 *   This function may execute in the interrupt context when called from
 *   icmpv6_wait().
 *
 ****************************************************************************/

int icmpv6_wait_cancel(FAR struct icmpv6_notify_s *notify)
{
  FAR struct icmpv6_notify_s *curr;
  FAR struct icmpv6_notify_s *prev;
  irqstate_t flags;
  int ret = -ENOENT;

  /* Remove our wait structure from the list (we may no longer be at the
   * head of the list).
   */

  flags = enter_critical_section();
  for (prev = NULL, curr = g_icmpv6_waiters;
       curr && curr != notify;
       prev = curr, curr = curr->nt_flink)
    {
    }

  DEBUGASSERT(curr && curr == notify);
  if (curr)
    {
      if (prev)
        {
          prev->nt_flink = notify->nt_flink;
        }
      else
        {
          g_icmpv6_waiters = notify->nt_flink;
        }

      ret = OK;
    }

  leave_critical_section(flags);
  nxsem_destroy(&notify->nt_sem);
  return ret;
}

/****************************************************************************
 * Name: icmpv6_wait
 *
 * Description:
 *   Called each time that a Neighbor Solicitation is sent.  This function
 *   will sleep until either: (1) the matching Neighbor Advertisement is
 *   received, or (2) a timeout occurs.
 *
 * Assumptions:
 *   This function is called from icmpv6_neighbor() and must execute with
 *   the network locked.
 *
 ****************************************************************************/

int icmpv6_wait(FAR struct icmpv6_notify_s *notify, unsigned int timeout)
{
  int ret;

  /* And wait for the Neighbor Advertisement (or a timeout). */

  ret = net_timedwait(&notify->nt_sem, timeout);
  if (ret >= 0)
    {
      ret = notify->nt_result;
    }

  /* Remove our wait structure from the list (we may no longer be at the
   * head of the list).
   */

  icmpv6_wait_cancel(notify);
  return ret;
}

/****************************************************************************
 * Name: icmpv6_notify
 *
 * Description:
 *   Called each time that a Neighbor Advertisement is received in order to
 *   wake-up any threads that may be waiting for this particular Neighbor
 *   Advertisement.
 *
 * Assumptions:
 *   This function is called from the MAC device driver indirectly through
 *   icmpv6_icmpv6in() will execute with the network locked.
 *
 ****************************************************************************/

void icmpv6_notify(net_ipv6addr_t ipaddr)
{
  FAR struct icmpv6_notify_s *curr;

  /* Find an entry with the matching IP address in the list of waiters */

  for (curr = g_icmpv6_waiters; curr; curr = curr->nt_flink)
    {
      /* Does this entry match?  If the result is okay, then we have
       * already notified this waiter and it has not yet taken the
       * entry from the list.
       */

      if (curr->nt_result != OK && net_ipv6addr_cmp(curr->nt_ipaddr, ipaddr))
        {
          /* Yes.. Signal the waiting, returning success */

          curr->nt_result = OK;
          nxsem_post(&curr->nt_sem);
          break;
        }
    }
}

#endif /* CONFIG_NET_ICMPv6_NEIGHBOR */
