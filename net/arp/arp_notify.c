/****************************************************************************
 * net/arp/arp_notify.c
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

#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <netinet/in.h>

#include <nuttx/irq.h>
#include <nuttx/net/net.h>

#include "arp/arp.h"

#ifdef CONFIG_NET_ARP_SEND

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* List of tasks waiting for ARP events */

static FAR struct arp_notify_s *g_arp_waiters;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arp_wait_setup
 *
 * Description:
 *   Called BEFORE an ARP request is sent.  This function sets up the ARP
 *   response timeout before the ARP request is sent so that there is
 *   no race condition when arp_wait() is called.
 *
 * Assumptions:
 *   This function is called from ARP send and executes in the normal
 *   tasking environment.
 *
 ****************************************************************************/

void arp_wait_setup(in_addr_t ipaddr, FAR struct arp_notify_s *notify)
{
  irqstate_t flags;

  /* Initialize the wait structure */

  notify->nt_ipaddr = ipaddr;
  notify->nt_result = -ETIMEDOUT;

  nxsem_init(&notify->nt_sem, 0, 0);

  /* Add the wait structure to the list with interrupts disabled */

  flags             = enter_critical_section();
  notify->nt_flink  = g_arp_waiters;
  g_arp_waiters     = notify;
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: arp_wait_cancel
 *
 * Description:
 *   Cancel any wait set after arp_wait_setup is called but before arp_wait()
 *   is called (arp_wait() will automatically cancel the wait).
 *
 * Assumptions:
 *   This function may execute with interrupts disabled when called from
 *   arp_wait().
 *
 ****************************************************************************/

int arp_wait_cancel(FAR struct arp_notify_s *notify)
{
  FAR struct arp_notify_s *curr;
  FAR struct arp_notify_s *prev;
  irqstate_t flags;
  int ret = -ENOENT;

  /* Remove our wait structure from the list (we may no longer be at the
   * head of the list).
   */

  flags = enter_critical_section();
  for (prev = NULL, curr = g_arp_waiters;
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
          g_arp_waiters = notify->nt_flink;
        }

      ret = OK;
    }

  leave_critical_section(flags);
  nxsem_destroy(&notify->nt_sem);
  return ret;
}

/****************************************************************************
 * Name: arp_wait
 *
 * Description:
 *   Called each time that a ARP request is sent.  This function will sleep
 *   until either: (1) the matching ARP response is received, or (2) a
 *   timeout occurs.
 *
 * Assumptions:
 *   This function is called from ARP send must execute with the network
 *   locked.
 *
 ****************************************************************************/

int arp_wait(FAR struct arp_notify_s *notify, unsigned int timeout)
{
  int ret;

  /* And wait for the ARP response (or a timeout). */

  net_sem_timedwait_uninterruptible(&notify->nt_sem, timeout);

  /* Then get the real result of the transfer */

  ret = notify->nt_result;

  /* Remove our wait structure from the list (we may no longer be at the
   * head of the list).
   */

  arp_wait_cancel(notify);
  return ret;
}

/****************************************************************************
 * Name: arp_notify
 *
 * Description:
 *   Called each time that a ARP response is received in order to wake-up
 *   any threads that may be waiting for this particular ARP response.
 *
 * Assumptions:
 *   This function is called from the MAC device driver indirectly through
 *   arp_input() will execute with the network locked.
 *
 ****************************************************************************/

void arp_notify(in_addr_t ipaddr)
{
  FAR struct arp_notify_s *curr;
  irqstate_t flags;

  flags = enter_critical_section();

  /* Find an entry with the matching IP address in the list of waiters */

  for (curr = g_arp_waiters; curr; curr = curr->nt_flink)
    {
      /* Does this entry match?  If the result is okay, then we have
       * already notified this waiter and it has not yet taken the
       * entry from the list.
       */

      if (curr->nt_result != OK && curr->nt_ipaddr == ipaddr)
        {
          /* Yes.. Signal the waiting, returning success */

          curr->nt_result = OK;
          nxsem_post(&curr->nt_sem);
          break;
        }
    }

  leave_critical_section(flags);
}

#endif /* CONFIG_NET_ARP_SEND */
