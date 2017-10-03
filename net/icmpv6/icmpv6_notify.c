/****************************************************************************
 * net/icmpv6/icmpv6_notify.c
 *
 *   Copyright (C) 2015-2016 Gregory Nutt. All rights reserved.
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

#include <string.h>
#include <time.h>
#include <semaphore.h>
#include <errno.h>
#include <debug.h>

#include <netinet/in.h>

#include <nuttx/irq.h>
#include <nuttx/semaphore.h>
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
 *   This function is called from icmpv6_neighbor() and executes in the normal
 *   tasking environment.
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

  (void)nxsem_init(&notify->nt_sem, 0, 0);
  nxsem_setprotocol(&notify->nt_sem, SEM_PRIO_NONE);

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
       prev = curr, curr = curr->nt_flink);

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
  (void)nxsem_destroy(&notify->nt_sem);
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

int icmpv6_wait(FAR struct icmpv6_notify_s *notify,
                FAR struct timespec *timeout)
{
  struct timespec abstime;
  irqstate_t flags;
  int ret;

  /* And wait for the Neighbor Advertisement (or a timeout).  Interrupts will
   * be re-enabled while we wait.
   */

  flags = enter_critical_section();
  DEBUGVERIFY(clock_gettime(CLOCK_REALTIME, &abstime));

  abstime.tv_sec  += timeout->tv_sec;
  abstime.tv_nsec += timeout->tv_nsec;
  if (abstime.tv_nsec >= 1000000000)
    {
      abstime.tv_sec++;
      abstime.tv_nsec -= 1000000000;
    }

  /* REVISIT:  If net_timedwait() is awakened with  signal, we will return
   * the wrong error code.
   */

  (void)net_timedwait(&notify->nt_sem, &abstime);
  ret = notify->nt_result;

  /* Remove our wait structure from the list (we may no longer be at the
   * head of the list).
   */

  (void)icmpv6_wait_cancel(notify);

  /* Re-enable interrupts and return the result of the wait */

  leave_critical_section(flags);
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
