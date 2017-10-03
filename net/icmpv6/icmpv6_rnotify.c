/****************************************************************************
 * net/icmpv6/icmpv6_rnotify.c
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
#include <nuttx/net/netdev.h>

#include "netdev/netdev.h"
#include "utils/utils.h"
#include "icmpv6/icmpv6.h"

#ifdef CONFIG_NET_ICMPv6_AUTOCONF

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

static struct icmpv6_rnotify_s *g_icmpv6_rwaiters;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: icmpv6_setaddresses
 *
 * Description:
 *   We successfully obtained the Router Advertisement.  See the new IPv6
 *   addresses in the driver structure.
 *
 ****************************************************************************/

static void icmpv6_setaddresses(FAR struct net_driver_s *dev,
                                const net_ipv6addr_t draddr,
                                const net_ipv6addr_t prefix,
                                unsigned int preflen)
{
  unsigned int i;

  /* Make sure that the network is down before changing any addresses */

  netdev_ifdown(dev);

  /* Create an address mask from the prefix */

  if (preflen > 128)
    {
      preflen = 128;
    }

  net_ipv6_pref2mask(preflen, dev->d_ipv6netmask);

  ninfo("preflen=%d netmask=%04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
        preflen, dev->d_ipv6netmask[0], dev->d_ipv6netmask[1],
        dev->d_ipv6netmask[2], dev->d_ipv6netmask[3], dev->d_ipv6netmask[4],
        dev->d_ipv6netmask[5], dev->d_ipv6netmask[6], dev->d_ipv6netmask[7]);

  /* Copy prefix to the current IPv6 address, applying the mask */

  for (i = 0; i < 7; i++)
    {
      dev->d_ipv6addr[i] = (dev->d_ipv6addr[i] & ~dev->d_ipv6netmask[i]) |
                           (prefix[i] & dev->d_ipv6netmask[i]);
    }

  ninfo("prefix=%04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
        prefix[0], prefix[1], prefix[2], prefix[3],
        prefix[4], prefix[6], prefix[6], prefix[7]);
  ninfo("IP address=%04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
        dev->d_ipv6addr[0], dev->d_ipv6addr[1], dev->d_ipv6addr[2],
        dev->d_ipv6addr[3], dev->d_ipv6addr[4], dev->d_ipv6addr[6],
        dev->d_ipv6addr[6], dev->d_ipv6addr[7]);

  /* Finally, copy the router address */

  net_ipv6addr_copy(dev->d_ipv6draddr, draddr);

  ninfo("DR address=%04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
        dev->d_ipv6draddr[0], dev->d_ipv6draddr[1], dev->d_ipv6draddr[2],
        dev->d_ipv6draddr[3], dev->d_ipv6draddr[4], dev->d_ipv6draddr[6],
        dev->d_ipv6draddr[6], dev->d_ipv6draddr[7]);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: icmpv6_rwait_setup
 *
 * Description:
 *   Called BEFORE an Router Solicitation is sent.  This function sets up
 *   the Router Advertisement timeout before the Router Solicitation
 *   is sent so that there is no race condition when icmpv6_rwait() is
 *   called.
 *
 * Assumptions:
 *   This function is called from icmpv6_autoconfig() and executes in the
 *   normal tasking environment.
 *
 ****************************************************************************/

void icmpv6_rwait_setup(FAR struct net_driver_s *dev,
                        FAR struct icmpv6_rnotify_s *notify)
{
  irqstate_t flags;

  /* Initialize the wait structure */

  memcpy(notify->rn_ifname, dev->d_ifname, IFNAMSIZ);
  notify->rn_result = -ETIMEDOUT;

  /* This semaphore is used for signaling and, hence, should not have
   * priority inheritance enabled.
   */

  (void)nxsem_init(&notify->rn_sem, 0, 0);
  nxsem_setprotocol(&notify->rn_sem, SEM_PRIO_NONE);

  /* Add the wait structure to the list with interrupts disabled */

  flags             = enter_critical_section();
  notify->rn_flink  = g_icmpv6_rwaiters;
  g_icmpv6_rwaiters  = notify;
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: icmpv6_rwait_cancel
 *
 * Description:
 *   Cancel any wait set after icmpv6_rwait_setup() is called but before
 *   icmpv6_rwait()is called (icmpv6_rwait() will automatically cancel the
 *   wait).
 *
 * Assumptions:
 *   This function may execute in the interrupt context when called from
 *   icmpv6_rwait().
 *
 ****************************************************************************/

int icmpv6_rwait_cancel(FAR struct icmpv6_rnotify_s *notify)
{
  FAR struct icmpv6_rnotify_s *curr;
  FAR struct icmpv6_rnotify_s *prev;
  irqstate_t flags;
  int ret = -ENOENT;

  ninfo("Cancelling...\n");

  /* Remove our wait structure from the list (we may no longer be at the
   * head of the list).
   */

  flags = enter_critical_section();
  for (prev = NULL, curr = g_icmpv6_rwaiters;
       curr && curr != notify;
       prev = curr, curr = curr->rn_flink);

  DEBUGASSERT(curr && curr == notify);
  if (curr)
    {
      if (prev)
        {
          prev->rn_flink = notify->rn_flink;
        }
      else
        {
          g_icmpv6_rwaiters = notify->rn_flink;
        }

      ret = OK;
    }

  leave_critical_section(flags);
  (void)nxsem_destroy(&notify->rn_sem);
  return ret;
}

/****************************************************************************
 * Name: icmpv6_rwait
 *
 * Description:
 *   Called each time that a Router Solicitation is sent.  This function
 *   will sleep until either: (1) the matching Router Advertisement is
 *   received, or (2) a timeout occurs.
 *
 * Assumptions:
 *   This function is called from icmpv6_autoconfig() and must execute with
 *   the network locked.
 *
 ****************************************************************************/

int icmpv6_rwait(FAR struct icmpv6_rnotify_s *notify,
                 FAR struct timespec *timeout)
{
  struct timespec abstime;
  irqstate_t flags;
  int ret;

  ninfo("Waiting...\n");

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

  (void)net_timedwait(&notify->rn_sem, &abstime);
  ret = notify->rn_result;

  /* Remove our wait structure from the list (we may no longer be at the
   * head of the list).
   */

  (void)icmpv6_rwait_cancel(notify);

  /* Re-enable interrupts and return the result of the wait */

  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: icmpv6_rnotify
 *
 * Description:
 *   Called each time that a Router Advertisement is received in order to
 *   wake-up any threads that may be waiting for this particular Router
 *   Advertisement.
 *
 *   NOTE:  On success the network has the new address applied and is in
 *   the down state.
 *
 * Assumptions:
 *   This function is called from the MAC device driver indirectly through
 *   icmpv6_icmpv6in() will execute with the network locked.
 *
 ****************************************************************************/

void icmpv6_rnotify(FAR struct net_driver_s *dev, const net_ipv6addr_t draddr,
                    const net_ipv6addr_t prefix, unsigned int preflen)
{
  FAR struct icmpv6_rnotify_s *curr;

  ninfo("Notified\n");

  /* Find an entry with the matching device name in the list of waiters */

  for (curr = g_icmpv6_rwaiters; curr; curr = curr->rn_flink)
    {
      /* Does this entry match?  If the result is okay, then we have
       * already notified this waiter and it has not yet taken the
       * entry from the list.
       */

      if (curr->rn_result != OK &&
          strncmp(curr->rn_ifname, dev->d_ifname, IFNAMSIZ) == 0)
        {
          /* Yes.. Set the new network addresses. */

          icmpv6_setaddresses(dev, draddr, prefix, preflen);

          /* And signal the waiting, returning success */

          curr->rn_result = OK;
          nxsem_post(&curr->rn_sem);
          break;
        }
    }
}

#endif /* CONFIG_NET_ICMPv6_AUTOCONF */
