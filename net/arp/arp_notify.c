/****************************************************************************
 * net/arp/arp_notify.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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

#include <time.h>
#include <semaphore.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <netinet/in.h>

#include <nuttx/net/net.h>
#include <arch/irq.h>

#include "arp/arp.h"

#ifdef CONFIG_NET_ARP_SEND

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* List of tasks waiting for ARP events */

static FAR struct arp_notify_s *g_arp_waiters;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: arp_wait_setup
 *
 * Description:
 *   Called BEFORE an ARP request is sent.  This function sets up the ARP
 *   response timeout before the the ARP request is sent so that there is
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
  (void)sem_init(&notify->nt_sem, 0, 0);

  /* Add the wait structure to the list with interrupts disabled */

  flags             = irqsave();
  notify->nt_flink  = g_arp_waiters;
  g_arp_waiters     = notify;
  irqrestore(flags);
}

/****************************************************************************
 * Function: arp_wait_cancel
 *
 * Description:
 *   Cancel any wait set after arp_wait_setup is called but before arp_wait()
 *   is called (arp_wait() will automatically cancel the wait).
 *
 * Assumptions:
 *   This function may execute in the interrupt context when called from
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

  flags = irqsave();
  for (prev = NULL, curr = g_arp_waiters;
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
          g_arp_waiters = notify->nt_flink;
        }

      ret = OK;
    }

  irqrestore(flags);
  (void)sem_destroy(&notify->nt_sem);
  return ret;
}

/****************************************************************************
 * Function: arp_wait
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

int arp_wait(FAR struct arp_notify_s *notify, FAR struct timespec *timeout)
{
  struct timespec abstime;
  irqstate_t flags;
  int errcode;
  int ret;

  /* And wait for the ARP response (or a timeout).  Interrupts will be re-
   * enabled while we wait.
   */

  flags = irqsave();
  DEBUGVERIFY(clock_gettime(CLOCK_REALTIME, &abstime));

  abstime.tv_sec  += timeout->tv_sec;
  abstime.tv_nsec += timeout->tv_nsec;
  if (abstime.tv_nsec >= 1000000000)
    {
      abstime.tv_sec++;
      abstime.tv_nsec -= 1000000000;
    }

  /* Wait to get either the correct response or a timeout. */

  do
    {
      /* The only errors that we expect would be if the abstime timeout
       * expires or if the wait were interrupted by a signal.
       */

      ret     = net_timedwait(&notify->nt_sem, &abstime);
      errcode = ((ret < 0) ? errno : 0);
    }
  while (ret < 0 && errcode == EINTR);

  /* Then get the real result of the transfer */

  ret = notify->nt_result;

  /* Remove our wait structure from the list (we may no longer be at the
   * head of the list).
   */

  (void)arp_wait_cancel(notify);

  /* Re-enable interrupts and return the result of the wait */

  irqrestore(flags);
  return ret;
}

/****************************************************************************
 * Function: arp_notify
 *
 * Description:
 *   Called each time that a ARP response is received in order to wake-up
 *   any threads that may be waiting for this particular ARP response.
 *
 * Assumptions:
 *   This function is called from the MAC device driver indirectly through
 *   arp_arpin() will execute with the network locked.
 *
 ****************************************************************************/

void arp_notify(in_addr_t ipaddr)
{
  FAR struct arp_notify_s *curr;

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
          sem_post(&curr->nt_sem);
          break;
        }
    }
}

#endif /* CONFIG_NET_ARP_SEND */
