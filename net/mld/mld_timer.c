/****************************************************************************
 * net/mld/mld_timer.c
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
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of CITEL Technologies Ltd nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY CITEL TECHNOLOGIES AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL CITEL TECHNOLOGIES OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <assert.h>
#include <debug.h>

#include <nuttx/wdog.h>
#include <nuttx/irq.h>
#include <nuttx/wqueue.h>
#include <nuttx/net/netconfig.h>
#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/netstats.h>
#include <nuttx/net/mld.h>

#include "devif/devif.h"
#include "netdev/netdev.h"
#include "mld/mld.h"
#include "utils/utils.h"

#ifdef CONFIG_NET_MLD

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MLD_FREE_WORK 4

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This is a small pool of work structures that are used for rare, general
 * timer and MLDv1 compatibility timers.
 *
 * REVISIT:  A better place would be in the device structure.  But then we
 * would either have to pass a device structure in the timeout (which could
 * become stale) or look up the device using an interface index.  But we
 * cannot access the network device list from the timer interrupt handler.
 */

static struct work_s g_mld_work[MLD_FREE_WORK];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  mld_find_work
 *
 * Description:
 *   Fin an available work queue structure.
 *
 * Assumptions:
 *   Called only from the timer interrupt handler with interrupts disabled.
 *
 ****************************************************************************/

static FAR struct work_s *mld_find_work(void)
{
  int i;

  for (i = 0; i < MLD_FREE_WORK; i++)
    {
      if (work_available(&g_mld_work[i]))
        {
          return &g_mld_work[i];
        }
    }

  return NULL;
}

/****************************************************************************
 * Name:  mld_gendog_work
 *
 * Description:
 *   General query watchdog timeout work.
 *
 * Assumptions:
 *   This function is called from a work queue thread.
 *
 ****************************************************************************/

static void mld_gendog_work(FAR void *arg)
{
  FAR struct net_driver_s *dev;
  int ifindex;

  /* Recover the interface index and find the device associated with the
   * index.
   */

  ifindex = (int)arg;
  DEBUGASSERT(ifindex > 0);

  net_lock();
  dev = netdev_findbyindex(ifindex);
  if (dev == NULL)
    {
      /* This could be a normal consequence if the device is unregistered
       * while the timer is pending.
       */

      fwarn("WARNING: No device associated with ifindex=%d\n", ifindex);
      net_unlock();
      return;
    }

  /* Check for an Other Querier Present Timeout.  This timer is set in non-
   * Querier mode to detect the case where we have lost the Querier.
   */

  if (!IS_MLD_QUERIER(dev->d_mld.flags))
    {
      /* We are not the Querier.  This is an Other Querier Present Timeout.
       * If this timeout expires, it means that there are no Queriers for
       * the group.  Let's revert to Querier mode.
       */

      SET_MLD_QUERIER(dev->d_mld.flags);
    }

  /* Check if this we are the Querier */

  if (IS_MLD_QUERIER(dev->d_mld.flags))
    {
      /* Schedule (and forget) the general query. */

      MLD_STATINCR(g_netstats.mld.query_sched);
      SET_MLD_GENPEND(dev->d_mld.flags);

      /* Notify the device that we have a packet to send */

      netdev_txnotify_dev(dev);

      /* Restart the Querier timer */

      mld_start_gentimer(dev, MSEC2TICK(MLD_QUERY_MSEC));
    }
  else
    {
      /* Not the Querier... Restart the Other Querier Present Timeout */

      mld_start_gentimer(dev, MSEC2TICK(MLD_OQUERY_MSEC));
    }

  net_unlock();
}

/****************************************************************************
 * Name:  mld_gendog_timout
 *
 * Description:
 *   General query watchdog handler.
 *
 * Assumptions:
 *   This function is called from the context of the timer interrupt handler.
 *
 ****************************************************************************/

static void mld_gendog_timout(wdparm_t arg)
{
  FAR struct work_s *work;
  int ret;

  mldinfo("Timeout!\n");
  DEBUGASSERT(arg != 0);

  /* Find an available work structure.  This is awkward. */

  work = mld_find_work();
  DEBUGASSERT(work != NULL);

  if (work == NULL)
    {
      mlderr("ERROR: No free work structures.\n");
    }
  else
    {
      /* Perform the general query-related operations on (preferably) the
       * low priority work queue.
       */

      ret = work_queue(LPWORK, work, mld_gendog_work, (FAR void *)arg, 0);
      if (ret < 0)
        {
          mlderr("ERROR: Failed to queue general query work: %d\n", ret);
        }
    }
}

/****************************************************************************
 * Name:  mld_v1dog_work
 *
 * Description:
 *   Timeout watchdog work
 *
 * Assumptions:
 *   This function is called from a work queue thread.
 *
 ****************************************************************************/

static void mld_v1dog_work(FAR void *arg)
{
  FAR struct net_driver_s *dev;
  int ifindex;

  /* Recover the interface index and find the device associated with the
   * index.
   */

  ifindex = (int)arg;
  DEBUGASSERT(ifindex > 0);

  net_lock();
  dev = netdev_findbyindex(ifindex);
  if (dev == NULL)
    {
      /* This could be a normal consequence if the device is unregistered
       * while the timer is pending.
       */

      fwarn("WARNING: No device associated with ifindex=%d\n", ifindex);
    }
  else
    {
      /* Exit MLDv1 compatibility mode. */

      CLR_MLD_V1COMPAT(dev->d_mld.flags);

      /* REVIST:  Whenever a host changes its compatibility mode, it cancels
       * all of its pending responses and retransmission timers.
       */
    }

  net_unlock();
}

/****************************************************************************
 * Name:  mld_v1dog_timout
 *
 * Description:
 *   Timeout watchdog handler.
 *
 * Assumptions:
 *   This function is called from the context of the timer interrupt handler.
 *
 ****************************************************************************/

static void mld_v1dog_timout(wdparm_t arg)
{
  FAR struct work_s *work;
  int ret;

  mldinfo("Timeout!\n");
  DEBUGASSERT(arg != 0);

  /* Find an available work structure.  This is awkward. */

  work = mld_find_work();
  DEBUGASSERT(work != NULL);

  if (work == NULL)
    {
      mlderr("ERROR: No free work structures.\n");
    }
  else
    {
      /* Perform the general query-related operations on (preferably) the
       * low priority work queue.
       */

      ret = work_queue(LPWORK, work, mld_v1dog_work, (FAR void *)arg, 0);
      if (ret < 0)
        {
          mlderr("ERROR: Failed to queue MLDv1 timeout work: %d\n", ret);
        }
    }
}

/****************************************************************************
 * Name:  mld_polldog_work
 *
 * Description:
 *   Timeout watchdog work
 *
 * Assumptions:
 *   This function is called from a work queue thread.
 *
 ****************************************************************************/

static void mld_polldog_work(FAR void *arg)
{
  FAR struct mld_group_s *group;
  FAR struct net_driver_s *dev;
  int ret;

  /* Recover the reference to the group */

  group = (FAR struct mld_group_s *)arg;
  DEBUGASSERT(group != NULL);

  /* Check if this a new join to the multicast group. */

  net_lock();
  if (IS_MLD_STARTUP(group->flags))
    {
      MLD_STATINCR(g_netstats.mld.report_sched);

      /* Get a reference to the device serving the sub-net */

      dev = netdev_findbyindex(group->ifindex);
      if (dev == NULL)
        {
          /* This could be a normal consequence if the device is
           * unregistered while the timer is pending.
           */

          fwarn("WARNING: No device associated with ifindex=%d\n",
                group->ifindex);
          return;
        }

      /* Schedule (and forget) the Report. */

      ret = mld_schedmsg(group, mld_report_msgtype(dev));
      if (ret < 0)
        {
          mlderr("ERROR: Failed to schedule message: %d\n", ret);
        }

      /* Send the report until the unsolicited report count goes to zero
       * then terminate the start-up sequence.
       */

      if (group->count > 0)
        {
          /* Decrement the count and restart the timer */

          group->count--;
          mld_start_polltimer(group, MSEC2TICK(MLD_UNSOLREPORT_MSEC));
        }
      else
        {
          /* Terminate the start-up sequence */

          CLR_MLD_STARTUP(group->flags);
        }
    }

  net_unlock();
}

/****************************************************************************
 * Name:  mld_polldog_timout
 *
 * Description:
 *   Timeout watchdog handler.
 *
 * Assumptions:
 *   This function is called from the context of the timer interrupt handler.
 *
 ****************************************************************************/

static void mld_polldog_timout(wdparm_t arg)
{
  FAR struct mld_group_s *group;
  int ret;

  mldinfo("Timeout!\n");

  /* Recover the reference to the group */

  group = (FAR struct mld_group_s *)arg;
  DEBUGASSERT(group != NULL);

  /* Perform the timeout-related operations on (preferably) the low priority
   * work queue.
   */

  ret = work_queue(LPWORK, &group->work, mld_polldog_work, group, 0);
  if (ret < 0)
    {
      mlderr("ERROR: Failed to queue timeout work: %d\n", ret);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  mld_start_gentimer
 *
 * Description:
 *   Start/Re-start the general query timer.
 *
 ****************************************************************************/

void mld_start_gentimer(FAR struct net_driver_s *dev, clock_t ticks)
{
  int ret;

  /* Start the timer */

  mldinfo("ticks: %lu\n", (unsigned long)ticks);

  ret = wd_start(&dev->d_mld.gendog, ticks,
                 mld_gendog_timout, dev->d_ifindex);

  DEBUGASSERT(ret == OK);
  UNUSED(ret);
}

/****************************************************************************
 * Name:  mld_start_v1timer
 *
 * Description:
 *   Start the MLDv1 compatibility timer.
 *
 *   REVISIT:  This should be a single global timer, not a per-group timer.
 *
 ****************************************************************************/

void mld_start_v1timer(FAR struct net_driver_s *dev, clock_t ticks)
{
  int ret;

  /* Start the timer */

  mldinfo("ticks: %lu\n", (unsigned long)ticks);

  ret = wd_start(&dev->d_mld.v1dog, ticks,
                 mld_v1dog_timout, dev->d_ifindex);

  DEBUGASSERT(ret == OK);
  UNUSED(ret);
}

/****************************************************************************
 * Name:  mld_start_polltimer
 *
 * Description:
 *   Start the MLD poll timer.
 *
 ****************************************************************************/

void mld_start_polltimer(FAR struct mld_group_s *group, clock_t ticks)
{
  int ret;

  /* Start the timer */

  mldinfo("ticks: %lu\n", (unsigned long)ticks);

  ret = wd_start(&group->polldog, ticks,
                 mld_polldog_timout, (wdparm_t)group);

  DEBUGASSERT(ret == OK);
  UNUSED(ret);
}

#endif /* CONFIG_NET_MLD */
