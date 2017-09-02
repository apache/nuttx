/****************************************************************************
 * net/igmp/igmp_timer.c
 *
 *   Copyright (C) 2010-2011, 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * The NuttX implementation of IGMP was inspired by the IGMP add-on for the
 * lwIP TCP/IP stack by Steve Reynolds:
 *
 *   Copyright (c) 2002 CITEL Technologies Ltd.
 *   All rights reserved.
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
 * 3. Neither the name of CITEL Technologies Ltd nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY CITEL TECHNOLOGIES AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL CITEL TECHNOLOGIES OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
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
#include <nuttx/net/netconfig.h>
#include <nuttx/net/net.h>
#include <nuttx/net/netstats.h>
#include <nuttx/net/igmp.h>

#include "devif/devif.h"
#include "igmp/igmp.h"
#include "utils/utils.h"

#ifdef CONFIG_NET_IGMP

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Debug ********************************************************************/

#undef IGMP_GTMRDEBUG /* Define to enable detailed IGMP group debug */

#ifndef CONFIG_NET_IGMP
#  undef IGMP_GTMRDEBUG
#endif

#ifdef CONFIG_CPP_HAVE_VARARGS
#  ifdef IGMP_GTMRDEBUG
#    define gtmrerr(format, ...)    nerr(format, ##__VA_ARGS__)
#    define gtmrinfo(format, ...)   ninfo(format, ##__VA_ARGS__)
#  else
#    define gtmrerr(x...)
#    define gtmrinfo(x...)
#  endif
#else
#  ifdef IGMP_GTMRDEBUG
#    define gtmrerr    nerr
#    define gtmrinfo   ninfo
#  else
#    define gtmrerr    (void)
#    define gtmrinfo   (void)
#  endif
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  igmp_timeout
 *
 * Description:
 *   Timeout watchdog handler.
 *
 * Assumptions:
 *   This function is called from the wdog timer handler which runs in the
 *   context of the timer interrupt handler.
 *
 ****************************************************************************/

static void igmp_timeout(int argc, uint32_t arg, ...)
{
  FAR struct igmp_group_s *group;

  /* If the state is DELAYING_MEMBER then we send a report for this group */

  ninfo("Timeout!\n");
  group = (FAR struct igmp_group_s *)arg;
  DEBUGASSERT(argc == 1 && group);

  /* If the group exists and is no an IDLE MEMBER, then it must be a DELAYING
   * member.  Race conditions are avoided because (1) the timer is not started
   * until after the first IGMPv2_MEMBERSHIP_REPORT during the join, and (2)
   * the timer is cancelled before sending the IGMP_LEAVE_GROUP during a leave.
   */

  if (!IS_IDLEMEMBER(group->flags))
    {
      /* Schedule (and forget) the Membership Report.  NOTE:
       * Since we are executing from a timer interrupt, we cannot wait
       * for the message to be sent.
       */

      IGMP_STATINCR(g_netstats.igmp.report_sched);
      igmp_schedmsg(group, IGMPv2_MEMBERSHIP_REPORT);

      /* Also note:  The Membership Report is sent at most two times becasue
       * the timer is not reset here.  Hmm.. does this mean that the group
       * is stranded if both reports were lost?  This is consistent with the
       * RFC that states: "To cover the possibility of the initial Membership
       * Report being lost or damaged, it is recommended that it be repeated
       * once or twice after short delays [Unsolicited Report Interval]..."
       */
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  igmp_startticks and igmp_starttimer
 *
 * Description:
 *   Start the IGMP timer with differing time units (ticks or deciseconds).
 *
 * Assumptions:
 *   This function may be called from most any context.
 *
 ****************************************************************************/

void igmp_startticks(FAR struct igmp_group_s *group, unsigned int ticks)
{
  int ret;

  /* Start the timer */

  gtmrinfo("ticks: %d\n", ticks);

  ret = wd_start(group->wdog, ticks, igmp_timeout, 1, (uint32_t)group);

  DEBUGASSERT(ret == OK);
  UNUSED(ret);
}

void igmp_starttimer(FAR struct igmp_group_s *group, uint8_t decisecs)
{
  /* Convert the decisec value to system clock ticks and start the timer.
   * Important!! this should be a random timer from 0 to decisecs
   */

  gtmrinfo("decisecs: %d\n", decisecs);
  igmp_startticks(group, net_dsec2tick(decisecs));
}

/****************************************************************************
 * Name:  igmp_cmptimer
 *
 * Description:
 *   Compare the timer remaining on the watching timer to the deci-second
 *   value. If maxticks > ticks-remaining, then (1) cancel the timer (to
 *   avoid race conditions) and return true.
 *
 * Assumptions:
 *   This function may be called from most any context.  If true is returned
 *   then the caller must call igmp_startticks() to restart the timer
 *
 ****************************************************************************/

bool igmp_cmptimer(FAR struct igmp_group_s *group, int maxticks)
{
  irqstate_t flags;
  int remaining;

  /* Disable interrupts so that there is no race condition with the actual
   * timer expiration.
   */

  flags = enter_critical_section();

  /* Get the timer remaining on the watchdog.  A time of <= zero means that
   * the watchdog was never started.
   */

  remaining = wd_gettime(group->wdog);

  /* A remaining time of zero means that the watchdog was never started
   * or has already expired.  That case should be covered in the following
   * test as well.
   */

  gtmrinfo("maxticks: %d remaining: %d\n", maxticks, remaining);
  if (maxticks > remaining)
    {
      /* Cancel the watchdog timer and return true */

      wd_cancel(group->wdog);
      leave_critical_section(flags);
      return true;
    }

  leave_critical_section(flags);
  return false;
}

#endif /* CONFIG_NET_IGMP */
