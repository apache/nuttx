/****************************************************************************
 * net/uip/uip_igmpleave.c
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#include <wdog.h>
#include <assert.h>
#include <debug.h>

#include <net/uip/uipopt.h>
#include <net/uip/uip.h>
#include <net/uip/uip-igmp.h>

#include "uip_internal.h"

#ifdef CONFIG_NET_IGMP

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  uip_igmptimeout
 *
 * Description:
 *   Timeout watchdog handler.
 *
 * Assumptions:
 *   This function is called from the wdog timer handler which runs in the
 *   context of the timer interrupt handler.
 *
 ****************************************************************************/

static void uip_igmptimeout(int argc, uint32_t arg, ...)
{
  FAR struct igmp_group_s *group;

  /* If the state is DELAYING_MEMBER then we send a report for this group */

  nvdbg("Timeout!\n");
  group = (FAR struct igmp_group_s *)arg;
  DEBUGASSERT(argc == 1 && group);

  /* If the group exists and is no an IDLE MEMBER, then it must be a DELAYING
   * member.  Race conditions are avoided because (1) the timer is not started
   * until after the first IGMPv2_MEMBERSHIP_REPORT during the join, and (2)
   * the timer is canceled before sending the IGMP_LEAVE_GROUP during a leave.
   */

  if (!IS_IDLEMEMBER(group->flags))
    {
      /* Schedule (and forget) the Membership Report.  NOTE:
       * Since we are executing from a timer interrupt, we cannot wait
       * for the message to be sent.
       */

      IGMP_STATINCR(uip_stat.igmp.report_sched);
      uip_igmpschedmsg(group, IGMPv2_MEMBERSHIP_REPORT);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  uip_igmpstarttimer
 *
 * Description:
 *   Start the IGMP timer.
 *
 * Assumptions:
 *   This function may be called from most any context.
 *
 ****************************************************************************/

int uip_decisec2tick(int decisecs)
{
  /* Convert the deci-second comparison value to clock ticks.  The CLK_TCK
   * value is the number of clock ticks per second; decisecs argument is the
   * maximum delay in 100's of milliseconds.  CLK_TCK/10 is then the number
   * of clock ticks in 100 milliseconds.
   */

  return CLK_TCK * decisecs / 10;
	
}
/****************************************************************************
 * Name:  uip_igmpstartticks and uip_igmpstarttimer
 *
 * Description:
 *   Start the IGMP timer with differing time units (ticks or deciseconds).
 *
 * Assumptions:
 *   This function may be called from most any context.
 *
 ****************************************************************************/

void uip_igmpstartticks(FAR struct igmp_group_s *group, int ticks)
{
  /* Start the timer */

  wd_start(group->wdog, ticks, uip_igmptimeout, 1, (uint32_t)group);
}

void uip_igmpstarttimer(FAR struct igmp_group_s *group, uint8_t decisecs)
{
  /* Convert the decisec value to system clock ticks and start the timer.
   * Important!! this should be a random timer from 0 to decisecs
   */
  
  uip_igmpstartticks(group, uip_decisec2tick(decisecs));
}

/****************************************************************************
 * Name:  uip_igmpcmptimer
 *
 * Description:
 *   Compare the timer remaining on the watching timer to the deci-second
 *   value. If maxticks > ticks-remaining, then (1) cancel the timer (to
 *   avoid race conditions) and return true.
 *
 * Assumptions:
 *   This function may be called from most any context.  If true is retuend
 *   then the caller must call uip_igmpstartticks() to restart the timer
 *
 ****************************************************************************/

bool uip_igmpcmptimer(FAR struct igmp_group_s *group, int maxticks)
{
  irqstate_t flags;
  int remaining;

  /* Disable interrupts so that there is no race condition with the actual
   * timer expiration.
   */

  flags = irqsave();

  /* Get the timer remaining on the watchdog.  A time of <= zero means that
   * the watchdog was never started.
   */

  remaining = wd_gettime(group->wdog);

  /* A remaining time of zero means that the watchdog was never started
   * or has already expired.  That case should be covered in the following
   * test as well.
   */

  if (maxticks > remaining)
    {
      /* Cancel the watchdog timer and return true */

      wd_cancel(group->wdog);
      irqrestore(flags);
      return true;
    }

  irqrestore(flags);
  return false;
}

#endif /* CONFIG_NET_IGMP */
