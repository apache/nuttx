/****************************************************************************
 * net/igmp/igmp_poll.c
 *
 *   Copyright (C) 2010, 2018 Gregory Nutt. All rights reserved.
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

#include <assert.h>
#include <debug.h>

#include <nuttx/semaphore.h>
#include <nuttx/net/netconfig.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/netstats.h>
#include <nuttx/net/ip.h>

#include "devif/devif.h"
#include "igmp/igmp.h"

#ifdef CONFIG_NET_IGMP

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  igmp_sched_send
 *
 * Description:
 *   Construct and send the IGMP message.
 *
 * Returned Value:
 *   Returns a non-zero value if an IGMP message is sent.
 *
 * Assumptions:
 *   This function ust be called with the network locked.
 *
 ****************************************************************************/

static inline void igmp_sched_send(FAR struct net_driver_s *dev,
                                   FAR struct igmp_group_s *group)
{
  in_addr_t *dest;

  /* Check what kind of message we need to send.  There are only two
   * possibilities:
   */

  if (group->msgid == IGMPv2_MEMBERSHIP_REPORT)
    {
      dest = &group->grpaddr;
      ninfo("Send IGMPv2_MEMBERSHIP_REPORT, dest=%08x flags=%02x\n",
             *dest, group->flags);
      IGMP_STATINCR(g_netstats.igmp.report_sched);
      SET_LASTREPORT(group->flags); /* Remember we were the last to report */
    }
  else
    {
      DEBUGASSERT(group->msgid == IGMP_LEAVE_GROUP);
      dest = &g_ipv4_allrouters;
      ninfo("Send IGMP_LEAVE_GROUP, dest=%08x flags=%02x\n",
             *dest, group->flags);
      IGMP_STATINCR(g_netstats.igmp.leave_sched);
    }

  /* Send the message */

  igmp_send(dev, group, dest);

  /* Indicate that the message has been sent */

  CLR_SCHEDMSG(group->flags);
  group->msgid = 0;

  /* If there is a thread waiting fore the message to be sent, wake it up */

  if (IS_WAITMSG(group->flags))
    {
      ninfo("Awakening waiter\n");
      nxsem_post(&group->sem);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  igmp_poll
 *
 * Description:
 *   Poll the groups associated with the device to see if any IGMP messages
 *   are pending transfer.
 *
 * Returned Value:
 *   Returns a non-zero value if a IGP message is sent.
 *
 * Assumptions:
 *   This function must be called with the network locked.
 *
 ****************************************************************************/

void igmp_poll(FAR struct net_driver_s *dev)
{
  FAR struct igmp_group_s *group;

  ninfo("Entry\n");

  /* Setup the poll operation */

  dev->d_appdata = &dev->d_buf[NET_LL_HDRLEN(dev) + IPIGMP_HDRLEN];
  dev->d_len     = 0;
  dev->d_sndlen  = 0;

  /* Check each member of the group */

  for (group = (FAR struct igmp_group_s *)dev->grplist.head;
       group;
       group = group->next)
    {
      /* Does this member have a pending outgoing message? */

      if (IS_SCHEDMSG(group->flags))
        {
          /* Yes, create the IGMP message in the driver buffer */

          igmp_sched_send(dev, group);

          /* Mark the message as sent and break out */

          CLR_SCHEDMSG(group->flags);
          break;
        }
    }
}
#endif /* CONFIG_NET_IGMP */
