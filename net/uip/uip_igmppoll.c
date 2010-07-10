/****************************************************************************
 * net/uip/uip-igmp.h
 * The definitions in this header file are intended only for internal use
 * by the NuttX port of the uIP stack.
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

#include <assert.h>
#include <debug.h>

#include <net/uip/uipopt.h>
#include <net/uip/uip.h>
#include <net/uip/uip-arch.h>

#include "uip_internal.h"

#ifdef CONFIG_NET_IGMP

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IGMPBUF ((struct uip_igmphdr_s *)&dev->d_buf[UIP_LLH_LEN])

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  uip_schedsend
 *
 * Description:
 *   Construct the .
 *
 * Returned Value:
 *   Returns a non-zero value if a IGP message is sent.
 *
 * Assumptions:
 *   This function is called from the driver polling logic... probably within
 *   an interrupt handler.
 *
 ****************************************************************************/

static inline void uip_schedsend(FAR struct uip_driver_s *dev, FAR struct igmp_group_s *group)
{
  uip_ipaddr_t *dest;

  /* IGMP header + IP header + 8 bytes of data */

  if (group->msgid == IGMPv2_MEMBERSHIP_REPORT)
    {
      nllvdbg("Send IGMPv2_MEMBERSHIP_REPORT\n");
      dest = &group->grpaddr;
      IGMP_STATINCR(uip_stat.igmp.report_sched);
      uiphdr_ipaddr_copy(IGMPBUF->grpaddr, &group->grpaddr);
      SET_LASTREPORT(group->flags); /* Remember we were the last to report */
    }
  else
    {
      nllvdbg("Send IGMP_LEAVE_GROUP\n");
      DEBUGASSERT(group->msgid == IGMP_LEAVE_GROUP);
      dest = &g_allrouters;
      IGMP_STATINCR(uip_stat.igmp.leave_sched);
      uiphdr_ipaddr_copy(IGMPBUF->grpaddr, &group->grpaddr);
    }

  uip_igmpsend(dev, dest);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  uip_igmppoll
 *
 * Description:
 *   Poll the groups associated with the device to see if any IGMP messages
 *   are pending transfer.
 *
 * Returned Value:
 *   Returns a non-zero value if a IGP message is sent.
 *
 * Assumptions:
 *   This function is called from the driver polling logic... probably within
 *   an interrupt handler.
 *
 ****************************************************************************/

void uip_igmppoll(FAR struct uip_driver_s *dev)
{
  FAR struct igmp_group_s *group;

  /* Setup the poll operation */

  dev->d_appdata = &dev->d_buf[UIP_LLH_LEN + UIP_IPIGMPH_LEN];
  dev->d_snddata = &dev->d_buf[UIP_LLH_LEN + UIP_IPIGMPH_LEN];

  dev->d_len     = 0;
  dev->d_sndlen  = 0;

  /* Check each member of the group */

  for (group = (FAR struct igmp_group_s *)dev->grplist.head; group; group = group->next)
    {
      /* Does this member have a pending outgoing message? */

      if (IS_SCHEDMSG(group->flags))
        {
          /* Yes, create the IGMP message in the driver buffer */

          uip_schedsend(dev, group);

          /* Mark the message as sent and break out */

          CLR_SCHEDMSG(group->flags);
          break;
        }
    }
}
#endif /* CONFIG_NET_IGMP */
