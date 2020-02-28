/****************************************************************************
 * net/mld/mld_poll.c
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

#include <nuttx/net/netconfig.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/netstats.h>
#include <nuttx/net/ip.h>

#include "devif/devif.h"
#include "mld/mld.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  mld_poll
 *
 * Description:
 *   Poll the groups associated with the device to see if any MLD messages
 *   are pending transfer.
 *
 * Returned Value:
 *   Returns a non-zero value if a IGP message is sent.
 *
 * Assumptions:
 *   This function must be called with the network locked.
 *
 ****************************************************************************/

void mld_poll(FAR struct net_driver_s *dev)
{
  FAR struct mld_group_s *group;

  /* Setup the poll operation */

  dev->d_appdata = &dev->d_buf[NET_LL_HDRLEN(dev) + IPv6_HDRLEN];
  dev->d_len     = 0;
  dev->d_sndlen  = 0;

  /* Check if a general query is pending */

  if (IS_MLD_GENPEND(dev->d_mld.flags))
    {
      /* Clear the pending flag */

      CLR_MLD_GENPEND(dev->d_mld.flags);

      /* Are we still the querier? */

      if (IS_MLD_QUERIER(dev->d_mld.flags))
        {
          /* Yes, send the general query and return */

          mld_send(dev, NULL, MLD_SEND_GENQUERY);
          return;
        }
    }

  /* Check each member of the group */

  for (group = (FAR struct mld_group_s *)dev->d_mld.grplist.head;
       group;
       group = group->next)
    {
      /* Does this member have a pending outgoing message? */

      if (IS_MLD_SCHEDMSG(group->flags))
        {
          /* Yes.. create the MLD message in the driver buffer */

          mld_send(dev, group, group->msgtype);

          /* Indicate that the message has been sent */

          CLR_MLD_SCHEDMSG(group->flags);
          group->msgtype = MLD_SEND_NONE;

          /* If there is a thread waiting fore the message to be sent, wake
           * it up.
           */

          if (IS_MLD_WAITMSG(group->flags))
            {
              mldinfo("Awakening waiter\n");

              CLR_MLD_WAITMSG(group->flags);
              nxsem_post(&group->sem);
            }

          /* And break out of the loop */

          break;
        }

      /* No.. does this message have a pending report still to be sent? */

      else if (IS_MLD_RPTPEND(group->flags))
        {
          /* Yes.. create the MLD message in the driver buffer */

          mld_send(dev, group, mld_report_msgtype(dev));

          /* Indicate that the report is no longer pending */

          CLR_MLD_RPTPEND(group->flags);

          /* And break out of the loop */

          break;
        }
    }
}
