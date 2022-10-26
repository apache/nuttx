/****************************************************************************
 * net/mld/mld_poll.c
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

  dev->d_appdata = IPBUF(IPv6_HDRLEN);
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
