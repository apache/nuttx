/****************************************************************************
 * net/igmp/igmp_poll.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include <nuttx/debug.h>
#include <inttypes.h>
#include <stdint.h>

#include <nuttx/semaphore.h>
#include <nuttx/net/netconfig.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/netstats.h>
#include <nuttx/net/ip.h>

#include "devif/devif.h"
#include "igmp/igmp.h"

#ifdef CONFIG_NET_IGMP

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Buffer layout */

#define RASIZE      (4)

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
  FAR const in_addr_t *dest;

  /* REVISIT:  This should be deferred to a work queue */

  /* Check what kind of message we need to send.  There are only two
   * possibilities:
   */

  if (group->msgid == IGMPv2_MEMBERSHIP_REPORT)
    {
      dest = &group->grpaddr;
      ninfo("Send IGMPv2_MEMBERSHIP_REPORT, dest=%08" PRIx32 " flags=%02x\n",
             (uint32_t)*dest, group->flags);
      SET_LASTREPORT(group->flags); /* Remember we were the last to report */
    }
  else
    {
      DEBUGASSERT(group->msgid == IGMP_LEAVE_GROUP);
      dest = &g_ipv4_allrouters;
      ninfo("Send IGMP_LEAVE_GROUP, dest=%08" PRIx32 " flags=%02x\n",
             (uint32_t)*dest, group->flags);
    }

  /* Send the message */

  igmp_send(dev, group, dest, group->msgid);

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

  /* Setup the poll operation */

  dev->d_len    = 0;
  dev->d_sndlen = 0;

  /* Check each member of the group */

  for (group = (FAR struct igmp_group_s *)dev->d_igmp_grplist.head;
       group;
       group = group->next)
    {
      /* Does this member have a pending outgoing message? */

      if (IS_SCHEDMSG(group->flags))
        {
          /* Yes, create the IGMP message in the driver buffer */

          igmp_sched_send(dev, group);
          break;
        }
    }
}
#endif /* CONFIG_NET_IGMP */
