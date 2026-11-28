/****************************************************************************
 * net/igmp/igmp_msg.c
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

#include <nuttx/net/netconfig.h>
#include <nuttx/net/net.h>
#include <nuttx/net/igmp.h>

#include "devif/devif.h"
#include "netdev/netdev.h"
#include "igmp/igmp.h"

#ifdef CONFIG_NET_IGMP

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: igmp_schedmsg
 *
 * Description:
 *   Schedule a message to be send at the next driver polling interval.
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

int igmp_schedmsg(FAR struct igmp_group_s *group, uint8_t msgid)
{
  FAR struct net_driver_s *dev;

  DEBUGASSERT(group != NULL);
  DEBUGASSERT(group->ifindex > 0);

  if (IS_SCHEDMSG(group->flags))
    {
      nerr("ERROR: The group %u is busy\n", group->ifindex);
      return -EBUSY;
    }

  /* Get the device instance associated with the interface index of the
   * group
   */

  dev = netdev_findbyindex(group->ifindex);
  if (dev == NULL)
    {
      nerr("ERROR: No device for this interface index: %u\n",
           group->ifindex);
      return -ENODEV;
    }

  /* Schedule the message */

  group->msgid = msgid;
  SET_SCHEDMSG(group->flags);

  /* Notify the device that we have a packet to send */

  netdev_lock(dev);
  netdev_txnotify_dev(dev, IGMP_POLL);
  netdev_unlock(dev);
  return OK;
}

/****************************************************************************
 * Name: igmp_waitmsg
 *
 * Description:
 *   Schedule a message to be send at the next driver polling interval and
 *   block, waiting for the message to be sent.
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

int igmp_waitmsg(FAR struct igmp_group_s *group, uint8_t msgid)
{
  int ret;

  /* Schedule to send the message */

  DEBUGASSERT(!IS_WAITMSG(group->flags));
  SET_WAITMSG(group->flags);

  ret = igmp_schedmsg(group, msgid);
  if (ret < 0)
    {
      nerr("ERROR: Failed to schedule the message: %d\n", ret);
      goto errout;
    }

  /* Then wait for the message to be sent */

  while (IS_SCHEDMSG(group->flags))
    {
      /* Wait for the semaphore to be posted */

      ret = nxsem_wait_uninterruptible(&group->sem);
      if (ret < 0)
        {
          break;
        }
    }

  /* The message has been sent and we are no longer waiting */

errout:
  CLR_WAITMSG(group->flags);
  return ret;
}

#endif /* CONFIG_NET_IGMP */
