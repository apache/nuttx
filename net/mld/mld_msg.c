/****************************************************************************
 * net/mld/mld_msg.c
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
#include <nuttx/net/net.h>
#include <nuttx/net/mld.h>

#include "devif/devif.h"
#include "netdev/netdev.h"
#include "mld/mld.h"

#ifdef CONFIG_NET_MLD

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mld_schedmsg
 *
 * Description:
 *   Schedule a message to be send at the next driver polling interval.
 *
 * Assumptions:
 *   The network is locked
 *
 ****************************************************************************/

int mld_schedmsg(FAR struct mld_group_s *group, uint8_t msgtype)
{
  FAR struct net_driver_s *dev;

  DEBUGASSERT(group != NULL && !IS_MLD_SCHEDMSG(group->flags));
  DEBUGASSERT(group->ifindex > 0);

  /* Get the device instance associated with the interface index
   * of the group
   */

  dev = netdev_findbyindex(group->ifindex);
  if (dev == NULL)
    {
      mlderr("ERROR: No device for this interface index: %u\n",
             group->ifindex);
      return -ENODEV;
    }

  /* Schedule the message */

  group->msgtype = msgtype;
  SET_MLD_SCHEDMSG(group->flags);

  /* Notify the device that we have a packet to send */

  netdev_txnotify_dev(dev);
  return OK;
}

/****************************************************************************
 * Name: mld_waitmsg
 *
 * Description:
 *   Schedule a message to be send at the next driver polling interval and
 *   block, waiting for the message to be sent.
 *
 * Assumptions:
 *   The network is locked
 *
 ****************************************************************************/

int mld_waitmsg(FAR struct mld_group_s *group, uint8_t msgtype)
{
  int ret;

  /* Schedule to send the message */

  DEBUGASSERT(!IS_MLD_WAITMSG(group->flags));
  SET_MLD_WAITMSG(group->flags);

  ret = mld_schedmsg(group, msgtype);
  if (ret < 0)
    {
      mlderr("ERROR: Failed to schedule the message: %d\n", ret);
      goto errout;
    }

  /* Then wait for the message to be sent */

  while (IS_MLD_SCHEDMSG(group->flags))
    {
      /* Wait for the semaphore to be posted */

      ret = net_lockedwait_uninterruptible(&group->sem);
      if (ret < 0)
        {
          break;
        }
    }

  /* The message has been sent and we are no longer waiting */

errout:
  CLR_MLD_WAITMSG(group->flags);
  return ret;
}

#endif /* CONFIG_NET_MLD */
