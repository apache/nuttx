/****************************************************************************
 * net/igmp/igmp_msg.c
 *
 *   Copyright (C) 2010-2011, 2018 Gregory Nutt. All rights reserved.
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

#include <assert.h>
#include <debug.h>

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

  DEBUGASSERT(group != NULL && !IS_SCHEDMSG(group->flags));
  DEBUGASSERT(group->ifindex > 0);

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

  netdev_txnotify_dev(dev);
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

      ret = net_sem_wait_uninterruptible(&group->sem);
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
