/****************************************************************************
 * net/arp/arp_acd.c
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

#include <debug.h>
#include <stdlib.h>
#include <unistd.h>

#include <nuttx/net/ethernet.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/net.h>
#include <nuttx/net/netconfig.h>
#include <nuttx/semaphore.h>

#include "arp/arp.h"
#include "netlink/netlink.h"
#include "utils/utils.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void arp_acd_try_announce(FAR void *net_dev);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arp_acd_arrange_announce
 *
 * Description:
 *   creat work_queue to send ARP announce
 *
 * Input Parameters:
 *   dev - The device driver structure to use in the send operation
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void arp_acd_arrange_announce(FAR struct net_driver_s *dev)
{
  if (dev->d_acd.need_announce == true)
    {
      return;
    }

  if (!work_available(&dev->d_acd.work))
    {
      nerr("ERROR work unavailable \n");
      return;
    }

  int ret = work_queue(LPWORK, &dev->d_acd.work, arp_acd_try_announce,
                       (FAR void *)dev, DSEC2TICK(dev->d_acd.ttw));
  if (ret != OK)
    {
      nerr("ERROR ret %d \n", ret);
    }
}

/****************************************************************************
 * Name: arp_acd_send_finish
 *
 * Description:
 *   send finish process of ARP Address Conflict Detection
 *
 * Input Parameters:
 *   dev - The device driver structure to use in the send operation
 *   result -  arp send result
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void arp_acd_send_finish(FAR struct net_driver_s *dev, int result)
{
  if (result < 0)
    {
      nerr("ERROR: arp_send result: %d\n", result);
    }
  else
    {
      arp_acd_arrange_announce(dev);
    }
}

/****************************************************************************
 * Name: arp_acd_try_announce
 *
 * Description:
 *   process status of ARP Address Conflict Detection
 *
 * Input Parameters:
 *   net_dev - The device driver structure to use in the send operation
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void arp_acd_try_announce(FAR void *net_dev)
{
  FAR struct net_driver_s *dev = net_dev;

  if (dev == NULL || dev->d_acd.state != ARP_ACD_STATE_ANNOUNCING)
    {
      return;
    }

  /* arp_acd_announce */

  arp_send_async(dev->d_ipaddr, arp_acd_send_finish);
  dev->d_acd.sendnum++;

  if (dev->d_acd.sendnum >= ANNOUNCE_NUM)
    {
      dev->d_acd.sendnum = 0;
      dev->d_acd.ttw = 0;
      dev->d_acd.state = ARP_ACD_STATE_FINISH;
    }
  else
    {
      dev->d_acd.ttw = ANNOUNCE_INTERVAL * ARP_ACD_TICKS_PER_SECOND;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arp_acd_update
 *
 * Description:
 *   interface of ARP Address Conflict Detection monitor
 *
 * Input Parameters:
 *   dev - The device driver structure to use in the send operation
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

void arp_acd_update(FAR struct net_driver_s *dev)
{
  FAR struct arp_hdr_s *arp = ARPBUF;
  clock_t now = clock_systime_ticks();

  if (dev->d_acd.conflict_flag == ARP_ACD_ADDRESS_CONFLICT)
    {
      return;
    }

  if (!net_ipv4addr_hdrcmp(arp->ah_sipaddr, &dev->d_ipaddr) ||
      (memcmp(arp->ah_shwaddr, dev->d_mac.ether.ether_addr_octet,
              sizeof(arp->ah_shwaddr)) == 0))
    {
      return;
    }

  if ((dev->d_acd.lastconflict > 0) &&
      (now - dev->d_acd.lastconflict) <
        DSEC2TICK(DEFEND_INTERVAL * ARP_ACD_TICKS_PER_SECOND))
    {
      nerr("ERROR: detect conflict again \n");
      dev->d_acd.lastconflict = 0;
      dev->d_acd.conflict_flag = ARP_ACD_ADDRESS_CONFLICT;
      if (dev->d_acd.state != ARP_ACD_STATE_ANNOUNCING)
        {
          dev->d_acd.state = ARP_ACD_STATE_INIT;
          dev->d_acd.sendnum = 0;
          dev->d_acd.ttw = 0;
        }
    }
  else
    {
      nerr("ERROR: detect conflict \n");
      dev->d_acd.lastconflict = now;
      if (dev->d_acd.state != ARP_ACD_STATE_ANNOUNCING)
        {
          arp_acd_arrange_announce(dev);

          dev->d_acd.state = ARP_ACD_STATE_ANNOUNCING;
          dev->d_acd.sendnum = 0;
          dev->d_acd.ttw =
            ANNOUNCE_WAIT * ARP_ACD_TICKS_PER_SECOND;
        }
    }
}

/****************************************************************************
 * Name: arp_acd_setup
 *
 * Description:
 *   set up interface of ARP Address Conflict Detection
 *
 * Input Parameters:
 *   dev - The device driver structure to use in the send operation
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

void arp_acd_setup(FAR struct net_driver_s *dev)
{
  if (dev->d_acd.need_announce == false)
    {
      return;
    }

  dev->d_acd.state = ARP_ACD_STATE_ANNOUNCING;
  dev->d_acd.sendnum = 0;
  dev->d_acd.ttw = 0;
  dev->d_acd.conflict_flag = ARP_ACD_ADDRESS_NO_CONFLICT;
  dev->d_acd.lastconflict = 0;
  dev->d_acd.need_announce = false;

  arp_acd_arrange_announce(dev);
}

/****************************************************************************
 * Name: arp_acd_set_addr
 *
 * Description:
 *   setting address interface of ARP Address Conflict Detection
 *
 * Input Parameters:
 *   dev - The device driver structure to use in the send operation
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

void arp_acd_set_addr(FAR struct net_driver_s *dev)
{
  if (!net_ipv4addr_cmp(dev->d_ipaddr, INADDR_ANY))
    {
      dev->d_acd.need_announce = true;
      if (IFF_IS_UP(dev->d_flags))
        {
          arp_acd_setup(dev);
        }
    }
  else
    {
      dev->d_acd.need_announce = false;
      dev->d_acd.state = ARP_ACD_STATE_INIT;
    }
}
