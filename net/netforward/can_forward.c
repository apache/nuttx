/****************************************************************************
 * net/netforward/can_forward.c
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

#include <string.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/mm/iob.h>
#include <nuttx/can.h>
#include <nuttx/net/can.h>
#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/netstats.h>

#include "devif/devif.h"
#include "netforward/netforward.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: can_dev_forward
 *
 * Description:
 *   This function is called from can_forward when it is necessary to
 *   forward a packet from the current device to different device.  In this
 *   case, the forwarding operation must be performed asynchronously when
 *   the TX poll is received from the forwarding device.
 *
 * Input Parameters:
 *   dev      - The device on which the packet was received and which
 *              contains the packet.
 *   fwdrule   - The forward target rule.
 *
 * Returned Value:
 *   Zero is returned if the packet was successfully forward;  A negated
 *   errno value is returned if the packet is not forwardable.
 *
 ****************************************************************************/

static int can_dev_forward(FAR struct net_driver_s *dev,
                           FAR struct cangw_rule_s *fwdrule)
{
  FAR struct forward_s *fwd;
  FAR struct net_driver_s *fwddev = fwdrule->dest_dev;
  int ret = -ENOMEM;

  /* If the interface isn't "Running", we can't forward. */

  if ((fwddev->d_flags & IFF_RUNNING) == 0)
    {
      nwarn("WARNING: device is DOWN\n");
      ret = -EHOSTUNREACH;
      goto errout;
    }

  /* Get a pre-allocated forwarding structure,  This structure will be
   * completely zeroed when we receive it.
   */

  fwd = netfwd_alloc();
  if (fwd == NULL)
    {
      nwarn("WARNING: Failed to allocate forwarding structure\n");
      goto errout;
    }

  /* Initialize the easy stuff in the forwarding structure */

  fwd->f_dev = fwddev;     /* Forwarding device */
  fwd->f_domain = PF_CAN;  /* CAN domain */

  /* Relay the device buffer */

  fwd->f_iob = can_iob_clone(dev);  /* IOB chain to be forwarded */
  if (fwd->f_iob == NULL)
    {
      nwarn("WARNING: Failed to clone IOB\n");
      goto errout_with_fwd;
    }

  /* Then set up to forward the packet according to the protocol. */

  ret = netfwd_forward(dev, fwd);
  if (ret >= 0)
    {
#ifdef CONFIG_NET_STATISTICS
      g_netstats.cangw.forward++;
#endif
      return OK;
    }

errout_with_fwd:
  if (fwd->f_iob != NULL)
    {
      iob_free_chain(fwd->f_iob);
    }

  netfwd_free(fwd);

errout:
#ifdef CONFIG_NET_STATISTICS
  g_netstats.cangw.drop++;
#endif
  return ret;
}

/****************************************************************************
 * Name: can_forward_filter
 *
 * Description:
 *   Check if the input id are match to one forward target rule.
 *
 * Input Parameters:
 *   filter   - The forward filter.
 *   id       - CAN msg ID.
 *
 * Returned Value:
 *   0 - not match, 1 = match
 *
 ****************************************************************************/

static inline_function int can_forward_filter(FAR struct can_filter *filter,
                                              canid_t id)
{
  if (id & CAN_INV_FILTER)
    {
      return ((id & filter->can_mask) !=
                ((filter->can_id & ~CAN_INV_FILTER) & filter->can_mask));
    }
  else
    {
      return ((id & filter->can_mask) ==
                (filter->can_id & filter->can_mask));
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: can_forward
 *
 * Description:
 *   This function is called from can_input when a packet is received that
 *   is not destined for us.  In this case, the packet may need to be
 *   forwarded to another device depending routing table information.
 *
 * Input Parameters:
 *   dev   - The device on which the packet was received and which contains
 *           the can packet.
 *
 *   On input:
 *   - dev->d_buf holds the received packet.
 *   - dev->d_len holds the length of the received packet.
 *
 * Returned Value:
 *   Always return Zero to let the packet be continue received by can_input;
 *
 ****************************************************************************/

int can_forward(FAR struct net_driver_s *dev)
{
#ifdef CONFIG_NET_CAN_CANFD
  FAR struct canfd_frame *frame = NETLLBUF;
#else
  FAR struct can_frame *frame = NETLLBUF;
#endif
  FAR const sq_entry_t *entry;
  FAR struct cangw_rule_s *fwdrule;

  if (frame->flags & CAN_TCF)
    {
      /* Do not forward transmit confirmation frames */

      return OK;
    }

  /* Search for a valid fwdrule */

  sq_for_every(&dev->d_cangw_rules, entry)
    {
      fwdrule = (FAR struct cangw_rule_s *)entry;
      switch (fwdrule->gwtype)
        {
          case CGW_TYPE_CAN_CAN:
            if (can_forward_filter(&fwdrule->gw.ccgw.filter, frame->can_id))
              {
                can_dev_forward(dev, fwdrule);
#ifdef CONFIG_NET_STATISTICS
                g_netstats.cangw.recv++;
#endif
              }
            break;

          default:
            break;
        }
    }

  /* Return success. can_input will return to the network driver with
   * dev->d_len set to the packet size and the network driver will perform
   * the transfer.
   */

  return OK;
}
