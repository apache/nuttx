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

#include <nuttx/can.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mm/iob.h>
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
  if (filter->can_id & CAN_INV_FILTER)
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
 * Name: add_cangw_rule
 *
 * Description:
 *   Add a CAN->CAN gateway rule to a dev gw rule list.
 *
 ****************************************************************************/

void add_cangw_rule(FAR struct net_driver_s *dev,
                    FAR struct cangw_rule_s *cgw_rules)
{
  DEBUGASSERT(dev && cgw_rules);

  netdev_lock(dev);
  sq_addlast(&cgw_rules->node, &dev->d_cangw_rules);
  netdev_unlock(dev);
}

/****************************************************************************
 * Name: cangw_rule_match
 *
 * Description:
 *   Check if a rule in the list matches the given criteria.
 *
 * Input Parameters:
 *   rtrule - The rule in the device's rule list.
 *   rule   - The match criteria provided by the caller.
 *
 * Returned Value:
 *   true if the rule matches, false otherwise.
 *
 ****************************************************************************/

static inline_function bool
cangw_rule_match(FAR const struct cangw_rule_s *rtrule,
                 FAR const struct cangw_rule_s *rule)
{
  /* Match filter if caller provided a non-zero filter */

  if (rule->gw.ccgw.filter.can_id != 0 ||
      rule->gw.ccgw.filter.can_mask != 0)
    {
      if (rtrule->gw.ccgw.filter.can_id !=
                                       rule->gw.ccgw.filter.can_id ||
          rtrule->gw.ccgw.filter.can_mask !=
                                       rule->gw.ccgw.filter.can_mask)
        {
          return false;
        }
    }

  /* Match destination device if specified */

  if (rule->dest_dev && rtrule->dest_dev != rule->dest_dev)
    {
      return false;
    }

  return true;
}

/****************************************************************************
 * Name: remove_cangw_rule
 *
 * Description:
 *   Remove CAN->CAN gateway rules from a device's rule list.
 *
 * Input Parameters:
 *   dev     - The network device whose rule list to modify.
 *   rule    - Match criteria. If NULL, all rules on the device are removed.
 *   removed - If non-NULL, removed nodes are collected into this queue
 *             and the caller is responsible for freeing them.
 *             If NULL, heap-allocated nodes are freed immediately;
 *             non-heap nodes (e.g. embedded in a static config struct)
 *             are only unlinked from the list.
 *
 * Returned Value:
 *   The number of rules removed.
 *
 ****************************************************************************/

int remove_cangw_rule(FAR struct net_driver_s *dev,
                      FAR const struct cangw_rule_s *rule,
                      FAR sq_queue_t *removed)
{
  FAR sq_entry_t *current;
  FAR sq_entry_t *tmp;
  int count = 0;

  DEBUGASSERT(dev);

  if (removed)
    {
      sq_init(removed);
    }

  netdev_lock(dev);

  sq_for_every_safe(&dev->d_cangw_rules, current, tmp)
    {
      FAR struct cangw_rule_s *rtrule =
                              (FAR struct cangw_rule_s *)current;

      /* If rule is NULL, remove all; otherwise match by criteria */

      if (rule == NULL || cangw_rule_match(rtrule, rule))
        {
          sq_rem(current, &dev->d_cangw_rules);
          if (removed)
            {
              /* Caller takes ownership — collect for deferred free */

              sq_addlast(current, removed);
            }
          else if (kmm_heapmember(current))
            {
              /* Only free nodes that were individually heap-allocated */

              kmm_free(current);
            }

          count++;
        }
    }

  netdev_unlock(dev);
  return count;
}

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
