/****************************************************************************
 * net/netfilter/ipt_nat.c
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

#include <debug.h>
#include <errno.h>
#include <string.h>

#include <nuttx/net/netfilter/nf_nat.h>

#include "nat/nat.h"
#include "netdev/netdev.h"
#include "netfilter/iptables.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: adjust_nat
 *
 * Description:
 *   Callback for netdev_foreach, enable/disable NAT on dev.
 *
 * Input Parameters:
 *   dev       - Device to adjust NAT.
 *   arg       - The input ipt_replace config of NAT.
 *
 ****************************************************************************/

static int adjust_nat(FAR struct net_driver_s *dev, FAR void *arg)
{
  FAR const struct ipt_replace *repl = arg;
  FAR struct ipt_entry *entry;
  FAR struct xt_entry_target *target;

  ipt_entry_for_every(entry, repl->entries, repl->size)
    {
      target = IPT_TARGET(entry);
      if (strcmp(target->u.user.name, XT_MASQUERADE_TARGET) == 0 &&
          strcmp(dev->d_ifname, entry->ip.outiface) == 0)
        {
          ipv4_nat_enable(dev);
          return 0;
        }
    }

  ipv4_nat_disable(dev);
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ipt_nat_init
 *
 * Description:
 *   Init NAT table data.
 *
 ****************************************************************************/

FAR struct ipt_replace *ipt_nat_init(void)
{
  return ipt_alloc_table(TABLE_NAME_NAT, 1 << NF_INET_POST_ROUTING);
}

/****************************************************************************
 * Name: ipt_nat_apply
 *
 * Description:
 *   Try to apply NAT rules, will do nothing if failed.
 *
 * Input Parameters:
 *   repl   - The config got from user space to control NAT table.
 *
 ****************************************************************************/

int ipt_nat_apply(FAR const struct ipt_replace *repl)
{
  FAR struct ipt_entry *entry;
  FAR struct xt_entry_target *target;

  /* Check config first. */

  ipt_entry_for_every(entry, repl->entries, repl->size)
    {
      target = IPT_TARGET(entry);
      if (strcmp(target->u.user.name, XT_MASQUERADE_TARGET) == 0 &&
          netdev_findbyname(entry->ip.outiface) == NULL)
        {
          /* MASQUERADE specified but no WAN device, don't apply. */

          nwarn("WARNING: Trying to apply nat on nonexistent dev: %s",
                entry->ip.outiface);
          return -EINVAL;
        }
    }

  /* Do actual work. */

  netdev_foreach(adjust_nat, (FAR void *)repl);

  return OK;
}
