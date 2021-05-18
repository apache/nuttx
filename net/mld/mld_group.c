/****************************************************************************
 * net/mld/mld_group.c
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
#include <nuttx/compiler.h>

#include <stdlib.h>
#include <string.h>
#include <queue.h>
#include <assert.h>
#include <debug.h>

#include <arch/irq.h>

#include <nuttx/arch.h>
#include <nuttx/wdog.h>
#include <nuttx/kmalloc.h>
#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/mld.h>

#include "devif/devif.h"
#include "mld/mld.h"

#ifdef CONFIG_NET_MLD

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  mld_ngroups
 *
 * Description:
 *   Return the number of groups joined by applications.
 *
 ****************************************************************************/

static int mld_ngroups(FAR struct net_driver_s *dev)
{
  FAR struct mld_group_s *group;
  int ngroups = 0;

  /* Count the number of groups in the group list */

  for (group = (FAR struct mld_group_s *)dev->d_mld.grplist.head;
       group != NULL;
       group = group->next)
    {
      ngroups++;
    }

  /* REVISIT:  Subtract one for the IPv6 allnodes group.  Why is this here?
   * what do we need it for?  It was cloned from IGMP and is probably not
   * needed.
   */

  return ngroups > 0 ? ngroups - 1 : 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  mld_grpalloc
 *
 * Description:
 *   Allocate a new group from heap memory.
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

FAR struct mld_group_s *mld_grpalloc(FAR struct net_driver_s *dev,
                                     FAR const net_ipv6addr_t addr)
{
  FAR struct mld_group_s *group;

  mldinfo("addr: %08x dev: %p\n", *addr, dev);
  group = (FAR struct mld_group_s *)kmm_zalloc(sizeof(struct mld_group_s));

  mldinfo("group: %p\n", group);

  /* Check if we successfully allocated a group structure */

  if (group != NULL)
    {
      /* Initialize the non-zero elements of the group structure */

      net_ipv6addr_copy(group->grpaddr, addr);

      /* This semaphore is used for signaling and, hence, should not have
       * priority inheritance enabled.
       */

      nxsem_init(&group->sem, 0, 0);
      nxsem_set_protocol(&group->sem, SEM_PRIO_NONE);

      /* Save the interface index */

      group->ifindex = dev->d_ifindex;

#ifndef CONFIG_NET_MLD_ROUTER
      /* Start the query timer if we are the Querier and this is the first
       * group member of the group.
       */

      if (mld_ngroups(dev) == 0)
        {
          mld_start_gentimer(dev, MSEC2TICK(MLD_QUERY_MSEC));
        }
#endif

      /* Add the group structure to the list in the device structure */

      sq_addfirst((FAR sq_entry_t *)group, &dev->d_mld.grplist);
    }

  return group;
}

/****************************************************************************
 * Name:  mld_grpfind
 *
 * Description:
 *   Find an existing group.
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

FAR struct mld_group_s *mld_grpfind(FAR struct net_driver_s *dev,
                                    FAR const net_ipv6addr_t addr)
{
  FAR struct mld_group_s *group;

  mldinfo("Searching for group: %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
          addr[0], addr[1], addr[2], addr[3], addr[4], addr[5], addr[6],
          addr[7]);

  for (group = (FAR struct mld_group_s *)dev->d_mld.grplist.head;
       group;
       group = group->next)
    {
      mldinfo("Compare: %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
              group->grpaddr[0], group->grpaddr[1], group->grpaddr[2],
              group->grpaddr[3], group->grpaddr[4], group->grpaddr[5],
              group->grpaddr[6], group->grpaddr[7]);

      if (net_ipv6addr_cmp(group->grpaddr, addr))
        {
          mldinfo("Match!\n");
          DEBUGASSERT(group->ifindex == dev->d_ifindex);
          break;
        }
    }

  return group;
}

/****************************************************************************
 * Name:  mld_grpallocfind
 *
 * Description:
 *   Find an existing group.  If not found, create a new group for the
 *   address.
 *
 ****************************************************************************/

FAR struct mld_group_s *mld_grpallocfind(FAR struct net_driver_s *dev,
                                         FAR const net_ipv6addr_t addr)
{
  FAR struct mld_group_s *group = mld_grpfind(dev, addr);

  mldinfo("group: %p addr: %08x\n", group, (int)*addr);
  if (!group)
    {
      group = mld_grpalloc(dev, addr);
    }

  mldinfo("group: %p\n", group);
  return group;
}

/****************************************************************************
 * Name:  mld_grpfree
 *
 * Description:
 *   Release a previously allocated group.
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

void mld_grpfree(FAR struct net_driver_s *dev, FAR struct mld_group_s *group)
{
  mldinfo("Free: %p flags: %02x\n", group, group->flags);

  /* Cancel the timers */

  wd_cancel(&group->polldog);

  /* Remove the group structure from the group list in the device structure */

  sq_rem((FAR sq_entry_t *)group, &dev->d_mld.grplist);

  /* Destroy the wait semaphore */

  nxsem_destroy(&group->sem);

  /* Cancel the watchdog timer */

  wd_cancel(&group->polldog);

  /* Then release the group structure resources. */

  mldinfo("Call kmm_free()\n");
  kmm_free(group);

#ifndef CONFIG_NET_MLD_ROUTER
  /* If there are no longer any groups, then stop the general query and v1
   * compatibility timers.
   */

  if (mld_ngroups(dev) == 0)
    {
      wd_cancel(&dev->d_mld.gendog);
      wd_cancel(&dev->d_mld.v1dog);
    }
#endif
}

/****************************************************************************
 * Name:  mld_new_pollcycle
 *
 * Description:
 *   Update accumulated membership at the beginning of each new poll cycle
 *
 ****************************************************************************/

#ifdef CONFIG_NET_MLD_ROUTER
void mld_new_pollcycle(FAR struct net_driver_s *dev)
{
  FAR struct mld_group_s *member;

  /* Update member ship in every group  */

  for (member = (FAR struct mld_group_s *)dev->d_mld.grplist.head;
       member;
       member = member->next)
    {
      /* Save the number of members that reported in the previous query
       * cycle; reset the number of members that have reported in the new
       * query cycle.
       */

      member->lstmbrs = member->members;
      member->members = 0;
    }
}
#endif

#endif /* CONFIG_NET_MLD */
