/****************************************************************************
 * net/igmp/igmp_group.c
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
#include <nuttx/compiler.h>

#include <inttypes.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <nuttx/debug.h>

#include <arch/irq.h>

#include <nuttx/arch.h>
#include <nuttx/wdog.h>
#include <nuttx/kmalloc.h>
#include <nuttx/queue.h>
#include <nuttx/semaphore.h>
#include <nuttx/net/net.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/igmp.h>

#include "devif/devif.h"
#include "igmp/igmp.h"
#include "utils/utils.h"

#ifdef CONFIG_NET_IGMP

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Debug ********************************************************************/

#undef IGMP_GRPDEBUG /* Define to enable detailed IGMP group debug */

#ifndef CONFIG_NET_IGMP
#  undef IGMP_GRPDEBUG
#endif

#ifdef IGMP_GRPDEBUG
#  define grperr    nerr
#  define grpinfo   ninfo
#else
#  define grperr    _none
#  define grpinfo   _none
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  igmp_grpalloc
 *
 * Description:
 *   Allocate a new group from heap memory.
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

FAR struct igmp_group_s *igmp_grpalloc(FAR struct net_driver_s *dev,
                                       FAR const in_addr_t *addr)
{
  FAR struct igmp_group_s *group;

  ninfo("addr: %08" PRIx32 " dev: %p\n", (uint32_t)*addr, dev);
  group = kmm_zalloc(sizeof(struct igmp_group_s));

  grpinfo("group: %p\n", group);

  /* Check if we successfully allocated a group structure */

  if (group != NULL)
    {
      /* Initialize the non-zero elements of the group structure */

      net_ipv4addr_copy(group->grpaddr, *addr);
      nxsem_init(&group->sem, 0, 0);

      /* Save the interface index */

      group->ifindex = dev->d_ifindex;

      /* Add the group structure to the list in the device structure */

      sq_addfirst((FAR sq_entry_t *)group, &dev->d_igmp_grplist);
    }

  return group;
}

/****************************************************************************
 * Name:  igmp_grpfind
 *
 * Description:
 *   Find an existing group.
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

FAR struct igmp_group_s *igmp_grpfind(FAR struct net_driver_s *dev,
                                      FAR const in_addr_t *addr)
{
  FAR struct igmp_group_s *group;

  grpinfo("Searching for addr %08x\n", (int)*addr);

  for (group = (FAR struct igmp_group_s *)dev->d_igmp_grplist.head;
       group;
       group = group->next)
    {
      grpinfo("Compare: %08" PRIx32 " vs. %08" PRIx32 "\n",
              (uint32_t)group->grpaddr, (uint32_t)*addr);
      if (net_ipv4addr_cmp(group->grpaddr, *addr))
        {
          grpinfo("Match!\n");
          DEBUGASSERT(group->ifindex == dev->d_ifindex);
          break;
        }
    }

  return group;
}

/****************************************************************************
 * Name:  igmp_grpallocfind
 *
 * Description:
 *   Find an existing group.  If not found, create a new group for the
 *   address.
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

FAR struct igmp_group_s *igmp_grpallocfind(FAR struct net_driver_s *dev,
                                           FAR const in_addr_t *addr)
{
  FAR struct igmp_group_s *group = igmp_grpfind(dev, addr);

  grpinfo("group: %p addr: %08x\n", group, (int)*addr);
  if (!group)
    {
      group = igmp_grpalloc(dev, addr);
    }

  grpinfo("group: %p\n", group);
  return group;
}

/****************************************************************************
 * Name:  igmp_grpfree
 *
 * Description:
 *   Release a previously allocated group.
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

void igmp_grpfree(FAR struct net_driver_s *dev,
                  FAR struct igmp_group_s *group)
{
  unsigned int count;
  int blresult;

  grpinfo("Free: %p flags: %02x\n", group, group->flags);

  /* Cancel the wdog */

  wd_cancel(&group->wdog);

  /* Cancel the workqueue */

  blresult = netdev_breaklock(dev, &count);
  work_cancel_sync(LPWORK, &group->work);
  if (blresult >= 0)
    {
      netdev_restorelock(dev, count);
    }

  /* Remove the group structure from the group list in the device structure */

  sq_rem((FAR sq_entry_t *)group, &dev->d_igmp_grplist);

  /* Destroy the wait semaphore */

  nxsem_destroy(&group->sem);

  /* Cancel the watchdog timer */

  wd_cancel(&group->wdog);

  /* Then release the group structure resources. */

  grpinfo("Call kmm_free()\n");
  kmm_free(group);
}

#endif /* CONFIG_NET_IGMP */
