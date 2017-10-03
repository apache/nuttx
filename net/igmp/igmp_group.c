/****************************************************************************
 * net/igmp/igmp_group.c
 * IGMP group data structure management logic
 *
 *   Copyright (C) 2010, 2013-2014, 2016 Gregory Nutt. All rights reserved.
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
 * 3. Neither the name of CITEL Technologies Ltd nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY CITEL TECHNOLOGIES AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL CITEL TECHNOLOGIES OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
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
#include <debug.h>

#include <arch/irq.h>

#include <nuttx/arch.h>
#include <nuttx/wdog.h>
#include <nuttx/kmalloc.h>
#include <nuttx/semaphore.h>
#include <nuttx/net/net.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/igmp.h>

#include "devif/devif.h"
#include "igmp/igmp.h"

#ifdef CONFIG_NET_IGMP

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifdef CONFIG_NET_IPv6
#  error "IGMP for IPv6 not supported"
#endif

/* Debug ********************************************************************/

#undef IGMP_GRPDEBUG /* Define to enable detailed IGMP group debug */

#ifndef CONFIG_NET_IGMP
#  undef IGMP_GRPDEBUG
#endif

#ifdef CONFIG_CPP_HAVE_VARARGS
#  ifdef IGMP_GRPDEBUG
#    define grperr(format, ...)    nerr(format, ##__VA_ARGS__)
#    define grpinfo(format, ...)   ninfo(format, ##__VA_ARGS__)
#  else
#    define grperr(x...)
#    define grpinfo(x...)
#  endif
#else
#  ifdef IGMP_GRPDEBUG
#    define grperr    nerr
#    define grpinfo   ninfo
#  else
#    define grperr    (void)
#    define grpinfo   (void)
#  endif
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
 ****************************************************************************/

FAR struct igmp_group_s *igmp_grpalloc(FAR struct net_driver_s *dev,
                                       FAR const in_addr_t *addr)
{
  FAR struct igmp_group_s *group;

  ninfo("addr: %08x dev: %p\n", *addr, dev);
  group = (FAR struct igmp_group_s *)kmm_zalloc(sizeof(struct igmp_group_s));

  grpinfo("group: %p\n", group);

  /* Check if we successfully allocated a group structure */

  if (group != NULL)
    {
      /* Initialize the non-zero elements of the group structure */

      net_ipv4addr_copy(group->grpaddr, *addr);

      /* This semaphore is used for signaling and, hence, should not have
       * priority inheritance enabled.
       */

      nxsem_init(&group->sem, 0, 0);
      nxsem_setprotocol(&group->sem, SEM_PRIO_NONE);

      /* Initialize the group timer (but don't start it yet) */

      group->wdog = wd_create();
      DEBUGASSERT(group->wdog);

      /* Interrupts must be disabled in order to modify the group list */

      net_lock();

      /* Add the group structure to the list in the device structure */

      sq_addfirst((FAR sq_entry_t *)group, &dev->grplist);
      net_unlock();
    }

  return group;
}

/****************************************************************************
 * Name:  igmp_grpfind
 *
 * Description:
 *   Find an existing group.
 *
 ****************************************************************************/

FAR struct igmp_group_s *igmp_grpfind(FAR struct net_driver_s *dev,
                                      FAR const in_addr_t *addr)
{
  FAR struct igmp_group_s *group;

  grpinfo("Searching for addr %08x\n", (int)*addr);

  net_lock();
  for (group = (FAR struct igmp_group_s *)dev->grplist.head;
       group;
       group = group->next)
    {
      grpinfo("Compare: %08x vs. %08x\n", group->grpaddr, *addr);
      if (net_ipv4addr_cmp(group->grpaddr, *addr))
        {
          grpinfo("Match!\n");
          break;
        }
    }

  net_unlock();
  return group;
}

/****************************************************************************
 * Name:  igmp_grpallocfind
 *
 * Description:
 *   Find an existing group.  If not found, create a new group for the
 *   address.
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
 ****************************************************************************/

void igmp_grpfree(FAR struct net_driver_s *dev, FAR struct igmp_group_s *group)
{
  grpinfo("Free: %p flags: %02x\n", group, group->flags);

  /* Cancel the wdog */

  net_lock();
  wd_cancel(group->wdog);

  /* Remove the group structure from the group list in the device structure */

  sq_rem((FAR sq_entry_t *)group, &dev->grplist);

  /* Destroy the wait semaphore */

  (void)nxsem_destroy(&group->sem);

  /* Destroy the wdog */

  wd_delete(group->wdog);

  /* Then release the group structure resources. */

  net_unlock();
  grpinfo("Call sched_kfree()\n");
  sched_kfree(group);
}

#endif /* CONFIG_NET_IGMP */

