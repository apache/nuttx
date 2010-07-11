/****************************************************************************
 * net/uip/uip_igmpgroup.c
 * IGMP group data structure management logic
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#include <stdlib.h>
#include <string.h>
#include <queue.h>
#include <debug.h>

#include <arch/irq.h>
#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>

#include <net/uip/uip.h>
#include <net/uip/uip-igmp.h>

#include "uip_internal.h"

#ifdef CONFIG_NET_IGMP

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_NET_IPv6
#  error "IGMP for IPv6 not supported"
#endif

#ifndef CONFIG_PREALLOC_IGMPGROUPS
#  define CONFIG_PREALLOC_IGMPGROUPS 4
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* malloc() cannot be called from an interrupt handler.  To work around this,
 * a small number of IGMP groups are preallocated just for use in interrupt
 * handling logic.
 */

static struct igmp_group_s g_preallocgrps[CONFIG_PREALLOC_IGMPGROUPS];
static FAR sq_queue_t g_freelist;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  uip_grpalloc
 *
 * Description:
 *   Allocate a new group from heap memory.
 *
 * Assumptions:
 *   Calls malloc and so cannot be called from an interrupt handler.
 *
 ****************************************************************************/

static inline FAR struct igmp_group_s *uip_grpheapalloc(void)
{
  return (FAR struct igmp_group_s *)zalloc(sizeof(struct igmp_group_s));
}

/****************************************************************************
 * Name:  uip_grpprealloc
 *
 * Description:
 *   Allocate a new group from the pre-allocated groups.
 *
 * Assumptions:
 *   This function should only be called from an interrupt handler (or with
 *   interrupts disabled).
 *
 ****************************************************************************/

static inline FAR struct igmp_group_s *uip_grpprealloc(void)
{
  FAR struct igmp_group_s *group = (FAR struct igmp_group_s *)sq_remfirst(&g_freelist);
  if (group)
    {
      memset(group, 0, sizeof(struct igmp_group_s));
      group->flags = IGMP_PREALLOCATED;
    }
  return group;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  uip_grpinit
 *
 * Description:
 *   One-time initialization of group data.
 *
 * Assumptions:
 *   Called only during early boot phases (pre-multitasking).
 *
 ****************************************************************************/

void uip_grpinit(void)
{
  FAR struct igmp_group_s *group;
  int i;

  for (i = 0; i < CONFIG_PREALLOC_IGMPGROUPS; i++)
    {
      group = &g_preallocgrps[i];
      sq_addfirst((FAR sq_entry_t *)group, &g_freelist);
    }
}

/****************************************************************************
 * Name:  uip_grpalloc
 *
 * Description:
 *   Allocate a new group from heap memory.
 *
 * Assumptions:
 *   May be called from either user or interrupt level processing.
 *
 ****************************************************************************/

FAR struct igmp_group_s *uip_grpalloc(FAR struct uip_driver_s *dev,
                                      FAR const uip_ipaddr_t *addr)
{
  FAR struct igmp_group_s *group;
  irqstate_t flags;

  nllvdbg("addr: %08x dev: %p\n", *addr, dev);
  if (up_interrupt_context())
    {
      group = uip_grpheapalloc();
    }
  else
    {
      group = uip_grpprealloc();
    }

  /* Check if we succesfully allocated a group structure */

  if (group)
    {
      /* Initialize the non-zero elements of the group structure */

      uip_ipaddr_copy(group->grpaddr, addr);
      sem_init(&group->sem, 0, 0);

      /* Interrupts must be disabled in order to modify the group list */

      flags = irqsave();

      /* Add the group structure to the list in the device structure */

      sq_addfirst((FAR sq_entry_t*)group, &dev->grplist);
      irqrestore(flags);
    }
  return group;
}

/****************************************************************************
 * Name:  uip_grpfind
 *
 * Description:
 *   Find an existing group.
 *
 * Assumptions:
 *   May be called from either user or interrupt level processing.
 *
 ****************************************************************************/

FAR struct igmp_group_s *uip_grpfind(FAR struct uip_driver_s *dev,
                                     FAR const uip_ipaddr_t *addr)
{
  FAR struct igmp_group_s *group;
  irqstate_t flags;

  /* We must disable interrupts because we don't which context we were called
   * from.
   */

  flags = irqsave();
  for (group = (FAR struct igmp_group_s *)dev->grplist.head; group; group = group->next)
    {
      if (uip_ipaddr_cmp(&group->grpaddr, addr))
        {
          break;
        }
    }
  irqrestore(flags);
  return group;
}

/****************************************************************************
 * Name:  uip_grpallocfind
 *
 * Description:
 *   Find an existing group.  If not found, create a new group for the
 *   address.
 *
 * Assumptions:
 *   May be called from either user or interrupt level processing.
 *
 ****************************************************************************/

FAR struct igmp_group_s *uip_grpallocfind(FAR struct uip_driver_s *dev,
                                          FAR const uip_ipaddr_t *addr)
{
  FAR struct igmp_group_s *group = uip_grpfind(dev, addr);
  if (!group)
    {
      group = uip_grpalloc(dev, addr);
    }
  return group;
}

/****************************************************************************
 * Name:  uip_grpfree
 *
 * Description:
 *   Release a previously allocated group.
 *
 * Assumptions:
 *   May be called from either user or interrupt level processing.
 *
 ****************************************************************************/

void uip_grpfree(FAR struct uip_driver_s *dev, FAR struct igmp_group_s *group)
{
  /* First, remove the group structure from the group list in the device
   * structure
   */

  irqstate_t flags = irqsave();
  sq_rem((FAR sq_entry_t*)group, &dev->grplist);
  
  /* Destroy the wait semapore */

  (void)sem_destroy(&group->sem);
  
  /* Then release the group structure resources.  Check first if this is one
   * of the pre-allocated group structures that we will retain in a free list.
   */

  if (IS_PREALLOCATED(group->flags))
    {
      sq_addlast((FAR sq_entry_t*)group, &g_freelist);
      irqrestore(flags);
    }
  else
    {
      /* No.. deallocate the group structure.  Use sched_free() just in case
       * this function is executing within an interrupt handler.
       */

      irqrestore(flags);
      sched_free(group);
    }
}

#endif /* CONFIG_NET_IGMP */
