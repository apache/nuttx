/****************************************************************************
 * net/devif/devif_callback.c
 *
 *   Copyright (C) 2008-2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#if defined(CONFIG_NET)

#include <stdint.h>
#include <string.h>
#include <debug.h>

#include <nuttx/net/netconfig.h>
#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>

#include "devif/devif.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct devif_callback_s g_cbprealloc[CONFIG_NET_NACTIVESOCKETS];
static FAR struct devif_callback_s *g_cbfreelist = NULL;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: devif_callback_init
 *
 * Description:
 *   Configure the pre-allocated callback structures into a free list.
 *   This is called internally as part of uIP initialization and should not
 *   be accessed from the application or socket layer.
 *
 * Assumptions:
 *   This function is called with interrupts disabled.
 *
 ****************************************************************************/

void devif_callback_init(void)
{
  int i;
  for (i = 0; i < CONFIG_NET_NACTIVESOCKETS; i++)
    {
      g_cbprealloc[i].flink = g_cbfreelist;
      g_cbfreelist          = &g_cbprealloc[i];
    }
}

/****************************************************************************
 * Function: devif_callback_alloc
 *
 * Description:
 *   Allocate a callback container from the free list.
 *   This is called internally as part of uIP initialization and should not
 *   be accessed from the application or socket layer.
 *
 * Assumptions:
 *   This function is called with interrupts disabled.
 *
 ****************************************************************************/

FAR struct devif_callback_s *devif_callback_alloc(FAR struct devif_callback_s **list)
{
  FAR struct devif_callback_s *ret;
  net_lock_t save;

  /* Check  the head of the free list */

  save = net_lock();
  ret  = g_cbfreelist;
  if (ret)
    {
      /* Remove the next instance from the head of the free list */

      g_cbfreelist = ret->flink;
      memset(ret, 0, sizeof(struct devif_callback_s));

      /* Add the newly allocated instance to the head of the specified list */

      if (list)
        {
           ret->flink = *list;
           *list      = ret;
        }
      else
        {
           ret->flink   = NULL;
        }
    }
#ifdef CONFIG_DEBUG
  else
    {
      nlldbg("Failed to allocate callback\n");
    }
#endif

  net_unlock(save);
  return ret;
}

/****************************************************************************
 * Function: devif_callback_free
 *
 * Description:
 *   Return a callback container to the free list.
 *   This is called internally as part of uIP initialization and should not
 *   be accessed from the application or socket layer.
 *
 * Assumptions:
 *   This function is called with interrupts disabled.
 *
 ****************************************************************************/

void devif_callback_free(FAR struct devif_callback_s *cb,
                         FAR struct devif_callback_s **list)
{
  FAR struct devif_callback_s *prev;
  FAR struct devif_callback_s *curr;
  net_lock_t save;

  if (cb)
    {
      save = net_lock();

#ifdef CONFIG_DEBUG
      /* Check for double freed callbacks */

      curr = g_cbfreelist;

      while (curr != NULL)
        {
          DEBUGASSERT(cb != curr);
          curr = curr->flink;
        }
#endif

      /* Find the callback structure in the connection's list */

      if (list)
        {
          for (prev = NULL, curr = *list;
               curr && curr != cb;
               prev = curr, curr = curr->flink);

          /* Remove the structure from the connection's list */

          if (curr)
            {
              if (prev)
                {
                  prev->flink = cb->flink;
                }
              else
                {
                  *list = cb->flink;
                }
            }
        }

      /* Put the structure into the free list */

      cb->flink    = g_cbfreelist;
      g_cbfreelist = cb;
      net_unlock(save);
    }
}

/****************************************************************************
 * Function: devif_callback_execute
 *
 * Description:
 *   Execute a list of callbacks.
 *   This is called internally as part of uIP initialization and should not
 *   be accessed from the application or socket layer.
 *
 * Assumptions:
 *   This function is called with interrupts disabled.
 *
 ****************************************************************************/

uint16_t devif_callback_execute(FAR struct net_driver_s *dev, void *pvconn,
                                uint16_t flags, FAR struct devif_callback_s *list)
{
  FAR struct devif_callback_s *next;
  net_lock_t save;

  /* Loop for each callback in the list and while there are still events
   * set in the flags set.
   */

  save = net_lock();
  while (list && flags)
    {
      /* Save the pointer to the next callback in the lists.  This is done
       * because the callback action might delete the entry pointed to by
       * list.
       */

      next = list->flink;

      /* Check if this callback handles any of the events in the flag set */

      if (list->event && (flags & list->flags) != 0)
        {
          /* Yes.. perform the callback.  Actions perform by the callback
           * may delete the current list entry or add a new list entry to
           * beginning of the list (which will be ignored on this pass)
           */

          nllvdbg("Call event=%p with flags=%04x\n", list->event, flags);
          flags = list->event(dev, pvconn, list->priv, flags);
        }

      /* Set up for the next time through the loop */

      list = next;
    }

  net_unlock(save);
  return flags;
}

#endif /* CONFIG_NET */
