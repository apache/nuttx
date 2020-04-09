/****************************************************************************
 *  net/ieee802154/ieee802154_container.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mm/iob.h>

#include "ieee802154/ieee802154.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The g_free_container is a list of IOB container structures that are
 * available for general use.  The number of messages in this list is a
 * system configuration item.
 *
 * Mutually exclusive access to this list is managed via the network lock:
 * i.e., the network must be locked beffore accessing this free list.
 */

static FAR struct ieee802154_container_s *g_free_container;

/* Pool of pre-allocated meta-data structures */

static struct ieee802154_container_s
  g_container_pool[CONFIG_NET_IEEE802154_NCONTAINERS];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ieee802154_container_initialize
 *
 * Description:
 *   This function initializes the container allocator.  This function must
 *   be called early in the initialization sequence before any socket
 *   activity.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called early in the initialization sequence
 *
 ****************************************************************************/

void ieee802154_container_initialize(void)
{
  FAR struct ieee802154_container_s *container;
  int i;

  /* Initialize g_free_container, the list of meta-data structures that
   * are available for allocation.
   */

  g_free_container = NULL;
  for (i = 0, container = g_container_pool;
      i < CONFIG_NET_IEEE802154_NCONTAINERS;
      i++, container++)
    {
      /* Add the next meta data structure from the pool to the list of
       * general structures.
       */

      container->ic_flink = g_free_container;
      g_free_container    = container;
    }
}

/****************************************************************************
 * Name: ieee802154_container_allocate
 *
 * Description:
 *   The ieee802154_container_allocate function will get a free container
 *   for use by the recvfrom() logic.
 *
 *   This function will first attempt to allocate from the g_free_container
 *   list.  If that the list is empty, then the meta-data structure will be
 *   allocated from the dynamic memory pool.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   A reference to the allocated container structure.
 *   All user fields in this structure have been zeroed.
 *   On a failure to allocate, NULL is returned.
 *
 * Assumptions:
 *   The caller has locked the network.
 *
 ****************************************************************************/

FAR struct ieee802154_container_s *ieee802154_container_allocate(void)
{
  FAR struct ieee802154_container_s *container;
  uint8_t pool;

  /* Try the free list first */

  net_lock();
  if (g_free_container != NULL)
    {
      container         = g_free_container;
      g_free_container  = container->ic_flink;
      pool             = IEEE802154_POOL_PREALLOCATED;
      net_unlock();
    }
  else
    {
      net_unlock();
      container = (FAR struct ieee802154_container_s *)
        kmm_malloc((sizeof (struct ieee802154_container_s)));
      pool     = IEEE802154_POOL_DYNAMIC;
    }

  /* We have successfully allocated memory from some source? */

  if (container != NULL)
    {
      /* Zero and tag the allocated meta-data structure. */

      memset(container, 0, sizeof(struct ieee802154_container_s));
      container->ic_pool = pool;
    }

  return container;
}

/****************************************************************************
 * Name: ieee802154_container_free
 *
 * Description:
 *   The ieee802154_container_free function will return a container structure
 *   to the free list of containers if it was a pre-allocated container
 *   structure. If the container structure was allocated dynamically it will
 *   be deallocated.
 *
 * Input Parameters:
 *   container - container structure to free
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The caller has locked the network.
 *
 ****************************************************************************/

void ieee802154_container_free(FAR struct ieee802154_container_s *container)
{
  /* If this is a pre-allocated meta-data structure, then just put it back
   * in the free list.
   */

  net_lock();
  if (container->ic_pool == IEEE802154_POOL_PREALLOCATED)
    {
      container->ic_flink = g_free_container;
      g_free_container    = container;
      net_unlock();
    }
  else
    {
      DEBUGASSERT(container->ic_pool == IEEE802154_POOL_DYNAMIC);

      /* Otherwise, deallocate it. */

      net_unlock();
      kmm_free(container);
    }
}
