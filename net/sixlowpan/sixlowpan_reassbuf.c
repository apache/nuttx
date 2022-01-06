/****************************************************************************
 * net/sixlowpan/sixlowpan_reassbuf.c
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

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mm/iob.h>

#include "sixlowpan_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Re-assembly timeout in clock ticks */

#define NET_6LOWPAN_TIMEOUT SEC2TICK(CONFIG_NET_6LOWPAN_MAXAGE)

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The g_free_reass is a list of reassembly buffer structures that are
 * available for general use.  The number of messages in this list is a
 * system configuration item.  Protected only by the network lock.
 */

static FAR struct sixlowpan_reassbuf_s *g_free_reass;

/* This is a list of active, allocated reassemby buffers */

static FAR struct sixlowpan_reassbuf_s *g_active_reass;

/* Pool of pre-allocated reassembly buffer structures */

static struct sixlowpan_reassbuf_s
              g_metadata_pool[CONFIG_NET_6LOWPAN_NREASSBUF];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sixlowpan_compare_fragsrc
 *
 * Description:
 *   Check if the fragment that we just received is from the same source as
 *   the previously received fragments.
 *
 * Input Parameters:
 *   radio    - Radio network device driver state instance
 *   fragsrc  - The source address of the fragment.
 *
 * Returned Value:
 *   true if the sources are the same.
 *
 ****************************************************************************/

static bool sixlowpan_compare_fragsrc(FAR struct sixlowpan_reassbuf_s *reass,
                                  FAR const struct netdev_varaddr_s *fragsrc)
{
  /* The addresses cannot match if they are not the same size */

  if (fragsrc->nv_addrlen == reass->rb_fragsrc.nv_addrlen)
    {
      /* The are the same size, return the address comparison */

      return (memcmp(fragsrc->nv_addr, reass->rb_fragsrc.nv_addr,
                     fragsrc->nv_addrlen) == 0);
    }

  return false;
}

/****************************************************************************
 * Name: sixlowpan_reass_expire
 *
 * Description:
 *   Free all expired or inactive reassembly buffers.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static void sixlowpan_reass_expire(void)
{
  FAR struct sixlowpan_reassbuf_s *reass;
  FAR struct sixlowpan_reassbuf_s *next;
  clock_t elapsed;

  /* If reassembly timed out, cancel it */

  for (reass = g_active_reass; reass != NULL; reass = next)
    {
      /* Needed if 'reass' is freed */

      next = reass->rb_flink;

      /* Free any inactive reassembly buffers.  This is done because the life
       * the reassembly buffer is not cerain.
       */

      if (!reass->rb_active)
        {
          sixlowpan_reass_free(reass);
        }
      else
        {
          /* Get the elpased time of the reassembly */

          elapsed = clock_systime_ticks() - reass->rb_time;

          /* If the reassembly has expired, then free the reassembly buffer */

          if (elapsed >= NET_6LOWPAN_TIMEOUT)
            {
              nwarn("WARNING: Reassembly timed out\n");
              sixlowpan_reass_free(reass);
            }
        }
    }
}

/****************************************************************************
 * Name: sixlowpan_remove_active
 *
 * Description:
 *   Remove a reassembly buffer from the active reassembly buffer list.
 *
 * Input Parameters:
 *   reass - The reassembly buffer to be removed.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static void sixlowpan_remove_active(FAR struct sixlowpan_reassbuf_s *reass)
{
  FAR struct sixlowpan_reassbuf_s *curr;
  FAR struct sixlowpan_reassbuf_s *prev;

  /* Find the reassembly buffer in the list of active reassembly buffers */

  for (prev = NULL, curr = g_active_reass;
       curr != NULL && curr != reass;
       prev = curr, curr = curr->rb_flink)
    {
    }

  /* Did we find it? */

  if (curr != NULL)
    {
      /* Yes.. remove it from the active reassembly buffer list */

      if (prev == NULL)
        {
          g_active_reass = reass->rb_flink;
        }
      else
        {
          prev->rb_flink = reass->rb_flink;
        }
    }

  reass->rb_flink = NULL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sixlowpan_reass_initialize
 *
 * Description:
 *   This function initializes the reassembly buffer allocator.  This
 *   function must be called early in the initialization sequence before
 *   any radios begin operation.
 *
 *   Called only once during network initialization.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sixlowpan_reass_initialize(void)
{
  FAR struct sixlowpan_reassbuf_s *reass;
  int i;

  /* Initialize g_free_reass, the list of reassembly buffer structures that
   * are available for allocation.
   */

  g_free_reass = NULL;
  for (i = 0, reass = g_metadata_pool;
       i < CONFIG_NET_6LOWPAN_NREASSBUF;
       i++, reass++)
    {
      /* Add the next meta data structure from the pool to the list of
       * general structures.
       */

      reass->rb_flink = g_free_reass;
      g_free_reass    = reass;
    }
}

/****************************************************************************
 * Name: sixlowpan_reass_allocate
 *
 * Description:
 *   The sixlowpan_reass_allocate function will get a free reassembly buffer
 *   structure for use by 6LoWPAN.
 *
 *   This function will first attempt to allocate from the g_free_reass
 *   list.  If that the list is empty, then the reassembly buffer structure
 *   will be allocated from the dynamic memory pool.
 *
 * Input Parameters:
 *   reasstag - The reassembly tag for subsequent lookup.
 *   fragsrc  - The source address of the fragment.
 *
 * Returned Value:
 *   A reference to the allocated reass structure.  All fields used by the
 *   reasembly logic have been zeroed.  On a failure to allocate, NULL is
 *   returned.
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

FAR struct sixlowpan_reassbuf_s *
  sixlowpan_reass_allocate(uint16_t reasstag,
                           FAR const struct netdev_varaddr_s *fragsrc)
{
  FAR struct sixlowpan_reassbuf_s *reass;
  uint8_t pool;

  /* First, removed any expired or inactive reassembly buffers.  This might
   * free up a pre-allocated buffer for this allocation.
   */

  sixlowpan_reass_expire();

  /* Now, try the free list first */

  if (g_free_reass != NULL)
    {
      reass         = g_free_reass;
      g_free_reass  = reass->rb_flink;
      pool          = REASS_POOL_PREALLOCATED;
    }
  else
    {
#ifdef CONFIG_NET_6LOWPAN_REASS_STATIC
      reass         = NULL;
#else
      /* If we cannot get a reassembly buffer instance from the free list,
       * then we will have to allocate one from the kernel memory pool.
       */

      reass = (FAR struct sixlowpan_reassbuf_s *)
        kmm_malloc((sizeof (struct sixlowpan_reassbuf_s)));
      pool  = REASS_POOL_DYNAMIC;
#endif
    }

  /* We have successfully allocated memory from some source? */

  if (reass != NULL)
    {
      /* Zero and tag the allocated reassembly buffer structure. */

      memset(reass, 0, sizeof(struct sixlowpan_reassbuf_s));
      memcpy(&reass->rb_fragsrc, fragsrc, sizeof(struct netdev_varaddr_s));
      reass->rb_pool     = pool;
      reass->rb_active   = true;
      reass->rb_reasstag = reasstag;
      reass->rb_time     = clock_systime_ticks();

      /* Add the reassembly buffer to the list of active reassembly buffers */

      reass->rb_flink   = g_active_reass;
      g_active_reass    = reass;
    }

  return reass;
}

/****************************************************************************
 * Name: sixlowpan_reass_find
 *
 * Description:
 *   Find a previously allocated, active reassembly buffer with the specified
 *   reassembly tag.
 *
 * Input Parameters:
 *   reasstag - The reassembly tag to match.
 *   fragsrc  - The source address of the fragment.
 *
 * Returned Value:
 *   A reference to the matching reass structure.
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

FAR struct sixlowpan_reassbuf_s *
  sixlowpan_reass_find(uint16_t reasstag,
                       FAR const struct netdev_varaddr_s *fragsrc)
{
  FAR struct sixlowpan_reassbuf_s *reass;

  /* First, removed any expired or inactive reassembly buffers (we don't want
   * to return old reassembly buffer with the same tag)
   */

  sixlowpan_reass_expire();

  /* Now search for the matching reassembly buffer in the remainng, active
   * reassembly buffers.
   */

  for (reass = g_active_reass; reass != NULL; reass = reass->rb_flink)
    {
      /* In order to be a match, it must have the same reassembly tag as
       * well as source address (different sources might use the same
       * reassembly tag).
       */

      if (reass->rb_reasstag == reasstag &&
          sixlowpan_compare_fragsrc(reass, fragsrc))
        {
          return reass;
        }
    }

  /* Not found */

  return NULL;
}

/****************************************************************************
 * Name: sixlowpan_reass_free
 *
 * Description:
 *   The sixlowpan_reass_free function will return a reass structure
 *   to the free list of  messages if it was a pre-allocated reass
 *   structure. If the reass structure was allocated dynamically it will
 *   be deallocated.
 *
 * Input Parameters:
 *   reass - reass structure to free
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

void sixlowpan_reass_free(FAR struct sixlowpan_reassbuf_s *reass)
{
  /* First, remove the reassembly buffer from the list of active reassembly
   * buffers.
   */

  sixlowpan_remove_active(reass);

  /* If this is a pre-allocated reassembly buffer structure, then just put it
   * back in the free list.
   */

  if (reass->rb_pool == REASS_POOL_PREALLOCATED)
    {
      reass->rb_flink = g_free_reass;
      g_free_reass    = reass;
    }
  else if (reass->rb_pool == REASS_POOL_DYNAMIC)
    {
#ifdef CONFIG_NET_6LOWPAN_REASS_STATIC
      DEBUGPANIC();
#else
      DEBUGASSERT(reass->rb_pool == REASS_POOL_DYNAMIC);

      /* Otherwise, deallocate it. */

      kmm_free(reass);
#endif
    }

  /* If the reassembly buffer structure was provided by the driver, nothing
   * needs to be freed.
   */
}
