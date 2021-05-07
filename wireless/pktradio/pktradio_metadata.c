/****************************************************************************
 * wireless/pktradio/pktradio_metadata.c
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
#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mm/iob.h>
#include <nuttx/semaphore.h>

#include <nuttx/wireless/pktradio.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The g_free_metadata is a list of meta-data structures that are available
 * for general use.  The number of messages in this list is a system
 * configuration item.
 */

static FAR struct pktradio_metadata_s *g_free_metadata;

/* Supports mutually exclusive access to the free list */

static sem_t g_metadata_sem;

/* Idempotence support */

static bool g_metadata_initialized;

/* Pool of pre-allocated meta-data structures */

static struct pktradio_metadata_s g_metadata_pool[CONFIG_PKTRADIO_NRXMETA];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pktradio_metadata_initialize
 *
 * Description:
 *   This function initializes the meta-data allocator.  This function must
 *   be called early in the initialization sequence before any radios
 *   begin operation.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void pktradio_metadata_initialize(void)
{
  FAR struct pktradio_metadata_s *metadata;
  int i;

  if (!g_metadata_initialized)
    {
      /* Initialize g_free_metadata, the list of meta-data structures that
       * are available for allocation.
       */

      g_free_metadata = NULL;
      for (i = 0, metadata = g_metadata_pool;
          i < CONFIG_PKTRADIO_NRXMETA;
          i++, metadata++)
        {
          /* Add the next meta data structure from the pool to the list of
           * general structures.
           */

          metadata->pm_flink = g_free_metadata;
          g_free_metadata    = metadata;
        }

      /* Initialize the mutual exclusion semaphore */

      nxsem_init(&g_metadata_sem, 0, 1);
      g_metadata_initialized = true;
    }
}

/****************************************************************************
 * Name: pktradio_metadata_allocate
 *
 * Description:
 *   The pktradio_metadata_allocate function will get a free meta-data
 *   structure for use by the packet radio.
 *
 *   This function will first attempt to allocate from the g_free_metadata
 *   list.  If that the list is empty, then the meta-data structure will be
 *   allocated from the dynamic memory pool.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   A reference to the allocated metadata structure.  All user fields in
 *   this structure have been zeroed.  On a failure to allocate, NULL is
 *   returned.
 *
 ****************************************************************************/

FAR struct pktradio_metadata_s *pktradio_metadata_allocate(void)
{
  FAR struct pktradio_metadata_s *metadata;
  uint8_t pool;

  /* Get exclusive access to the free list */

  nxsem_wait_uninterruptible(&g_metadata_sem);

  /* Try the free list first */

  if (g_free_metadata != NULL)
    {
      metadata         = g_free_metadata;
      g_free_metadata  = metadata->pm_flink;
      pool             = PKTRADIO_POOL_PREALLOCATED;

      /* We are finished with the free list */

      nxsem_post(&g_metadata_sem);
    }
  else
    {
      /* If we cannot get a meta-data instance from the free list, then we
       * will have to allocate one from the kernel memory pool.  We won't
       * access the free list.
       */

      nxsem_post(&g_metadata_sem);

      metadata = (FAR struct pktradio_metadata_s *)
        kmm_malloc((sizeof (struct pktradio_metadata_s)));
      pool     = PKTRADIO_POOL_DYNAMIC;
    }

  /* We have successfully allocated memory from some source? */

  if (metadata != NULL)
    {
      /* Zero and tag the allocated meta-data structure. */

      memset(metadata, 0, sizeof(struct pktradio_metadata_s));
      metadata->pm_pool = pool;
    }

  return metadata;
}

/****************************************************************************
 * Name: pktradio_metadata_free
 *
 * Description:
 *   The pktradio_metadata_free function will return a metadata structure
 *   to the free list of  messages if it was a pre-allocated metadata
 *   structure. If the metadata structure was allocated dynamically it will
 *   be deallocated.
 *
 * Input Parameters:
 *   metadata - metadata structure to free
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void pktradio_metadata_free(FAR struct pktradio_metadata_s *metadata)
{
  /* Get exclusive access to the free list */

  nxsem_wait_uninterruptible(&g_metadata_sem);

  /* If this is a pre-allocated meta-data structure, then just put it back
   * in the free list.
   */

  if (metadata->pm_pool == PKTRADIO_POOL_PREALLOCATED)
    {
      metadata->pm_flink = g_free_metadata;
      g_free_metadata    = metadata;

      /* We are finished with the free list */

      nxsem_post(&g_metadata_sem);
    }
  else
    {
      DEBUGASSERT(metadata->pm_pool == PKTRADIO_POOL_DYNAMIC);

      /* Otherwise, deallocate it.  We won't access the free list */

      nxsem_post(&g_metadata_sem);
      kmm_free(metadata);
    }
}
