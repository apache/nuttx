/****************************************************************************
 *  wireless/pktradio/pktradio_metadata.c
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
#include <semaphore.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mm/iob.h>

#include <nuttx/wireless/pktradio.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The g_free_metadata is a list of meta-data structures that are available for
 * general use.  The number of messages in this list is a system configuration
 * item.
 */

static FAR struct pktradio_metadata_s *g_free_metadata;

/* Supports mutually exclusive access to the free list */

static sem_t g_metadata_sem;

/* Idempotence support */

static bool g_metadata_initialized;

/* Pool of pre-allocated meta-data stuctures */

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
 *   A reference to the allocated metadata structure.  All user fields in this
 *   structure have been zeroed.  On a failure to allocate, NULL is
 *   returned.
 *
 ****************************************************************************/

FAR struct pktradio_metadata_s *pktradio_metadata_allocate(void)
{
  FAR struct pktradio_metadata_s *metadata;
  uint8_t pool;
  int ret;

  /* Get exclusive access to the free list */

  do
    {
      /* Take the semaphore (perhaps waiting) */

      ret = nxsem_wait(&g_metadata_sem);

      /* The only case that an error should occur here is if the wait was
       * awakened by a signal.
       */

      DEBUGASSERT(ret == OK || ret == -EINTR);
    }
  while (ret == -EINTR);

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
       * will have to allocate one from the kernal memory pool.  We won't
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
  int ret;

  /* Get exclusive access to the free list */

  do
    {
      /* Take the semaphore (perhaps waiting) */

      ret = nxsem_wait(&g_metadata_sem);

      /* The only case that an error should occur here is if the wait was
       * awakened by a signal.
       */

      DEBUGASSERT(ret == OK || ret == -EINTR);
    }
  while (ret == -EINTR);

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
      sched_kfree(metadata);
    }
}
