/****************************************************************************
 *  wireless/ieee802154/ieee802154_indalloc.c
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
#include <string.h>

#include <nuttx/kmalloc.h>

#include <nuttx/mm/iob.h>

#include <nuttx/wireless/ieee802154/ieee802154_mac.h>

#include "mac802154.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* NOTE:  The CONFIG_IEEE802154_IND_IRQRESERVE options is marked as marked
 * 'experimental' and with the default 0 zero because there are no interrupt
 * level allocations performed by the current IEEE 802.15.4 MAC code.
 */

#if !defined(CONFIG_IEEE802154_IND_PREALLOC) || \
    CONFIG_IEEE802154_IND_PREALLOC < 0
#  undef CONFIG_IEEE802154_IND_PREALLOC
#  define CONFIG_IEEE802154_IND_PREALLOC 20
#endif

#if !defined(CONFIG_IEEE802154_IND_IRQRESERVE) || \
    CONFIG_IEEE802154_IND_IRQRESERVE < 0
#  undef CONFIG_IEEE802154_IND_IRQRESERVE
#  define CONFIG_IEEE802154_IND_IRQRESERVE 0
#endif

#if CONFIG_IEEE802154_IND_IRQRESERVE > CONFIG_IEEE802154_IND_PREALLOC
#  undef CONFIG_IEEE802154_IND_IRQRESERVE
#  define CONFIG_IEEE802154_IND_IRQRESERVE CONFIG_IEEE802154_IND_PREALLOC
#endif

/* Memory Pools */

#define POOL_IND_GENERAL  0
#define POOL_IND_IRQ      1
#define POOL_IND_DYNAMIC  2

/****************************************************************************
 * Private Data Types
 ****************************************************************************/

/* Private data type that extends the ieee802154_data_ind_s struct */

struct ieee802154_priv_ind_s
{
  /* Must be first member so we can cast to/from */

  struct ieee802154_data_ind_s pub;
  FAR struct ieee802154_priv_ind_s *flink;
  uint8_t pool;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if CONFIG_IEEE802154_IND_PREALLOC > 0
#if CONFIG_IEEE802154_IND_PREALLOC > CONFIG_IEEE802154_IND_IRQRESERVE
/* The g_indfree is a list of meta-data structures that are available for
 * general use.  The number of messages in this list is a system configuration
 * item.
 */

static struct ieee802154_priv_ind_s *g_indfree;
#endif

#if CONFIG_IEEE802154_IND_IRQRESERVE > 0
/* The g_indfree_irq is a list of meta-data structures that are reserved for
 * use by only by interrupt handlers.
 */

static struct ieee802154_priv_ind_s *g_indfree_irq;
#endif

/* Pool of pre-allocated meta-data stuctures */

static struct ieee802154_priv_ind_s g_indpool[CONFIG_IEEE802154_IND_PREALLOC];
#endif /* CONFIG_IEEE802154_IND_PREALLOC > 0 */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ieee802154_indpool_initialize
 *
 * Description:
 *   This function initializes the meta-data allocator.  This function must
 *   be called early in the initialization sequence before any radios
 *   begin operation.
 *
 * Inputs:
 *   None
 *
 * Return Value:
 *   None
 *
 ****************************************************************************/

void ieee802154_indpool_initialize(void)
{
#if CONFIG_IEEE802154_IND_PREALLOC > 0
  FAR struct ieee802154_priv_ind_s *pool = g_indpool;
  int remaining = CONFIG_IEEE802154_IND_PREALLOC;

#if CONFIG_IEEE802154_IND_PREALLOC > CONFIG_IEEE802154_IND_IRQRESERVE
  /* Initialize g_indfree, thelist of meta-data structures that are available
   * for general use.
   */

  g_indfree = NULL;
  while (remaining > CONFIG_IEEE802154_IND_IRQRESERVE)
    {
      FAR struct ieee802154_priv_ind_s *ind = pool;

      /* Add the next meta data structure from the pool to the list of
       * general structures.
       */

      ind->flink = g_indfree;
      g_indfree  = ind;

      /* Set up for the next structure from the pool */

      pool++;
      remaining--;
    }
#endif

#if CONFIG_IEEE802154_IND_IRQRESERVE > 0
  /* Initialize g_indfree_irq is a list of meta-data structures reserved for
   * use by only by interrupt handlers.
   */

  g_indfree_irq = NULL;
  while (remaining > 0)
    {
      FAR struct ieee802154_priv_ind_s *ind = pool;

      /* Add the next meta data structure from the pool to the list of
       * general structures.
       */

      ind->flink    = g_indfree_irq;
      g_indfree_irq = ind;

      /* Set up for the next structure from the pool */

      pool++;
      remaining--;
    }
#endif
#endif /* CONFIG_IEEE802154_IND_PREALLOC > 0 */
}

/****************************************************************************
 * Name: ieee802154_ind_allocate
 *
 * Description:
 *   The ieee802154_ind_allocate function will get a free meta-data
 *   structure for use by the IEEE 802.15.4 MAC.
 *
 *   Interrupt handling logic will first attempt to allocate from the
 *   g_indfree list.  If that list is empty, it will attempt to allocate
 *   from its reserve, g_indfree_irq.  If that list is empty, then the
 *   allocation fails (NULL is returned).
 *
 *   Non-interrupt handler logic will attempt to allocate from g_indfree
 *   list.  If that the list is empty, then the meta-data structure will be
 *   allocated from the dynamic memory pool.
 *
 * Inputs:
 *   None
 *
 * Return Value:
 *   A reference to the allocated msg structure.  All user fields in this
 *   structure have been zeroed.  On a failure to allocate, NULL is
 *   returned.
 *
 ****************************************************************************/

FAR struct ieee802154_data_ind_s *ieee802154_ind_allocate(void)
{
#if CONFIG_IEEE802154_IND_PREALLOC > 0
  FAR struct ieee802154_priv_ind_s *ind;
  irqstate_t flags;
  uint8_t pool;

  /* If we were called from an interrupt handler, then try to get the meta-
   * data structure from generally available list of messages. If this fails,
   * then try the list of messages reserved for interrupt handlers
   */

  flags = enter_critical_section(); /* Always necessary in SMP mode */
  if (up_interrupt_context())
    {
#if CONFIG_IEEE802154_IND_PREALLOC > CONFIG_IEEE802154_IND_IRQRESERVE
      /* Try the general free list */

      if (g_indfree != NULL)
        {
          ind           = g_indfree;
          g_indfree     = ind->flink;

          leave_critical_section(flags);
          pool          = POOL_IND_GENERAL;
        }
      else
#endif
#if CONFIG_IEEE802154_IND_IRQRESERVE > 0
      /* Try the list list reserved for interrupt handlers */

      if (g_indfree_irq != NULL)
        {
          ind           = g_indfree_irq;
          g_indfree_irq = ind->flink;

          leave_critical_section(flags);
          pool          = POOL_IND_IRQ;
        }
      else
#endif
        {
          leave_critical_section(flags);
          return NULL;
        }
    }

  /* We were not called from an interrupt handler. */

  else
    {
#if CONFIG_IEEE802154_IND_PREALLOC > CONFIG_IEEE802154_IND_IRQRESERVE
      /* Try the general free list */

      if (g_indfree != NULL)
        {
          ind           = g_indfree;
          g_indfree     = ind->flink;

          leave_critical_section(flags);
          pool          = POOL_IND_GENERAL;
        }
      else
#endif
        {
          /* If we cannot a meta-data structure from the free list, then we
           * will have to allocate one from the kernal memory pool.
           */

          leave_critical_section(flags);
          ind = (FAR struct ieee802154_priv_ind_s *)
            kmm_malloc((sizeof (struct ieee802154_priv_ind_s)));

          /* Check if we allocated the meta-data structure */

          if (ind != NULL)
            {
              /* Yes... remember that this meta-data structure was dynamically allocated */

              pool = POOL_IND_DYNAMIC;
            }
        }
    }

  /* We have successfully allocated memory from some source.
   * Zero and tag the alloated meta-data structure.
   */

  ind->pool = pool;
  memset(&ind->pub, 0, sizeof(struct ieee802154_data_ind_s));

  /* Allocate the IOB for the frame */

  ind->pub.frame = iob_alloc(true);
  if (ind->pub.frame == NULL)
    {
      /* Deallocate the ind */

      ieee802154_ind_free(&ind->pub);

      return NULL;
    }

  ind->pub.frame->io_flink  = NULL;
  ind->pub.frame->io_len    = 0;
  ind->pub.frame->io_offset = 0;
  ind->pub.frame->io_pktlen = 0;

  return &ind->pub;
#else
  return NULL;
#endif
}

/****************************************************************************
 * Name: ieee802154_ind_free
 *
 * Description:
 *   The ieee802154_ind_free function will return a meta-data structure to
 *   the free pool of  messages if it was a pre-allocated meta-data
 *   structure. If the meta-data structure was allocated dynamically it will
 *   be deallocated.
 *
 * Inputs:
 *   ind - meta-data structure to free
 *
 * Return Value:
 *   None
 *
 ****************************************************************************/

void ieee802154_ind_free(FAR struct ieee802154_data_ind_s *ind)
{
#if CONFIG_IEEE802154_IND_PREALLOC > 0
  irqstate_t flags;
  FAR struct ieee802154_priv_ind_s *priv =
    (FAR struct ieee802154_priv_ind_s *)ind;

  /* Check if the IOB is not NULL. The only time it should be NULL is if we
   * allocated the data_ind, but the IOB allocation failed so we now have to
   * free the data_ind but not the IOB. This really should happen rarely if at all.
   */

  if (ind->frame != NULL)
    {
      iob_free(ind->frame);
    }

#if CONFIG_IEEE802154_IND_PREALLOC > CONFIG_IEEE802154_IND_IRQRESERVE
  /* If this is a generally available pre-allocated meta-data structure,
   * then just put it back in the free list.
   */

  if (priv->pool == POOL_IND_GENERAL)
    {
      /* Make sure we avoid concurrent access to the free
       * list from interrupt handlers.
       */

      flags = enter_critical_section();
      priv->flink = g_indfree;
      g_indfree  = priv;
      leave_critical_section(flags);
    }
  else
#endif

#if CONFIG_IEEE802154_IND_IRQRESERVE > 0
  /* If this is a meta-data structure pre-allocated for interrupts,
   * then put it back in the correct free list.
   */

  if (priv->pool == POOL_IND_IRQ)
    {
      /* Make sure we avoid concurrent access to the free
       * list from interrupt handlers.
       */

      flags = enter_critical_section();
      priv->flink    = g_indfree_irq;
      g_indfree_irq  = priv;
      leave_critical_section(flags);
    }
  else
#endif

    {
      /* Otherwise, deallocate it. */

      DEBUGASSERT(priv->pool == POOL_IND_DYNAMIC);
      sched_kfree(priv);
    }
#endif
}
