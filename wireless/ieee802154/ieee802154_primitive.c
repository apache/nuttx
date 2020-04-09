/****************************************************************************
 *  wireless/ieee802154/ieee802154_primitive.c
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
#include <nuttx/wireless/ieee802154/ieee802154_mac.h>

#include "mac802154.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* NOTE:  The CONFIG_IEEE802154_PRIMITIVE_IRQRESERVE options is marked as
 * marked 'experimental' and with the default 0 zero because there are no
 * interrupt level allocations performed by the current IEEE 802.15.4 MAC
 * code.
 */

#if !defined(CONFIG_IEEE802154_PRIMITIVE_PREALLOC) || \
    CONFIG_IEEE802154_PRIMITIVE_PREALLOC < 0
#  undef CONFIG_IEEE802154_PRIMITIVE_PREALLOC
#  define CONFIG_IEEE802154_PRIMITIVE_PREALLOC 20
#endif

#if !defined(CONFIG_IEEE802154_PRIMITIVE_IRQRESERVE) || \
    CONFIG_IEEE802154_PRIMITIVE_IRQRESERVE < 0
#  undef CONFIG_IEEE802154_PRIMITIVE_IRQRESERVE
#  define CONFIG_IEEE802154_PRIMITIVE_IRQRESERVE 0
#endif

#if CONFIG_IEEE802154_PRIMITIVE_IRQRESERVE > CONFIG_IEEE802154_PRIMITIVE_PREALLOC
#  undef CONFIG_IEEE802154_PRIMITIVE_IRQRESERVE
#  define CONFIG_IEEE802154_PRIMITIVE_IRQRESERVE CONFIG_IEEE802154_PRIMITIVE_PREALLOC
#endif

/* Memory Pools */

#define POOL_PRIMITIVE_GENERAL  0
#define POOL_PRIMITIVE_IRQ      1
#define POOL_PRIMITIVE_DYNAMIC  2

/****************************************************************************
 * Private Data Types
 ****************************************************************************/

/* Private data type that extends the ieee802154_primitive_s struct */

struct ieee802154_priv_primitive_s
{
  /* Must be first member so we can cast to/from */

  struct ieee802154_primitive_s pub;
  FAR struct ieee802154_priv_primitive_s *flink;
  uint8_t pool;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if CONFIG_IEEE802154_PRIMITIVE_PREALLOC > 0
#if CONFIG_IEEE802154_PRIMITIVE_PREALLOC > CONFIG_IEEE802154_PRIMITIVE_IRQRESERVE
/* The g_primfree is a list of primitive structures that are available for
 * general use.  The number of messages in this list is a system
 * configuration item.
 */

static struct ieee802154_priv_primitive_s *g_primfree;
#endif

#if CONFIG_IEEE802154_PRIMITIVE_IRQRESERVE > 0
/* The g_primfree_irq is a list of primitive structures that are reserved for
 * use by only by interrupt handlers.
 */

static struct ieee802154_priv_primitive_s *g_primfree_irq;
#endif

/* Pool of pre-allocated primitive structures */

static struct ieee802154_priv_primitive_s
                g_primpool[CONFIG_IEEE802154_PRIMITIVE_PREALLOC];
#endif /* CONFIG_IEEE802154_PRIMITIVE_PREALLOC > 0 */

static bool g_poolinit = false;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ieee802154_primitivepool_initialize
 *
 * Description:
 *   This function initializes the primitive allocator.  This function must
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

void ieee802154_primitivepool_initialize(void)
{
  /* Only allow the pool to be initialized once */

  if (g_poolinit)
    {
      return;
    }

  g_poolinit = true;

#if CONFIG_IEEE802154_PRIMITIVE_PREALLOC > 0
  FAR struct ieee802154_priv_primitive_s *pool = g_primpool;
  int remaining = CONFIG_IEEE802154_PRIMITIVE_PREALLOC;

#if CONFIG_IEEE802154_PRIMITIVE_PREALLOC > CONFIG_IEEE802154_PRIMITIVE_IRQRESERVE
  /* Initialize g_primfree, the list of primitive structures that are
   * available for general use.
   */

  g_primfree = NULL;
  while (remaining > CONFIG_IEEE802154_PRIMITIVE_IRQRESERVE)
    {
      FAR struct ieee802154_priv_primitive_s *prim = pool;

      /* Add the next meta data structure from the pool to the list of
       * general structures.
       */

      prim->flink = g_primfree;
      g_primfree  = prim;

      /* Set up for the next structure from the pool */

      pool++;
      remaining--;
    }
#endif

#if CONFIG_IEEE802154_PRIMITIVE_IRQRESERVE > 0
  /* Initialize g_primfree_irq is a list of primitive structures reserved for
   * use by only by interrupt handlers.
   */

  g_primfree_irq = NULL;
  while (remaining > 0)
    {
      FAR struct ieee802154_priv_primitive_s *prim = pool;

      /* Add the next meta data structure from the pool to the list of
       * general structures.
       */

      prim->flink    = g_primfree_irq;
      g_primfree_irq = prim;

      /* Set up for the next structure from the pool */

      pool++;
      remaining--;
    }
#endif
#endif /* CONFIG_IEEE802154_PRIMITIVE_PREALLOC > 0 */
}

/****************************************************************************
 * Name: ieee802154_primitive_allocate
 *
 * Description:
 *   The ieee802154_primitive_allocate function will get a free primitive
 *   structure for use by the IEEE 802.15.4 MAC.
 *
 *   Interrupt handling logic will first attempt to allocate from the
 *   g_primfree list.  If that list is empty, it will attempt to allocate
 *   from its reserve, g_primfree_irq.  If that list is empty, then the
 *   allocation fails (NULL is returned).
 *
 *   Non-interrupt handler logic will attempt to allocate from g_primfree
 *   list.  If that the list is empty, then the primitive structure will be
 *   allocated from the dynamic memory pool.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   A reference to the allocated primitive structure.
 *   All user fields in this structure have been zeroed.
 *   On a failure to allocate, NULL is returned.
 *
 ****************************************************************************/

FAR struct ieee802154_primitive_s *ieee802154_primitive_allocate(void)
{
#if CONFIG_IEEE802154_PRIMITIVE_PREALLOC > 0
  FAR struct ieee802154_priv_primitive_s *prim;
  irqstate_t flags;
  uint8_t pool;

  /* If we were called from an interrupt handler, then try to get the meta-
   * data structure from generally available list of messages. If this fails,
   * then try the list of messages reserved for interrupt handlers
   */

  flags = enter_critical_section(); /* Always necessary in SMP mode */
  if (up_interrupt_context())
    {
#if CONFIG_IEEE802154_PRIMITIVE_PREALLOC > CONFIG_IEEE802154_PRIMITIVE_IRQRESERVE
      /* Try the general free list */

      if (g_primfree != NULL)
        {
          prim           = g_primfree;
          g_primfree     = prim->flink;

          leave_critical_section(flags);
          pool          = POOL_PRIMITIVE_GENERAL;
        }
      else
#endif
#if CONFIG_IEEE802154_PRIMITIVE_IRQRESERVE > 0
      /* Try the list list reserved for interrupt handlers */

      if (g_primfree_irq != NULL)
        {
          prim           = g_primfree_irq;
          g_primfree_irq = prim->flink;

          leave_critical_section(flags);
          pool          = POOL_PRIMITIVE_IRQ;
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
#if CONFIG_IEEE802154_PRIMITIVE_PREALLOC > CONFIG_IEEE802154_PRIMITIVE_IRQRESERVE
      /* Try the general free list */

      if (g_primfree != NULL)
        {
          prim           = g_primfree;
          g_primfree     = prim->flink;

          leave_critical_section(flags);
          pool          = POOL_PRIMITIVE_GENERAL;
        }
      else
#endif
        {
          /* If we cannot a primitive structure from the free list, then we
           * will have to allocate one from the kernel memory pool.
           */

          leave_critical_section(flags);
          prim = (FAR struct ieee802154_priv_primitive_s *)
            kmm_malloc((sizeof (struct ieee802154_priv_primitive_s)));

          /* Check if we allocated the primitive structure */

          if (prim == NULL)
            {
              /* No..  memory not available */

              wlerr("ERROR: Failed to allocate primitive.\n");
              return NULL;
            }

          /* Remember that this primitive structure
           * was dynamically allocated
           */

          pool = POOL_PRIMITIVE_DYNAMIC;
        }
    }

  /* We have successfully allocated memory from some source.
   * Zero and tag the allocated primitive structure.
   */

  prim->pool = pool;
  memset(&prim->pub, 0, sizeof(struct ieee802154_primitive_s));

  wlinfo("Primitive allocated: %p\n", prim);
  return &prim->pub;
#else
  return NULL;
#endif
}

/****************************************************************************
 * Name: ieee802154_primitive_free
 *
 * Description:
 *   The ieee802154_primitive_free function will return a primitive structure
 *   to the free pool of  messages if it was a pre-allocated primitive
 *   structure. If the primitive structure was allocated dynamically it will
 *   be deallocated.
 *
 * Input Parameters:
 *   prim - primitive structure to free
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void ieee802154_primitive_free(FAR struct ieee802154_primitive_s *prim)
{
  if (--prim->nclients > 0)
    {
      wlinfo("Remaining Clients: %d\n", prim->nclients);
      return;
    }

#if CONFIG_IEEE802154_PRIMITIVE_PREALLOC > 0
  irqstate_t flags;
  FAR struct ieee802154_priv_primitive_s *priv =
    (FAR struct ieee802154_priv_primitive_s *)prim;

#if CONFIG_IEEE802154_PRIMITIVE_PREALLOC > CONFIG_IEEE802154_PRIMITIVE_IRQRESERVE
  /* If this is a generally available pre-allocated primitive structure,
   * then just put it back in the free list.
   */

  if (priv->pool == POOL_PRIMITIVE_GENERAL)
    {
      /* Make sure we avoid concurrent access to the free
       * list from interrupt handlers.
       */

      flags = enter_critical_section();
      priv->flink = g_primfree;
      g_primfree  = priv;
      leave_critical_section(flags);
    }
  else
#endif

#if CONFIG_IEEE802154_PRIMITIVE_IRQRESERVE > 0
  /* If this is a primitive structure pre-allocated for interrupts,
   * then put it back in the correct free list.
   */

  if (priv->pool == POOL_PRIMITIVE_IRQ)
    {
      /* Make sure we avoid concurrent access to the free
       * list from interrupt handlers.
       */

      flags = enter_critical_section();
      priv->flink    = g_primfree_irq;
      g_primfree_irq  = priv;
      leave_critical_section(flags);
    }
  else
#endif

    {
      /* Otherwise, deallocate it. */

      DEBUGASSERT(priv->pool == POOL_PRIMITIVE_DYNAMIC);
      kmm_free(priv);
    }
#endif

  wlinfo("Primitive freed: %p\n", prim);
}
