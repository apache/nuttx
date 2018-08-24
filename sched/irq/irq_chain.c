/****************************************************************************
 * sched/irq/irq_chain.c
 *
 *   Copyright (C) 2018 Pinecone Inc. All rights reserved.
 *   Author: Zhu GuanqQing <zhuguangqing@pinecone.net>
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

#include "irq/irq.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct irqchain_s
{
  FAR struct irqchain_s *next;

  xcpt_t handler;    /* Address of the interrupt handler */
  FAR void *arg;     /* The argument provided to the interrupt handler. */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* g_irqchainpool is a list of pre-allocated irq chain. The number of irq chains
 * in the pool is a configuration item.
 */

static struct irqchain_s g_irqchainpool[CONFIG_PREALLOC_IRQCHAIN];

/* The g_irqchainfreelist data structure is a single linked list of irqchains
 * available to the system for delayed function use.
 */

static sq_queue_t g_irqchainfreelist;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void irqchain_detach_all(int ndx)
{
  FAR struct irqchain_s *curr;
  FAR struct irqchain_s *prev;

  g_irqvector[ndx].handler = irq_unexpected_isr;

  curr = g_irqvector[ndx].arg;
  while (curr != NULL)
    {
      prev = curr;
      curr = curr->next;
      sq_addlast((FAR struct sq_entry_s *)prev, &g_irqchainfreelist);
    }
}

static int irqchain_dispatch(int irq, FAR void *context, FAR void *arg)
{
  FAR struct irqchain_s *curr;
  FAR struct irqchain_s *prev;
  int ndx;
  int ret = 0;

#ifdef CONFIG_ARCH_MINIMAL_VECTORTABLE
  ndx = g_irqmap[irq];
#else
  ndx = irq;
#endif

  curr = g_irqvector[ndx].arg;
  while (curr != NULL)
    {
      prev = curr;
      curr = curr->next;
      ret |= prev->handler(irq, context, prev->arg);
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void irqchain_initialize(void)
{
  FAR struct irqchain_s *irqchain = g_irqchainpool;
  int i;

  /* Initialize irqchain free lists */

  sq_init(&g_irqchainfreelist);

  /* The g_irqchainfreelist must be loaded at initialization time to hold the
   * configured number of irqchain.
   */

  for (i = 0; i < CONFIG_PREALLOC_IRQCHAIN; i++)
    {
      sq_addlast((FAR struct sq_entry_s *)irqchain++, &g_irqchainfreelist);
    }
}

bool is_irqchain(int ndx, xcpt_t isr)
{
  if (g_irqvector[ndx].handler == irq_unexpected_isr ||
      g_irqvector[ndx].handler == NULL)
    {
      return false;
    }
  else if (g_irqvector[ndx].handler == irqchain_dispatch)
    {
      return true;
    }
  else
    {
      return isr != irq_unexpected_isr;
    }
}

int irqchain_attach(int ndx, xcpt_t isr, FAR void *arg)
{
  FAR struct irqchain_s *node;
  FAR struct irqchain_s *curr;

  if (isr != irq_unexpected_isr)
    {
      if (g_irqvector[ndx].handler != irqchain_dispatch)
        {
          if (sq_count(&g_irqchainfreelist) < 2)
            {
              return -ENOMEM;
            }

          node = (FAR struct irqchain_s *)sq_remfirst(&g_irqchainfreelist);
          DEBUGASSERT(node != NULL);

          node->handler = g_irqvector[ndx].handler;
          node->arg     = g_irqvector[ndx].arg;
          node->next    = NULL;

          g_irqvector[ndx].handler = irqchain_dispatch;
          g_irqvector[ndx].arg     = node;
       }

      node = (FAR struct irqchain_s *)sq_remfirst(&g_irqchainfreelist);
      if (node == NULL)
        {
          return -ENOMEM;
        }

      node->handler = isr;
      node->arg     = arg;
      node->next    = NULL;

      curr = g_irqvector[ndx].arg;
      while (curr->next != NULL)
        {
          curr = curr->next;
        }

      curr->next = node;
    }
  else
    {
      irqchain_detach_all(ndx);
    }

  return OK;
}

int irqchain_detach(int irq, xcpt_t isr, FAR void *arg)
{
#if NR_IRQS > 0
  FAR struct irqchain_s *prev;
  FAR struct irqchain_s *curr;
  FAR struct irqchain_s *first;
  int ret = -EINVAL;

  if ((unsigned)irq < NR_IRQS)
    {
      irqstate_t flags;
      int ndx;

#ifdef CONFIG_ARCH_MINIMAL_VECTORTABLE
      /* Is there a mapping for this IRQ number? */

      ndx = g_irqmap[irq];
      if ((unsigned)ndx >= CONFIG_ARCH_NUSER_INTERRUPTS)
        {
          /* No.. then return failure. */

          return ret;
        }
#else
      ndx = irq;
#endif

      flags = enter_critical_section();

      if (g_irqvector[ndx].handler == irqchain_dispatch)
        {
          first = g_irqvector[ndx].arg;
          for (prev = NULL, curr = first;
               curr != NULL;
               prev = curr, curr = curr->next)
            {
              if (curr->handler == isr && curr->arg == arg)
                {
                  if (curr == first)
                    {
                      g_irqvector[ndx].arg = curr->next;
                    }
                  else if (curr->next == NULL)
                    {
                      prev->next = NULL;
                    }
                  else
                    {
                      prev->next = curr->next;
                    }

                  sq_addlast((FAR struct sq_entry_s *)curr, &g_irqchainfreelist);

                  first = g_irqvector[ndx].arg;
                  if (first->next == NULL)
                    {
                      g_irqvector[ndx].handler = first->handler;
                      g_irqvector[ndx].arg     = first->arg;
                      sq_addlast((FAR struct sq_entry_s *)first, &g_irqchainfreelist);
                    }

                  ret = OK;
                  break;
                }
            }
        }
      else
        {
          ret = irq_detach(irq);
        }

      leave_critical_section(flags);
    }

  return ret;
#else
  return OK;
#endif
}
