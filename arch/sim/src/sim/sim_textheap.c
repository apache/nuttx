/****************************************************************************
 * arch/sim/src/sim/sim_textheap.c
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

#include <nuttx/arch.h>
#include <nuttx/queue.h>
#include <nuttx/kmalloc.h>

#include "sim_internal.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Record memory allocated for text sections by sq_queue_t */

struct textheap_s
{
  sq_entry_t entry;
  void *p;
  size_t size;
};

static sq_queue_t g_textheap_list;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_textheap_memalign
 *
 * Description:
 *   Allocate memory for text sections with the specified alignment.
 *
 ****************************************************************************/

void *up_textheap_memalign(size_t align, size_t size)
{
  struct textheap_s *node;
  void *p;
  irqstate_t flags;

  /* host_allocheap (mmap) returns memory aligned to the page size, which
   * is always a multiple of the alignment (4/8) for text section. So, we
   * don't need to do anything here.
   */

  p = host_allocheap(size);

  flags = up_irq_save();

  if (p)
    {
      node = kmm_malloc(sizeof(*node));

      if (node)
        {
          node->p = p;
          node->size = size;

          sq_addlast(&node->entry, &g_textheap_list);
        }
      else
        {
          host_freeheap(p);
          p = NULL;
        }
    }

  up_irq_restore(flags);

  return p;
}

/****************************************************************************
 * Name: up_textheap_free
 *
 * Description:
 *   Free memory allocated for text sections.
 *
 ****************************************************************************/

void up_textheap_free(void *p)
{
  sq_entry_t *node;
  struct textheap_s *entry;
  irqstate_t flags = up_irq_save();

  for (node = sq_peek(&g_textheap_list);
       node != NULL;
       node = sq_next(node))
    {
      entry = (struct textheap_s *)node;

      if (entry->p == p)
        {
          sq_rem(node, &g_textheap_list);
          host_freeheap(p);
          kmm_free(entry);
          break;
        }
    }

  up_irq_restore(flags);
}

/****************************************************************************
 * Name: up_textheap_heapmember
 *
 * Description:
 *   Test if memory is from text heap.
 *
 ****************************************************************************/

bool up_textheap_heapmember(void *p)
{
  bool ret = false;
  sq_entry_t *node;
  struct textheap_s *entry;
  irqstate_t flags = up_irq_save();

  for (node = sq_peek(&g_textheap_list);
       node != NULL;
       node = sq_next(node))
    {
      entry = (struct textheap_s *)node;

      if (entry->p == p)
        {
          ret = true;
          break;
        }
    }

  up_irq_restore(flags);
  return ret;
}
