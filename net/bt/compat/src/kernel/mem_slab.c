/****************************************************************************
 * net/bt/compat/src/kernel/mem_slab.c
 *
 * Copyright (c) ByteDance Ltd. and/or its affiliates.
 *
 * Author: chao an <anchao.archer@bytedance.com>
 *
 * SPDX-License-Identifier: Apache-2.0
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

/* SPDX-License-Identifier: Apache-2.0 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <errno.h>
#include <nuttx/net/bluetooth/compat/kernel.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void k_mem_slab_build_free_list(struct k_mem_slab *slab)
{
  char *p = slab->buffer;

  slab->free_list = NULL;

  for (uint32_t i = 0; i < slab->num_blocks; i++)
    {
      *(char **)p = slab->free_list;
      slab->free_list = p;
      p += slab->block_size;
    }

  slab->num_used = 0;
  slab->initialized = true;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void k_mem_slab_init(struct k_mem_slab *slab,
                     void *buffer,
                     size_t block_size,

                     uint32_t num_blocks)
{
  slab->buffer = (char *)buffer;
  slab->block_size = block_size;
  slab->num_blocks = num_blocks;
  slab->lock = SP_UNLOCKED;
  k_mem_slab_build_free_list(slab);
}

int k_mem_slab_alloc(struct k_mem_slab *slab, void **mem, clock_t timeout)
{
  (void)timeout;

  irqstate_t flags = spin_lock_irqsave(&slab->lock);

  if (!slab->initialized)
    {
      k_mem_slab_build_free_list(slab);
    }

  if (slab->free_list == NULL)
    {
      spin_unlock_irqrestore(&slab->lock, flags);
      return -ENOMEM;
    }

  *mem = slab->free_list;
  slab->free_list = *(char **)slab->free_list;
  slab->num_used++;

  spin_unlock_irqrestore(&slab->lock, flags);
  return 0;
}

void k_mem_slab_free(struct k_mem_slab *slab, void *mem)
{
  irqstate_t flags = spin_lock_irqsave(&slab->lock);

  *(char **)mem = slab->free_list;
  slab->free_list = (char *)mem;

  if (slab->num_used > 0)
    {
      slab->num_used--;
    }

  spin_unlock_irqrestore(&slab->lock, flags);
}

uint32_t k_mem_slab_num_free_get(struct k_mem_slab *slab)
{
  return slab->num_blocks - slab->num_used;
}
