/****************************************************************************
 * arch/arm64/src/qemu/qemu_textheap.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/arch.h>
#include <nuttx/queue.h>
#include <nuttx/kmalloc.h>

#include "arm64_mmu.h"
#include "arm64_internal.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern uint8_t _sload[];
extern uint8_t _szload[];

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct mm_heap_s *g_textheap;

/* Mark load segment cacheable, write-read and executable */

static struct arm_mmu_region g_mmu_load =
  MMU_REGION_FLAT_ENTRY("nx_load",
                        (uint64_t)_sload,
                        (uint64_t)_szload,
                        MT_NORMAL | MT_RW | MT_EXECUTE | MT_SECURE);

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
  if (g_textheap == NULL)
    {
      if (arm64_mmu_set_memregion(&g_mmu_load) != 0)
        {
          return NULL;
        }

      g_textheap = mm_initialize("textheap", (void *)&_sload,
                                 (size_t)_szload);
    }

  return mm_memalign(g_textheap, align, size);
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
  if (g_textheap != NULL)
    {
      mm_free(g_textheap, p);
    }
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
  if (g_textheap != NULL)
    {
      return mm_heapmember(g_textheap, p);
    }

  return false;
}
