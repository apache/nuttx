/****************************************************************************
 * arch/sim/src/sim/sim_sectionheap.c
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
#include <nuttx/mm/mm.h>

#include "sim_internal.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct mm_heap_s *g_textheap;
static struct mm_heap_s *g_dataheap;

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

#ifdef CONFIG_ARCH_USE_SEPARATED_SECTION
void *up_textheap_memalign(const char *sectname, size_t align, size_t size)
#else
void *up_textheap_memalign(size_t align, size_t size)
#endif
{
  if (g_textheap == NULL)
    {
      g_textheap = mm_initialize("textheap",
                                 host_allocheap(SIM_HEAP_SIZE, true),
                                 SIM_HEAP_SIZE);
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
  return g_textheap != NULL && mm_heapmember(g_textheap, p);
}

/****************************************************************************
 * Name: up_dataheap_memalign
 *
 * Description:
 *   Allocate memory for data sections with the specified alignment.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_USE_SEPARATED_SECTION
void *up_dataheap_memalign(const char *sectname, size_t align, size_t size)
#else
void *up_dataheap_memalign(size_t align, size_t size)
#endif
{
  if (g_dataheap == NULL)
    {
      g_dataheap = mm_initialize("dataheap",
                                 host_allocheap(SIM_HEAP_SIZE, false),
                                 SIM_HEAP_SIZE);
    }

  return mm_memalign(g_dataheap, align, size);
}

/****************************************************************************
 * Name: up_dataheap_free
 *
 * Description:
 *   Free memory allocated for data sections.
 *
 ****************************************************************************/

void up_dataheap_free(void *p)
{
  if (g_dataheap != NULL)
    {
      mm_free(g_dataheap, p);
    }
}

/****************************************************************************
 * Name: up_dataheap_heapmember
 *
 * Description:
 *   Test if memory is from data heap.
 *
 ****************************************************************************/

bool up_dataheap_heapmember(void *p)
{
  return g_dataheap != NULL && mm_heapmember(g_dataheap, p);
}
