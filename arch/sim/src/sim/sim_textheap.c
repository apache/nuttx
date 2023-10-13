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
#include <nuttx/mm/mm.h>

#include "sim_internal.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct mm_heap_s *g_textheap;

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
