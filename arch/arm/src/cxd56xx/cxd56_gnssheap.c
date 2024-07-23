/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_gnssheap.c
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

#include <nuttx/arch.h>
#include <nuttx/mm/mm.h>
#include <arch/chip/gnssram.h>

#include "cxd56_clock.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct mm_heap_s *g_gnssheap;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_gnssram_initialize
 *
 * Description:
 *   Initialize the GNSS heap.
 *
 ****************************************************************************/

void up_gnssram_initialize(void)
{
  void  *start;
  size_t size;

  /* These values come from the linker scripts. */

  extern uint8_t _sgnssheap[];
  extern uint8_t _egnssheap[];

  cxd56_gnssram_clock_enable();

  start = (void *)_sgnssheap;
  size  = (size_t)(_egnssheap - _sgnssheap);
  g_gnssheap = mm_initialize("gnssheap", start, size);
}

/****************************************************************************
 * Name: up_gnssram_uninitialize
 *
 * Description:
 *   Uninitialize the GNSS heap.
 *
 ****************************************************************************/

void up_gnssram_uninitialize(void)
{
  mm_uninitialize(g_gnssheap);
}

/****************************************************************************
 * Name: up_gnssram_malloc
 *
 * Description:
 *   Allocate memory from the GNSS heap.
 *
 ****************************************************************************/

void *up_gnssram_malloc(size_t size)
{
  return mm_malloc(g_gnssheap, size);
}

/****************************************************************************
 * Name: up_gnssram_calloc
 *
 * Description:
 *   Calculates the size of the allocation and allocate memory from
 *   the GNSS heap.
 *
 ****************************************************************************/

void *up_gnssram_calloc(size_t n, size_t elem_size)
{
  return mm_calloc(g_gnssheap, n, elem_size);
}

/****************************************************************************
 * Name: up_gnssram_realloc
 *
 * Description:
 *   Reallocate memory from the GNSS heap.
 *
 ****************************************************************************/

void *up_gnssram_realloc(void *ptr, size_t size)
{
  return mm_realloc(g_gnssheap, ptr, size);
}

/****************************************************************************
 * Name: up_gnssram_zalloc
 *
 * Description:
 *   Allocate and zero memory from the GNSS heap.
 *
 ****************************************************************************/

void *up_gnssram_zalloc(size_t size)
{
  return mm_zalloc(g_gnssheap, size);
}

/****************************************************************************
 * Name: up_gnssram_free
 *
 * Description:
 *   Free memory from the GNSS heap.
 *
 ****************************************************************************/

void up_gnssram_free(void *mem)
{
  mm_free(g_gnssheap, mem);
}

/****************************************************************************
 * Name: up_gnssram_memalign
 *
 * Description:
 *   memalign requests more than enough space from malloc, finds a region
 *   within that chunk that meets the alignment request and then frees any
 *   leading or trailing space.
 *
 *   The alignment argument must be a power of two (not checked).  8-byte
 *   alignment is guaranteed by normal malloc calls.
 *
 ****************************************************************************/

void *up_gnssram_memalign(size_t alignment, size_t size)
{
  return mm_memalign(g_gnssheap, alignment, size);
}

/****************************************************************************
 * Name: up_gnssram_heapmember
 *
 * Description:
 *   Check if an address lies in the GNSS heap.
 *
 * Parameters:
 *   mem - The address to check
 *
 * Return Value:
 *   true if the address is a member of the GNSS heap.  false if not
 *
 ****************************************************************************/

bool up_gnssram_heapmember(void *mem)
{
  return mm_heapmember(g_gnssheap, mem);
}

/****************************************************************************
 * Name: up_gnssram_mallinfo
 *
 * Description:
 *   mallinfo returns a copy of updated current heap information for the
 *   user heap.
 *
 ****************************************************************************/

struct mallinfo up_gnssram_mallinfo(void)
{
  return mm_mallinfo(g_gnssheap);
}
