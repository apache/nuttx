/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_scufifo.c
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

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <debug.h>

#include <arch/chip/scu.h>

#include "chip.h"
#include "arm_internal.h"
#include "cxd56_scufifo.h"
#include "hardware/cxd56_scufifo.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define FIFOMEMSIZE 40960

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct memchunk
{
  struct memchunk *next;
  uint16_t start;
  uint16_t size;
};

struct fifomem
{
  struct memchunk chunk[14];
  struct memchunk *allocated;
  struct memchunk *freelist;
  uint16_t size;
};

/****************************************************************************
 * Private Variables
 ****************************************************************************/

struct fifomem g_fifomem;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fifomem_alloc
 *
 * Description:
 *  Allocate FIFO memory
 *
 ****************************************************************************/

uint16_t scufifo_memalloc(uint16_t size)
{
  struct memchunk *c;
  struct memchunk *last;
  uint16_t start;

  if (g_fifomem.size < size)
    {
      return FIFOMEM_INVALID;
    }

  if (g_fifomem.freelist == NULL)
    {
      return FIFOMEM_INVALID;
    }

  start = 0;
  last  = NULL;
  for (c = g_fifomem.allocated; c; c = c->next)
    {
      start = c->start + c->size;
      last  = c;
      if (c->next && c->next->start - start > size)
        {
          break;
        }
    }

  if (start + size > g_fifomem.size)
    {
      return FIFOMEM_INVALID;
    }

  /* Remove from free list */

  c                  = g_fifomem.freelist;
  g_fifomem.freelist = c->next;

  /* Append file chunk */

  if (last == NULL)
    {
      g_fifomem.allocated = c;
      c->next             = NULL;
    }
  else
    {
      c->next    = last->next;
      last->next = c;
    }

  c->start = start;
  c->size  = size;

  return start;
}

/****************************************************************************
 * Name: scufifo_memfree
 *
 * Description:
 *  Free allocated FIFO memory
 *
 ****************************************************************************/

void scufifo_memfree(uint16_t start)
{
  struct memchunk *c;
  struct memchunk *prev;

  prev = g_fifomem.allocated;
  for (c = g_fifomem.allocated; c; c = c->next)
    {
      if (c->start == start)
        {
          if (g_fifomem.allocated == c)
            {
              g_fifomem.allocated = c->next;
            }
          else
            {
              prev->next = c->next;
            }

          c->next            = g_fifomem.freelist;
          g_fifomem.freelist = c;
          break;
        }

      prev = c;
    }
}

/****************************************************************************
 * Name: scufifo_initialize
 *
 * Description:
 *  Initialize FIFO memory allocator
 *
 ****************************************************************************/

void scufifo_initialize(void)
{
  struct memchunk *c;
  int i;

  g_fifomem.allocated = NULL;
  g_fifomem.freelist  = g_fifomem.chunk;
  g_fifomem.size      = FIFOMEMSIZE;

  for (i = 1, c = g_fifomem.freelist; i < 14; i++, c = c->next)
    {
      c->size = 0;
      c->next = &g_fifomem.chunk[i];
    }

  c->next = NULL;
}
