/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_scufifo.c
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
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
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
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

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <debug.h>

#include <arch/chip/scu.h>

#include "chip.h"
#include "arm_arch.h"

#include "cxd56_scufifo.h"
#include "hardware/cxd56_scufifo.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define FIFOMEMSIZE 40960

#define __unused __attribute__((unused))

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
