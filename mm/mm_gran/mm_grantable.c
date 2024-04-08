/****************************************************************************
 * mm/mm_gran/mm_grantable.c
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

#include <assert.h>
#include <errno.h>
#include <strings.h>
#include <debug.h>

#include <nuttx/bits.h>
#include <nuttx/mm/gran.h>

#include "mm_gran/mm_gran.h"
#include "mm_gran/mm_grantable.h"

#ifdef CONFIG_GRAN

/****************************************************************************
 * Preprocessors
 ****************************************************************************/

#define GATCFULL         0xffffffffu    /* a full GAT cell */
#define DEBRUJIN_NUM     0x077CB531UL   /* the de Bruijn Sequence */

/****************************************************************************
 * Private data
 ****************************************************************************/

#if !defined(CONFIG_HAVE_BUILTIN_CLZ) || !defined(CONFIG_HAVE_BUILTIN_CTZ)

/* The de Bruijn lookup table to get n from BIT(n). */

static const uint8_t DEBRUJIN_LUT[32] =
{
  0, 1, 28, 2, 29, 14, 24, 3, 30, 22, 20, 15, 25, 17, 4, 8,
  31, 27, 13, 23, 21, 19, 16, 7, 26, 12, 18, 6, 11, 5, 10, 9
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* return BIT(MSB(n)) */

uint32_t msb_mask(uint32_t n)
{
  /* see https://www.geeksforgeeks.org/find-significant-set-bit-number */

  DEBUGASSERT(n);
  n |= n >> 1;
  n |= n >> 2;
  n |= n >> 4;
  n |= n >> 8;
  n |= n >> 16;

  n = ((n + 1) >> 1) | (n & (1 << ((sizeof(n) << 3)-1)));
  return n;
}

/* return BIT(LSB(n)) */

uint32_t lsb_mask(uint32_t n)
{
  DEBUGASSERT(n);
  return (-n & n) & GATCFULL;
}

/* set or clear a GAT cell with given bit mask */

static void cell_set(gran_t *gran, uint32_t cell, uint32_t mask, bool val)
{
  if (val)
    {
      gran->gat[cell] |= mask;
    }
  else
    {
      gran->gat[cell] &= ~mask;
    }
}

/* set or clear a range of GAT bits */

static void gran_set_(gran_t *gran, gatr_t *rang, bool val)
{
  uint32_t c;

  cell_set(gran, rang->sidx, rang->smask, val);
  if (rang->sidx != rang->eidx)
    {
      cell_set(gran, rang->eidx, rang->emask, val);
      c = rang->sidx + 1;
      for (; c < rang->eidx; c++)
        {
          cell_set(gran, c, GATCFULL, val);
        }
    }

  return;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* prepare given GAT range instance for later use. */

int gran_range(const gran_t *gran, size_t posi, size_t size,
               gatr_t *rang)
{
  if (!gran || gran->ngranules < posi + size)
    {
      return -EINVAL;
    }

  if (rang == NULL)
    {
      return -ENOMEM;
    }

  rang->width = GATC_BITS(gran);

  rang->sidx = posi / rang->width;
  rang->soff = posi % rang->width;

  posi += size - 1;
  rang->eidx = posi / GATC_BITS(gran);
  rang->eoff = posi % GATC_BITS(gran);

  rang->smask = ~(BIT(rang->soff) - 1);
  rang->emask = (BIT(rang->eoff) - 1) | BIT(rang->eoff);

  if (rang->sidx == rang->eidx)
    {
      rang->smask &= rang->emask;  /* combine the masks */
      rang->emask = rang->smask;
    }

  return OK;
}

/* checks if a range of granule matches the expected status */

bool gran_match(const gran_t *gran, size_t posi, size_t size, bool used,
                size_t *mpos)
{
  uint32_t c;   /* cell index */
  uint32_t v;   /* masked cell value */
  uint32_t e;   /* expected cell value */
  gatr_t   r;   /* range helper */

  gran_range(gran, posi, size, &r);

  /* check the ending cell */

  c = r.eidx;
  e = used ? r.emask : 0 ;
  v = gran->gat[c] & r.emask;
  if (v != e)
    {
      goto failure;
    }

  if (r.sidx == r.eidx)
    {
      return true;
    }

  /* check cells in between */

  c = r.eidx - 1;
  e = used ? GATCFULL : 0;
  for (; c > r.sidx; c--)
    {
      v = gran->gat[c];
      if (v != e)
        {
          goto failure;
        }
    }

  /* check the starting cell */

  c = r.sidx;
  e = used ? r.smask : 0 ;
  v = gran->gat[c] & r.smask;
  if (v != e)
    {
      goto failure;
    }

  return true;

failure:

  if (mpos && !used)
    {
      /* offset of last used when matching for free */

      DEBUGASSERT(v);
#ifdef CONFIG_HAVE_BUILTIN_CLZ
      *mpos = 31 - __builtin_clz(v);
#else
      *mpos = (uint32_t)((msb_mask(v)) * DEBRUJIN_NUM) >> 27;
      DEBUGASSERT(*mpos < sizeof(DEBRUJIN_LUT));
      *mpos = DEBRUJIN_LUT[*mpos];
#endif
      *mpos += c * GATC_BITS(gran);
    }

  return false;
}

/* returns granule number of free range or negative error */

int gran_search(const gran_t *gran, size_t size)
{
  int ret = -EINVAL;

  if (gran == NULL || gran->ngranules < size)
    {
      return ret;
    }

  ret = -ENOMEM;
  for (size_t i = 0; i <= gran->ngranules - size; i++)
    {
      if (gran_match(gran, i, size, 0, &i))
        {
          ret = i;
          break;
        }
    }

  return ret;
}

/* set a range of granules */

int gran_set(gran_t *gran, size_t posi, size_t size)
{
  gatr_t rang;
  int ret = gran_range(gran, posi, size, &rang);

  if (ret == OK)
    {
      gran_set_(gran, &rang, true);
    }

  return ret;
}

/* clear a range of granules */

int gran_clear(gran_t *gran, size_t posi, size_t size)
{
  gatr_t rang;
  int ret = gran_range(gran, posi, size, &rang);

  if (ret == OK)
    {
      gran_set_(gran, &rang, false);
    }

  return ret;
}

#endif /* CONFIG_GRAN */
