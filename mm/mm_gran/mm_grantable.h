/****************************************************************************
 * mm/mm_gran/mm_grantable.h
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

#ifndef __MM_MM_GRAN_MM_GRANTABLE_H
#define __MM_MM_GRAN_MM_GRANTABLE_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Granule arithmetics */

#define GRANSIZE(g)         (1 << g->log2gran)
#define GRANMASK(g)         (GRANSIZE(g) - 1)
#define NGRANULE(g, s)      ((s + GRANMASK(g)) >> g->log2gran)

#define GRANBYTE(g)         ((size_t)g->ngranules << g->log2gran)
#define GRANENDA(g)         (GRANBYTE(g) + g->heapstart)
#define MEM2GRAN(g, m)      ((((uintptr_t)m) - g->heapstart) >> g->log2gran)
#define GRAN2MEM(g, x)      ((((uintptr_t)x) << g->log2gran) + g->heapstart)

#define GRAN_ALIGNED(g, m)  ((((uintptr_t)(m)) & GRANMASK(g)) == 0)
#define GRAN_INRANGE(g, m)  (g->heapstart <= (uintptr_t)(m) && \
                              (uintptr_t)(m) < GRANENDA(g))
#define GRAN_PRODUCT(g, m)  (GRAN_ALIGNED(g, m) && GRAN_INRANGE(g, m))

#define ALIGNDN(g, m)       (((size_t)m) & ~GRANMASK(g))
#define ALIGNUP(g, m)       ((((size_t)m) + GRANMASK(g)) & ~GRANMASK(g))

/* gran_reserve related */

#define MEM_RSRV(g, m)      ALIGNDN(g, m)
#define END_RSRV(g, m, s)   ALIGNUP(g, (((size_t)m) + s - 1))
#define NUM_RSRV(g, m, s)   (((END_RSRV(g, m, s) - MEM_RSRV(g, m)) \
                                 >> g->log2gran) + 1)
#define LEN_RSRV(g, m, s)   ((size_t)(NUM_RSRV(g, m, s) << g->log2gran))

/* GAT table related */

#define GATC_BITS(g)        (sizeof(g->gat[0]) << 3)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Structure for a range of granules in GAT */

struct gran_range_s
{
  uint16_t  sidx;        /* index of the starting GAT cell */
  uint16_t  eidx;        /* index of the ending GAT cell */
  uint8_t   soff;        /* offset of bit in starting cell */
  uint8_t   eoff;        /* offset of bit in ending cell */
  uint16_t width;        /* width of cell in bits */
  uint32_t smask;        /* mask of the starting GAT cell */
  uint32_t emask;        /* mask of the ending GAT cell */
};

typedef struct gran_range_s gatr_t;
typedef struct gran_s       gran_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: gran_range
 *
 * Description:
 *   populate a gran_range_s instance for later use
 *
 * Input Parameters:
 *   gran - Pointer to the gran state
 *   posi - Position of starting granule
 *   size - Length of range
 *
 * Output Parameters:
 *   rang - The range instance to be prepared
 *
 * Return value:
 *   OK or negative errno
 ****************************************************************************/

int gran_range(const gran_t *gran, size_t posi, size_t size, gatr_t *rang);

/****************************************************************************
 * Name: gran_match
 *
 * Description:
 *   check if a continuous range of granules all have expected status
 *
 * Input Parameters:
 *   gran - Pointer to the gran state
 *   posi - Position of starting granule
 *   size - Length of range
 *   used - Expected state, true for used, false for empty.
 *
 * Output Parameters:
 *   mism - Optional last failed position upon free range matching.
 *
 * Return value:
 *   true for match, false otherwise.
 ****************************************************************************/

bool gran_match(const gran_t *gran, size_t posi, size_t size, bool used,
                size_t *mism);

/****************************************************************************
 * Name: gran_search
 *
 * Description:
 *   search for continuous range of free granules
 *
 * Input Parameters:
 *   gran - Pointer to the gran state
 *   size - Length of range
 *
 * Return value:
 *   position of negative error number.
 ****************************************************************************/

int gran_search(const gran_t *gran, size_t size);

/****************************************************************************
 * Name: gran_set, gran_clear
 *
 * Description:
 *   Set or clear a range of granule in the GAT
 *
 * Input Parameters:
 *   gran   - Pointer to the gran state
 *   posi   - Range starting bit index
 *   size   - Range size
 *
 * Return value:
 *   OK on success or negative value on error
 ****************************************************************************/

int gran_set(gran_t *gran, size_t posi, size_t size);
int gran_clear(gran_t *gran, size_t posi, size_t size);

#endif /* __MM_MM_GRAN_MM_GRANTABLE_H */
