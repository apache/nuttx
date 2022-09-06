/****************************************************************************
 * mm/mm_gran/mm_gran.h
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

#ifndef __MM_MM_GRAN_MM_GRAN_H
#define __MM_MM_GRAN_MM_GRAN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <arch/types.h>
#include <nuttx/mm/gran.h>
#include <nuttx/mutex.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Sizes of things */

#define SIZEOF_GAT(n) \
  ((n + 31) >> 5)
#define SIZEOF_GRAN_S(n) \
  (sizeof(struct gran_s) + sizeof(uint32_t) * (SIZEOF_GAT(n) - 1))

/* Debug */

#ifdef CONFIG_DEBUG_GRAM
#  define granerr                    _err
#  define granwarn                   _warn
#  define graninfo                   _info
#else
#  define granerr                    merr
#  define granwarn                   mwarn
#  define graninfo                   minfo
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure represents the state of one granule allocation */

struct gran_s
{
  uint8_t    log2gran;  /* Log base 2 of the size of one granule */
  uint16_t   ngranules; /* The total number of (aligned) granules in the heap */
#ifdef CONFIG_GRAN_INTR
  irqstate_t irqstate;  /* For exclusive access to the GAT */
#else
  mutex_t    lock;       /* For exclusive access to the GAT */
#endif
  uintptr_t  heapstart; /* The aligned start of the granule heap */
  uint32_t   gat[1];    /* Start of the granule allocation table */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: gran_enter_critical and gran_leave_critical
 *
 * Description:
 *   Critical section management for the granule allocator.
 *
 * Input Parameters:
 *   priv - Pointer to the gran state
 *
 * Returned Value:
 *   gran_enter_critical() may return any error reported by
 *   nxsem_wait_uninterruptible()
 *
 ****************************************************************************/

int  gran_enter_critical(FAR struct gran_s *priv);
void gran_leave_critical(FAR struct gran_s *priv);

/****************************************************************************
 * Name: gran_mark_allocated
 *
 * Description:
 *   Mark a range of granules as allocated.
 *
 * Input Parameters:
 *   priv  - The granule heap state structure.
 *   alloc - The address of the allocation.
 *   ngranules - The number of granules allocated
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void gran_mark_allocated(FAR struct gran_s *priv, uintptr_t alloc,
                         unsigned int ngranules);

#endif /* __MM_MM_GRAN_MM_GRAN_H */
