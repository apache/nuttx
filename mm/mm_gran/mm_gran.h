/****************************************************************************
 * mm/mm_gran/mm_gran.h
 *
 *   Copyright (C) 2012, 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
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

#ifndef __MM_MM_GRAN_MM_GRAN_H
#define __MM_MM_GRAN_MM_GRAN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <arch/types.h>
#include <nuttx/mm/gran.h>
#include <nuttx/semaphore.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Sizes of things */

#define SIZEOF_GAT(n) \
  ((n + 31) >> 5)
#define SIZEOF_GRAN_S(n) \
  (sizeof(struct gran_s) + sizeof(uint32_t) * (SIZEOF_GAT(n) - 1))

/* Debug */

#ifdef CONFIG_CPP_HAVE_VARARGS
#  ifdef CONFIG_DEBUG_GRAM
#    define granerr(format, ...)       _err(format, ##__VA_ARGS__)
#    define granwarn(format, ...)      _warn(format, ##__VA_ARGS__)
#    define graninfo(format, ...)      _info(format, ##__VA_ARGS__)
#  else
#    define granerr(format, ...)       merr(format, ##__VA_ARGS__)
#    define granwarn(format, ...)      mwarn(format, ##__VA_ARGS__)
#    define graninfo(format, ...)      minfo(format, ##__VA_ARGS__)
#  endif
#else
#  ifdef CONFIG_DEBUG_GRAM
#    define granerr                    _err
#    define granwarn                   _warn
#    define graninfo                   _info
#  else
#    define granerr                    merr
#    define granwarn                   mwarn
#    define graninfo                   minfo
#  endif
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
  sem_t      exclsem;   /* For exclusive access to the GAT */
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
