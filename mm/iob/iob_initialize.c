/****************************************************************************
 * mm/iob/iob_initialize.c
 *
 *   Copyright (C) 2014, 2017 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>

#include <nuttx/semaphore.h>
#include <nuttx/mm/iob.h>

#include "iob.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef NULL
#  define NULL ((FAR void *)0)
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This is a pool of pre-allocated I/O buffers */

static struct iob_s        g_iob_pool[CONFIG_IOB_NBUFFERS];
#if CONFIG_IOB_NCHAINS > 0
static struct iob_qentry_s g_iob_qpool[CONFIG_IOB_NCHAINS];
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* A list of all free, unallocated I/O buffers */

FAR struct iob_s *g_iob_freelist;

/* A list of I/O buffers that are committed for allocation */

FAR struct iob_s *g_iob_committed;

#if CONFIG_IOB_NCHAINS > 0
/* A list of all free, unallocated I/O buffer queue containers */

FAR struct iob_qentry_s *g_iob_freeqlist;

/* A list of I/O buffer queue containers that are committed for allocation */

FAR struct iob_qentry_s *g_iob_qcommitted;
#endif

/* Counting semaphores that tracks the number of free IOBs/qentries */

sem_t g_iob_sem;            /* Counts free I/O buffers */
#if CONFIG_IOB_THROTTLE > 0
sem_t g_throttle_sem;       /* Counts available I/O buffers when throttled */
#endif
#if CONFIG_IOB_NCHAINS > 0
sem_t g_qentry_sem;         /* Counts free I/O buffer queue containers */
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: iob_initialize
 *
 * Description:
 *   Set up the I/O buffers for normal operations.
 *
 ****************************************************************************/

void iob_initialize(void)
{
  static bool initialized = false;
  int i;

  /* Perform one-time initialization */

  if (!initialized)
    {
      /* Add each I/O buffer to the free list */

      for (i = 0; i < CONFIG_IOB_NBUFFERS; i++)
        {
          FAR struct iob_s *iob = &g_iob_pool[i];

          /* Add the pre-allocate I/O buffer to the head of the free list */

          iob->io_flink  = g_iob_freelist;
          g_iob_freelist = iob;
        }

      g_iob_committed = NULL;

      nxsem_init(&g_iob_sem, 0, CONFIG_IOB_NBUFFERS);
#if CONFIG_IOB_THROTTLE > 0
      nxsem_init(&g_throttle_sem, 0, CONFIG_IOB_NBUFFERS - CONFIG_IOB_THROTTLE);
#endif

#if CONFIG_IOB_NCHAINS > 0
      /* Add each I/O buffer chain queue container to the free list */

      for (i = 0; i < CONFIG_IOB_NCHAINS; i++)
        {
          FAR struct iob_qentry_s *iobq = &g_iob_qpool[i];

          /* Add the pre-allocate buffer container to the head of the free list */

          iobq->qe_flink  = g_iob_freeqlist;
          g_iob_freeqlist = iobq;
        }

      g_iob_qcommitted = NULL;

      nxsem_init(&g_qentry_sem, 0, CONFIG_IOB_NCHAINS);
#endif
      initialized = true;
    }
}
