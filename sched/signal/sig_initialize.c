/****************************************************************************
 * sched/signal/sig_initialize.c
 *
 *   Copyright (C) 2007, 2009, 2011, 2017-2018 Gregory Nutt. All rights
 *     reserved.
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

#include <stdint.h>
#include <queue.h>
#include <assert.h>

#include <nuttx/kmalloc.h>

#include "signal/signal.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* The g_sigfreeaction data structure is a list of available signal
 * action structures.
 */

sq_queue_t  g_sigfreeaction;

/* The g_sigpendingaction data structure is a list of available pending
 * signal action structures.
 */

sq_queue_t  g_sigpendingaction;

/* The g_sigpendingirqaction is a list of available pending signal actions
 * that are reserved for use by interrupt handlers.
 */

sq_queue_t  g_sigpendingirqaction;

/* The g_sigpendingsignal data structure is a list of available pending
 * signal structures.
 */

sq_queue_t  g_sigpendingsignal;

/* The g_sigpendingirqsignal data structure is a list of available
 * pending signal structures that are reserved for use by interrupt
 * handlers.
 */

sq_queue_t  g_sigpendingirqsignal;

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* g_sigactionalloc is a pointer to the start of the allocated blocks of
 * signal actions.
 */

static sigactq_t  *g_sigactionalloc;

/* g_sigpendingactionalloc is a pointer to the start of the allocated
 * blocks of pending signal actions.
 */

static sigq_t     *g_sigpendingactionalloc;

/* g_sigpendingirqactionalloc is a pointer to the start of the allocated
 * block of pending signal actions.
 */

static sigq_t     *g_sigpendingirqactionalloc;

/* g_sigpendingsignalalloc is a pointer to the start of the allocated
 * blocks of pending signals.
 */

static sigpendq_t *g_sigpendingsignalalloc;

/* g_sigpendingirqsignalalloc is a pointer to the start of the allocated
 * blocks of pending signals.
 */

static sigpendq_t *g_sigpendingirqsignalalloc;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static sigq_t     *nxsig_alloc_block(sq_queue_t *siglist, uint16_t nsigs,
                                     uint8_t sigtype);
static sigpendq_t *nxsig_alloc_pendingsignalblock(sq_queue_t *siglist,
                                                  uint16_t nsigs,
                                                  uint8_t sigtype);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsig_alloc_block
 *
 * Description:
 *   Allocate a block of pending signal actions and place them
 *   on the free list.
 *
 ****************************************************************************/

static FAR sigq_t *nxsig_alloc_block(sq_queue_t *siglist, uint16_t nsigs,
                                     uint8_t sigtype)
{
  FAR sigq_t *sigqalloc;
  FAR sigq_t *sigq;
  int i;

  /* Allocate a block of pending signal actions. */

  sigqalloc = (FAR sigq_t *)kmm_malloc((sizeof(sigq_t)) * nsigs);
  if (sigqalloc != NULL)
    {
      sigq = sigqalloc;
      for (i = 0; i < nsigs; i++)
        {
          sigq->type = sigtype;
          sq_addlast((FAR sq_entry_t *)sigq++, siglist);
        }
    }

  return sigqalloc;
}

/****************************************************************************
 * Name: nxsig_alloc_pendingsignalblock
 *
 * Description:
 *   Allocate a block of pending signal structures  and place them on
 *   the free list.
 *
 ****************************************************************************/

static sigpendq_t *nxsig_alloc_pendingsignalblock(sq_queue_t *siglist,
                                                  uint16_t nsigs,
                                                  uint8_t sigtype)
{
  FAR sigpendq_t *sigpendalloc;
  FAR sigpendq_t *sigpend;
  int i;

  /* Allocate a block of pending signal structures  */

  sigpendalloc =
    (FAR sigpendq_t *)kmm_malloc((sizeof(sigpendq_t)) * nsigs);

  if (sigpendalloc != NULL)
    {
      sigpend = sigpendalloc;
      for (i = 0; i < nsigs; i++)
        {
          sigpend->type = sigtype;
          sq_addlast((FAR sq_entry_t *)sigpend++, siglist);
        }
    }

  return sigpendalloc;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsig_initialize
 *
 * Description:
 *   Perform one-time power-up initialization
 *
 ****************************************************************************/

void nxsig_initialize(void)
{
  /* Initialize free lists */

  sq_init(&g_sigfreeaction);
  sq_init(&g_sigpendingaction);
  sq_init(&g_sigpendingirqaction);
  sq_init(&g_sigpendingsignal);
  sq_init(&g_sigpendingirqsignal);

  /* Add a block of signal structures to each list */

  g_sigpendingactionalloc =
    nxsig_alloc_block(&g_sigpendingaction,
                      NUM_PENDING_ACTIONS,
                      SIG_ALLOC_FIXED);
  DEBUGASSERT(g_sigpendingactionalloc != NULL);

  g_sigpendingirqactionalloc =
    nxsig_alloc_block(&g_sigpendingirqaction,
                      NUM_PENDING_INT_ACTIONS,
                      SIG_ALLOC_IRQ);
  DEBUGASSERT(g_sigpendingirqactionalloc != NULL);

  g_sigpendingsignalalloc =
    nxsig_alloc_pendingsignalblock(&g_sigpendingsignal,
                                   NUM_SIGNALS_PENDING,
                                   SIG_ALLOC_FIXED);
  DEBUGASSERT(g_sigpendingsignalalloc != NULL);

  g_sigpendingirqsignalalloc =
    nxsig_alloc_pendingsignalblock(&g_sigpendingirqsignal,
                                   NUM_INT_SIGNALS_PENDING,
                                   SIG_ALLOC_IRQ);
  DEBUGASSERT(g_sigpendingirqsignalalloc != NULL);
}

/****************************************************************************
 * Name: nxsig_alloc_actionblock
 *
 * Description:
 *   Allocate a block of signal actions and place them
 *   on the free list.
 *
 ****************************************************************************/

void nxsig_alloc_actionblock(void)
{
  FAR sigactq_t *sigact;
  int i;

  /* Allocate a block of signal actions */

  g_sigactionalloc =
    (FAR sigactq_t *)kmm_malloc((sizeof(sigactq_t)) * NUM_SIGNAL_ACTIONS);

  if (g_sigactionalloc != NULL)
    {
      sigact = g_sigactionalloc;
      for (i = 0; i < NUM_SIGNAL_ACTIONS; i++)
        {
          sq_addlast((FAR sq_entry_t *)sigact++, &g_sigfreeaction);
        }
    }
}
