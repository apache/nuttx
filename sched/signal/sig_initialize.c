/****************************************************************************
 * sched/signal/sig_initialize.c
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
                      CONFIG_SIG_PREALLOC_IRQ_ACTIONS,
                      SIG_ALLOC_IRQ);
  DEBUGASSERT(g_sigpendingirqactionalloc != NULL);

  g_sigpendingsignalalloc =
    nxsig_alloc_pendingsignalblock(&g_sigpendingsignal,
                                   NUM_SIGNALS_PENDING,
                                   SIG_ALLOC_FIXED);
  DEBUGASSERT(g_sigpendingsignalalloc != NULL);

  g_sigpendingirqsignalalloc =
    nxsig_alloc_pendingsignalblock(&g_sigpendingirqsignal,
                                   CONFIG_SIG_PREALLOC_IRQ_ACTIONS,
                                   SIG_ALLOC_IRQ);
  DEBUGASSERT(g_sigpendingirqsignalalloc != NULL);
}
