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
#include <assert.h>

#include <nuttx/kmalloc.h>
#include <nuttx/queue.h>
#include <nuttx/trace.h>

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
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsig_init_block
 *
 * Description:
 *   Initialize a block of pending signal actions and place them
 *   on the free list.
 *
 ****************************************************************************/

static void *nxsig_init_block(sq_queue_t *siglist, FAR sigq_t *sigq,
                              uint16_t nsigs, uint8_t sigtype)
{
  int i;

  for (i = 0; i < nsigs; i++)
    {
      sigq->type = sigtype;
      sq_addlast((FAR sq_entry_t *)sigq++, siglist);
    }

  return sigq;
}

/****************************************************************************
 * Name: nxsig_init_pendingsignalblock
 *
 * Description:
 *   Initialize a block of pending signal structures  and place them on
 *   the free list.
 *
 ****************************************************************************/

static void *nxsig_init_pendingsignalblock(FAR sq_queue_t *siglist,
                                           FAR sigpendq_t *sigpend,
                                           uint16_t nsigs,
                                           uint8_t sigtype)
{
  int i;

  for (i = 0; i < nsigs; i++)
    {
      sigpend->type = sigtype;
      sq_addlast((FAR sq_entry_t *)sigpend++, siglist);
    }

  return sigpend;
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
  FAR void *sigpool;

  sched_trace_begin();

  /* Initialize free lists */

  sq_init(&g_sigfreeaction);
  sq_init(&g_sigpendingaction);
  sq_init(&g_sigpendingirqaction);
  sq_init(&g_sigpendingsignal);
  sq_init(&g_sigpendingirqsignal);

  /* Add a block of signal structures to each list */

  sigpool =
    kmm_malloc(sizeof(sigq_t) *
               (NUM_PENDING_ACTIONS + CONFIG_SIG_PREALLOC_IRQ_ACTIONS)
               + sizeof(sigpendq_t) *
                 (NUM_SIGNALS_PENDING + CONFIG_SIG_PREALLOC_IRQ_ACTIONS));

  DEBUGASSERT(sigpool != NULL);

  sigpool = nxsig_init_block(&g_sigpendingaction, sigpool,
                             NUM_PENDING_ACTIONS, SIG_ALLOC_FIXED);
  sigpool = nxsig_init_block(&g_sigpendingirqaction, sigpool,
                             CONFIG_SIG_PREALLOC_IRQ_ACTIONS,
                             SIG_ALLOC_IRQ);
  sigpool = nxsig_init_pendingsignalblock(&g_sigpendingsignal, sigpool,
                                          NUM_SIGNALS_PENDING,
                                          SIG_ALLOC_FIXED);
  sigpool = nxsig_init_pendingsignalblock(&g_sigpendingirqsignal, sigpool,
                                          CONFIG_SIG_PREALLOC_IRQ_ACTIONS,
                                          SIG_ALLOC_IRQ);
  sched_trace_end();
}
