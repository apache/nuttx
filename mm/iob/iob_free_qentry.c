/****************************************************************************
 * mm/iob/iob_free_qentry.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/mm/iob.h>

#include "iob.h"

#if CONFIG_IOB_NCHAINS > 0

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: iob_free_qentry
 *
 * Description:
 *   Free the I/O buffer chain container by returning it to the free list.
 *   The link to  the next I/O buffer in the chain is return.
 *
 ****************************************************************************/

FAR struct iob_qentry_s *iob_free_qentry(FAR struct iob_qentry_s *iobq)
{
  FAR struct iob_qentry_s *nextq = iobq->qe_flink;
  irqstate_t flags;

  /* Free the I/O buffer chain container by adding it to the head of the
   * free or the committed list. We don't know what context we are called
   * from so we use extreme measures to protect the free list:  We disable
   * interrupts very briefly.
   */

  flags = spin_lock_irqsave(&g_iob_lock);

  /* Which list?  If there is a task waiting for an IOB chain, then put
   * the IOB chain on either the free list or on the committed list where
   * it is reserved for that allocation (and not available to
   * iob_tryalloc_qentry()).
   */

  if (g_qentry_count < 0)
    {
      iobq->qe_flink   = g_iob_qcommitted;
      g_iob_qcommitted = iobq;
      g_qentry_count++;
      spin_unlock_irqrestore(&g_iob_lock, flags);
      nxsem_post(&g_qentry_sem);
    }
  else
    {
      g_qentry_count++;
      iobq->qe_flink   = g_iob_freeqlist;
      g_iob_freeqlist  = iobq;
      spin_unlock_irqrestore(&g_iob_lock, flags);
    }

  /* And return the I/O buffer chain container after the one that was freed */

  return nextq;
}

#endif /* CONFIG_IOB_NCHAINS > 0 */
