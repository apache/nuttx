/****************************************************************************
 * mm/iob/iob.h
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

#ifndef __MM_IOB_IOB_H
#define __MM_IOB_IOB_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>

#include <nuttx/mm/iob.h>
#include <nuttx/semaphore.h>

#ifdef CONFIG_MM_IOB

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_DEBUG_FEATURES) && defined(CONFIG_IOB_DEBUG)
#  define ioberr                 _err
#  define iobwarn                _warn
#  define iobinfo                _info
#else
#  define ioberr                 _none
#  define iobwarn                _none
#  define iobinfo                _none
#endif /* CONFIG_DEBUG_FEATURES && CONFIG_IOB_DEBUG */

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* A list of all free, unallocated I/O buffers */

extern FAR struct iob_s *g_iob_freelist;

/* A list of I/O buffers that are committed for allocation */

extern FAR struct iob_s *g_iob_committed;

#if CONFIG_IOB_NCHAINS > 0
/* A list of all free, unallocated I/O buffer queue containers */

extern FAR struct iob_qentry_s *g_iob_freeqlist;

/* A list of I/O buffer queue containers that are committed for allocation */

extern FAR struct iob_qentry_s *g_iob_qcommitted;
#endif

/* Counting semaphores that tracks the number of free IOBs/qentries */

extern sem_t g_iob_sem;       /* Counts free I/O buffers */
#if CONFIG_IOB_THROTTLE > 0
extern sem_t g_throttle_sem;  /* Counts available I/O buffers when throttled */
#endif
#if CONFIG_IOB_NCHAINS > 0
extern sem_t g_qentry_sem;    /* Counts free I/O buffer queue containers */
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: iob_alloc_qentry
 *
 * Description:
 *   Allocate an I/O buffer chain container by taking the buffer at the head
 *   of the free list. This function is intended only for internal use by
 *   the IOB module.
 *
 ****************************************************************************/

FAR struct iob_qentry_s *iob_alloc_qentry(void);

/****************************************************************************
 * Name: iob_tryalloc_qentry
 *
 * Description:
 *   Try to allocate an I/O buffer chain container by taking the buffer at
 *   the head of the free list without waiting for the container to become
 *   free. This function is intended only for internal use by the IOB module.
 *
 ****************************************************************************/

FAR struct iob_qentry_s *iob_tryalloc_qentry(void);

/****************************************************************************
 * Name: iob_free_qentry
 *
 * Description:
 *   Free the I/O buffer chain container by returning it to the  free list.
 *   The link to  the next I/O buffer in the chain is return.
 *
 ****************************************************************************/

FAR struct iob_qentry_s *iob_free_qentry(FAR struct iob_qentry_s *iobq);

/****************************************************************************
 * Name: iob_notifier_signal
 *
 * Description:
 *   An IOB has become available.  Signal all threads waiting for an IOB
 *   that an IOB is available.
 *
 *   When an IOB becomes available, *all* of the waiters in this thread will
 *   be signaled.  If there are multiple waiters then only the highest
 *   priority thread will get the IOB.  Lower priority threads will need to
 *   call iob_notifier_setup() once again.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_IOB_NOTIFIER
void iob_notifier_signal(void);
#endif

#endif /* CONFIG_MM_IOB */
#endif /* __MM_IOB_IOB_H */
