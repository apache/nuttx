/****************************************************************************
 * sched/pg_internal.h
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#ifndef __SCHED_PG_INTERNAL_H
#define __SCHED_PG_INTERNAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/
 
#include <nuttx/config.h>
#include <queue.h>

#ifdef CONFIG_PAGING

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_PAGING_WORKPRIORITY
#  define CONFIG_PAGING_WORKPRIORITY 50
#endif

#ifndef CONFIG_PAGING_WORKSTACKSIZE
#  define CONFIG_PAGING_WORKSTACKSIZE CONFIG_IDLETHREAD_STACKSIZE
#endif

#ifdef CONFIG_DISABLE_SIGNALS
#  warning "Page fill support requires signals"
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY

/* If is the PID of the page fill worker thread */

extern pid_t g_pgworker;

/* If non-NULL, this points to the TCB of the task currently being filled */

extern FAR _TCB *g_filltcb;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: pg_worker
 *
 * Description:
 *   This is the entry point of the worker thread that performs the actual
 *   page file.
 *
 * Input parameters:
 *   argc, argv (not used)
 *
 * Returned Value:
 *   Does not return
 *
 ****************************************************************************/

extern int pg_worker(int argc, char *argv[]);

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_PAGING */
#endif /* __SCHED_PG_INTERNAL_H */
