/****************************************************************************
 * sched/paging/paging.h
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

#ifndef __SCHED_PAGING_PAGING_H
#define __SCHED_PAGING_PAGING_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <signal.h>

#ifdef CONFIG_PAGING

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Supply reasonable (but probably non-optimal) default settings if
 * configuration items are omitted.
 */

#ifndef CONFIG_PAGING_DEFPRIO
#  define CONFIG_PAGING_DEFPRIO 50
#endif

#ifndef CONFIG_PAGING_WORKPERIOD
#  define CONFIG_PAGING_WORKPERIOD (500*1000) /* 1/2 second */
#endif

#ifndef CONFIG_PAGING_STACKSIZE
#  define CONFIG_PAGING_STACKSIZE  CONFIG_IDLETHREAD_STACKSIZE
#endif

#define SIGPAGING SIGRTMIN

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY

/* This is the task IDof the page fill worker thread.  This value was set in
 * nx_start when the page fill worker thread was started.
 */

extern pid_t g_pgworker;

/* The page fill worker thread maintains a static variable called g_pftcb.
 * If no page fill is in progress, g_pftcb will be NULL. Otherwise, g_pftcb
 * will point to the TCB of the task which is receiving the fill that is
 * in progress.
 *
 * NOTE: I think that this is the only state in which a TCB does not reside
 * in some list.  Here is it in limbo, outside of the normally queuing while
 * the page file is in progress.  Where here, it will be marked with
 * TSTATE_TASK_INVALID.
 */

extern FAR struct tcb_s *g_pftcb;

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
 * Input Parameters:
 *   argc, argv (not used)
 *
 * Returned Value:
 *   Does not return
 *
 ****************************************************************************/

int pg_worker(int argc, char *argv[]);

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_PAGING */
#endif /* __SCHED_PAGING_PAGING_H */
