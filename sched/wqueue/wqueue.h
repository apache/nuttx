/****************************************************************************
 * sched/wqueue/wqueue.h
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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

#ifndef __SCHED_WQUEUE_WQUEUE_H
#define __SCHED_WQUEUE_WQUEUE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_SCHED_WORKQUEUE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Customize kernel thread names */

#ifdef CONFIG_SCHED_HPWORK
#  if defined(CONFIG_SCHED_LPWORK)
#    define HPWORKNAME "hpwork"
#    define LPWORKNAME "lpwork"
#  elif defined(CONFIG_SCHED_USRWORK)
#    define HPWORKNAME "kwork"
#  else
#    define HPWORKNAME "work"
#  endif
#endif

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef CONFIG_SCHED_HPWORK
/* The state of the kernel mode, high priority work queue. */

extern struct wqueue_s g_hpwork;
#endif

#ifdef CONFIG_SCHED_LPWORK
/* The state of the kernel mode, low priority work queue(s). */

extern struct wqueue_s g_lpwork;
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: work_hpstart
 *
 * Description:
 *   Start the high-priority, kernel-mode work queue.
 *
 * Input parameters:
 *   None
 *
 * Returned Value:
 *   The task ID of the worker thread is returned on success.  A negated
 *   errno value is returned on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SCHED_HPWORK
int work_hpstart(void);
#endif

/****************************************************************************
 * Name: work_lpstart
 *
 * Description:
 *   Start the low-priority, kernel-mode worker thread(s)
 *
 * Input parameters:
 *   None
 *
 * Returned Value:
 *   The task ID of the worker thread is returned on success.  A negated
 *   errno value is returned on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SCHED_LPWORK
int work_lpstart(void);
#endif

#endif /* CONFIG_SCHED_WORKQUEUE */
#endif /* __SCHED_WQUEUE_WQUEUE_H */
