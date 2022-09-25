/****************************************************************************
 * sched/wdog/wdog.h
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

#ifndef __SCHED_WDOG_WDOG_H
#define __SCHED_WDOG_WDOG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/compiler.h>
#include <nuttx/clock.h>
#include <nuttx/queue.h>
#include <nuttx/wdog.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Name: wd_elapse
 *
 * Description:
 *   This function is used to get time-elapse from last time wd_timer() be
 *   called. In case of CONFIG_SCHED_TICKLESS configured, wd_timer() may
 *   take lots of ticks, during this time, wd_start()/wd_cancel() may
 *   called, so we need wd_elapse() to correct the delay/lag.
 *
 ****************************************************************************/

#ifdef CONFIG_SCHED_TICKLESS
#  define wd_elapse() (clock_systime_ticks() - g_wdtickbase)
#else
#  define wd_elapse() (0)
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/* The g_wdactivelist data structure is a singly linked list ordered by
 * watchdog expiration time. When watchdog timers expire,the functions on
 * this linked list are removed and the function is called.
 */

extern sq_queue_t g_wdactivelist;

/* This is wdog tickbase, for wd_gettime() may called many times
 * between 2 times of wd_timer(), we use it to update wd_gettime().
 */

#ifdef CONFIG_SCHED_TICKLESS
extern clock_t g_wdtickbase;
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: wd_timer
 *
 * Description:
 *   This function is called from the timer interrupt handler to determine
 *   if it is time to execute a watchdog function.  If so, the watchdog
 *   function will be executed in the context of the timer interrupt
 *   handler.
 *
 * Input Parameters:
 *   ticks - If CONFIG_SCHED_TICKLESS is defined then the number of ticks
 *     in the interval that just expired is provided.  Otherwise,
 *     this function is called on each timer interrupt and a value of one
 *     is implicit.
 *   noswitches - True: Can't do context switches now.
 *
 * Returned Value:
 *   If CONFIG_SCHED_TICKLESS is defined then the number of ticks for the
 *   next delay is provided (zero if no delay).  Otherwise, this function
 *   has no returned value.
 *
 * Assumptions:
 *   Called from interrupt handler logic with interrupts disabled.
 *
 ****************************************************************************/

#ifdef CONFIG_SCHED_TICKLESS
unsigned int wd_timer(int ticks, bool noswitches);
#else
void wd_timer(void);
#endif

/****************************************************************************
 * Name: wd_recover
 *
 * Description:
 *   This function is called from nxtask_recover() when a task is deleted via
 *   task_delete() or via pthread_cancel(). It checks if the deleted task
 *   is waiting for a timed event and if so cancels the timeout
 *
 * Input Parameters:
 *   tcb - The TCB of the terminated task or thread
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *   This function is called from task deletion logic in a safe context.
 *
 ****************************************************************************/

struct tcb_s;
void wd_recover(FAR struct tcb_s *tcb);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __SCHED_WDOG_WDOG_H */
