/****************************************************************************
 * include/nuttx/page.h
 * This file defines interfaces used to support NuttX On-Demand Paging.
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

#ifndef __NUTTX_PAGE_H
#define __NUTTX_PAGE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <nuttx/wqueue.h>

#ifdef CONFIG_PAGING

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY
 
#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pg_miss
 *
 * Description:
 *   This function is called from architecture-specific memory segmentation
 *   fault handling logic.  This function will perform the following
 *   operations:
 *
 *   1) Sanity checking.
 *      - ASSERT if the currently executing task is the page fill worker
 *        thread.  The page fill worker thread is how the the page fault
 *        is resolved and all logic associated with the page fill worker
 *        must be "locked" and always present in memory.
 *   2) Block the currently executing task.
 *      - Call up_block_task() to block the task at the head of the ready-
 *        to-run list.  This should cause an interrupt level context switch
 *        to the next highest priority task.
 *      - The blocked task will be marked with state TSTATE_WAIT_PAGEFILL
 *        and will be retained in the g_waitingforfill prioritized task
 *        list.
 *   3) Boost the page fill worker thread priority.
 *      - Check the priority of the task at the head of the g_waitingforfill
 *        list.  If the priority of that task is higher than the current
 *        priority of the page fill worker thread, then boost the priority
 *        of the page fill worker thread to that priority.
 *   4) Signal the page fill worker thread.
 *      - Is there a page fill pending?  If not then signal the worker
 *        thread to start working on the queued page fill requests.
 *
 * Input Parameters:
 *   None - The head of the ready-to-run list is assumed to be task that
 *   caused the exception.
 *
 * Returned Value:
 *   None - Either this function function succeeds or an assertion occurs.
 *
 * Assumptions:
 *   - It is assumed that this function is called from the level of an
 *     exception handler and that all interrupts are disabled.
 *   - It is assumed that currently executing task (the one at the head of
 *     the ready-to-run list) is the one that cause the fault.  This will
 *     always be true unless the page fault occurred in an interrupt handler.
 *     Interrupt handling logic must always be present and "locked" into
 *     memory.
 *   - The chip-specific page fault exception handler has already verified
 *     that the exception did not occur from interrupt/exception handling
 *     logic.
 *   - As mentioned above, the task causing the page fault must not be the
 *     page fill worker thread because that is the only way to complete the
 *     page fill.
 *
 * NOTES:
 *   1. One way to accomplish this would be a two pass link phase:
 *      - In the first phase, create a partially linked objected containing
 *        all interrupt/exception handling logic, the page fill worker thread
 *        plus all parts of the IDLE thread (which must always be available
 *        for execution).
 *      - All of the .text and .rodata sections of this partial link should
 *        be collected into a single section.
 *      - The second link would link the partially linked object along with
 *        the remaining object to produce the final binary.  The linker
 *        script should position the "special" section so that it lies
 *        in a reserved, "non-swappable" region.
 *
 ****************************************************************************/

EXTERN void pg_miss(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY */
#endif /* CONFIG_PAGING */
#endif /* __NUTTX_PAGE_H */
