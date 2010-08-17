/****************************************************************************
 * arch/arm/src/arm/up_allocpage.c
 * Allocate a new page and map it to the fault address of a task.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>

#include <nuttx/sched.h>
#include <nuttx/page.h>

#ifdef CONFIG_PAGING

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_allocpage()
 *
 * Description:
 *  This architecture-specific function will set aside page in memory and map
 *  the page to its correct virtual address.  Architecture-specific context
 *  information saved within the TCB will provide the function with the
 *  information needed to identify the virtual miss address.
 *
 *  This function will return the allocated physical page address in vpage.
 *  The size of the underlying physical page is determined by the
 *  configuration setting CONFIG_PAGING_PAGESIZE.
 *
 *  NOTE 1: This function must always return a page allocation. If all
 *  available pages are in-use (the typical case), then this function will
 *  select a page in-use, un-map it, and make it available.
 *
 *  NOTE 2: Allocating and filling a page is a two step process.  up_allocpage()
 *  allocates the page, and up_fillpage() fills it with data from some non-
 *  volatile storage device.  This distinction is made because up_allocpage()
 *  can probably be implemented in board-independent logic whereas up_fillpage()
 *  probably must be implemented as board-specific logic.
 *
 * Input Parameters:
 *   tcb - A reference to the task control block of the task that needs to
 *         have a page fill.  Architecture-specific logic can retrieve page
 *         fault information from the architecture-specific context
 *         information in this TCB to perform the mapping.
 *
 * Returned Value:
 *   This function will return zero (OK) if the allocation was successful.
 *   A negated errno value may be returned if an error occurs.  All errors,
 *   however, are fatal.
 *
 * Assumptions:
 *   - This function is called from the normal tasking context (but with
 *     interrupts disabled).  The implementation must take whatever actions
 *     are necessary to assure that the operation is safe within this
 *     context.
 *
 ****************************************************************************/

int up_allocpage(FAR _TCB *tcb, FAR void **vpage);
{
# warning "Not implemented"
  return -ENOSYS;
}

#endif /* CONFIG_PAGING */
