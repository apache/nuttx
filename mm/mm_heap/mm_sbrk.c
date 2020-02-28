/****************************************************************************
 * mm/mm_heap/mm_sbrk.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/mm/mm.h>
#include <nuttx/pgalloc.h>

#ifdef CONFIG_BUILD_KERNEL

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mm_sbrk
 *
 * Description:
 *    The sbrk() function is used to change the amount of space allocated
 *    for the calling process. The change is made by resetting the process's
 *    break value and allocating the appropriate amount of space.  The amount
 *    of allocated space increases as the break value increases.
 *
 *    The sbrk() function adds 'incr' bytes to the break value and changes
 *    the allocated space accordingly. If incr is negative, the amount of
 *    allocated space is decreased by incr bytes. The current value of the
 *    program break is returned by sbrk(0).
 *
 * Input Parameters:
 *    heap - A reference to the data structure that defines this heap.
 *    incr - Specifies the number of bytes to add or to remove from the
 *      space allocated for the process.
 *    maxbreak - The maximum permissible break address.
 *
 * Returned Value:
 *    Upon successful completion, sbrk() returns the prior break value.
 *    Otherwise, it returns (void *)-1 and sets errno to indicate the
 *    error:
 *
 *      ENOMEM - The requested change would allocate more space than
 *        allowed under system limits.
 *      EAGAIN - The total amount of system memory available for allocation
 *        to this process is temporarily insufficient. This may occur even
 *        though the space requested was less than the maximum data segment
 *        size.
 *
 ****************************************************************************/

FAR void *mm_sbrk(FAR struct mm_heap_s *heap, intptr_t incr,
                  uintptr_t maxbreak)
{
  uintptr_t brkaddr;
  uintptr_t allocbase;
  unsigned int pgincr;
  size_t bytesize;
  int errcode;

  DEBUGASSERT(incr >= 0);
  if (incr < 0)
    {
      errcode = ENOSYS;
      goto errout;
    }

  /* Get the current break address (NOTE: assumes region 0).  If
   * the memory manager is uninitialized, mm_brkaddr() will return
   * zero.
   */

  brkaddr = (uintptr_t)mm_brkaddr(heap, 0);
  if (incr > 0)
    {
      /* Convert the increment to multiples of the page size */

      pgincr = MM_NPAGES(incr);

      /* Check if this increment would exceed the maximum break value */

      if ((brkaddr > 0) && ((maxbreak - brkaddr) < (pgincr << MM_PGSHIFT)))
        {
          errcode = ENOMEM;
          goto errout;
        }

      /* Allocate the requested number of pages and map them to the
       * break address.  If we provide a zero brkaddr to pgalloc(),  it
       * will create the first block in the correct virtual address
       * space and return the start address of that block.
       */

      allocbase = pgalloc(brkaddr, pgincr);
      if (allocbase == 0)
        {
          errcode = EAGAIN;
          goto errout;
        }

      /* Has the been been initialized?  brkaddr will be zero if the
       * memory manager has not yet been initialized.
       */

      bytesize = pgincr << MM_PGSHIFT;
      if (brkaddr == 0)
        {
          /* No... then initialize it now */

          mm_initialize(heap, (FAR void *)allocbase, bytesize);
        }
      else
        {
          /* Extend the heap (region 0) */

          mm_extend(heap, (FAR void *)allocbase, bytesize, 0);
        }
    }

  return (FAR void *)brkaddr;

errout:
  set_errno(errcode);
  return (FAR void *)-1;
}
#endif /* CONFIG_BUILD_KERNEL */
