/************************************************************
 * mm_mallinfo.c
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
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
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
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
 ************************************************************/

/************************************************************
 * Included Files
 ************************************************************/

#include <assert.h>
#include "mm_environment.h"
#include "mm_internal.h"

/************************************************************
 * Definitions
 ************************************************************/

/************************************************************
 * Private Data
 ************************************************************/

/************************************************************
 * Private Functions
 ************************************************************/

/************************************************************
 * Public Functions
 ************************************************************/

/************************************************************
 * mallinfo
 *
 * Description:
 *   mallinfo returns a copy of updated current mallinfo.
 *
 ************************************************************/

struct mallinfo mallinfo(void)
{
  static struct mallinfo stats;
  struct mm_allocnode_s *node;
  uint32 mxordblk = 0; 
  int    ordblks  = 0;  /* Number of non-inuse chunks */
  uint32 uordblks = 0;  /* Total allocated space */
  uint32 fordblks = 0;  /* Total non-inuse space */

  /* Visit each node in physical memory */

  for (node = g_heapstart;
       node < g_heapend;
       node = (struct mm_allocnode_s *)((char*)node + node->size))
    {
      if (node->preceding & MM_ALLOC_BIT)
        {
          uordblks += node->size;
        }
      else
        {
          ordblks++;
          fordblks += node->size;
          if (node->size > mxordblk)
            {
              mxordblk = node->size;
            }
        }
    }

  DEBUGASSERT(node == g_heapend);
  uordblks += SIZEOF_MM_ALLOCNODE; /* account for the tail node */
  DEBUGASSERT(uordblks + fordblks == g_heapsize);

  stats.arena    = g_heapsize;
  stats.ordblks  = ordblks;
  stats.mxordblk = mxordblk;
  stats.uordblks = uordblks;
  stats.fordblks = fordblks;
  return stats;
}
