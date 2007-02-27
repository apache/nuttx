/************************************************************
 * mm_initialize.c
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

#include "mm_environment.h"
#include "mm_internal.h"

/************************************************************
 * Definitions
 ************************************************************/

/************************************************************
 * Public Variables
 ************************************************************/

/* This is the size of the heap provided to mm */

size_t g_heapsize;

/* This is the first and last nodes of the heap */

FAR struct mm_allocnode_s *g_heapstart;
FAR struct mm_allocnode_s *g_heapend;

/* All free nodes are maintained in a doubly linked list.  This
 * array provides some hooks into the list at various points to
 * speed searches for free nodes.
 */

FAR struct mm_freenode_s g_nodelist[MM_NNODES];

/************************************************************
 * Public Functions
 ************************************************************/

/************************************************************
 * Function:  mm_initialize
 *
 * Description:
 *   This is an internal OS function called only at power-up
 *   boot time.
 *
 * Parameters:
 *   None
 *
 * Return Value:
 *   None
 *
 * Assumptions:
 *
 ************************************************************/

void mm_initialize(FAR void *heapstart, size_t heapsize)
{
  FAR struct mm_freenode_s *node;
  size_t heapbase;
  size_t heapend;
  int i;

  CHECK_ALLOCNODE_SIZE;
  CHECK_FREENODE_SIZE;

  /* Adjust the provide heap start and size so that they are
   * both aligned with the MM_MIN_CHUNK size.
   */

  heapbase = MM_ALIGN_UP((size_t)heapstart);
  heapend  = MM_ALIGN_DOWN((size_t)heapstart + (size_t)heapsize);

  /* Save the size of the heap */

  g_heapsize = heapend - heapbase;

  /* Initialize the node array */

  memset(g_nodelist, 0, sizeof(struct mm_freenode_s) * MM_NNODES);
  for (i = 1; i < MM_NNODES; i++)
    {
      g_nodelist[i-1].flink = &g_nodelist[i];
      g_nodelist[i].blink   = &g_nodelist[i-1];
    }

  /* Create two "allocated" guard nodes at the beginning and end of
   * the heap.  These only serve to keep us from allocating outside
   * of the heap.
   * 
   * And create one free node between the guard nodes that contains
   * all available memory.
   */

  g_heapstart            = (FAR struct mm_allocnode_s *)heapbase;
  g_heapstart->size      = SIZEOF_MM_ALLOCNODE;
  g_heapstart->preceding = MM_ALLOC_BIT;

  node                   = (FAR struct mm_freenode_s *)(heapbase + SIZEOF_MM_ALLOCNODE);
  node->size             = g_heapsize - 2*SIZEOF_MM_ALLOCNODE;
  node->preceding        = SIZEOF_MM_ALLOCNODE;

  g_heapend              = (FAR struct mm_allocnode_s *)(heapend - SIZEOF_MM_ALLOCNODE);
  g_heapend->size        = SIZEOF_MM_ALLOCNODE;
  g_heapend->preceding   = node->size | MM_ALLOC_BIT;

  /* Add the single, large free node to the nodelist */

  mm_addfreechunk(node);

  /* Initialize the malloc semaphore to one (to support one-at-
   * a-time access to private data sets.
   */

  mm_seminitialize();
}
