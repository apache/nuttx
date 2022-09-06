/****************************************************************************
 * mm/mm_heap/mm_lock.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <unistd.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/mm/mm.h>

#include "mm_heap/mm.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mm_lock
 *
 * Description:
 *   Take the MM mutex. This may be called from the OS in certain conditions
 *   when it is impossible to wait on a mutex:
 *     1.The idle process performs the memory corruption check.
 *     2.The task/thread free the memory in the exiting process.
 *
 * Input Parameters:
 *   heap  - heap instance want to take mutex
 *
 * Returned Value:
 *   true if the lock can be taken, otherwise false.
 *
 ****************************************************************************/

bool mm_lock(FAR struct mm_heap_s *heap)
{
#if defined(CONFIG_BUILD_FLAT) || defined(__KERNEL__)
  /* Check current environment */

  if (up_interrupt_context())
    {
#if !defined(CONFIG_SMP)
      /* Check the mutex value, if held by someone, then return false.
       * Or, touch the heap internal data directly.
       */

      return !nxmutex_is_locked(&heap->mm_lock);
#else
      /* Can't take mutex in SMP interrupt handler */

      return false;
#endif
    }
  else
#endif

  /* getpid() returns the task ID of the task at the head of the ready-to-
   * run task list.  mm_lock() may be called during context
   * switches.  There are certain situations during context switching when
   * the OS data structures are in flux and then can't be freed immediately
   * (e.g. the running thread stack).
   *
   * This is handled by getpid() to return the special value -ESRCH to
   * indicate this special situation.
   */

  if (getpid() < 0)
    {
      return false;
    }
  else
    {
      return nxmutex_lock(&heap->mm_lock) >= 0;
    }
}

/****************************************************************************
 * Name: mm_unlock
 *
 * Description:
 *   Release the MM mutex when it is not longer needed.
 *
 ****************************************************************************/

void mm_unlock(FAR struct mm_heap_s *heap)
{
#if defined(CONFIG_BUILD_FLAT) || defined(__KERNEL__)
  if (up_interrupt_context())
    {
      return;
    }
#endif

  DEBUGVERIFY(nxmutex_unlock(&heap->mm_lock));
}
