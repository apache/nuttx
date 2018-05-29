/****************************************************************************
 * libs/libc/pthread/pthread_setcanceltype.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
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

#include <pthread.h>
#include <sched.h>
#include <errno.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* The following are defined in different header files but must have the
 * same values.
 */

#if PTHREAD_CANCEL_DEFERRED != TASK_CANCEL_DEFERRED
#  error We must have  PTHREAD_CANCEL_DEFERRED == TASK_CANCEL_DEFERRED
#endif

#if PTHREAD_CANCEL_ASYNCHRONOUS != TASK_CANCEL_ASYNCHRONOUS
#  error We must have  PTHREAD_CANCEL_ASYNCHRONOUS == TASK_CANCEL_ASYNCHRONOUS
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_setcancelstate
 *
 * Description:
 *   The pthread_setcanceltype() function atomically both sets the calling
 *   thread's cancelability type to the indicated type and returns the
 *   previous cancelability type at the location referenced by oldtype
 *   Legal values for type are PTHREAD_CANCEL_DEFERRED and
 *   PTHREAD_CANCEL_ASYNCHRONOUS.
 *
 *   The cancelability state and type of any newly created threads,
 *   including the thread in which main() was first invoked, are
 *   PTHREAD_CANCEL_ENABLE and PTHREAD_CANCEL_DEFERRED respectively.
 *
 ****************************************************************************/

int pthread_setcanceltype(int type, FAR int *oldtype)
{
  int ret;

  /* task_setcanceltype() can do this */

  ret = task_setcanceltype(type, oldtype);
  if (ret < 0)
    {
      ret = get_errno();
    }

  return ret;
}
