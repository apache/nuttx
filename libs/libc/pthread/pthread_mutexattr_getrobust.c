/****************************************************************************
 * libs/libc/pthread/pthread_mutexattr_getrobust.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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
#include <pthread.h>
#include <errno.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_mutexattr_getrobust
 *
 * Description:
 *   Return the mutex robustneess from the mutex attributes.
 *
 * Input Parameters:
 *   attr   - The mutex attributes to query
 *   robust - Location to return the robustness indication
 *
 * Returned Value:
 *   0, if the robustness was successfully return in 'robust', or
 *   EINVAL, if any NULL pointers provided.
 *
 * Assumptions:
 *
 ****************************************************************************/

int pthread_mutexattr_getrobust(FAR const pthread_mutexattr_t *attr,
                                FAR int *robust)
{
  if (attr != NULL && robust != NULL)
    {
#if defined(CONFIG_PTHREAD_MUTEX_UNSAFE)
      *robust = PTHREAD_MUTEX_STALLED;
#elif defined(CONFIG_PTHREAD_MUTEX_BOTH)
      *robust = attr->robust;
#else /* Default: CONFIG_PTHREAD_MUTEX_ROBUST */
      *robust = PTHREAD_MUTEX_ROBUST;
#endif
      return 0;
    }

  return EINVAL;
}
