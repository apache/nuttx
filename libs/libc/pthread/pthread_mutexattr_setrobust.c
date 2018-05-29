/****************************************************************************
 * libs/libc/pthread/pthread_mutexattr_setrobust.c
 *
 *   Copyright (C) 201t Gregory Nutt. All rights reserved.
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
 * Name: pthread_mutexattr_setrobust
 *
 * Description:
 *   Set the mutex robustness in the mutex attributes.
 *
 * Input Parameters:
 *   attr   - The mutex attributes in which to set the mutex type.
 *   robust - The mutex type value to set.
 *
 * Returned Value:
 *   0, if the mutex robustness was successfully set in 'attr', or
 *   EINVAL, if 'attr' is NULL or 'robust' unrecognized.
 *
 * Assumptions:
 *
 ****************************************************************************/

int pthread_mutexattr_setrobust(pthread_mutexattr_t *attr, int robust)
{
#if defined(CONFIG_PTHREAD_MUTEX_UNSAFE)

  if (attr != NULL && robust == PTHREAD_MUTEX_STALLED)
    {
      return OK;
    }

  return EINVAL;

#elif defined(CONFIG_PTHREAD_MUTEX_BOTH)

  if (attr != NULL && (robust == PTHREAD_MUTEX_STALLED || robust == _PTHREAD_MFLAGS_ROBUST))
    {
      attr->robust = robust;
      return OK;
    }

  return EINVAL;

#else /* Default: CONFIG_PTHREAD_MUTEX_ROBUST */

  if (attr != NULL && robust == _PTHREAD_MFLAGS_ROBUST)
    {
      return OK;
    }

  return EINVAL;
#endif
}
