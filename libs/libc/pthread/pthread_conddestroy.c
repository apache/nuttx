/****************************************************************************
 * libs/libc/pthread/pthread_conddestroy.c
 *
 *   Copyright (C) 2007-2009, 2017 Gregory Nutt. All rights reserved.
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
#include <debug.h>
#include <errno.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_cond_destroy
 *
 * Description:
 *   A thread can delete condition variables.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK (0) on success; a (non-negated) errno value on failure. The errno
 *   variable is not set. EBUSY is returned when the implementation has
 *   detected an attempt to destroy the object referenced by cond while
 *   it is referenced. EINVAL is returned when cond is invalid.
 *
 * Assumptions:
 *
 ****************************************************************************/

int pthread_cond_destroy(FAR pthread_cond_t *cond)
{
  int ret = OK;
  int sval = 0;

  sinfo("cond=0x%p\n", cond);

  if (!cond)
    {
      ret = EINVAL;
    }

  /* Destroy the semaphore contained in the structure */

  else
    {
      ret = sem_getvalue(&cond->sem, &sval);
      if (ret < 0)
        {
          ret = -ret;
        }
      else
        {
          if (sval < 0)
            {
              ret = EBUSY;
            }
          else if (sem_destroy(&cond->sem) != OK)
            {
              ret = get_errno();
            }
        }
    }

  sinfo("Returning %d\n", ret);
  return ret;
}
