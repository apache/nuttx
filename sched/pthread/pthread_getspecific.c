/************************************************************************
 * sched/pthread/pthread_getspecific.c
 *
 *   Copyright (C) 2007, 2009, 2013 Gregory Nutt. All rights reserved.
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
 ************************************************************************/

/************************************************************************
 * Included Files
 ************************************************************************/

#include <nuttx/config.h>

#include <sched.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include "sched/sched.h"
#include "pthread/pthread.h"

/************************************************************************
 * Pre-processor Definitions
 ************************************************************************/

/************************************************************************
 * Private Type Declarations
 ************************************************************************/

/************************************************************************
 * Global Variables
 ************************************************************************/

/************************************************************************
 * Private Variables
 ************************************************************************/

/************************************************************************
 * Private Functions
 ************************************************************************/

/************************************************************************
 * Public Functions
 ************************************************************************/

/************************************************************************
 * Name: pthread_getspecific
 *
 * Description:
 *   The pthread_getspecific() function returns the value currently
 *   bound to the specified key on behalf of the calling thread.
 *
 *   The effect of calling pthread_getspecific() with with a key value
 *   not obtained from pthread_create() or after a key has been deleted
 *   with pthread_key_delete() is undefined.
 *
 * Parameters:
 *   key = The data key to get or set
 *
 * Return Value:
 *   The function pthread_getspecific() returns the thread-specific data
 *   associated with the given key.  If no thread specific data is
 *   associated with the key, then the value NULL is returned.
 *
 *      EINVAL - The key value is invalid.
 *
 * Assumptions:
 *
 * POSIX Compatibility:
 *   - Both calling pthread_setspecific() and pthread_getspecific()
 *     may be called from a thread-specific data destructor
 *     function.
 *
 ************************************************************************/

FAR void *pthread_getspecific(pthread_key_t key)
{
#if CONFIG_NPTHREAD_KEYS > 0
  FAR struct pthread_tcb_s *rtcb = (FAR struct pthread_tcb_s*)g_readytorun.head;
  FAR struct task_group_s *group = rtcb->cmn.group;
  FAR void *ret = NULL;

  DEBUGASSERT(group &&
              (rtcb->cmn.flags & TCB_FLAG_TTYPE_MASK) == TCB_FLAG_TTYPE_PTHREAD);

  /* Check if the key is valid. */

  if (key < group->tg_nkeys)
    {
      /* Return the stored value. */

      ret = rtcb->pthread_data[key];
    }

  return ret;
#else
  return NULL;
#endif
}

