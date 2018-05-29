/****************************************************************************
 * libs/libc/semaphore/sem_getprotocol.c
 *
 *   Copyright (C) 2016-2017 Gregory Nutt. All rights reserved.
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

#include <assert.h>

#include <nuttx/semaphore.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sem_getprotocol
 *
 * Description:
 *    Return the value of the semaphore protocol attribute.
 *
 * Input Parameters:
 *    sem      - A pointer to the semaphore whose attributes are to be
 *               queried.
 *    protocol - The user provided location in which to store the protocol
 *               value.
 *
 * Returned Value:
 *   This function is exposed as a non-standard application interface.  It
 *   returns zero (OK) if successful.  Otherwise, -1 (ERROR) is returned and
 *   the errno value is set appropriately.
 *
 ****************************************************************************/

int sem_getprotocol(FAR sem_t *sem, FAR int *protocol)
{
  DEBUGASSERT(sem != NULL && protocol != NULL);

#ifdef CONFIG_PRIORITY_INHERITANCE
  if ((sem->flags & PRIOINHERIT_FLAGS_DISABLE) != 0)
    {
      *protocol = SEM_PRIO_NONE;
    }
  else
    {
      *protocol = SEM_PRIO_INHERIT;
    }

#else
  *protocol = SEM_PRIO_NONE;
#endif

  return OK;
}
