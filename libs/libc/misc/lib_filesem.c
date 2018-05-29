/****************************************************************************
 * libs/libc/misc/lib_filesem.c
 *
 *   Copyright (C) 2007, 2009, 2011, 2017 Gregory Nutt. All rights reserved.
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

#include <sys/types.h>
#include <unistd.h>
#include <semaphore.h>
#include <errno.h>
#include <assert.h>

#include <nuttx/semaphore.h>
#include <nuttx/fs/fs.h>

#ifdef CONFIG_SMP
#  include <nuttx/irq.h>
#endif

#include "libc.h"

#ifndef CONFIG_STDIO_DISABLE_BUFFERING

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * lib_sem_initialize
 ****************************************************************************/

void lib_sem_initialize(FAR struct file_struct *stream)
{
  /* Initialize the LIB semaphore to one (to support one-at-a-time access
   * to private data sets.
   */

  (void)nxsem_init(&stream->fs_sem, 0, 1);

  stream->fs_holder = -1;
  stream->fs_counts = 0;
}

/****************************************************************************
 * lib_take_semaphore
 ****************************************************************************/

void lib_take_semaphore(FAR struct file_struct *stream)
{
#ifdef CONFIG_SMP
  irqstate_t flags = enter_critical_section();
#endif

  pid_t my_pid = getpid();
  int ret;

  /* Do I already have the semaphore? */

  if (stream->fs_holder == my_pid)
    {
      /* Yes, just increment the number of references that I have */

      stream->fs_counts++;
    }
  else
    {
      /* Take the semaphore (perhaps waiting) */

      while ((ret = _SEM_WAIT(&stream->fs_sem)) < 0)
        {
          /* The only case that an error should occr here is if the wait
           * was awakened by a signal.
           */

          DEBUGASSERT(_SEM_ERRNO(ret) == EINTR || _SEM_ERRNO(ret) == ECANCELED);
          UNUSED(ret);
        }

      /* We have it.  Claim the stak and return */

      stream->fs_holder = my_pid;
      stream->fs_counts = 1;
    }

#ifdef CONFIG_SMP
  leave_critical_section(flags);
#endif
}

/****************************************************************************
 * lib_give_semaphore
 ****************************************************************************/

void lib_give_semaphore(FAR struct file_struct *stream)
{
#ifdef CONFIG_SMP
  irqstate_t flags = enter_critical_section();
#endif

  /* I better be holding at least one reference to the semaphore */

  DEBUGASSERT(stream->fs_holder == getpid());

  /* Do I hold multiple references to the semphore */

  if (stream->fs_counts > 1)
    {
      /* Yes, just release one count and return */

      stream->fs_counts--;
    }
  else
    {
      /* Nope, this is the last reference I have */

      stream->fs_holder = -1;
      stream->fs_counts = 0;
      DEBUGVERIFY(_SEM_POST(&stream->fs_sem));
    }

#ifdef CONFIG_SMP
  leave_critical_section(flags);
#endif
}

#endif /* CONFIG_STDIO_DISABLE_BUFFERING */
