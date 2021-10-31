/****************************************************************************
 * libs/libc/stdio/lib_libfilesem.c
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

#include <sys/types.h>
#include <unistd.h>
#include <errno.h>
#include <assert.h>

#include <nuttx/semaphore.h>
#include <nuttx/fs/fs.h>

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

  _SEM_INIT(&stream->fs_sem, 0, 1);

  stream->fs_holder = -1;
  stream->fs_counts = 0;
}

/****************************************************************************
 * lib_take_semaphore
 ****************************************************************************/

void lib_take_semaphore(FAR struct file_struct *stream)
{
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

          DEBUGASSERT(_SEM_ERRNO(ret) == EINTR ||
                      _SEM_ERRNO(ret) == ECANCELED);
          UNUSED(ret);
        }

      /* We have it.  Claim the stak and return */

      stream->fs_holder = my_pid;
      stream->fs_counts = 1;
    }
}

/****************************************************************************
 * lib_give_semaphore
 ****************************************************************************/

void lib_give_semaphore(FAR struct file_struct *stream)
{
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
}

#endif /* CONFIG_STDIO_DISABLE_BUFFERING */
