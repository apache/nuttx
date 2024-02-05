/****************************************************************************
 * libs/libc/semaphore/sem_open.c
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

#include <stdarg.h>
#include <string.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/semaphore.h>

#ifdef CONFIG_FS_NAMED_SEMAPHORES

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sem_open
 *
 * Description:
 *   This function establishes a connection between named semaphores and a
 *   task.  Following a call to sem_open() with the semaphore name, the task
 *   may reference the semaphore associated with name using the address
 *   returned by this call.  The semaphore may be used in subsequent calls
 *   to sem_wait(), sem_trywait(), and sem_post().  The semaphore remains
 *   usable until the semaphore is closed by a successful call to
 *   sem_close().
 *
 *   If a task makes multiple calls to sem_open() with the same name, then
 *   the same semaphore address is returned (provided there have been no
 *   calls to sem_unlink()).
 *
 * Input Parameters:
 *   name  - Semaphore name
 *   oflags - Semaphore creation options.  This may either or both of the
 *     following bit settings.
 *     oflags = 0:  Connect to the semaphore only if it already exists.
 *     oflags = O_CREAT:  Connect to the semaphore if it exists, otherwise
 *        create the semaphore.
 *     oflags = O_CREAT|O_EXCL:  Create a new semaphore
 *        unless one of this name already exists.
 *   Optional parameters.  When the O_CREAT flag is specified, two optional
 *     parameters are expected:
 *     1. mode_t mode, and
 *     2. unsigned int value.  This initial value of the semaphore. Valid
 *        initial values of the semaphore must be less than or equal to
 *        SEM_VALUE_MAX.
 *
 * Returned Value:
 *   A pointer to sem_t or SEM_FAILED if unsuccessful.
 *
 * Assumptions:
 *
 ****************************************************************************/

FAR sem_t *sem_open(FAR const char *name, int oflags, ...)
{
  FAR sem_t *sem;
  va_list ap;
  mode_t mode;
  unsigned int value;

  /* Make sure that a non-NULL name is supplied */

  DEBUGASSERT(name != NULL);

  if (name[0] == '/')
    {
      if (strlen(name) >= PATH_MAX)
        {
          set_errno(ENAMETOOLONG);
          return SEM_FAILED;
        }

      if (strlen(strrchr(name, '/') + 1) >= NAME_MAX)
        {
          set_errno(ENAMETOOLONG);
          return SEM_FAILED;
        }
    }

  /* Off-load the variadic list */

  va_start(ap, oflags);
  mode = va_arg(ap, mode_t);
  value = va_arg(ap, unsigned int);
  va_end(ap);

  /* Let nxsem_open() do the work */

  sem = nxsem_open(name, oflags, mode, value);
  if (sem < 0)
    {
      set_errno(-((intptr_t)sem));
      return SEM_FAILED;
    }

  return sem;
}

#endif /* CONFIG_FS_NAMED_SEMAPHORES */
