/****************************************************************************
 * fs/semaphore/sem_open.c
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

#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <sched.h>
#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/semaphore.h>
#include <nuttx/fs/fs.h>

#include "inode/inode.h"
#include "semaphore/semaphore.h"

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
 *     1. mode_t mode (ignored), and
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

FAR sem_t *sem_open (FAR const char *name, int oflags, ...)
{
  FAR struct inode *inode;
  FAR struct nsem_inode_s *nsem;
  FAR sem_t *sem = (FAR sem_t *)ERROR;
  struct inode_search_s desc;
  char fullpath[MAX_SEMPATH];
  mode_t mode;
  unsigned value;
  int errcode;
  int ret;

  /* Make sure that a non-NULL name is supplied */

  DEBUGASSERT(name != NULL);

  /* The POSIX specification requires that the "check for the existence
   * of a semaphore and the creation of the semaphore if it does not
   * exist shall be atomic with respect to other processes executing
   * sem_open()..."  A simple sched_lock() should be sufficient to meet
   * this requirement.
   */

  sched_lock();

  /* Get the full path to the semaphore */

  snprintf(fullpath, MAX_SEMPATH, CONFIG_FS_NAMED_SEMPATH "/%s", name);

  /* Get the inode for this semaphore.  This should succeed if the
   * semaphore has already been created.  In this case, inode_find()
   * will have incremented the reference count on the inode.
   */

  SETUP_SEARCH(&desc, fullpath, false);

  ret = inode_find(&desc);
  if (ret >= 0)
    {
      /* Something exists at this path.  Get the search results */

      inode = desc.node;
      DEBUGASSERT(inode != NULL);

      /* Verify that the inode is a semaphore */

      if (!INODE_IS_NAMEDSEM(inode))
        {
          errcode = ENXIO;
          goto errout_with_inode;
        }

      /* It exists and is a semaphore.  Check if the caller wanted to
       * create a new semaphore with this name.
       */

      if ((oflags & (O_CREAT | O_EXCL)) == (O_CREAT | O_EXCL))
        {
          errcode = EEXIST;
          goto errout_with_inode;
        }

      /* Return a reference to the semaphore, retaining the reference
       * count on the inode.
       */

      sem = &inode->u.i_nsem->ns_sem;
    }
  else
    {
      va_list ap;

      /* The semaphore does not exists.  Were we asked to create it? */

      if ((oflags & O_CREAT) == 0)
        {
          /* The semaphore does not exist and O_CREAT is not set */

          errcode = ENOENT;
          goto errout_with_lock;
        }

      /* Create the semaphore.  First we have to extract the additional
       * parameters from the variable argument list.
       * REVISIT:  Mode parameter is not currently used.
       */

      va_start(ap, oflags);
      mode  = va_arg(ap, mode_t);
      value = va_arg(ap, unsigned);
      va_end(ap);

      UNUSED(mode);

      /* Check the semaphore value */

      if (value > SEM_VALUE_MAX)
        {
          errcode = EINVAL;
          goto errout_with_lock;
        }

      /* Create an inode in the pseudo-filesystem at this path.  The new
       * inode will be created with a reference count of zero.
       */

      ret = inode_semtake();
      if (ret < 0)
        {
          errcode = -ret;
          goto errout_with_lock;
        }

      ret = inode_reserve(fullpath, &inode);
      inode_semgive();

      if (ret < 0)
        {
          errcode = -ret;
          goto errout_with_lock;
        }

      /* Allocate the semaphore structure (using the appropriate allocator
       * for the group)
       */

      nsem = group_malloc(NULL, sizeof(struct nsem_inode_s));
      if (!nsem)
        {
          errcode = ENOMEM;
          goto errout_with_inode;
        }

      /* Link to the inode */

      inode->u.i_nsem = nsem;
      nsem->ns_inode  = inode;

      /* Initialize the inode */

      INODE_SET_NAMEDSEM(inode);
      inode->i_crefs = 1;

      /* Initialize the semaphore */

      nxsem_init(&nsem->ns_sem, 0, value);

      /* Return a reference to the semaphore */

      sem = &nsem->ns_sem;
    }

  RELEASE_SEARCH(&desc);
  sched_unlock();
  return sem;

errout_with_inode:
  inode_release(inode);

errout_with_lock:
  RELEASE_SEARCH(&desc);
  set_errno(errcode);
  sched_unlock();
  return SEM_FAILED;
}

#endif /* CONFIG_FS_NAMED_SEMAPHORES */
