/****************************************************************************
 * fs/semaphore/sem_open.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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

#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include <fcntl.h>
#include <semaphore.h>
#include <string.h>
#include <sched.h>
#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/semaphore.h>
#include <nuttx/fs/fs.h>

#include "fs.h"

#ifdef CONFIG_FS_NAMED_SEMAPHORES

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MAX_SEMPATH 64

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Global Variables
 ****************************************************************************/

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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
 *   usable until the semaphore is closed by a successful call to sem_close().
 *
 *   If a task makes multiple calls to sem_open() with the same name, then
 *   the same semaphore address is returned (provided there have been no
 *   calls to sem_unlink()).
 *
 * Parameters:
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
 * Return Value:
 *   A pointer to sem_t or -1 (ERROR) if unsuccessful.
 *
 * Assumptions:
 *
 ****************************************************************************/

FAR sem_t *sem_open (FAR const char *name, int oflags, ...)
{
  FAR struct filelist *list;
  FAR struct inode *inode;
  FAR const char *relpath = NULL;
  mode_t mode;
  FAR struct nsem_inode_s *nsem;
  FAR sem_t *sem = (FAR sem_t*)ERROR;
  char fullpath[MAX_SEMPATH];
  unsigned value;
  int errcode;
  int ret;

  /* Make sure that a non-NULL name is supplied */

  if (name)
    {
      /* The POSIX specification requires that the "check for the existence
       * of a semaphore and the creation of the semaphore if it does not 
       * exist shall be atomic with respect to other processes executing
       * sem_open()..."  A simple sched_lock() should be sufficient to meet
       * this requirement.
       */

      sched_lock();

      /* Get the full path to the semaphore */

      snprintf(fullpath, MAX_SEMPATH, CONFIG_FS_NAMED_SEMPATH "/%s", name);

      /* Get the thread-specific file list */

      list = sched_getfiles();
      DEBUGASSERT(list);

      /* Get the inode for this file.  This should succeed if the semaphore
       * has already been created.
       */

      inode = inode_find(fullpath, &relpath);
      if (inode)
        {
          /* It exists.  Verify that the inode is a semaphore */

          if (!INODE_IS_NAMEDSEM(inode))
            {
              errcode = ENXIO;
              goto errout_with_inode;
            }

          /* It exists and is a semaphore.  Check if the caller wanted to
           * create a new semaphore with this name.
           */

          if ((oflags & (O_CREAT|O_EXCL)) == (O_CREAT|O_EXCL))
            {
              errcode = EEXIST;
              goto errout_with_inode;
            }

          /* Allow a new connection to the semaphore */

          nsem = inode->u.i_nsem;
          nsem->ns_refs++;
          sem  = &nsem->ns_sem;

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

          /* Create an inode in the pseudo-filesystem at this path */

          inode_semtake();
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
              goto errout_with_lock;
            }

          /* Initialize the semaphore */

          inode->u.i_nsem = nsem;
          nsem->ns_refs   = 1;
          sem             = &nsem->ns_sem;
        }
    }

  sched_unlock();
  return sem;

 errout_with_inode:
  inode_release(inode);
 errout_with_lock:
  sched_unlock();
  set_errno(errcode);
  return (FAR sem_t *)ERROR;
}

#endif /* CONFIG_FS_NAMED_SEMAPHORES */