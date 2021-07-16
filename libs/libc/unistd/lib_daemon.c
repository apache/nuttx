/****************************************************************************
 * libs/libc/unistd/lib_daemon.c
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

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>

/* Not appropriate for use within the OS */

#if defined(CONFIG_BUILD_FLAT) || !defined(__KERNEL__)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: daemon
 *
 * Description:
 *   The daemon() function is for programs wishing to detach themselves from
 *   the controlling terminal and run in the background as system daemons.
 *
 *   NOTE: daemon() is a non-standard GNU C library interface (based on a
 *   BSD interface of the same name which behaves slightly differently).
 *   The interfaces defined at OpenGroup.org are the interfaces that govern
 *   NuttX.  daemon() is only described in the Linux man pages like:
 *   http://man7.org/linux/man-pages/man3/daemon.3.html
 *
 * Limitations:
 *   NuttX does not implement controlling terminals so the primary purpose
 *   of this interface cannot be supported.  Additional operations controlled
 *   by input parameters are supported, however.
 *
 *   It is not possible to support a true Linux daemon() helper function
 *   because it is designed to work only in the Linux environment and cannot
 *   be translated to NuttX.  The above mentioned web page states:
 *
 *     (This function forks, and if the fork(2) succeeds, the parent calls
 *     _exit(2), so that further errors are seen by the child only.)
 *
 *   So the basic operation of the NuttX daemon() is different.  The GNU C
 *   library daemon() begins will logic like:
 *
 *     switch (fork())
 *       {
 *         case -1:
 *           return -1;
 *         case 0:
 *           break;
 *         default:
 *           exit(0);
 *       }
 *
 *    This implementation begins essentially 'pretending' that we forked
 *    and that the fork was successful and that we are the child of the
 *    fork.
 *
 *    REVISIT:  Ideally we should remove the parent information from the
 *    the new daemon's group data structure.  That would have a similar
 *    effect to the above fork() operation.
 *
 * Input Parameters:
 *   nochdir - If nochdir is zero, daemon() changes the process's current
 *     working directory to the root directory ("/"); otherwise, the current
 *     working  directory is left unchanged.
 *   noclose - If noclose is zero, daemon() redirects standard input,
 *     standard output and standard error to /dev/null; otherwise, no changes
 *     are made to these file descriptors.
 *
 * Returned Value:
 *   0 is returned on success; otherwise, -1 is returned with errno set
 *   appropriately.
 *
 ****************************************************************************/

int daemon(int nochdir, int noclose)
{
#ifndef CONFIG_DISABLE_ENVIRON
  if (nochdir == 0)
    {
      int ret = chdir("/");
      if (ret < 0)
        {
          return -1;  /* chdir() will set the errno variable */
        }
    }
#endif

  if (noclose == 0)
    {
      int fd = open("/dev/null", O_RDWR);
      if (fd < 0)
        {
          return -1;
        }

#ifdef CONFIG_FILE_STREAM
      /* Make sure the stdout, and stderr are flushed */

      fflush(stdout);
      fflush(stderr);
#endif
      /* Dup the fd to create standard fd 0-2 */

      dup2(fd, 0);
      dup2(fd, 1);
      dup2(fd, 2);

      /* We can close the original file descriptor now (unless it was
       * one of* 0-2)
       */

      if (fd > 2)
        {
          close(fd);
        }
    }

  return OK;
}

#endif /* CONFIG_BUILD_FLAT=y || !__KERNEL__ */
