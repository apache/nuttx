/****************************************************************************
 * libs/libc/unistd/lib_daemon.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
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
 *   BSD interface of the same way which behaves slightly differently).
 *   The interfaces defined at OpenGroup.org are the interfaces that govern
 *   NuttX.  deamon() is only described in the Linux man pages like:
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
 *     _exit(2), so that further errors are seen by the child only.)  On
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

#if CONFIG_NFILE_DESCRIPTORS > 0
  if (noclose == 0)
    {
      int fd = open("/dev/null", O_RDWR);
      if (fd < 0)
        {
          return -1;
        }

      /* Make sure the stdin, stdout, and stderr are closed */

      (void)fclose(stdin);
      (void)fclose(stdout);
      (void)fclose(stderr);

      /* Dup the fd to create standard fd 0-2 */

      (void)dup2(fd, 0);
      (void)dup2(fd, 1);
      (void)dup2(fd, 2);

      /* fdopen to get the stdin, stdout and stderr streams. The
       * following logic depends on the fact that the library layer
       * will allocate FILEs in order.  And since we closed stdin,
       * stdout, and stderr above, that is what we should get.
       *
       * fd = 0 is stdin  (read-only)
       * fd = 1 is stdout (write-only, append)
       * fd = 2 is stderr (write-only, append)
       */

      (void)fdopen(0, "r");
      (void)fdopen(1, "a");
      (void)fdopen(2, "a");

      /* We can close the original file descriptor now (unless it was
       * one of* 0-2)
       */

      if (fd > 2)
       {
         (void)close(fd);
       }
    }
#endif

  return OK;
}

#endif /* CONFIG_BUILD_FLAT=y || !__KERNEL__ */
