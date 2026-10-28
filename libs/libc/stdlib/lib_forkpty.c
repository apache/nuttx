/****************************************************************************
 * libs/libc/stdlib/lib_forkpty.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <errno.h>
#include <pty.h>
#include <sys/ioctl.h>
#include <unistd.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: forkpty
 *
 * Description:
 *   Open a pseudo terminal pair with openpty() and fork.  The child starts
 *   a new session, makes the slave its controlling terminal and its
 *   standard input, output and error.  The parent gets the master.
 *
 * Input Parameters:
 *   master - The master of the pair (parent only)
 *   name   - The name of the slave, or NULL
 *   term   - The terminal attributes of the slave, or NULL
 *   win    - The window size of the slave, or NULL
 *
 * Returned Value:
 *   In the parent the process ID of the child, in the child 0.  -1 with
 *   errno set if openpty() or fork() fails.
 *
 ****************************************************************************/

pid_t forkpty(FAR int *master, FAR char *name,
              FAR const struct termios *term,
              FAR const struct winsize *win)
{
  int errcode;
  int slave;
  int fd;
  pid_t pid;

  if (openpty(&fd, &slave, name, term, win) < 0)
    {
      return -1;
    }

  pid = fork();
  if (pid < 0)
    {
      errcode = errno;
      close(fd);
      close(slave);
      errno = errcode;
      return -1;
    }

  if (pid == 0)
    {
      close(fd);
      setsid();

      /* Without CONFIG_TTY_SIGINT and CONFIG_TTY_SIGTSTP a terminal has no
       * controlling process, and TIOCSCTTY fails: the slave is still the
       * standard input and output.
       */

      ioctl(slave, TIOCSCTTY, 0);
      if (dup2(slave, STDIN_FILENO) < 0 ||
          dup2(slave, STDOUT_FILENO) < 0 ||
          dup2(slave, STDERR_FILENO) < 0)
        {
          _exit(1);
        }

      if (slave > STDERR_FILENO)
        {
          close(slave);
        }

      return 0;
    }

  close(slave);
  *master = fd;
  return pid;
}
