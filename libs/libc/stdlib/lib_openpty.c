/****************************************************************************
 * libs/libc/stdlib/lib_openpty.c
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

#include <fcntl.h>
#include <pty.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <limits.h>

#include <nuttx/serial/pty.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: posix_openpt
 *
 * Description:
 *   The posix_openpt() function establish a connection between a master
 *   device for a pseudo-terminal and a file descriptor. The file descriptor
 *   is used by other I/O functions that refer to that pseudo-terminal.
 *
 *   The file status flags and file access modes of the open file description
 *   shall be set according to the value of oflag.
 *
 *   Values for oflag are constructed by a bitwise-inclusive OR of flags from
 *   the following list, defined in <fcntl.h>:
 *
 *   O_RDWR
 *       Open for reading and writing.
 *   O_NOCTTY
 *       If set posix_openpt() shall not cause the terminal device to become
 *       the controlling terminal for the process.
 *
 *   The behavior of other values for the oflag argument is unspecified.
 *
 * Returned Value:
 *   Upon successful completion, the posix_openpt() function shall open
 *   a master pseudo-terminal device and return a non-negative integer
 *   representing the lowest numbered unused file descriptor. Otherwise,
 *   -1 shall be returned and errno set to indicate the error.
 *
 ****************************************************************************/

int posix_openpt(int oflag)
{
#ifdef CONFIG_PSEUDOTERM_SUSV1
  return open("/dev/ptmx", oflag);
#else
  int minor;

  for (minor = 0; minor < 256; minor++)
    {
      char devname[16];
      int fd;

      snprintf(devname, 16, "/dev/pty%d", minor);
      fd = open(devname, oflag);
      if (fd < 0)
        {
          /* Fail, register and try again */

          pty_register(minor);
          fd = open(devname, oflag);
        }

      if (fd >= 0)
        {
          return fd;
        }
    }

  return -1;
#endif
}

/****************************************************************************
 * Name: openpty
 *
 * Description:
 *   The openpty() function finds an available pseudoterminal and
 *   returns file descriptors in master and slave.  If name is not
 *   NULL, the filename of the slave is returned in name.  If term
 *   is not NULL, the terminal parameters of the slave will be set
 *   to the values in term.  If win is not NULL, the window size of
 *   the slave will be set to the values in win.
 *
 * Returned Value:
 *   If a call to openpty() is not successful, -1 is returned and
 *   errno is set to indicate the error.  Otherwise, return 0.
 *
 ****************************************************************************/

int openpty(FAR int *master, FAR int *slave, FAR char *name,
            FAR const struct termios *term, FAR const struct winsize *win)
{
  char buf[64];
  int ret;

  /* Open the pseudo terminal master */

  ret = posix_openpt(O_RDWR);
  if (ret < 0)
    {
      return ret;
    }

  *master = ret;

  /* Configure the pseudo terminal master */

  ret = grantpt(*master);
  if (ret < 0)
    {
      goto err;
    }

  ret = unlockpt(*master);
  if (ret < 0)
    {
      goto err;
    }

  /* Open the pseudo terminal slave */

  ret = ptsname_r(*master, buf, sizeof(buf));
  if (ret < 0)
    {
      goto err;
    }

  ret = open(buf, O_RDWR | O_NOCTTY);
  if (ret < 0)
    {
      goto err;
    }

  *slave = ret;

  if (name != NULL)
    {
      strlcpy(name, buf, NAME_MAX);
    }

  /* Configure the pseudo terminal slave */

  if (term != NULL)
    {
      tcsetattr(*slave, TCSAFLUSH, term);
    }

  if (win != NULL)
    {
      ioctl(*slave, TIOCSWINSZ, win);
    }

  return 0;

err:
  close(*master);
  return ret;
}
