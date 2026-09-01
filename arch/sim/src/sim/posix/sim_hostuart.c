/****************************************************************************
 * arch/sim/src/sim/posix/sim_hostuart.c
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

#include <sys/ioctl.h>
#include <fcntl.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <termios.h>
#include <poll.h>
#include <errno.h>
#include <stdarg.h>
#include <stdio.h>

#include "sim_internal.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct termios g_cooked;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: setrawmode
 ****************************************************************************/

static void setrawmode(int fd)
{
  struct termios raw;

  tcgetattr(fd, &raw);

  /* Switch to raw mode */

  cfmakeraw(&raw);

  /* Disable output processing, We need to exclude stdout to prevent the
   * terminal configuration from being changed after an abnormal exit.
   */

  if (fd == 0)
    {
      raw.c_oflag |= OPOST;
    }

  tcsetattr(fd, TCSANOW, &raw);
}

/****************************************************************************
 * Name: restoremode
 ****************************************************************************/

static void restoremode(void)
{
  /* Restore the original terminal mode */

  tcsetattr(0, TCSANOW, &g_cooked);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: host_uart_start
 ****************************************************************************/

void host_uart_start(void)
{
  int nonblock = 1;

  /* Get the current stdin terminal mode */

  tcgetattr(0, &g_cooked);

  /* Put stdin into raw mode */

  setrawmode(0);

  host_uninterruptible_no_return(ioctl, 0, FIONBIO, &nonblock);

  /* Set stdout to non-blocking to prevent write(1, ...) from blocking
   * the entire sim process when the host pipe buffer is full.
   */

  host_uninterruptible_no_return(ioctl, 1, FIONBIO, &nonblock);

  /* Restore the original terminal mode before exit */

  atexit(restoremode);
}

/****************************************************************************
 * Name: host_uart_open
 ****************************************************************************/

int host_uart_open(const char *pathname)
{
  int fd;

  fd = open(pathname, O_RDWR | O_NONBLOCK);
  if (fd >= 0)
    {
      /* keep raw mode */

      setrawmode(fd);
    }
  else
    {
      fd = -errno;
    }

  return fd;
}

/****************************************************************************
 * Name: host_uart_openpty
 ****************************************************************************/

int host_uart_openpty(const char *name)
{
#ifdef CONFIG_HOST_LINUX
  unsigned int ptyno;
  int lock = 0;
  int oflags;
  int ret;
  int fd;

  oflags = O_RDWR | O_NOCTTY | O_NONBLOCK;
#ifdef O_CLOEXEC
  oflags |= O_CLOEXEC;
#endif
  fd = open("/dev/ptmx", oflags);
  if (fd < 0)
    {
      return -errno;
    }

  ret = ioctl(fd, TIOCGPTN, &ptyno);
  if (ret < 0)
    {
      ret = -errno;
      goto errout;
    }

  ret = ioctl(fd, TIOCSPTLCK, &lock);
  if (ret < 0)
    {
      ret = -errno;
      goto errout;
    }

  setrawmode(fd);

  printf("%s connected to pseudotty: /dev/pts/%u\n", name, ptyno);

  return fd;

errout:
  close(fd);
  return ret;
#else
  (void)name;
  return -ENOSYS;
#endif
}

/****************************************************************************
 * Name: host_uart_close
 ****************************************************************************/

void host_uart_close(int fd)
{
  close(fd);
}

/****************************************************************************
 * Name: host_uart_puts
 ****************************************************************************/

int host_uart_puts(int fd, const char *buf, size_t size)
{
  int ret;

  do
    {
      ret = write(fd, buf, size);
    }
  while (ret < 0 && errno == EINTR);

  return ret < 0 ? -errno : ret;
}

/****************************************************************************
 * Name: host_uart_gets
 ****************************************************************************/

int host_uart_gets(int fd, char *buf, size_t size)
{
  int ret;

  do
    {
      ret = read(fd, buf, size);
    }
  while (ret < 0 && errno == EINTR);

  return ret < 0 ? -errno : ret;
}

/****************************************************************************
 * Name: host_uart_getcflag
 ****************************************************************************/

int host_uart_getcflag(int fd, unsigned int *cflag)
{
  struct termios t;
  int ret;

  ret = tcgetattr(fd, &t);
  if (ret < 0)
    {
      ret = -errno;
    }
  else
    {
      *cflag = t.c_cflag;
    }

  return ret;
}

/****************************************************************************
 * Name: host_uart_setcflag
 ****************************************************************************/

int host_uart_setcflag(int fd, unsigned int cflag)
{
  struct termios t;
  int ret;

  ret = tcgetattr(fd, &t);
  if (!ret)
    {
      t.c_cflag = cflag;
      ret = tcsetattr(fd, TCSANOW, &t);
    }

  if (ret < 0)
    {
      ret = -errno;
    }

  return ret;
}

/****************************************************************************
 * Name: host_uart_checkin
 ****************************************************************************/

bool host_uart_checkin(int fd)
{
  struct pollfd pfd;

  pfd.fd     = fd;
  pfd.events = POLLIN;
  return poll(&pfd, 1, 0) == 1 && (pfd.revents & POLLIN) != 0;
}

/****************************************************************************
 * Name: host_uart_checkout
 ****************************************************************************/

bool host_uart_checkout(int fd)
{
  struct pollfd pfd;

  pfd.fd     = fd;
  pfd.events = POLLOUT;
  return poll(&pfd, 1, 0) == 1 && (pfd.revents & POLLOUT) != 0;
}

/****************************************************************************
 * Name: host_printf
 ****************************************************************************/

void host_printf(const char *fmt, ...)
{
  va_list ap;

  va_start(ap, fmt);
  vprintf(fmt, ap);
  va_end(ap);
}
