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
  /* Get the current stdin terminal mode */

  host_uninterruptible_no_return(tcgetattr, 0, &g_cooked);

  /* Put stdin into raw mode */

  host_uninterruptible_no_return(setrawmode, 0);

  /* Restore the original terminal mode before exit */

  host_uninterruptible_no_return(atexit, restoremode);
}

/****************************************************************************
 * Name: host_uart_open
 ****************************************************************************/

int host_uart_open(const char *pathname)
{
  int fd;

  fd = host_uninterruptible_errno(open, pathname, O_RDWR | O_NONBLOCK);
  if (fd >= 0)
    {
      /* keep raw mode */

      host_uninterruptible_no_return(setrawmode, fd);
    }

  return fd;
}

/****************************************************************************
 * Name: host_uart_close
 ****************************************************************************/

void host_uart_close(int fd)
{
  host_uninterruptible(close, fd);
}

/****************************************************************************
 * Name: host_uart_puts
 ****************************************************************************/

int host_uart_puts(int fd, const char *buf, size_t size)
{
  int ret;

  do
    {
      ret = host_uninterruptible_errno(write, fd, buf, size);
    }
  while (ret < 0 && ret == -EINTR);

  return ret;
}

/****************************************************************************
 * Name: host_uart_gets
 ****************************************************************************/

int host_uart_gets(int fd, char *buf, size_t size)
{
  int ret;

  do
    {
      ret = host_uninterruptible_errno(read, fd, buf, size);
    }
  while (ret < 0 && ret == -EINTR);

  return ret;
}

/****************************************************************************
 * Name: host_uart_getcflag
 ****************************************************************************/

int host_uart_getcflag(int fd, unsigned int *cflag)
{
  struct termios t;
  int ret;

  ret = host_uninterruptible_errno(tcgetattr, fd, &t);
  if (ret >= 0)
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

  ret = host_uninterruptible_errno(tcgetattr, fd, &t);
  if (!ret)
    {
      t.c_cflag = cflag;
      ret = host_uninterruptible_errno(tcsetattr, fd, TCSANOW, &t);
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
  return host_uninterruptible(poll, &pfd, 1, 0) == 1;
}

/****************************************************************************
 * Name: host_uart_checkout
 ****************************************************************************/

bool host_uart_checkout(int fd)
{
  struct pollfd pfd;

  pfd.fd     = fd;
  pfd.events = POLLOUT;
  return host_uninterruptible(poll, &pfd, 1, 0) == 1;
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
