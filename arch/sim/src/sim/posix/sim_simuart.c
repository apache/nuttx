/****************************************************************************
 * arch/sim/src/sim/posix/sim_simuart.c
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

  raw.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR |
                   ICRNL | IXON);
  raw.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
  raw.c_cflag &= ~(CSIZE | PARENB);
  raw.c_cflag |= CS8;

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
 * Name: simuart_start
 ****************************************************************************/

void simuart_start(void)
{
  /* Get the current stdin terminal mode */

  tcgetattr(0, &g_cooked);

  /* Put stdin into raw mode */

  setrawmode(0);

  /* Restore the original terminal mode before exit */

  atexit(restoremode);
}

/****************************************************************************
 * Name: simuart_open
 ****************************************************************************/

int simuart_open(const char *pathname)
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
 * Name: simuart_close
 ****************************************************************************/

void simuart_close(int fd)
{
  close(fd);
}

/****************************************************************************
 * Name: simuart_putc
 ****************************************************************************/

int simuart_putc(int fd, int ch)
{
  return write(fd, &ch, 1) == 1 ? ch : -1;
}

/****************************************************************************
 * Name: simuart_getc
 ****************************************************************************/

int simuart_getc(int fd)
{
  int ret;
  unsigned char ch;

  ret = read(fd, &ch, 1);
  return ret < 0 ? ret : ch;
}

/****************************************************************************
 * Name: simuart_getcflag
 ****************************************************************************/

int simuart_getcflag(int fd, unsigned int *cflag)
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
 * Name: simuart_setcflag
 ****************************************************************************/

int simuart_setcflag(int fd, unsigned int cflag)
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
 * Name: simuart_checkc
 ****************************************************************************/

bool simuart_checkc(int fd)
{
  struct pollfd pfd;

  pfd.fd     = fd;
  pfd.events = POLLIN;
  return poll(&pfd, 1, 0) == 1;
}
