/****************************************************************************
 * arch/sim/src/sim/up_simuart.c
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

#include <fcntl.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <termios.h>
#include <poll.h>
#include <errno.h>

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
  raw.c_oflag &= ~OPOST;
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

int simuart_getcflag(int fd, tcflag_t *cflag)
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

int simuart_setcflag(int fd, tcflag_t cflag)
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
