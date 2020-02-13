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

#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <termios.h>
#include <poll.h>

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

static void setrawmode(void)
{
  struct termios raw;

  /* Get the current stdin terminal mode */

  tcgetattr(0, &g_cooked);

  /* Switch to raw mode */

  memcpy(&raw, &g_cooked, sizeof(struct termios));

  raw.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
  raw.c_oflag &= ~OPOST;
  raw.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
  raw.c_cflag &= ~(CSIZE | PARENB);
  raw.c_cflag |= CS8;

  tcsetattr(0, TCSANOW, &raw);
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
 * Name: simuart_putraw
 ****************************************************************************/

int simuart_putraw(int ch)
{
  ssize_t nwritten;
  unsigned char buf = ch;

  nwritten = write(1, &buf, 1);
  if (nwritten != 1)
    {
      return -1;
    }

  return ch;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: simuart_start
 ****************************************************************************/

void simuart_start(void)
{
  /* Put stdin into raw mode */

  setrawmode();

  /* Restore the original terminal mode before exit */

  atexit(restoremode);
}

/****************************************************************************
 * Name: simuart_putc
 ****************************************************************************/

int simuart_putc(int ch)
{
  int ret = ch;

  if (ch == '\n')
    {
      ret = simuart_putraw('\r');
    }

  if (ret >= 0)
    {
      ret = simuart_putraw(ch);
    }

  return ret;
}

/****************************************************************************
 * Name: simuart_getc
 ****************************************************************************/

int simuart_getc(void)
{
  int ret;
  unsigned char ch;

  ret = read(0, &ch, 1);
  return ret < 0 ? ret : ch;
}

/****************************************************************************
 * Name: simuart_checkc
 ****************************************************************************/

bool simuart_checkc(void)
{
  struct pollfd pfd;

  pfd.fd     = 0;
  pfd.events = POLLIN;
  return poll(&pfd, 1, 0) == 1;
}
