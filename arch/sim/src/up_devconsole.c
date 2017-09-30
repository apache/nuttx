/****************************************************************************
 * arch/sim/src/up_devconsole.c
 *
 *   Copyright (C) 2007-2009, 2013 Gregory Nutt. All rights reserved.
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

#include <sys/types.h>
#include <stdbool.h>
#include <sched.h>
#include <errno.h>
#include <fcntl.h>

#include <nuttx/fs/fs.h>

#include "up_internal.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static ssize_t devconsole_read(FAR struct file *filep, FAR char *buffer,
                 size_t buflen);
static ssize_t devconsole_write(FAR struct file *filep,
                 FAR const char *buffer, size_t buflen);
#ifndef CONFIG_DISABLE_POLL
static int     devconsole_poll(FAR struct file *filep, FAR struct pollfd *fds,
                 bool setup);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations devconsole_fops =
{
  .read   = devconsole_read,
  .write  = devconsole_write,
#ifndef CONFIG_DISABLE_POLL
  .poll   = devconsole_poll,
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: devconsole_read
 ****************************************************************************/

static ssize_t devconsole_read(struct file *filep, char *buffer, size_t len)
{
  size_t remaining = len;
  ssize_t nread;
  int ch;

  /* Loop until all requested bytes have been read.  No error checking */

  sched_lock();
  for (remaining = len, nread = 0; remaining > 0; remaining--)
    {
      /* Read the next character from the console, we should only wait
       * on the first read.
       */

      ch = simuart_getc(!(filep->f_oflags & O_NONBLOCK));
      if (ch < 0)
        {
          /*  errno is set in upper layer according to returned value */

          sched_unlock();
          return ch;
        }

      *buffer++ = ch;
       nread++;

      /* We have at least one character.  Return now if no further
       * characters are available without waiting.
       */

      if (!simuart_checkc())
        {
          break;
        }
    }

  sched_unlock();
  return nread;
}

/****************************************************************************
 * Name: devconsole_write
 ****************************************************************************/

static ssize_t devconsole_write(struct file *filep, const char *buffer, size_t len)
{
  int remaining;
  int ret = OK;

  for (remaining = len; remaining > 0 && ret >= 0; remaining--)
    {
      unsigned char ch = *buffer++;
      ret = simuart_putc((int)ch);
    }

  if (ret < 0)
    {
      return -ret;
    }

  return len;
}

/****************************************************************************
 * Name: devconsole_poll
 ****************************************************************************/

#ifndef CONFIG_DISABLE_POLL
static int devconsole_poll(FAR struct file *filep, FAR struct pollfd *fds,
                           bool setup)
{
  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_devconsole
 ****************************************************************************/

void up_devconsole(void)
{
  (void)register_driver("/dev/console", &devconsole_fops, 0666, NULL);
}

/****************************************************************************
 * Name: up_putc
 ****************************************************************************/

int up_putc(int ch)
{
  /* Just map to the host simuart_putc routine */

  return simuart_putc(ch);
}

