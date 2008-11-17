/****************************************************************************
 * examples/poll/poll_listener.c
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
#include <sys/stat.h>

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <pthread.h>
#include <poll.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#include "poll_internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: poll_listener
 ****************************************************************************/

void *poll_listener(pthread_addr_t pvarg)
{
  struct pollfd fds;
  char buffer[64];
  ssize_t nbytes;
  boolean timeout;
  boolean pollin;
  int fd;
  int ret;

  /* Open the FIFO for non-blocking read */

  message("poll_listener: Opening %s for non-blocking read\n", FIFO_PATH);
  fd = open(FIFO_PATH, O_RDONLY|O_NONBLOCK);
  if (fd < 0)
    {
      message("poll_listener: ERROR Failed to open FIFO %s: %d\n",
              FIFO_PATH, errno);
      (void)close(fd);
      return (void*)-1;
    }

  /* Loop forever */

  for (;;)
    {
      message("poll_listener: Calling poll()\n");

      memset(&fds, 0, sizeof(struct pollfd));
      fds.fd      = fd;
      fds.events  = POLLIN;
      fds.revents = 0;

      timeout     = FALSE;
      pollin      = FALSE;

      ret = poll(&fds, 1, LISTENER_DELAY);
      if (ret < 0)
        {
          message("poll_listener: ERROR poll failed: %d\n");
        }
      else if (ret == 0)
        {
          message("poll_listener: Timeout, revents=%02x\n", fds.revents);
          timeout = TRUE;
          if (fds.revents != 0)
            {
              message("poll_listener: ERROR? expected revents=00, received revents=%02x\n",
                      fds.revents);
            }
        }
      else
        {
          if (ret != 1)
            {
              message("poll_listener: ERROR poll reported: %d\n");
            }
          else
            {
              pollin = TRUE;
            }

          message("poll_listener: revents=%02x\n", fds.revents);
          if (fds.revents != POLLIN)
            {
              message("poll_listener: ERROR expected revents=%02x, received revents=%02x\n",
                      fds.revents);
              message("               (might just be a race condition)\n");
            }
        }

      /* In any event, read until the pipe is empty */

      do
        {
          nbytes = read(fd, buffer, 63);
          if (nbytes <= 0)
            {
              if (nbytes == 0 || errno == EAGAIN)
                {
                  if (timeout)
                    {
                      message("poll_listener: No read data available\n");
                    }
                  else if (pollin)
                    {
                      message("poll_listener: ERROR no read data\n");
                    }
                }
              else if (errno != EINTR)
                {
                  message("poll_listener: read failed: %d\n", errno);
                }
              nbytes = 0;
            }
          else
            {
              if (timeout)
                {
                  message("poll_listener: ERROR? Poll timeout, but data read\n");
                  message("               (might just be a race condition)\n");
                }

              buffer[nbytes] = '\0';
              message("poll_listener: Read '%s' (%d bytes)\n", buffer, nbytes);
            }

          timeout = FALSE;
          pollin  = FALSE;
        }
      while (nbytes > 0);

      /* Make sure that everything is displayed */

      msgflush();
    }

  /* Won't get here */

  (void)close(fd);
  return NULL;
}
