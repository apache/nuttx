/****************************************************************************
 * net/local/local_sendpacket.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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
#if defined(CONFIG_NET) && defined(CONFIG_NET_LOCAL)

#include <sys/types.h>
#include <stdint.h>
#include <unistd.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include "local/local.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LOCAL_PREAMBLE_SIZE 8

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint8_t g_preamble[LOCAL_PREAMBLE_SIZE] =
{
  LOCAL_SYNC_BYTE, LOCAL_SYNC_BYTE, LOCAL_SYNC_BYTE, LOCAL_SYNC_BYTE,
  LOCAL_SYNC_BYTE, LOCAL_SYNC_BYTE, LOCAL_SYNC_BYTE, LOCAL_END_BYTE
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: local_fifo_write
 *
 * Description:
 *   Write a data on the write-only FIFO.
 *
 * Parameters:
 *   fd       File descriptor of write-only FIFO.
 *   buf      Data to send
 *   len      Length of data to send
 *
 * Return:
 *   Zero is returned on success; a negated errno value is returned on any
 *   failure.
 *
 ****************************************************************************/

static int local_fifo_write(int fd, FAR const uint8_t *buf, size_t len)
{
  ssize_t nwritten;

  while (len > 0)
    {
      nwritten = write(fd, buf, len);
      if (nwritten < 0)
        {
          int errcode = get_errno();
          DEBUGASSERT(errcode > 0);

          if (errcode != EINTR)
            {
              ndbg("ERROR: Write failed: %d\n", errcode);
              return -errcode;
            }

          nvdbg("Ignoring signal\n");
        }
      else
        {
          DEBUGASSERT(nwritten > 0 && nwritten <= len);
          len -= nwritten;
          buf += nwritten;
        }
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: local_send_packet
 *
 * Description:
 *   Send a packet on the write-only FIFO.
 *
 * Parameters:
 *   fd       File descriptor of write-only FIFO.
 *   buf      Data to send
 *   len      Length of data to send
 *
 * Return:
 *   Zero is returned on success; a negated errno value is returned on any
 *   failure.
 *
 ****************************************************************************/

int local_send_packet(int fd, FAR const uint8_t *buf, size_t len)
{
  uint16_t len16;
  int ret;

  /* Send the packet preamble */

  ret = local_fifo_write(fd, g_preamble, LOCAL_PREAMBLE_SIZE);
  if (ret == OK)
    {
      /* Send the packet length */

      len16 = len;
      ret = local_fifo_write(fd, (FAR const uint8_t *)&len16, sizeof(uint16_t));
      if (ret == OK)
        {
          /* Send the packet data */

          ret = local_fifo_write(fd, buf, len);
        }
    }

  return ret;
}

#endif /* CONFIG_NET && CONFIG_NET_LOCAL */
