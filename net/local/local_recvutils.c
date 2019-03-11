/****************************************************************************
 * net/local/local_recvpacket.c
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

#include <sys/types.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/fs/fs.h>

#include "local/local.h"

#if defined(CONFIG_NET) && defined(CONFIG_NET_LOCAL)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: local_fifo_read
 *
 * Description:
 *   Read a data from the read-only FIFO.
 *
 * Input Parameters:
 *   filep - File structure of write-only FIFO.
 *   buf   - Local to store the received data
 *   len   - Length of data to receive [in]
 *           Length of data actually received [out]
 *
 * Returned Value:
 *   Zero is returned on success; a negated errno value is returned on any
 *   failure.  If -ECONNRESET is received, then the sending side has closed
 *   the FIFO. In this case, the returned data may still be valid (if the
 *   returned len > 0).
 *
 ****************************************************************************/

int local_fifo_read(FAR struct file *filep, FAR uint8_t *buf, size_t *len)
{
  ssize_t remaining;
  ssize_t nread;
  int ret;

  DEBUGASSERT(buf && len);

  remaining = *len;
  while (remaining > 0)
    {
      nread = file_read(filep, buf, remaining);
      if (nread < 0)
        {
          if (nread != -EINTR)
            {
              ret = (int)nread;
              nerr("ERROR: nx_read() failed: %d\n", ret);
              goto errout;
            }

          ninfo("Ignoring signal\n");
        }
      else if (nread == 0)
        {
          /* The FIFO returns zero if the sending side of the connection
           * has closed the FIFO.
           */

          ret = -ECONNRESET;
          goto errout;
        }
      else
        {
          DEBUGASSERT(nread <= remaining);
          remaining -= nread;
          buf       += nread;
        }
    }

  ret = OK;

errout:
  *len -= remaining;
  return ret;
}

/****************************************************************************
 * Name: local_sync
 *
 * Description:
 *   Read a sync bytes until the start of the packet is found.
 *
 * Input Parameters:
 *   filep - File structure of write-only FIFO.
 *
 * Returned Value:
 *   The non-zero size of the following packet is returned on success; a
 *   negated errno value is returned on any failure.
 *
 ****************************************************************************/

int local_sync(FAR struct file *filep)
{
  size_t readlen;
  uint16_t pktlen;
  uint8_t sync;
  int ret;

  /* Loop until a valid pre-amble is encountered:  SYNC bytes followed
   * by one END byte.
   */

  do
    {
      /* Read until we encounter a sync byte */

      do
        {
          readlen = sizeof(uint8_t);
          ret     = local_fifo_read(filep, &sync, &readlen);
          if (ret < 0)
            {
              nerr("ERROR: Failed to read sync bytes: %d\n", ret);
              return ret;
            }
        }
      while (sync != LOCAL_SYNC_BYTE);

      /* Then read to the end of the SYNC sequence */

      do
        {
          readlen = sizeof(uint8_t);
          ret     = local_fifo_read(filep, &sync, &readlen);
          if (ret < 0)
            {
              nerr("ERROR: Failed to read sync bytes: %d\n", ret);
              return ret;
            }
        }
      while (sync == LOCAL_SYNC_BYTE);
    }
  while (sync != LOCAL_END_BYTE);

  /* Then read the packet length */

  readlen = sizeof(uint16_t);
  ret     = local_fifo_read(filep, (FAR uint8_t *)&pktlen, &readlen);
  return ret < 0 ? ret : pktlen;
}

/****************************************************************************
 * Name: local_getaddr
 *
 * Description:
 *   Return the Unix domain address of a connection.
 *
 * Input Parameters:
 *   conn - The connection
 *   addr - The location to return the address
 *   addrlen - The size of the memory allocated by the caller to receive the
 *             address.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int local_getaddr(FAR struct local_conn_s *conn, FAR struct sockaddr *addr,
                  FAR socklen_t *addrlen)
{
  FAR struct sockaddr_un *unaddr;
  int totlen;
  int pathlen;

  DEBUGASSERT(conn && addr && addrlen && *addrlen >= sizeof(sa_family_t));

  /* Get the length of the path (minus the NUL terminator) and the length
   * of the whole Unix domain address.
   */

  pathlen = strnlen(conn->lc_path, UNIX_PATH_MAX - 1);
  totlen  = sizeof(sa_family_t) + pathlen + 1;

  /* If the length of the whole Unix domain address is larger than the
   * buffer provided by the caller, then truncate the address to fit.
   */

  if (totlen > *addrlen)
    {
      pathlen    -= (totlen - *addrlen);
      totlen      = *addrlen;
    }

  /* Copy the Unix domain address */

  unaddr = (FAR struct sockaddr_un *)addr;
  unaddr->sun_family = AF_LOCAL;
  memcpy(unaddr->sun_path, conn->lc_path, pathlen);
  unaddr->sun_path[pathlen] = '\0';

  /* Return the Unix domain address size */

  *addrlen = totlen;
  return OK;
}

#endif /* CONFIG_NET && CONFIG_NET_LOCAL */
