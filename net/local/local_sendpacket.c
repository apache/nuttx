/****************************************************************************
 * net/local/local_sendpacket.c
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

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <assert.h>
#include <nuttx/debug.h>

#include <nuttx/fs/fs.h>
#include "devif/devif.h"

#include "local/local.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Preamble and path segments plus this many payload segments are gathered
 * on the stack.  A datagram never needs more than this, which the sender
 * asserts before it builds the frame.
 */

#define LOCAL_FRAME_NIOV 8

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: local_fifo_write
 *
 * Description:
 *   Write a data on the write-only FIFO.
 *
 * Input Parameters:
 *   filep    File structure of write-only FIFO.
 *   buf      Data to send
 *   len      Length of data to send
 *
 * Returned Value:
 *   On success, the number of bytes written are returned (zero indicates
 *   nothing was written).  On any failure, a negated errno value is returned
 *
 ****************************************************************************/

static ssize_t local_fifo_write(FAR struct file *filep,
                                FAR const uint8_t *buf,
                                size_t len)
{
  ssize_t nwritten = 0;
  ssize_t ret = 0;

  while (len != nwritten)
    {
      ret = file_write(filep, buf + nwritten, len - nwritten);
      if (ret < 0)
        {
          if (ret == -EINTR)
            {
              continue;
            }
          else if (ret == -EAGAIN)
            {
              break;
            }
          else
            {
              nerr("ERROR: file_write failed: %zd\n", ret);
              break;
            }
        }

      nwritten += ret;
    }

  return nwritten > 0 ? nwritten : ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: local_send_frame
 *
 * Description:
 *   Send one datagram on the write-only FIFO as a single atomic writev, so
 *   that the reader is never woken on a preamble whose payload has not been
 *   written yet.
 *
 * Input Parameters:
 *   conn     A reference to local connection structure
 *   buf      Payload to send
 *   len      Number of payload iovec segments
 *   rcvsize  Receive buffer size the peer advertised, preamble room
 *            included
 *
 * Returned Value:
 *   Payload length is returned on success; a negated errno value is
 *   returned on any failure.  -EMSGSIZE means the whole frame, preamble
 *   included, does not fit in rcvsize.
 *
 ****************************************************************************/

ssize_t local_send_frame(FAR struct local_conn_s *conn,
                         FAR const struct iovec *buf, size_t len,
                         size_t rcvsize)
{
  struct iovec stackiov[LOCAL_FRAME_NIOV];
  lc_size_t preamble[2];
  lc_size_t pathlen;
  size_t niov = len + 2;
  size_t pktlen;
  ssize_t ret;
  size_t i;

  /* The frame is the preamble plus the path plus every payload segment.
   * The on-stack iovec holds at most LOCAL_FRAME_NIOV segments, two of
   * which are reserved for the preamble and the path.  len comes straight
   * from the caller's msg_iovlen (bounded only by IOV_MAX), so reject a
   * request that would not fit instead of overrunning stackiov.
   */

  if (niov > LOCAL_FRAME_NIOV)
    {
      nerr("ERROR: Too many iovec segments: %zu > %d\n",
           niov, LOCAL_FRAME_NIOV);
      return -EMSGSIZE;
    }

  for (pktlen = 0, i = 0; i < len; i++)
    {
      pktlen += buf[i].iov_len;
    }

  pathlen = strlen(conn->lc_path);

  /* Hold the frame to the capacity the receiver advertised.  The driver
   * refuses only what its FIFO can never hold, which is a different limit:
   * the FIFO may have been created or resized for another rcvsize.
   */

  if (pktlen + sizeof(preamble) + pathlen > rcvsize)
    {
      nerr("ERROR: Frame is too big: %zu > %zu\n",
           pktlen + sizeof(preamble) + (size_t)pathlen, rcvsize);
      return -EMSGSIZE;
    }

  /* Wire format is [pathlen][pktlen][path][payload]; the receiver parses
   * the two lengths before it knows how much of either follows.
   */

  preamble[0] = pathlen;
  preamble[1] = pktlen;

  stackiov[0].iov_base = preamble;
  stackiov[0].iov_len  = sizeof(preamble);
  stackiov[1].iov_base = conn->lc_path;
  stackiov[1].iov_len  = pathlen;
  memcpy(&stackiov[2], buf, len * sizeof(struct iovec));

  /* The FIFO is in atomic mode, so this either publishes the whole frame
   * or writes nothing at all.
   */

  ret = file_writev(&conn->lc_outfile, stackiov, niov);

  if (ret < 0)
    {
      if (ret != -EAGAIN)
        {
          nerr("ERROR: local send frame failed ret: %zd\n", ret);
        }

      return ret;
    }

  return pktlen;
}

/****************************************************************************
 * Name: local_send_packet
 *
 * Description:
 *   Send a packet on the write-only FIFO.
 *
 * Input Parameters:
 *   filep    File structure of write-only FIFO.
 *   buf      Data to send
 *   len      Length of data to send
 *
 * Returned Value:
 *   Packet length is returned on success; a negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

ssize_t local_send_packet(FAR struct file *filep,
                          FAR const struct iovec *buf,
                          size_t len)
{
  FAR const struct iovec *end = buf + len;
  FAR const struct iovec *iov;
  ssize_t ret = 0;
  ssize_t sendlen = 0;

  if (len == 0)
    {
      return 0;
    }

  for (iov = buf; iov != end; iov++)
    {
      ret = local_fifo_write(filep, iov->iov_base, iov->iov_len);
      if (ret < 0)
        {
          if (ret != -EAGAIN)
            {
              nerr("ERROR: local send packet failed ret: %zd\n", ret);
            }

          break;
        }

      if (ret > 0)
        {
          sendlen += ret;
          if (ret != iov->iov_len)
            {
              break;
            }
        }
    }

  return sendlen > 0 ? sendlen : ret;
}
