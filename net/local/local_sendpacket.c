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
#include <nuttx/kmalloc.h>
#include "devif/devif.h"

#include "local/local.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The segments of a frame (preamble, path and payload) that the stack
 * holds.  A frame with more segments is copied into one buffer.
 */

#define LOCAL_FRAME_NIOV 8

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
 *   path     The address of the sender ("" for none)
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
                         FAR const char *path,
                         FAR const struct iovec *buf, size_t len,
                         size_t rcvsize)
{
  struct iovec stackiov[LOCAL_FRAME_NIOV];
  FAR uint8_t *frame = NULL;
  lc_size_t preamble[2];
  lc_size_t pathlen;
  size_t niov = len + 2;
  size_t total;
  size_t i;
  ssize_t ret;

  pathlen = strlen(path);

  /* Hold the frame to the capacity the receiver advertised.  The driver
   * refuses only what its FIFO can never hold, which is a different limit:
   * the FIFO may have been created or resized for another rcvsize.  Add
   * the segments one at a time, so that the sum cannot overflow.
   */

  total = sizeof(preamble) + pathlen;
  for (i = 0; i < len && total <= rcvsize; i++)
    {
      total = buf[i].iov_len > rcvsize - total ? rcvsize + 1 :
              total + buf[i].iov_len;
    }

  if (total > rcvsize)
    {
      nerr("ERROR: Frame is too big for %zu\n", rcvsize);
      return -EMSGSIZE;
    }

  /* Wire format is [pathlen][pktlen][path][payload]; the receiver parses
   * the two lengths before it knows how much of either follows.
   */

  preamble[0] = pathlen;
  preamble[1] = total - sizeof(preamble) - pathlen;

  if (niov <= LOCAL_FRAME_NIOV)
    {
      stackiov[0].iov_base = preamble;
      stackiov[0].iov_len  = sizeof(preamble);
      stackiov[1].iov_base = (FAR char *)path;
      stackiov[1].iov_len  = pathlen;
      memcpy(&stackiov[2], buf, len * sizeof(struct iovec));
    }
  else
    {
      FAR uint8_t *ptr;

      /* Too many segments for the stack: copy the frame into one buffer.
       * rcvsize limits its size.
       */

      frame = kmm_malloc(total);
      if (frame == NULL)
        {
          return -ENOMEM;
        }

      memcpy(frame, preamble, sizeof(preamble));
      ptr = frame + sizeof(preamble);
      memcpy(ptr, path, pathlen);
      ptr += pathlen;
      for (i = 0; i < len; i++)
        {
          if (buf[i].iov_base == NULL && buf[i].iov_len > 0)
            {
              kmm_free(frame);
              return -EFAULT;
            }

          memcpy(ptr, buf[i].iov_base, buf[i].iov_len);
          ptr += buf[i].iov_len;
        }

      stackiov[0].iov_base = frame;
      stackiov[0].iov_len  = total;
      niov = 1;
    }

  /* The FIFO is in atomic mode, so this either publishes the whole frame
   * or writes nothing at all.
   */

  ret = file_writev(&conn->lc_outfile, stackiov, niov);
  kmm_free(frame);

  if (ret < 0)
    {
      if (ret != -EAGAIN)
        {
          nerr("ERROR: local send frame failed ret: %zd\n", ret);
        }

      return ret;
    }

  return preamble[1];
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
 *   The number of bytes sent is returned on success.  It is less than the
 *   length of the data when the FIFO is full and does not wait, or when a
 *   signal came while the send waited.  A negated errno value is returned
 *   on any failure (-EINTR: a signal came before the first byte).
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
      ret = file_write(filep, iov->iov_base, iov->iov_len);
      if (ret < 0)
        {
          if (ret != -EAGAIN && ret != -EINTR)
            {
              nerr("ERROR: local send packet failed ret: %zd\n", ret);
            }

          break;
        }

      /* A short write: the FIFO is full and does not wait, or a signal came
       * while the write waited.
       */

      sendlen += ret;
      if (ret != iov->iov_len)
        {
          break;
        }
    }

  return sendlen > 0 ? sendlen : ret;
}
