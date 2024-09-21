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
#include <unistd.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/fs/fs.h>
#include "devif/devif.h"

#include "local/local.h"

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

static int local_fifo_write(FAR struct file *filep, FAR const uint8_t *buf,
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
 * Name: local_send_preamble
 *
 * Description:
 *   Send a packet on the write-only FIFO.
 *
 * Input Parameters:
 * conn      A reference to local connection structure
 * filep     File structure of write-only FIFO.
 * buf       Data to send
 * len       Length of data to send
 *
 * Returned Value:
 *   Packet length is returned on success; a negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

int local_send_preamble(FAR struct local_conn_s *conn,
                        FAR struct file *filep,
                        FAR const struct iovec *buf,
                        size_t len, size_t rcvsize)
{
  FAR const struct iovec *end = buf + len;
  FAR const struct iovec *iov;
  int ret;
  lc_size_t pathlen;
  lc_size_t pktlen;

  /* Send the packet length */

  for (pktlen = 0, iov = buf; iov != end; iov++)
    {
      pktlen += iov->iov_len;
    }

  if (pktlen > rcvsize - sizeof(lc_size_t))
    {
      nerr("ERROR: Packet is too big: %d\n", pktlen);
      return -EMSGSIZE;
    }

  pathlen = strlen(conn->lc_path);
  ret = local_fifo_write(&conn->lc_outfile, (FAR const uint8_t *)&pathlen,
                         sizeof(lc_size_t));
  if (ret != sizeof(lc_size_t))
    {
      nerr("ERROR: local send path length failed ret: %d\n", ret);
      return ret;
    }

  ret = local_fifo_write(filep, (FAR const uint8_t *)&pktlen,
                         sizeof(lc_size_t));
  if (ret != sizeof(lc_size_t))
    {
      return ret;
    }

  return local_fifo_write(&conn->lc_outfile, (uint8_t *)conn->lc_path,
                          pathlen);
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

int local_send_packet(FAR struct file *filep, FAR const struct iovec *buf,
                      size_t len)
{
  FAR const struct iovec *end = buf + len;
  FAR const struct iovec *iov;
  int ret = -EINVAL;
  lc_size_t sendlen;

  for (sendlen = 0, iov = buf; iov != end; iov++)
    {
      ret = local_fifo_write(filep, iov->iov_base, iov->iov_len);
      if (ret < 0)
        {
          if (ret != -EAGAIN)
            {
              nerr("ERROR: local send packet failed ret: %d\n", ret);
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
