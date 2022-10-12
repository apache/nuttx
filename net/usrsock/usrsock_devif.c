/****************************************************************************
 * net/usrsock/usrsock_devif.c
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
#if defined(CONFIG_NET) && defined(CONFIG_NET_USRSOCK)

#include <sys/types.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/random.h>
#include <nuttx/semaphore.h>
#include <nuttx/net/net.h>
#include <nuttx/net/usrsock.h>

#include "usrsock/usrsock.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct usrsock_req_s
{
  sem_t    sem;               /* Request semaphore (only one outstanding
                               * request) */
  sem_t    acksem;            /* Request acknowledgment notification */
  uint32_t newxid;            /* New transcation Id */
  uint32_t ackxid;            /* Exchange id for which waiting ack */
  uint16_t nbusy;             /* Number of requests blocked from different
                               * threads */

  /* Connection instance to receive data buffers. */

  FAR struct usrsock_conn_s *datain_conn;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* only support 1 usrsock network interface for the moment,
 * define it into array or construct a list
 * if multiple usrsock network interfaces are needed in the future
 */

static struct usrsock_req_s g_usrsock_req =
{
  NXSEM_INITIALIZER(1, PRIOINHERIT_FLAGS_DISABLE),
  NXSEM_INITIALIZER(0, PRIOINHERIT_FLAGS_DISABLE),
  0,
  0,
  0,
  NULL
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usrsock_iovec_do() - copy to/from iovec from/to buffer.
 ****************************************************************************/

static ssize_t usrsock_iovec_do(FAR void *srcdst, size_t srcdstlen,
                                FAR struct iovec *iov, int iovcnt,
                                size_t pos, bool from_iov, FAR bool *done)
{
  FAR uint8_t *ioout = srcdst;
  FAR uint8_t *iovbuf;
  ssize_t total = 0;
  size_t srclen = 0;

  /* Rewind to correct position. */

  while (pos >= 0 && iovcnt > 0)
    {
      if (iov->iov_len <= pos)
        {
          pos -= iov->iov_len;
          iov++;
          iovcnt--;
        }
      else
        {
          break;
        }
    }

  if (iovcnt == 0)
    {
      /* Position beyond iovec. */

      total = -EINVAL;
      goto out;
    }

  iovbuf = iov->iov_base;
  srclen = iov->iov_len;
  iovbuf += pos;
  srclen -= pos;
  iov++;
  iovcnt--;

  while ((srclen > 0 || iovcnt > 0) && srcdstlen > 0)
    {
      size_t clen = srclen;

      if (srclen == 0)
        {
          /* Skip empty iovec. */

          iovbuf = iov->iov_base;
          srclen = iov->iov_len;
          iov++;
          iovcnt--;

          continue;
        }

      if (clen > srcdstlen)
        {
          clen = srcdstlen;
        }

      if (from_iov)
        {
          memmove(ioout, iovbuf, clen);
        }
      else
        {
          memmove(iovbuf, ioout, clen);
        }

      ioout += clen;
      srcdstlen -= clen;
      iovbuf += clen;
      srclen -= clen;
      total += clen;

      if (srclen == 0)
        {
          if (iovcnt == 0)
            {
              break;
            }

          iovbuf = iov->iov_base;
          srclen = iov->iov_len;
          iov++;
          iovcnt--;
        }
    }

out:
  if (done)
    {
      *done = !srclen && !iovcnt;
    }

  return total;
}

/****************************************************************************
 * Name: usrsock_handle_event
 ****************************************************************************/

static ssize_t usrsock_handle_event(FAR const void *buffer, size_t len)
{
  FAR const struct usrsock_message_common_s *common = buffer;

  switch (common->msgid)
    {
    case USRSOCK_MESSAGE_SOCKET_EVENT:
      {
        FAR const struct usrsock_message_socket_event_s *hdr = buffer;
        FAR struct usrsock_conn_s *conn;
        int ret;

        if (len < sizeof(*hdr))
          {
            nwarn("message too short, %zu < %zu.\n", len, sizeof(*hdr));
            return -EINVAL;
          }

        /* Get corresponding usrsock connection. */

        conn = usrsock_active(hdr->usockid);
        if (!conn)
          {
            nwarn("no active connection for usockid=%d.\n", hdr->usockid);
            return -ENOENT;
          }

#ifdef CONFIG_DEV_RANDOM
        /* Add randomness. */

        add_sw_randomness((hdr->head.events << 16) - hdr->usockid);
#endif

        /* Handle event. */

        conn->resp.events = hdr->head.events & ~USRSOCK_EVENT_INTERNAL_MASK;
        ret = usrsock_event(conn);
        if (ret < 0)
          {
            return ret;
          }

        len = sizeof(*hdr);
      }
      break;

    default:
      nwarn("Unknown event type: %d\n", common->msgid);
      return -EINVAL;
    }

  return len;
}

/****************************************************************************
 * Name: usrsock_handle_response
 ****************************************************************************/

static ssize_t usrsock_handle_response(FAR struct usrsock_conn_s *conn,
                                       FAR const void *buffer,
                                       size_t len)
{
  FAR const struct usrsock_message_req_ack_s *hdr = buffer;

  if (USRSOCK_MESSAGE_REQ_IN_PROGRESS(hdr->head.flags))
    {
      /* In-progress response is acknowledgment that response was
       * received.
       */

      conn->resp.inprogress = true;

      /* This branch indicates successful processing and waiting
       * for USRSOCK_EVENT_CONNECT_READY event.
       */

      conn->resp.result = 0;
    }
  else
    {
      conn->resp.inprogress = false;
      conn->resp.xid = 0;

      /* Get result for common request. */

      conn->resp.result = hdr->result;

      /* Done with request/response. */

      usrsock_event(conn);
    }

  return sizeof(*hdr);
}

/****************************************************************************
 * Name: usrsock_handle_datareq_response
 ****************************************************************************/

static ssize_t
usrsock_handle_datareq_response(FAR struct usrsock_conn_s *conn,
                                FAR const void *buffer,
                                size_t len)
{
  FAR const struct usrsock_message_datareq_ack_s *datahdr = buffer;
  FAR const struct usrsock_message_req_ack_s *hdr = &datahdr->reqack;
  FAR struct usrsock_req_s *req = &g_usrsock_req;
  int num_inbufs;
  int iovpos;

  if (USRSOCK_MESSAGE_REQ_IN_PROGRESS(hdr->head.flags))
    {
      if (datahdr->reqack.result > 0)
        {
          ninfo("error: request in progress, and result > 0.\n");
          return -EINVAL;
        }
      else if (datahdr->valuelen > 0)
        {
          ninfo("error: request in progress, and valuelen > 0.\n");
          return -EINVAL;
        }

      /* In-progress response is acknowledgment that response was
       * received.
       */

      conn->resp.inprogress = true;

      /* This branch indicates successful processing and waiting
       * for USRSOCK_EVENT_CONNECT_READY event.
       */

      conn->resp.result = 0;

      return sizeof(*datahdr);
    }

  conn->resp.inprogress = false;
  conn->resp.xid = 0;

  /* Prepare to read buffers. */

  conn->resp.result = hdr->result;
  conn->resp.valuelen = datahdr->valuelen;
  conn->resp.valuelen_nontrunc = datahdr->valuelen_nontrunc;

  if (conn->resp.result < 0)
    {
      /* Error, valuelen must be zero. */

      if (datahdr->valuelen > 0 || datahdr->valuelen_nontrunc > 0)
        {
          nerr("error: response result negative, and valuelen or "
               "valuelen_nontrunc non-zero.\n");
          return -EINVAL;
        }

      /* Done with request/response. */

      usrsock_event(conn);
      return sizeof(*datahdr);
    }

  /* Check that number of buffers match available. */

  num_inbufs = (hdr->result > 0) + 1;

  if (conn->resp.datain.iovcnt < num_inbufs)
    {
      nwarn("not enough recv buffers (need: %d, have: %d).\n", num_inbufs,
            conn->resp.datain.iovcnt);
      return -EINVAL;
    }

  /* Adjust length of receiving buffers. */

  conn->resp.datain.total = 0;
  iovpos = 0;

  /* Value buffer is always the first */

  if (conn->resp.datain.iov[iovpos].iov_len < datahdr->valuelen)
    {
      nwarn("%dth buffer not large enough (need: %d, have: %zu).\n",
            iovpos, datahdr->valuelen,
            conn->resp.datain.iov[iovpos].iov_len);
      return -EINVAL;
    }

  /* Adjust read size. */

  conn->resp.datain.iov[iovpos].iov_len = datahdr->valuelen;
  conn->resp.datain.total += conn->resp.datain.iov[iovpos].iov_len;
  iovpos++;

  if (hdr->result > 0)
    {
      /* Value buffer is always the first */

      if (conn->resp.datain.iov[iovpos].iov_len < hdr->result)
        {
          nwarn("%dth buffer not large enough "
                "(need: %" PRId32 ", have: %zu).\n",
                iovpos, hdr->result,
                conn->resp.datain.iov[iovpos].iov_len);
          return -EINVAL;
        }

      /* Adjust read size. */

      conn->resp.datain.iov[iovpos].iov_len = hdr->result;
      conn->resp.datain.total += conn->resp.datain.iov[iovpos].iov_len;
      iovpos++;
    }

  DEBUGASSERT(num_inbufs == iovpos);

  conn->resp.datain.iovcnt = num_inbufs;

  /* Next written buffers are redirected to data buffers. */

  req->datain_conn = conn;
  return sizeof(*datahdr);
}

/****************************************************************************
 * Name: usrsock_handle_req_response
 ****************************************************************************/

static ssize_t usrsock_handle_req_response(FAR const void *buffer,
                                           size_t len, FAR bool *req_done)
{
  FAR const struct usrsock_message_req_ack_s *hdr = buffer;
  FAR struct usrsock_conn_s *conn = NULL;
  FAR struct usrsock_req_s *req = &g_usrsock_req;
  ssize_t (*handle_response)(FAR struct usrsock_conn_s *conn,
                             FAR const void *buffer,
                             size_t len);
  size_t hdrlen;
  ssize_t ret;

  switch (hdr->head.msgid)
    {
    case USRSOCK_MESSAGE_RESPONSE_ACK:
      hdrlen = sizeof(struct usrsock_message_req_ack_s);
      handle_response = &usrsock_handle_response;
      break;

    case USRSOCK_MESSAGE_RESPONSE_DATA_ACK:
      hdrlen = sizeof(struct usrsock_message_datareq_ack_s);
      handle_response = &usrsock_handle_datareq_response;
      break;

    default:
      nwarn("unknown message type: %d, flags: %d, xid: %" PRIu32 ", "
            "result: %" PRId32 "\n",
            hdr->head.msgid, hdr->head.flags, hdr->xid, hdr->result);
      return -EINVAL;
    }

  if (len < hdrlen)
    {
      nwarn("message too short, %zu < %zu.\n", len, hdrlen);
      return -EINVAL;
    }

  net_lock();

  /* Get corresponding usrsock connection for this transfer */

  while ((conn = usrsock_nextconn(conn)) != NULL &&
         conn->resp.xid != hdr->xid);
  if (!conn)
    {
      /* No connection waiting for this message. */

      nwarn("Could find connection waiting for response"
            "with xid=%" PRIu32 "\n", hdr->xid);

      ret = -EINVAL;
      goto unlock_out;
    }

  if (req->ackxid == hdr->xid)
    {
      req->ackxid = 0;
      if (req_done)
        {
          *req_done = true;
        }

      /* Signal that request was received and read by daemon and
       * acknowledgment response was received.
       */

      nxsem_post(&req->acksem);
    }

  conn->resp.events = hdr->head.events | USRSOCK_EVENT_REQ_COMPLETE;
  ret = handle_response(conn, buffer, len);

unlock_out:
  net_unlock();
  return ret;
}

/****************************************************************************
 * Name: usrsock_handle_message
 ****************************************************************************/

static ssize_t usrsock_handle_message(FAR const void *buffer, size_t len,
                                      FAR bool *req_done)
{
  FAR const struct usrsock_message_common_s *common = buffer;

  if (USRSOCK_MESSAGE_IS_EVENT(common->flags))
    {
      return usrsock_handle_event(buffer, len);
    }

  if (USRSOCK_MESSAGE_IS_REQ_RESPONSE(common->flags))
    {
      return usrsock_handle_req_response(buffer, len, req_done);
    }

  return -EINVAL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usrsock_response() - handle usrsock request's ack/response
 ****************************************************************************/

ssize_t usrsock_response(FAR const char *buffer, size_t len,
                         FAR bool *req_done)
{
  FAR struct usrsock_req_s *req = &g_usrsock_req;
  FAR struct usrsock_conn_s *conn;
  size_t origlen = len;
  int ret = 0;

  if (!req->datain_conn)
    {
      /* Start of message, buffer length should be at least size of common
       * message header.
       */

      if (len < sizeof(struct usrsock_message_common_s))
        {
          nwarn("message too short, %zu < %zu.\n", len,
                sizeof(struct usrsock_message_common_s));
          return -EINVAL;
        }

      /* Handle message. */

      ret = usrsock_handle_message(buffer, len, req_done);
      if (ret >= 0)
        {
          buffer += ret;
          len -= ret;
          ret = origlen - len;
        }
    }

  if (req->datain_conn)
    {
      conn = req->datain_conn;

      /* Copy data from user-space. */

      if (len != 0)
        {
          ret = usrsock_iovec_put(conn->resp.datain.iov,
                                  conn->resp.datain.iovcnt,
                                  conn->resp.datain.pos, buffer, len);
          if (ret < 0)
            {
              /* Tried writing beyond buffer. */

              conn->resp.result = ret;
              conn->resp.datain.pos = conn->resp.datain.total;
            }
          else
            {
              conn->resp.datain.pos += ret;
              buffer += ret;
              len -= ret;
              ret = origlen - len;
            }
        }

      if (conn->resp.datain.pos == conn->resp.datain.total)
        {
          req->datain_conn = NULL;

          /* Done with data response. */

          usrsock_event(conn);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: usrsock_iovec_get() - copy from iovec to buffer.
 ****************************************************************************/

ssize_t usrsock_iovec_get(FAR void *dst, size_t dstlen,
                          FAR const struct iovec *iov, int iovcnt,
                          size_t pos, FAR bool *done)
{
  return usrsock_iovec_do(dst, dstlen, (FAR struct iovec *)iov, iovcnt,
                          pos, true, done);
}

/****************************************************************************
 * Name: usrsock_iovec_put() - copy to iovec from buffer.
 ****************************************************************************/

ssize_t usrsock_iovec_put(FAR struct iovec *iov, int iovcnt, size_t pos,
                          FAR const void *src, size_t srclen)
{
  return usrsock_iovec_do((FAR void *)src, srclen, iov, iovcnt,
                          pos, false, NULL);
}

/****************************************************************************
 * Name: usrsock_request() - finish usrsock's request
 ****************************************************************************/

int usrsock_do_request(FAR struct usrsock_conn_s *conn,
                       FAR struct iovec *iov, unsigned int iovcnt)
{
  FAR struct usrsock_request_common_s *req_head = NULL;
  FAR struct usrsock_req_s *req = &g_usrsock_req;
  int ret;

  /* Get exchange id. */

  req_head = iov[0].iov_base;

  /* Set outstanding request for daemon to handle. */

  net_lockedwait_uninterruptible(&req->sem);
  if (++req->newxid == 0)
    {
      ++req->newxid;
    }

  req_head->xid = req->newxid;

  /* Prepare connection for response. */

  conn->resp.xid = req_head->xid;
  conn->resp.result = -EACCES;

  req->ackxid = req_head->xid;

  ret = usrsock_request(iov, iovcnt);
  if (ret == OK)
    {
      /* Wait ack for request. */

      ++req->nbusy; /* net_lock held. */
      net_lockedwait_uninterruptible(&req->acksem);
      --req->nbusy; /* net_lock held. */
    }

  /* Free request line for next command. */

  nxsem_post(&req->sem);
  return ret;
}

/****************************************************************************
 * Name: usrsock_abort() - abort all usrsock's operations
 ****************************************************************************/

void usrsock_abort(void)
{
  FAR struct usrsock_req_s *req = &g_usrsock_req;
  FAR struct usrsock_conn_s *conn = NULL;
  int ret;

  net_lock();

  /* Set active usrsock sockets to aborted state. */

  while ((conn = usrsock_nextconn(conn)) != NULL)
    {
      conn->resp.inprogress = false;
      conn->resp.xid = 0;
      conn->resp.events = USRSOCK_EVENT_ABORT;
      usrsock_event(conn);
    }

  do
    {
      /* Give other threads short time window to complete recently completed
       * requests.
       */

      ret = net_timedwait(&req->sem, 10);
      if (ret < 0)
        {
          if (ret != -ETIMEDOUT && ret != -EINTR)
            {
              ninfo("net_timedwait errno: %d\n", ret);
              DEBUGASSERT(false);
            }
        }
      else
        {
          nxsem_post(&req->sem);
        }

      /* Wake-up pending requests. */

      if (req->nbusy == 0)
        {
          break;
        }

      nxsem_post(&req->acksem);
    }
  while (true);

  net_unlock();
}

#endif /* CONFIG_NET && CONFIG_NET_USRSOCK */
