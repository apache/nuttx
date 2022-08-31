/****************************************************************************
 * net/usrsock/usrsock_dev.c
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
#include <poll.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <arch/irq.h>

#include <nuttx/random.h>
#include <nuttx/fs/fs.h>
#include <nuttx/semaphore.h>
#include <nuttx/net/net.h>
#include <nuttx/net/usrsock.h>

#include "devif/devif.h"
#include "usrsock/usrsock.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_NET_USRSOCKDEV_NPOLLWAITERS
#  define CONFIG_NET_USRSOCKDEV_NPOLLWAITERS 1
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct usrsockdev_s
{
  sem_t    devsem; /* Lock for device node */
  uint8_t  ocount; /* The number of times the device has been opened */
  uint32_t newxid; /* New transcation Id */

  struct
  {
    FAR const struct iovec *iov; /* Pending request buffers */
    int       iovcnt;            /* Number of request buffers */
    size_t    pos;               /* Reader position on request buffer */
    sem_t     sem;               /* Request semaphore (only one outstanding
                                  * request) */
    sem_t     acksem;            /* Request acknowledgment notification */
    uint64_t  ackxid;            /* Exchange id for which waiting ack */
    uint16_t  nbusy;             /* Number of requests blocked from different
                                  * threads */
  } req;

  FAR struct usrsock_conn_s *datain_conn; /* Connection instance to receive
                                           * data buffers. */
  struct pollfd *pollfds[CONFIG_NET_USRSOCKDEV_NPOLLWAITERS];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Character driver methods */

static ssize_t usrsockdev_read(FAR struct file *filep, FAR char *buffer,
                               size_t len);

static ssize_t usrsockdev_write(FAR struct file *filep,
                                FAR const char *buffer, size_t len);

static off_t usrsockdev_seek(FAR struct file *filep, off_t offset,
                             int whence);

static int usrsockdev_open(FAR struct file *filep);

static int usrsockdev_close(FAR struct file *filep);

static int usrsockdev_poll(FAR struct file *filep, FAR struct pollfd *fds,
                           bool setup);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_usrsockdevops =
{
  usrsockdev_open,    /* open */
  usrsockdev_close,   /* close */
  usrsockdev_read,    /* read */
  usrsockdev_write,   /* write */
  usrsockdev_seek,    /* seek */
  NULL,               /* ioctl */
  usrsockdev_poll     /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL              /* unlink */
#endif
};

static struct usrsockdev_s g_usrsockdev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: iovec_do() - copy to/from iovec from/to buffer.
 ****************************************************************************/

static ssize_t iovec_do(FAR void *srcdst, size_t srcdstlen,
                        FAR struct iovec *iov, int iovcnt, size_t pos,
                        bool from_iov)
{
  ssize_t total;
  size_t srclen;
  FAR uint8_t *ioout = srcdst;
  FAR uint8_t *iovbuf;

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

      return -1;
    }

  iovbuf = iov->iov_base;
  srclen = iov->iov_len;
  iovbuf += pos;
  srclen -= pos;
  iov++;
  iovcnt--;
  total = 0;

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

  return total;
}

/****************************************************************************
 * Name: iovec_get() - copy from iovec to buffer.
 ****************************************************************************/

static ssize_t iovec_get(FAR void *dst, size_t dstlen,
                         FAR const struct iovec *iov, int iovcnt, size_t pos)
{
  return iovec_do(dst, dstlen, (FAR struct iovec *)iov, iovcnt, pos, true);
}

/****************************************************************************
 * Name: iovec_put() - copy to iovec from buffer.
 ****************************************************************************/

static ssize_t iovec_put(FAR struct iovec *iov, int iovcnt, size_t pos,
                         FAR const void *src, size_t srclen)
{
  return iovec_do((FAR void *)src, srclen, iov, iovcnt, pos, false);
}

/****************************************************************************
 * Name: usrsockdev_semtake() and usrsockdev_semgive()
 *
 * Description:
 *   Take/give semaphore
 *
 ****************************************************************************/

static int usrsockdev_semtake(FAR sem_t *sem)
{
  return nxsem_wait_uninterruptible(sem);
}

static void usrsockdev_semgive(FAR sem_t *sem)
{
  nxsem_post(sem);
}

/****************************************************************************
 * Name: usrsockdev_is_opened
 ****************************************************************************/

static bool usrsockdev_is_opened(FAR struct usrsockdev_s *dev)
{
  bool ret = true;

  if (dev->ocount == 0)
    {
      ret = false; /* No usrsock daemon running. */
    }

  return ret;
}

/****************************************************************************
 * Name: usrsockdev_pollnotify
 ****************************************************************************/

static void usrsockdev_pollnotify(FAR struct usrsockdev_s *dev,
                                  pollevent_t eventset)
{
  int i;
  for (i = 0; i < ARRAY_SIZE(dev->pollfds); i++)
    {
      struct pollfd *fds = dev->pollfds[i];
      if (fds)
        {
          fds->revents |= (fds->events & eventset);
          if (fds->revents != 0)
            {
              ninfo("Report events: %08" PRIx32 "\n", fds->revents);
              nxsem_post(fds->sem);
            }
        }
    }
}

/****************************************************************************
 * Name: usrsockdev_read
 ****************************************************************************/

static ssize_t usrsockdev_read(FAR struct file *filep, FAR char *buffer,
                               size_t len)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct usrsockdev_s *dev;
  int                      ret;

  if (len == 0)
    {
      return 0;
    }

  if (buffer == NULL)
    {
      return -EINVAL;
    }

  DEBUGASSERT(inode);

  dev = inode->i_private;

  DEBUGASSERT(dev);

  ret = usrsockdev_semtake(&dev->devsem);
  if (ret < 0)
    {
      return ret;
    }

  net_lock();

  /* Is request available? */

  if (dev->req.iov)
    {
      ssize_t rlen;

      /* Copy request to user-space. */

      rlen = iovec_get(buffer, len, dev->req.iov, dev->req.iovcnt,
                       dev->req.pos);
      if (rlen < 0)
        {
          /* Tried reading beyond buffer. */

          len = 0;
        }
      else
        {
          dev->req.pos += rlen;
          len = rlen;
        }
    }
  else
    {
      len = 0;
    }

  net_unlock();
  usrsockdev_semgive(&dev->devsem);

  return len;
}

/****************************************************************************
 * Name: usrsockdev_seek
 ****************************************************************************/

static off_t usrsockdev_seek(FAR struct file *filep, off_t offset,
                             int whence)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct usrsockdev_s *dev;
  off_t pos;
  int ret;

  if (whence != SEEK_CUR && whence != SEEK_SET)
    {
      return -EINVAL;
    }

  DEBUGASSERT(inode);

  dev = inode->i_private;

  DEBUGASSERT(dev);

  ret = usrsockdev_semtake(&dev->devsem);
  if (ret < 0)
    {
      return ret;
    }

  net_lock();

  /* Is request available? */

  if (dev->req.iov)
    {
      ssize_t rlen;

      if (whence == SEEK_CUR)
        {
          pos = dev->req.pos + offset;
        }
      else
        {
          pos = offset;
        }

      /* Copy request to user-space. */

      rlen = iovec_get(NULL, 0, dev->req.iov, dev->req.iovcnt, pos);
      if (rlen < 0)
        {
          /* Tried seek beyond buffer. */

          pos = -EINVAL;
        }
      else
        {
          dev->req.pos = pos;
        }
    }
  else
    {
      pos = 0;
    }

  net_unlock();
  usrsockdev_semgive(&dev->devsem);

  return pos;
}

/****************************************************************************
 * Name: usrsockdev_handle_event
 ****************************************************************************/

static ssize_t usrsockdev_handle_event(FAR struct usrsockdev_s *dev,
                                       FAR const void *buffer,
                                       size_t len)
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

        ret = usrsock_event(conn,
                            hdr->head.events & ~USRSOCK_EVENT_INTERNAL_MASK);
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
 * Name: usrsockdev_handle_response
 ****************************************************************************/

static ssize_t usrsockdev_handle_response(FAR struct usrsockdev_s *dev,
                                          FAR struct usrsock_conn_s *conn,
                                          FAR const void *buffer)
{
  FAR const struct usrsock_message_req_ack_s *hdr = buffer;

  if (USRSOCK_MESSAGE_REQ_IN_PROGRESS(hdr->head.flags))
    {
      /* In-progress response is acknowledgment that response was
       * received.
       */

      conn->resp.inprogress = true;
    }
  else
    {
      conn->resp.inprogress = false;
      conn->resp.xid = 0;

      /* Get result for common request. */

      conn->resp.result = hdr->result;

      /* Done with request/response. */

      usrsock_event(conn, USRSOCK_EVENT_REQ_COMPLETE);
    }

  return sizeof(*hdr);
}

/****************************************************************************
 * Name: usrsockdev_handle_datareq_response
 ****************************************************************************/

static ssize_t
usrsockdev_handle_datareq_response(FAR struct usrsockdev_s *dev,
                                   FAR struct usrsock_conn_s *conn,
                                   FAR const void *buffer)
{
  FAR const struct usrsock_message_datareq_ack_s *datahdr = buffer;
  FAR const struct usrsock_message_req_ack_s *hdr = &datahdr->reqack;
  int num_inbufs;
  int iovpos;
  ssize_t ret;

  if (USRSOCK_MESSAGE_REQ_IN_PROGRESS(hdr->head.flags))
    {
      if (datahdr->reqack.result > 0)
        {
          ninfo("error: request in progress, and result > 0.\n");
          ret = -EINVAL;
          goto unlock_out;
        }
      else if (datahdr->valuelen > 0)
        {
          ninfo("error: request in progress, and valuelen > 0.\n");
          ret = -EINVAL;
          goto unlock_out;
        }

      /* In-progress response is acknowledgment that response was
       * received.
       */

      conn->resp.inprogress = true;

      ret = sizeof(*datahdr);
      goto unlock_out;
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

          ret = -EINVAL;
          goto unlock_out;
        }

      /* Done with request/response. */

      usrsock_event(conn, USRSOCK_EVENT_REQ_COMPLETE);

      ret = sizeof(*datahdr);
      goto unlock_out;
    }

  /* Check that number of buffers match available. */

  num_inbufs = (hdr->result > 0) + 1;

  if (conn->resp.datain.iovcnt < num_inbufs)
    {
      nwarn("not enough recv buffers (need: %d, have: %d).\n", num_inbufs,
            conn->resp.datain.iovcnt);

      ret = -EINVAL;
      goto unlock_out;
    }

  /* Adjust length of receiving buffers. */

  conn->resp.datain.total = 0;
  iovpos = 0;

  /* Value buffer is always the first */

  if (conn->resp.datain.iov[iovpos].iov_len < datahdr->valuelen)
    {
      nwarn("%dth buffer not large enough (need: %d, have: %zu).\n",
            iovpos,
            datahdr->valuelen,
            conn->resp.datain.iov[iovpos].iov_len);

      ret = -EINVAL;
      goto unlock_out;
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
                iovpos,
                hdr->result,
                conn->resp.datain.iov[iovpos].iov_len);

          ret = -EINVAL;
          goto unlock_out;
        }

      /* Adjust read size. */

      conn->resp.datain.iov[iovpos].iov_len = hdr->result;
      conn->resp.datain.total += conn->resp.datain.iov[iovpos].iov_len;
      iovpos++;
    }

  DEBUGASSERT(num_inbufs == iovpos);

  conn->resp.datain.iovcnt = num_inbufs;

  /* Next written buffers are redirected to data buffers. */

  dev->datain_conn = conn;
  ret = sizeof(*datahdr);

unlock_out:
  return ret;
}

/****************************************************************************
 * Name: usrsockdev_handle_req_response
 ****************************************************************************/

static ssize_t usrsockdev_handle_req_response(FAR struct usrsockdev_s *dev,
                                              FAR const void *buffer,
                                              size_t len)
{
  FAR const struct usrsock_message_req_ack_s *hdr = buffer;
  FAR struct usrsock_conn_s *conn = NULL;
  unsigned int hdrlen;
  ssize_t ret;
  ssize_t (*handle_response)(FAR struct usrsockdev_s *dev,
                             FAR struct usrsock_conn_s *conn,
                             FAR const void *buffer);

  switch (hdr->head.msgid)
    {
    case USRSOCK_MESSAGE_RESPONSE_ACK:
      hdrlen = sizeof(struct usrsock_message_req_ack_s);
      handle_response = &usrsockdev_handle_response;
      break;

    case USRSOCK_MESSAGE_RESPONSE_DATA_ACK:
      hdrlen = sizeof(struct usrsock_message_datareq_ack_s);
      handle_response = &usrsockdev_handle_datareq_response;
      break;

    default:
      nwarn("unknown message type: %d, flags: %d, xid: %" PRIu32 ", "
            "result: %" PRId32 "\n",
            hdr->head.msgid, hdr->head.flags, hdr->xid, hdr->result);
      return -EINVAL;
    }

  if (len < hdrlen)
    {
      nwarn("message too short, %zu < %u.\n", len, hdrlen);

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

  if (dev->req.ackxid == hdr->xid && dev->req.iov)
    {
      /* Signal that request was received and read by daemon and
       * acknowledgment response was received.
       */

      dev->req.iov = NULL;

      nxsem_post(&dev->req.acksem);
    }

  ret = handle_response(dev, conn, buffer);

unlock_out:
  net_unlock();
  return ret;
}

/****************************************************************************
 * Name: usrsockdev_handle_message
 ****************************************************************************/

static ssize_t usrsockdev_handle_message(FAR struct usrsockdev_s *dev,
                                         FAR const void *buffer,
                                         size_t len)
{
  FAR const struct usrsock_message_common_s *common = buffer;

  if (USRSOCK_MESSAGE_IS_EVENT(common->flags))
    {
      return usrsockdev_handle_event(dev, buffer, len);
    }

  if (USRSOCK_MESSAGE_IS_REQ_RESPONSE(common->flags))
    {
      return usrsockdev_handle_req_response(dev, buffer, len);
    }

  return -EINVAL;
}

/****************************************************************************
 * Name: usrsockdev_write
 ****************************************************************************/

static ssize_t usrsockdev_write(FAR struct file *filep,
                                FAR const char *buffer, size_t len)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct usrsock_conn_s *conn;
  FAR struct usrsockdev_s *dev;
  size_t origlen = len;
  ssize_t ret = 0;

  if (len == 0)
    {
      return 0;
    }

  if (buffer == NULL)
    {
      return -EINVAL;
    }

  DEBUGASSERT(inode);

  dev = inode->i_private;

  DEBUGASSERT(dev);

  ret = (ssize_t)usrsockdev_semtake(&dev->devsem);
  if (ret < 0)
    {
      return ret;
    }

  if (!dev->datain_conn)
    {
      /* Start of message, buffer length should be at least size of common
       * message header.
       */

      if (len < sizeof(struct usrsock_message_common_s))
        {
          nwarn("message too short, %zu < %zu.\n", len,
                sizeof(struct usrsock_message_common_s));

          ret = -EINVAL;
          goto errout;
        }

      /* Handle message. */

      ret = usrsockdev_handle_message(dev, buffer, len);
      if (ret >= 0)
        {
          buffer += ret;
          len -= ret;
          ret = origlen - len;
        }
    }

  /* Data input handling. */

  if (dev->datain_conn)
    {
      conn = dev->datain_conn;

      /* Copy data from user-space. */

      if (len != 0)
        {
          ret = iovec_put(conn->resp.datain.iov, conn->resp.datain.iovcnt,
                          conn->resp.datain.pos, buffer, len);
          if (ret < 0)
            {
              /* Tried writing beyond buffer. */

              ret = -EINVAL;
              conn->resp.result = -EINVAL;
              conn->resp.datain.pos =
                  conn->resp.datain.total;
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
          dev->datain_conn = NULL;

          /* Done with data response. */

          usrsock_event(conn, USRSOCK_EVENT_REQ_COMPLETE);
        }
    }

errout:
  usrsockdev_semgive(&dev->devsem);
  return ret;
}

/****************************************************************************
 * Name: usrsockdev_open
 ****************************************************************************/

static int usrsockdev_open(FAR struct file *filep)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct usrsockdev_s *dev;
  int ret;
  int tmp;

  DEBUGASSERT(inode);

  dev = inode->i_private;

  DEBUGASSERT(dev);

  ret = usrsockdev_semtake(&dev->devsem);
  if (ret < 0)
    {
      return ret;
    }

  ninfo("opening /dev/usrsock\n");

  /* Increment the count of references to the device. */

  tmp = dev->ocount + 1;
  if (tmp > 1)
    {
      /* Only one reference is allowed. */

      nwarn("failed to open\n");

      ret = -EPERM;
    }
  else
    {
      dev->ocount = tmp;
      ret = OK;
    }

  usrsockdev_semgive(&dev->devsem);

  return ret;
}

/****************************************************************************
 * Name: usrsockdev_close
 ****************************************************************************/

static int usrsockdev_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct usrsock_conn_s *conn = NULL;
  FAR struct usrsockdev_s *dev;
  int ret;

  DEBUGASSERT(inode);

  dev = inode->i_private;

  DEBUGASSERT(dev);

  ret = usrsockdev_semtake(&dev->devsem);
  if (ret < 0)
    {
      return ret;
    }

  ninfo("closing /dev/usrsock\n");

  net_lock();

  /* Set active usrsock sockets to aborted state. */

  while ((conn = usrsock_nextconn(conn)) != NULL)
    {
      conn->resp.inprogress = false;
      conn->resp.xid = 0;
      usrsock_event(conn, USRSOCK_EVENT_ABORT);
    }

  /* Decrement the references to the driver. */

  dev->ocount--;
  DEBUGASSERT(dev->ocount == 0);
  ret = OK;

  do
    {
      /* Give other threads short time window to complete recently completed
       * requests.
       */

      ret = net_timedwait(&dev->req.sem, 10);
      if (ret < 0)
        {
          if (ret != -ETIMEDOUT && ret != -EINTR)
            {
              ninfo("net_timedwait errno: %d\n", ret);
              DEBUGPANIC();
            }
        }
      else
        {
          usrsockdev_semgive(&dev->req.sem);
        }

      /* Wake-up pending requests. */

      if (dev->req.nbusy == 0)
        {
          break;
        }

      dev->req.iov = NULL;
      nxsem_post(&dev->req.acksem);
    }
  while (true);

  net_unlock();

  /* Check if request line is active */

  if (dev->req.iov != NULL)
    {
      dev->req.iov = NULL;
    }

  usrsockdev_semgive(&dev->devsem);

  return ret;
}

/****************************************************************************
 * Name: usrsockdev_poll
 ****************************************************************************/

static int usrsockdev_poll(FAR struct file *filep, FAR struct pollfd *fds,
                           bool setup)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct usrsockdev_s *dev;
  pollevent_t eventset;
  int ret;
  int i;

  DEBUGASSERT(inode);

  dev = inode->i_private;

  DEBUGASSERT(dev);

  /* Some sanity checking */

  if (!dev || !fds)
    {
      return -ENODEV;
    }

  /* Are we setting up the poll?  Or tearing it down? */

  ret = usrsockdev_semtake(&dev->devsem);
  if (ret < 0)
    {
      return ret;
    }

  net_lock();
  if (setup)
    {
      /* This is a request to set up the poll.  Find an available
       * slot for the poll structure reference
       */

      for (i = 0; i < ARRAY_SIZE(dev->pollfds); i++)
        {
          /* Find an available slot */

          if (!dev->pollfds[i])
            {
              /* Bind the poll structure and this slot */

              dev->pollfds[i] = fds;
              fds->priv = &dev->pollfds[i];
              break;
            }
        }

      if (i >= ARRAY_SIZE(dev->pollfds))
        {
          fds->priv = NULL;
          ret = -EBUSY;
          goto errout;
        }

      /* Should immediately notify on any of the requested events? */

      eventset = 0;

      /* Notify the POLLIN event if pending request. */

      if (dev->req.iov != NULL &&
          !(iovec_get(NULL, 0, dev->req.iov,
                      dev->req.iovcnt, dev->req.pos) < 0))
        {
          eventset |= POLLIN;
        }

      if (eventset)
        {
          usrsockdev_pollnotify(dev, eventset);
        }
    }
  else
    {
      /* This is a request to tear down the poll. */

      FAR struct pollfd **slot = (FAR struct pollfd **)fds->priv;

      if (!slot)
        {
          ret = -EIO;
          goto errout;
        }

      /* Remove all memory of the poll setup */

      *slot = NULL;
      fds->priv = NULL;
    }

errout:
  net_unlock();
  usrsockdev_semgive(&dev->devsem);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usrsockdev_do_request
 ****************************************************************************/

int usrsockdev_do_request(FAR struct usrsock_conn_s *conn,
                          FAR struct iovec *iov, unsigned int iovcnt)
{
  FAR struct usrsockdev_s *dev = &g_usrsockdev;
  FAR struct usrsock_request_common_s *req_head = iov[0].iov_base;

  if (!usrsockdev_is_opened(dev))
    {
      ninfo("usockid=%d; daemon has closed /dev/usrsock.\n", conn->usockid);

      return -ENETDOWN;
    }

  /* Get exchange id. */

  if (++dev->newxid == 0)
    {
      ++dev->newxid;
    }

  req_head->xid = dev->newxid;

  /* Prepare connection for response. */

  conn->resp.xid = req_head->xid;
  conn->resp.result = -EACCES;

  ++dev->req.nbusy; /* net_lock held. */

  /* Set outstanding request for daemon to handle. */

  net_lockedwait_uninterruptible(&dev->req.sem);

  if (usrsockdev_is_opened(dev))
    {
      DEBUGASSERT(dev->req.iov == NULL);
      dev->req.ackxid = req_head->xid;
      dev->req.iov = iov;
      dev->req.pos = 0;
      dev->req.iovcnt = iovcnt;

      /* Notify daemon of new request. */

      usrsockdev_pollnotify(dev, POLLIN);

      /* Wait ack for request. */

      net_lockedwait_uninterruptible(&dev->req.acksem);
    }
  else
    {
      ninfo("usockid=%d; daemon abruptly closed /dev/usrsock.\n",
            conn->usockid);
    }

  /* Free request line for next command. */

  usrsockdev_semgive(&dev->req.sem);

  --dev->req.nbusy; /* net_lock held. */

  return OK;
}

/****************************************************************************
 * Name: usrsockdev_register
 *
 * Description:
 *   Register /dev/usrsock
 *
 ****************************************************************************/

void usrsockdev_register(void)
{
  /* Initialize device private structure. */

  g_usrsockdev.ocount = 0;
  g_usrsockdev.req.nbusy = 0;
  nxsem_init(&g_usrsockdev.devsem, 0, 1);
  nxsem_init(&g_usrsockdev.req.sem, 0, 1);
  nxsem_init(&g_usrsockdev.req.acksem, 0, 0);
  nxsem_set_protocol(&g_usrsockdev.req.acksem, SEM_PRIO_NONE);

  register_driver("/dev/usrsock", &g_usrsockdevops, 0666,
                  &g_usrsockdev);
}

#endif /* CONFIG_NET && CONFIG_NET_USRSOCK */
