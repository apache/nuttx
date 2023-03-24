/****************************************************************************
 * drivers/usrsock/usrsock_rpmsg_server.c
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

#include <debug.h>
#include <errno.h>
#include <inttypes.h>
#include <poll.h>
#include <string.h>

#include <sys/ioctl.h>

#include <nuttx/mutex.h>
#include <nuttx/net/dns.h>
#include <nuttx/net/net.h>
#include <nuttx/rptun/openamp.h>
#include <nuttx/usrsock/usrsock_rpmsg.h>
#ifdef CONFIG_NETDEV_WIRELESS_IOCTL
#include <nuttx/wireless/wireless.h>
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct usrsock_rpmsg_s
{
  rmutex_t                  mutex;
  struct iovec              iov[CONFIG_NET_USRSOCK_RPMSG_SERVER_NIOVEC];
  struct socket             socks[CONFIG_NET_USRSOCK_RPMSG_SERVER_NSOCKS];
  FAR struct rpmsg_endpoint *epts[CONFIG_NET_USRSOCK_RPMSG_SERVER_NSOCKS];
  struct pollfd             pfds[CONFIG_NET_USRSOCK_RPMSG_SERVER_NSOCKS];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static bool usrsock_rpmsg_available(FAR struct socket *psock, int cmd);

static int usrsock_rpmsg_send_ack(FAR struct rpmsg_endpoint *ept,
                                  uint16_t events,
                                  uint32_t xid, int32_t result);
static int usrsock_rpmsg_send_data_ack(FAR struct rpmsg_endpoint *ept,
                              FAR struct usrsock_message_datareq_ack_s *ack,
                              uint16_t events,
                              uint32_t xid, int32_t result,
                              uint16_t valuelen,
                              uint16_t valuelen_nontrunc,
                              int32_t datalen);
static int usrsock_rpmsg_send_event(FAR struct rpmsg_endpoint *ept,
                                    int16_t usockid, uint16_t events);

static int usrsock_rpmsg_socket_handler(FAR struct rpmsg_endpoint *ept,
                                        FAR void *data, size_t len,
                                        uint32_t src, FAR void *priv_);
static int usrsock_rpmsg_close_handler(FAR struct rpmsg_endpoint *ept,
                                       FAR void *data, size_t len,
                                       uint32_t src, FAR void *priv_);
static int usrsock_rpmsg_connect_handler(FAR struct rpmsg_endpoint *ept,
                                         FAR void *data, size_t len,
                                         uint32_t src, FAR void *priv_);
static int usrsock_rpmsg_sendto_handler(FAR struct rpmsg_endpoint *ept,
                                        FAR void *data, size_t len,
                                        uint32_t src, FAR void *priv_);
static int usrsock_rpmsg_recvfrom_handler(FAR struct rpmsg_endpoint *ept,
                                          FAR void *data, size_t len,
                                          uint32_t src, FAR void *priv_);
static int usrsock_rpmsg_setsockopt_handler(FAR struct rpmsg_endpoint *ept,
                                            FAR void *data, size_t len,
                                            uint32_t src, FAR void *priv_);
static int usrsock_rpmsg_getsockopt_handler(FAR struct rpmsg_endpoint *ept,
                                            FAR void *data, size_t len,
                                            uint32_t src, FAR void *priv_);
static int usrsock_rpmsg_getsockname_handler(FAR struct rpmsg_endpoint *ept,
                                             FAR void *data, size_t len,
                                             uint32_t src, FAR void *priv_);
static int usrsock_rpmsg_getpeername_handler(FAR struct rpmsg_endpoint *ept,
                                             FAR void *data, size_t len,
                                             uint32_t src, FAR void *priv_);
static int usrsock_rpmsg_bind_handler(FAR struct rpmsg_endpoint *ept,
                                      FAR void *data, size_t len,
                                      uint32_t src, FAR void *priv_);
static int usrsock_rpmsg_listen_handler(FAR struct rpmsg_endpoint *ept,
                                        FAR void *data, size_t len,
                                        uint32_t src, FAR void *priv_);
static int usrsock_rpmsg_accept_handler(FAR struct rpmsg_endpoint *ept,
                                        FAR void *data, size_t len,
                                        uint32_t src, FAR void *priv_);
static int usrsock_rpmsg_ioctl_handler(FAR struct rpmsg_endpoint *ept,
                                       FAR void *data, size_t len,
                                       uint32_t src, FAR void *priv_);
static int usrsock_rpmsg_shutdown_handler(FAR struct rpmsg_endpoint *ept,
                                          FAR void *data, size_t len,
                                          uint32_t src, FAR void *priv_);
static int usrsock_rpmsg_dns_handler(FAR struct rpmsg_endpoint *ept,
                                     FAR void *data, size_t len,
                                     uint32_t src, FAR void *priv_);

static bool usrsock_rpmsg_ns_match(FAR struct rpmsg_device *rdev,
                                   FAR void *priv_, FAR const char *name,
                                   uint32_t dest);
static void usrsock_rpmsg_ns_bind(FAR struct rpmsg_device *rdev,
                                  FAR void *priv_, FAR const char *name,
                                  uint32_t dest);
static void usrsock_rpmsg_ns_unbind(FAR struct rpmsg_endpoint *ept);
static int usrsock_rpmsg_ept_cb(FAR struct rpmsg_endpoint *ept,
                                FAR void *data, size_t len, uint32_t src,
                                FAR void *priv);

static void usrsock_rpmsg_poll_cb(FAR struct pollfd *pfds);
static void usrsock_rpmsg_poll_setup(FAR struct pollfd *pfds,
                                     pollevent_t events);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const rpmsg_ept_cb g_usrsock_rpmsg_handler[] =
{
  usrsock_rpmsg_socket_handler,
  usrsock_rpmsg_close_handler,
  usrsock_rpmsg_connect_handler,
  usrsock_rpmsg_sendto_handler,
  usrsock_rpmsg_recvfrom_handler,
  usrsock_rpmsg_setsockopt_handler,
  usrsock_rpmsg_getsockopt_handler,
  usrsock_rpmsg_getsockname_handler,
  usrsock_rpmsg_getpeername_handler,
  usrsock_rpmsg_bind_handler,
  usrsock_rpmsg_listen_handler,
  usrsock_rpmsg_accept_handler,
  usrsock_rpmsg_ioctl_handler,
  usrsock_rpmsg_shutdown_handler,
  usrsock_rpmsg_dns_handler,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static bool usrsock_rpmsg_available(FAR struct socket *psock, int cmd)
{
  int len;

  if (psock_ioctl(psock, cmd, &len, sizeof(len)) == 0)
    {
      if (len > 0)
        {
          return true;
        }
    }

  return false;
}

static int usrsock_rpmsg_send_ack(FAR struct rpmsg_endpoint *ept,
                                  uint16_t events,
                                  uint32_t xid, int32_t result)
{
  struct usrsock_message_req_ack_s ack;

  ack.head.msgid  = USRSOCK_MESSAGE_RESPONSE_ACK;
  ack.head.flags  = (result == -EINPROGRESS);
  ack.head.events = events;

  ack.xid    = xid;
  ack.result = result == -EINPROGRESS ? 0 : result;

  return rpmsg_send(ept, &ack, sizeof(ack));
}

static int usrsock_rpmsg_send_data_ack(FAR struct rpmsg_endpoint *ept,
                              FAR struct usrsock_message_datareq_ack_s *ack,
                              uint16_t events,
                              uint32_t xid, int32_t result,
                              uint16_t valuelen,
                              uint16_t valuelen_nontrunc,
                              int32_t datalen)
{
  ack->reqack.head.msgid  = USRSOCK_MESSAGE_RESPONSE_DATA_ACK;
  ack->reqack.head.flags  = 0;
  ack->reqack.head.events = events;

  ack->reqack.xid    = xid;
  ack->reqack.result = result;

  if (result < 0)
    {
      valuelen           = 0;
      valuelen_nontrunc  = 0;
      datalen            = 0;
    }
  else if (valuelen > valuelen_nontrunc)
    {
      valuelen           = valuelen_nontrunc;
    }

  ack->valuelen          = valuelen;
  ack->valuelen_nontrunc = valuelen_nontrunc;

  return rpmsg_send_nocopy(ept, ack, sizeof(*ack) + valuelen + datalen);
}

static int usrsock_rpmsg_send_frag_ack(FAR struct rpmsg_endpoint *ept,
                              FAR struct usrsock_message_frag_ack_s *ack,
                              uint16_t events,
                              uint32_t xid, int32_t result,
                              uint32_t datalen)
{
  ack->reqack.head.msgid  = USRSOCK_RPMSG_FRAG_RESPONSE;
  ack->reqack.head.flags  = 0;
  ack->reqack.head.events = events;
  ack->reqack.xid         = xid;
  ack->reqack.result      = result;
  ack->datalen            = datalen;

  return rpmsg_send_nocopy(ept, ack, sizeof(*ack) + datalen);
}

static int usrsock_rpmsg_send_event(FAR struct rpmsg_endpoint *ept,
                                    int16_t usockid, uint16_t events)
{
  struct usrsock_message_socket_event_s event;

  event.head.msgid  = USRSOCK_MESSAGE_SOCKET_EVENT;
  event.head.flags  = USRSOCK_MESSAGE_FLAG_EVENT;
  event.head.events = events;

  event.usockid = usockid;

  return rpmsg_send(ept, &event, sizeof(event));
}

static int usrsock_rpmsg_socket_handler(FAR struct rpmsg_endpoint *ept,
                                        FAR void *data, size_t len,
                                        uint32_t src, FAR void *priv_)
{
  FAR struct usrsock_request_socket_s *req = data;
  FAR struct usrsock_rpmsg_s *priv = priv_;
  uint16_t events = 0;
  int ret = -ENFILE;
  int retr;
  int i;

  nxrmutex_lock(&priv->mutex);
  for (i = 0; i < CONFIG_NET_USRSOCK_RPMSG_SERVER_NSOCKS; i++)
    {
      if (priv->epts[i] == NULL)
        {
          priv->epts[i] = ept;
          nxrmutex_unlock(&priv->mutex);
          ret = psock_socket(req->domain, req->type | SOCK_NONBLOCK,
                             req->protocol, &priv->socks[i]);
          if (ret >= 0)
            {
              ret = i; /* Return index as the usockid */
              if (req->type != SOCK_STREAM && req->type != SOCK_SEQPACKET)
                {
                  events = USRSOCK_EVENT_SENDTO_READY;
                }
            }
          else
            {
              priv->epts[i] = NULL;
            }

          break;
        }
    }

  if (i == CONFIG_NET_USRSOCK_RPMSG_SERVER_NSOCKS)
    {
      nxrmutex_unlock(&priv->mutex);
    }

  retr = usrsock_rpmsg_send_ack(ept, events, req->head.xid, ret);
  if (retr >= 0 && ret >= 0 &&
      req->type != SOCK_STREAM && req->type != SOCK_SEQPACKET)
    {
      usrsock_rpmsg_poll_setup(&priv->pfds[ret], POLLIN);
    }

  return retr;
}

static int usrsock_rpmsg_close_handler(FAR struct rpmsg_endpoint *ept,
                                       FAR void *data, size_t len,
                                       uint32_t src, FAR void *priv_)
{
  FAR struct usrsock_request_close_s *req = data;
  FAR struct usrsock_rpmsg_s *priv = priv_;
  int ret = -EBADF;

  if (req->usockid >= 0 &&
      req->usockid < CONFIG_NET_USRSOCK_RPMSG_SERVER_NSOCKS)
    {
      usrsock_rpmsg_poll_setup(&priv->pfds[req->usockid], 0);

      /* It's safe to close sock here */

      ret = psock_close(&priv->socks[req->usockid]);
      nxrmutex_lock(&priv->mutex);
      priv->epts[req->usockid] = NULL;
      nxrmutex_unlock(&priv->mutex);
    }

  return usrsock_rpmsg_send_ack(ept, 0, req->head.xid, ret);
}

static int usrsock_rpmsg_connect_handler(FAR struct rpmsg_endpoint *ept,
                                         FAR void *data, size_t len,
                                         uint32_t src, FAR void *priv_)
{
  FAR struct usrsock_request_connect_s *req = data;
  FAR struct usrsock_rpmsg_s *priv = priv_;
  bool inprogress = false;
  int ret = -EBADF;
  int retr;

  if (req->usockid >= 0 &&
      req->usockid < CONFIG_NET_USRSOCK_RPMSG_SERVER_NSOCKS)
    {
      ret = psock_connect(&priv->socks[req->usockid],
              (FAR const struct sockaddr *)(req + 1), req->addrlen);
    }

  retr = usrsock_rpmsg_send_ack(ept, 0, req->head.xid, ret);
  if (ret == -EINPROGRESS)
    {
      inprogress = true;
      ret = 0;
    }

  if (retr >= 0 && ret >= 0)
    {
      pollevent_t events = POLLIN;

      if (inprogress)
        {
          events |= POLLOUT;
        }

      usrsock_rpmsg_poll_setup(&priv->pfds[req->usockid], events);
      if (!inprogress)
        {
          retr = usrsock_rpmsg_send_event(ept, req->usockid,
                                          USRSOCK_EVENT_SENDTO_READY);
        }
    }

  return retr;
}

static int usrsock_rpmsg_sendto_handler(FAR struct rpmsg_endpoint *ept,
                                        FAR void *data, size_t len,
                                        uint32_t src, FAR void *priv_)
{
  FAR struct usrsock_request_sendto_s *req;
  FAR struct usrsock_rpmsg_s *priv = priv_;
  uint16_t events = 0;
  ssize_t ret = -EBADF;
  size_t total;
  int retr;
  int i;

  if (priv->iov[0].iov_base)
    {
      size_t hlen;
      struct msghdr msg =
      {
      };

      req = priv->iov[0].iov_base;
      hlen = sizeof(*req) + req->addrlen;

      total = len;
      for (i = 0; i < CONFIG_NET_USRSOCK_RPMSG_SERVER_NIOVEC; i++)
        {
          if (!priv->iov[i].iov_base)
            {
              priv->iov[i].iov_base = data;
              priv->iov[i].iov_len = len;
              rpmsg_hold_rx_buffer(ept, data);
              break;
            }

          total += priv->iov[i].iov_len;
        }

      if (i == CONFIG_NET_USRSOCK_RPMSG_SERVER_NIOVEC)
        {
          ret = -ENOMEM;
          goto out;
        }

      /* Partial packet ? continue to fetch */

      if (req->buflen > total - hlen)
        {
          return 0;
        }
      else if (req->buflen < total - hlen)
        {
          ret = -EINVAL;
          goto out;
        }

      /* Skip the sendto header from I/O vector */

      priv->iov[0].iov_base = (FAR char *)priv->iov[0].iov_base + hlen;
      priv->iov[0].iov_len -= hlen;

      msg.msg_name = req->addrlen ? (FAR void *)(req + 1) : NULL;
      msg.msg_namelen = req->addrlen;
      msg.msg_iov = priv->iov;
      msg.msg_iovlen = i + 1;

      ret = psock_sendmsg(&priv->socks[req->usockid], &msg, req->flags);

      /* Recover the I/O vector */

      priv->iov[0].iov_base = (FAR char *)priv->iov[0].iov_base - hlen;
      priv->iov[0].iov_len += hlen;
    }
  else
    {
      req = data;

      if (req->usockid >= 0 &&
          req->usockid < CONFIG_NET_USRSOCK_RPMSG_SERVER_NSOCKS)
        {
          total = sizeof(*req) + req->addrlen + req->buflen;
          if (total > len)
            {
              priv->iov[0].iov_base = data;
              priv->iov[0].iov_len = len;

              rpmsg_hold_rx_buffer(ept, data);
              return 0;
            }
          else
            {
              ret = psock_sendto(&priv->socks[req->usockid],
                  (FAR const void *)(req + 1) + req->addrlen, req->buflen,
                  req->flags,
                  req->addrlen ?
                        (FAR const struct sockaddr *)(req + 1) : NULL,
                  req->addrlen);
            }
        }
    }

out:
  if (ret > 0 &&
      usrsock_rpmsg_available(&priv->socks[req->usockid], FIONSPACE))
    {
      events |= USRSOCK_EVENT_SENDTO_READY;
    }

  retr = usrsock_rpmsg_send_ack(ept, events, req->head.xid, ret);
  if (retr >= 0 && events == 0)
    {
      usrsock_rpmsg_poll_setup(&priv->pfds[req->usockid], POLLOUT);
    }

  if (priv->iov[0].iov_base)
    {
      for (i = 0; i < CONFIG_NET_USRSOCK_RPMSG_SERVER_NIOVEC; i++)
        {
          if (priv->iov[i].iov_base == NULL)
            {
              break;
            }

            rpmsg_release_rx_buffer(ept, priv->iov[i].iov_base);
            priv->iov[i].iov_base = NULL;
            priv->iov[i].iov_len = 0;
        }
    }

  return retr;
}

static int usrsock_rpmsg_recvfrom_handler(FAR struct rpmsg_endpoint *ept,
                                          FAR void *data, size_t len_,
                                          uint32_t src, FAR void *priv_)
{
  struct iovec iov[CONFIG_NET_USRSOCK_RPMSG_SERVER_NIOVEC];
  FAR struct usrsock_request_recvfrom_s *req = data;
  FAR struct usrsock_message_datareq_ack_s *ack;
  FAR struct usrsock_rpmsg_s *priv = priv_;
  socklen_t outaddrlen = req->max_addrlen;
  socklen_t inaddrlen = req->max_addrlen;
  size_t buflen = req->max_buflen;
  size_t totlen = 0;
  ssize_t ret = -EBADF;
  uint32_t len = buflen;
  uint16_t events = 0;
  uint8_t i = 0;
  int retr;

  ack = rpmsg_get_tx_payload_buffer(ept, &len, true);
  if (sizeof(*ack) + inaddrlen + buflen < len)
    {
      len = sizeof(*ack) + inaddrlen + buflen;
    }

  memset(iov, 0, sizeof(iov));
  if (req->usockid >= 0 &&
      req->usockid < CONFIG_NET_USRSOCK_RPMSG_SERVER_NSOCKS)
    {
      ret = psock_recvfrom(&priv->socks[req->usockid],
              (FAR void *)(ack + 1) + inaddrlen,
              len - sizeof(*ack) - inaddrlen,
              req->flags,
              outaddrlen ? (FAR struct sockaddr *)(ack + 1) : NULL,
              outaddrlen ? &outaddrlen : NULL);
      totlen = ret;
      if (ret > 0 && (priv->socks[req->usockid].s_type & SOCK_TYPE_MASK) ==
                      SOCK_STREAM)
        {
          if (outaddrlen < inaddrlen)
            {
              memcpy((FAR void *)(ack + 1) + outaddrlen,
                     (FAR void *)(ack + 1) + inaddrlen, ret);
            }

          while (totlen < buflen &&
                 i < CONFIG_NET_USRSOCK_RPMSG_SERVER_NIOVEC)
            {
              uint32_t offset = sizeof(struct usrsock_message_frag_ack_s);

              if (!usrsock_rpmsg_available(&priv->socks[req->usockid],
                                           FIONREAD))
                {
                  break;
                }

              iov[i].iov_base = rpmsg_get_tx_payload_buffer(ept,
                                                            &len,
                                                            false);
              if (!iov[i].iov_base)
                {
                  events |= USRSOCK_EVENT_RECVFROM_AVAIL;
                  break;
                }

              DEBUGASSERT(len > offset);
              if (buflen - totlen < len - offset)
                {
                  len = buflen - totlen + offset;
                }

              /* Should never wait */

              iov[i].iov_len =
                psock_recvfrom(&priv->socks[req->usockid],
                    (FAR char *)iov[i].iov_base + offset,
                    len - offset,
                    req->flags | MSG_DONTWAIT,
                    NULL, NULL);
              if ((ssize_t)iov[i].iov_len > 0)
                {
                  totlen += iov[i].iov_len;
                  if (iov[i].iov_len < len - offset)
                    {
                      break;
                    }
                }
              else
                {
                  iov[i].iov_len = 0;
                  events |= USRSOCK_EVENT_RECVFROM_AVAIL;
                  break;
                }

              i++;
            }
        }
    }

  retr = usrsock_rpmsg_send_data_ack(ept,
                                     ack, events, req->head.xid,
                                     totlen, inaddrlen, outaddrlen,
                                     ret);
  for (i = 0; i < CONFIG_NET_USRSOCK_RPMSG_SERVER_NIOVEC; i++)
    {
      if (!iov[i].iov_base)
        {
          break;
        }

      if (!iov[i].iov_len || retr <= 0)
        {
          rpmsg_release_tx_buffer(ept, iov[i].iov_base);
          iov[i].iov_base = NULL;
          continue;
        }

      usrsock_rpmsg_send_frag_ack(ept,
            (FAR struct usrsock_message_frag_ack_s *)iov[i].iov_base,
            events, req->head.xid, totlen, iov[i].iov_len);
    }

  if (retr >= 0 && events == 0)
    {
      usrsock_rpmsg_poll_setup(&priv->pfds[req->usockid], POLLIN);
    }

  return retr;
}

static int usrsock_rpmsg_setsockopt_handler(FAR struct rpmsg_endpoint *ept,
                                            FAR void *data, size_t len,
                                            uint32_t src, FAR void *priv_)
{
  FAR struct usrsock_request_setsockopt_s *req = data;
  FAR struct usrsock_rpmsg_s *priv = priv_;
  int ret = -EBADF;

  if (req->usockid >= 0 &&
      req->usockid < CONFIG_NET_USRSOCK_RPMSG_SERVER_NSOCKS)
    {
      ret = psock_setsockopt(&priv->socks[req->usockid],
              req->level, req->option, req + 1, req->valuelen);
    }

  return usrsock_rpmsg_send_ack(ept, 0, req->head.xid, ret);
}

static int usrsock_rpmsg_getsockopt_handler(FAR struct rpmsg_endpoint *ept,
                                            FAR void *data, size_t len_,
                                            uint32_t src, FAR void *priv_)
{
  FAR struct usrsock_request_getsockopt_s *req = data;
  FAR struct usrsock_message_datareq_ack_s *ack;
  FAR struct usrsock_rpmsg_s *priv = priv_;
  socklen_t optlen = req->max_valuelen;
  int ret = -EBADF;
  uint32_t len;

  ack = rpmsg_get_tx_payload_buffer(ept, &len, true);
  if (req->usockid >= 0 &&
      req->usockid < CONFIG_NET_USRSOCK_RPMSG_SERVER_NSOCKS)
    {
      ret = psock_getsockopt(&priv->socks[req->usockid],
              req->level, req->option, ack + 1, &optlen);
    }

  return usrsock_rpmsg_send_data_ack(ept,
          ack, 0, req->head.xid, ret, optlen, optlen, ret);
}

static int usrsock_rpmsg_getsockname_handler(FAR struct rpmsg_endpoint *ept,
                                             FAR void *data, size_t len_,
                                             uint32_t src, FAR void *priv_)
{
  FAR struct usrsock_request_getsockname_s *req = data;
  FAR struct usrsock_message_datareq_ack_s *ack;
  FAR struct usrsock_rpmsg_s *priv = priv_;
  socklen_t outaddrlen = req->max_addrlen;
  socklen_t inaddrlen = req->max_addrlen;
  int ret = -EBADF;
  uint32_t len;

  ack = rpmsg_get_tx_payload_buffer(ept, &len, true);
  if (req->usockid >= 0 &&
      req->usockid < CONFIG_NET_USRSOCK_RPMSG_SERVER_NSOCKS)
    {
      ret = psock_getsockname(&priv->socks[req->usockid],
              (FAR struct sockaddr *)(ack + 1), &outaddrlen);
    }

  return usrsock_rpmsg_send_data_ack(ept,
          ack, 0, req->head.xid, ret, inaddrlen, outaddrlen, ret);
}

static int usrsock_rpmsg_getpeername_handler(FAR struct rpmsg_endpoint *ept,
                                             FAR void *data, size_t len_,
                                             uint32_t src, FAR void *priv_)
{
  FAR struct usrsock_request_getpeername_s *req = data;
  FAR struct usrsock_message_datareq_ack_s *ack;
  FAR struct usrsock_rpmsg_s *priv = priv_;
  socklen_t outaddrlen = req->max_addrlen;
  socklen_t inaddrlen = req->max_addrlen;
  int ret = -EBADF;
  uint32_t len;

  ack = rpmsg_get_tx_payload_buffer(ept, &len, true);
  if (req->usockid >= 0 &&
      req->usockid < CONFIG_NET_USRSOCK_RPMSG_SERVER_NSOCKS)
    {
      ret = psock_getpeername(&priv->socks[req->usockid],
              (FAR struct sockaddr *)(ack + 1), &outaddrlen);
    }

  return usrsock_rpmsg_send_data_ack(ept,
          ack, 0, req->head.xid, ret, inaddrlen, outaddrlen, ret);
}

static int usrsock_rpmsg_bind_handler(FAR struct rpmsg_endpoint *ept,
                                      FAR void *data, size_t len,
                                      uint32_t src, FAR void *priv_)
{
  FAR struct usrsock_request_bind_s *req = data;
  FAR struct usrsock_rpmsg_s *priv = priv_;
  int ret = -EBADF;

  if (req->usockid >= 0 &&
      req->usockid < CONFIG_NET_USRSOCK_RPMSG_SERVER_NSOCKS)
    {
      ret = psock_bind(&priv->socks[req->usockid],
              (FAR const struct sockaddr *)(req + 1), req->addrlen);
    }

  return usrsock_rpmsg_send_ack(ept, 0, req->head.xid, ret);
}

static int usrsock_rpmsg_listen_handler(FAR struct rpmsg_endpoint *ept,
                                        FAR void *data, size_t len,
                                        uint32_t src, FAR void *priv_)
{
  FAR struct usrsock_request_listen_s *req = data;
  FAR struct usrsock_rpmsg_s *priv = priv_;
  int ret = -EBADF;
  int retr;

  if (req->usockid >= 0 &&
      req->usockid < CONFIG_NET_USRSOCK_RPMSG_SERVER_NSOCKS)
    {
      ret = psock_listen(&priv->socks[req->usockid], req->backlog);
    }

  retr = usrsock_rpmsg_send_ack(ept, 0, req->head.xid, ret);
  if (retr >= 0 && ret >= 0)
    {
      usrsock_rpmsg_poll_setup(&priv->pfds[req->usockid], POLLIN);
    }

  return retr;
}

static int usrsock_rpmsg_accept_handler(FAR struct rpmsg_endpoint *ept,
                                        FAR void *data, size_t len_,
                                        uint32_t src, FAR void *priv_)
{
  FAR struct usrsock_request_accept_s *req = data;
  FAR struct usrsock_message_datareq_ack_s *ack;
  FAR struct usrsock_rpmsg_s *priv = priv_;
  socklen_t outaddrlen = req->max_addrlen;
  socklen_t inaddrlen = req->max_addrlen;
  uint32_t len;
  int ret = -EBADF;
  int i = 0;
  int retr;

  ack = rpmsg_get_tx_payload_buffer(ept, &len, true);
  if (req->usockid >= 0 &&
      req->usockid < CONFIG_NET_USRSOCK_RPMSG_SERVER_NSOCKS)
    {
      ret = -ENFILE; /* Assume no free socket handler */
      nxrmutex_lock(&priv->mutex);
      for (i = 0; i < CONFIG_NET_USRSOCK_RPMSG_SERVER_NSOCKS; i++)
        {
          if (priv->epts[i] == NULL)
            {
              priv->epts[i] = ept;
              nxrmutex_unlock(&priv->mutex);
              ret = psock_accept(&priv->socks[req->usockid],
                      outaddrlen ? (FAR struct sockaddr *)(ack + 1) : NULL,
                      outaddrlen ? &outaddrlen : NULL, &priv->socks[i],
                      SOCK_NONBLOCK);
              if (ret >= 0)
                {
                  /* Append index as usockid to the payload */

                  if (outaddrlen <= inaddrlen)
                    {
                      *(FAR int16_t *)
                      ((FAR void *)(ack + 1) + outaddrlen) = i;
                    }
                  else
                    {
                      *(FAR int16_t *)
                      ((FAR void *)(ack + 1) + inaddrlen) = i;
                    }

                  ret = sizeof(int16_t); /* Return usockid size */
                }
              else
                {
                  priv->epts[i] = NULL;
                }

              break;
            }
        }

      if (i == CONFIG_NET_USRSOCK_RPMSG_SERVER_NSOCKS)
        {
          nxrmutex_unlock(&priv->mutex);
        }
    }

  retr = usrsock_rpmsg_send_data_ack(ept,
    ack, 0, req->head.xid, ret, inaddrlen, outaddrlen, ret);
  if (retr >= 0 && ret >= 0)
    {
      usrsock_rpmsg_poll_setup(&priv->pfds[req->usockid], POLLIN);
      usrsock_rpmsg_poll_setup(&priv->pfds[i], POLLIN);
      usrsock_rpmsg_send_event(ept, i, USRSOCK_EVENT_SENDTO_READY);
    }

  return retr;
}

static int usrsock_rpmsg_ioctl_handler(FAR struct rpmsg_endpoint *ept,
                                       FAR void *data, size_t len_,
                                       uint32_t src, FAR void *priv_)
{
  FAR struct usrsock_request_ioctl_s *req = data;
  FAR struct usrsock_message_datareq_ack_s *ack;
  FAR struct usrsock_rpmsg_s *priv = priv_;
#ifdef CONFIG_NETDEV_WIRELESS_IOCTL
  FAR struct iwreq *wlreq;
  FAR struct iwreq *wlack;
#endif
  int ret = -EBADF;
  uint32_t len;

  ack = rpmsg_get_tx_payload_buffer(ept, &len, true);
  if (req->usockid >= 0 &&
      req->usockid < CONFIG_NET_USRSOCK_RPMSG_SERVER_NSOCKS)
    {
      memcpy(ack + 1, req + 1, len_ - sizeof(*req));
#ifdef CONFIG_NETDEV_WIRELESS_IOCTL
      wlreq = (FAR struct iwreq *)(req + 1);
      wlack = (FAR struct iwreq *)(ack + 1);
      if (WL_IS80211POINTERCMD(req->cmd) && wlreq->u.data.pointer)
        {
          wlack->u.data.pointer = wlack + 1;
        }
#endif

      ret = psock_ioctl(&priv->socks[req->usockid],
              req->cmd, (unsigned long)(ack + 1));

#ifdef CONFIG_NETDEV_WIRELESS_IOCTL
      if (WL_IS80211POINTERCMD(req->cmd) && wlreq->u.data.pointer)
        {
          if (ret >= 0)
            {
              ret = wlreq->u.data.length;
            }

          wlack->u.data.pointer = wlreq->u.data.pointer;
        }
#endif
    }

  return usrsock_rpmsg_send_data_ack(ept,
           ack, 0, req->head.xid, ret, req->arglen, req->arglen, ret);
}

static int usrsock_rpmsg_shutdown_handler(FAR struct rpmsg_endpoint *ept,
                                       FAR void *data, size_t len,
                                       uint32_t src, FAR void *priv_)
{
  FAR struct usrsock_request_shutdown_s *req = data;
  FAR struct usrsock_rpmsg_s *priv = priv_;
  int ret = -EBADF;

  if (req->usockid >= 0 &&
      req->usockid < CONFIG_NET_USRSOCK_RPMSG_SERVER_NSOCKS)
    {
      ret = psock_shutdown(&priv->socks[req->usockid], req->how);
    }

  return usrsock_rpmsg_send_ack(ept, 0, req->head.xid, ret);
}

static int usrsock_rpmsg_dns_handler(FAR struct rpmsg_endpoint *ept,
                                     FAR void *data, size_t len,
                                     uint32_t src, FAR void *priv_)
{
#ifdef CONFIG_NETDB_DNSCLIENT
  FAR struct usrsock_rpmsg_dns_request_s *dns = data;

  dns_add_nameserver((FAR struct sockaddr *)(dns + 1), dns->addrlen);
#endif

  return 0;
}

#ifdef CONFIG_NETDB_DNSCLIENT
static int usrsock_rpmsg_send_dns_event(FAR void *arg,
                                        FAR struct sockaddr *addr,
                                        socklen_t addrlen)
{
  FAR struct rpmsg_endpoint *ept = arg;
  FAR struct usrsock_rpmsg_dns_event_s *dns;
  uint32_t len;

  dns = rpmsg_get_tx_payload_buffer(ept, &len, true);

  dns->head.msgid = USRSOCK_RPMSG_DNS_EVENT;
  dns->head.flags = USRSOCK_MESSAGE_FLAG_EVENT;

  dns->addrlen = addrlen;
  memcpy(dns + 1, addr, addrlen);

  return rpmsg_send_nocopy(ept, dns, sizeof(*dns) + addrlen);
}
#endif

static bool usrsock_rpmsg_ns_match(FAR struct rpmsg_device *rdev,
                                   FAR void *priv_, FAR const char *name,
                                   uint32_t dest)
{
  return !strcmp(name, USRSOCK_RPMSG_EPT_NAME);
}

static void usrsock_rpmsg_ns_bind(FAR struct rpmsg_device *rdev,
                                  FAR void *priv_, FAR const char *name,
                                  uint32_t dest)
{
  FAR struct usrsock_rpmsg_s *priv = priv_;
  FAR struct rpmsg_endpoint *ept;
  int ret;

  ept = kmm_zalloc(sizeof(struct rpmsg_endpoint));
  if (!ept)
    {
      return;
    }

  ept->priv = priv;

  ret = rpmsg_create_ept(ept, rdev, USRSOCK_RPMSG_EPT_NAME,
                         RPMSG_ADDR_ANY, dest,
                         usrsock_rpmsg_ept_cb, usrsock_rpmsg_ns_unbind);
  if (ret < 0)
    {
      kmm_free(ept);
      return;
    }

#ifdef CONFIG_NETDB_DNSCLIENT
  dns_register_notify(usrsock_rpmsg_send_dns_event, ept);
#endif
}

static void usrsock_rpmsg_ns_unbind(FAR struct rpmsg_endpoint *ept)
{
  FAR struct usrsock_rpmsg_s *priv = ept->priv;
  int i;

#ifdef CONFIG_NETDB_DNSCLIENT
  dns_unregister_notify(usrsock_rpmsg_send_dns_event, ept);
#endif

  /* Collect all socks belong to the dead client */

  for (i = 0; i < CONFIG_NET_USRSOCK_RPMSG_SERVER_NSOCKS; i++)
    {
      if (priv->epts[i] == ept)
        {
          usrsock_rpmsg_poll_setup(&priv->pfds[i], 0);

          /* It's safe to close socks here */

          psock_close(&priv->socks[i]);
          nxrmutex_lock(&priv->mutex);
          priv->epts[i] = NULL;
          nxrmutex_unlock(&priv->mutex);
        }
    }

  rpmsg_destroy_ept(ept);
}

static int usrsock_rpmsg_ept_cb(FAR struct rpmsg_endpoint *ept,
                                FAR void *data, size_t len, uint32_t src,
                                FAR void *priv_)
{
  FAR struct usrsock_request_common_s *common = data;
  FAR struct usrsock_rpmsg_s *priv = priv_;

  if (priv->iov[0].iov_base)
    {
      return usrsock_rpmsg_sendto_handler(ept, data, len, src, priv);
    }
  else if (common->reqid >= 0 && common->reqid <= USRSOCK_REQUEST__MAX)
    {
      return g_usrsock_rpmsg_handler[common->reqid](ept, data, len,
                                                    src, priv);
    }

  return -EINVAL;
}

static void usrsock_rpmsg_poll_setup(FAR struct pollfd *pfds,
                                     pollevent_t events)
{
  FAR struct usrsock_rpmsg_s *priv = (FAR struct usrsock_rpmsg_s *)pfds->arg;
  FAR struct socket *psock = &priv->socks[pfds->fd];
  int ret = 0;

  /* No poll for SOCK_CTRL. */

  if (psock->s_type == SOCK_CTRL)
    {
      return;
    }

  net_lock();

  if (events)
    {
      if (!pfds->events)
        {
          pfds->revents = 0;
          pfds->events = events;
          ret = psock_poll(psock, pfds, true);
        }
      else
        {
          pfds->events = events;
        }
    }
  else
    {
      pfds->events = 0;
      ret = psock_poll(psock, pfds, false);
    }

  if (ret < 0)
    {
      nerr("psock_poll failed. ret %d domain %u type %u pfds->fd %d"
           ", pfds->events %08" PRIx32 ", pfds->revents %08" PRIx32,
           ret, psock->s_domain, psock->s_type, pfds->fd,
           pfds->events, pfds->revents);
    }

  net_unlock();
}

static void usrsock_rpmsg_poll_cb(FAR struct pollfd *pfds)
{
  FAR struct usrsock_rpmsg_s *priv = (FAR struct usrsock_rpmsg_s *)pfds->arg;
  int events = 0;

  nxrmutex_lock(&priv->mutex);

  if (!priv->epts[pfds->fd])
    {
      nxrmutex_unlock(&priv->mutex);
      return;
    }

  if (pfds->revents & POLLIN)
    {
      events |= USRSOCK_EVENT_RECVFROM_AVAIL;

      /* Stop poll in until recv get called */

      pfds->events &= ~POLLIN;
      pfds->revents &= ~POLLIN;
    }

  if (pfds->revents & POLLOUT)
    {
      events |= USRSOCK_EVENT_SENDTO_READY;

      /* Stop poll out until send get called */

      pfds->events &= ~POLLOUT;
      pfds->revents &= ~POLLOUT;
    }

  if (pfds->revents & (POLLHUP | POLLERR))
    {
      events |= USRSOCK_EVENT_REMOTE_CLOSED;

      /* Check data that has not been recv */

      if (usrsock_rpmsg_available(&priv->socks[pfds->fd], FIONREAD))
        {
          events |= USRSOCK_EVENT_RECVFROM_AVAIL;
        }
    }

  if (!(pfds->events & (POLLIN | POLLOUT)))
    {
      usrsock_rpmsg_poll_setup(pfds, 0);
    }

  if (events != 0)
    {
      usrsock_rpmsg_send_event(priv->epts[pfds->fd], pfds->fd, events);
    }

  nxrmutex_unlock(&priv->mutex);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int usrsock_rpmsg_server_initialize(void)
{
  FAR struct usrsock_rpmsg_s *priv;
  int ret;
  int i;

  priv = kmm_calloc(1, sizeof(*priv));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  nxrmutex_init(&priv->mutex);

  /* Setup poll callback settings */

  for (i = 0; i < CONFIG_NET_USRSOCK_RPMSG_SERVER_NSOCKS; i++)
    {
      priv->pfds[i].fd  = i;
      priv->pfds[i].arg = priv;
      priv->pfds[i].cb  = usrsock_rpmsg_poll_cb;
    }

  ret = rpmsg_register_callback(priv,
                                NULL,
                                NULL,
                                usrsock_rpmsg_ns_match,
                                usrsock_rpmsg_ns_bind);
  if (ret >= 0)
    {
      return ret;
    }

  nxrmutex_destroy(&priv->mutex);
  kmm_free(priv);
  return ret;
}
