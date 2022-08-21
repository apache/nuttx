/****************************************************************************
 * arch/sim/src/sim/up_usrsock.c
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

#include <fcntl.h>
#include <errno.h>
#include <poll.h>
#include <syslog.h>
#include <string.h>

#include <nuttx/fs/fs.h>
#include <nuttx/net/usrsock.h>

#include "up_usrsock_host.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SIM_USRSOCK_BUFSIZE (400 * 1024)

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

struct usrsock_s
{
  struct file usock;
  uint8_t     in [SIM_USRSOCK_BUFSIZE];
  uint8_t     out[SIM_USRSOCK_BUFSIZE];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

typedef int (*usrsock_handler_t)(struct usrsock_s *usrsock,
                                 const void *data, size_t len);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct usrsock_s g_usrsock;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int usrsock_send(struct usrsock_s *usrsock,
                        const void *buf, size_t len)
{
  uint8_t *data = (uint8_t *)buf;
  ssize_t ret;

  while (len > 0)
    {
      ret = file_write(&usrsock->usock, data, len);
      if (ret < 0)
        {
          return ret;
        }

      data += ret;
      len  -= ret;
    }

  return 0;
}

static int usrsock_send_ack(struct usrsock_s *usrsock,
                            uint32_t xid, int32_t result)
{
  struct usrsock_message_req_ack_s ack;

  ack.head.msgid = USRSOCK_MESSAGE_RESPONSE_ACK;
  ack.head.flags = (result == -EINPROGRESS);
  ack.head.events = 0;

  ack.xid    = xid;
  ack.result = result;

  return usrsock_send(usrsock, &ack, sizeof(ack));
}

static int usrsock_send_dack(struct usrsock_s *usrsock,
                             struct usrsock_message_datareq_ack_s *ack,
                             uint32_t xid, int32_t result,
                             uint16_t valuelen,
                             uint16_t valuelen_nontrunc)
{
  ack->reqack.head.msgid = USRSOCK_MESSAGE_RESPONSE_DATA_ACK;
  ack->reqack.head.flags = 0;
  ack->reqack.head.events = 0;

  ack->reqack.xid    = xid;
  ack->reqack.result = result;

  if (result < 0)
    {
      result             = 0;
      valuelen           = 0;
      valuelen_nontrunc  = 0;
    }
  else if (valuelen > valuelen_nontrunc)
    {
      valuelen           = valuelen_nontrunc;
    }

  ack->valuelen          = valuelen;
  ack->valuelen_nontrunc = valuelen_nontrunc;

  return usrsock_send(usrsock, ack, sizeof(*ack) + valuelen + result);
}

static int usrsock_send_event(struct usrsock_s *usrsock,
                              int16_t usockid, uint16_t events)
{
  struct usrsock_message_socket_event_s event;

  event.head.msgid  = USRSOCK_MESSAGE_SOCKET_EVENT;
  event.head.flags  = USRSOCK_MESSAGE_FLAG_EVENT;
  event.head.events = events;

  event.usockid = usockid;

  return usrsock_send(usrsock, &event, sizeof(event));
}

static int usrsock_socket_handler(struct usrsock_s *usrsock,
                                  const void *data, size_t len)
{
  const struct usrsock_request_socket_s *req = data;
  int fd = usrsock_host_socket(req->domain, req->type, req->protocol);
  int ret = usrsock_send_ack(usrsock, req->head.xid, fd);

  if (ret >= 0 && fd >= 0)
    {
      ret = usrsock_send_event(usrsock, fd, USRSOCK_EVENT_SENDTO_READY);
    }

  return ret;
}

static int usrsock_close_handler(struct usrsock_s *usrsock,
                                 const void *data, size_t len)
{
  const struct usrsock_request_close_s *req = data;
  int ret = usrsock_host_close(req->usockid);

  return usrsock_send_ack(usrsock, req->head.xid, ret);
}

static int usrsock_connect_handler(struct usrsock_s *usrsock,
                                   const void *data, size_t len)
{
  const struct usrsock_request_connect_s *req = data;
  int ret = usrsock_host_connect(req->usockid,
                                 (const struct sockaddr *)(req + 1),
                                 req->addrlen);

  return usrsock_send_ack(usrsock, req->head.xid, ret);
}

static int usrsock_sendto_handler(struct usrsock_s *usrsock,
                                  const void *data, size_t len)
{
  const struct usrsock_request_sendto_s *req = data;
  int ret = usrsock_host_sendto(req->usockid,
                                (const void *)(req + 1) + req->addrlen,
                                req->buflen, req->flags,
                                req->addrlen ?
                                (const struct sockaddr *)(req + 1) :
                                NULL, req->addrlen);
  bool sent = (ret > 0);

  ret = usrsock_send_ack(usrsock, req->head.xid, ret);
  if (ret >= 0 && sent)
    {
      ret = usrsock_send_event(usrsock, req->usockid,
                               USRSOCK_EVENT_SENDTO_READY);
    }

  return ret;
}

static int usrsock_recvfrom_handler(struct usrsock_s *usrsock,
                                    const void *data, size_t len)
{
  const struct usrsock_request_recvfrom_s *req = data;
  struct usrsock_message_datareq_ack_s *ack;
  socklen_t outaddrlen = req->max_addrlen;
  socklen_t inaddrlen = req->max_addrlen;
  size_t buflen = req->max_buflen;
  int ret;

  ack = (struct usrsock_message_datareq_ack_s *)usrsock->out;
  if (sizeof(*ack) + inaddrlen + buflen > sizeof(usrsock->out))
    {
      buflen = sizeof(usrsock->out) - sizeof(*ack) - inaddrlen;
    }

  ret = usrsock_host_recvfrom(req->usockid,
                              (void *)(ack + 1) + inaddrlen,
                              buflen, req->flags,
                              outaddrlen ?
                              (struct sockaddr *)(ack + 1) : NULL,
                              outaddrlen ? &outaddrlen : NULL);
  if (ret > 0 && outaddrlen < inaddrlen)
    {
      memcpy((void *)(ack + 1) + outaddrlen,
             (void *)(ack + 1) + inaddrlen, ret);
    }

  return usrsock_send_dack(usrsock, ack, req->head.xid,
                           ret, inaddrlen, outaddrlen);
}

static int usrsock_setsockopt_handler(struct usrsock_s *usrsock,
                                      const void *data, size_t len)
{
  const struct usrsock_request_setsockopt_s *req = data;
  int ret = usrsock_host_setsockopt(req->usockid, req->level,
                                    req->option, req + 1, req->valuelen);

  return usrsock_send_ack(usrsock, req->head.xid, ret);
}

static int usrsock_getsockopt_handler(struct usrsock_s *usrsock,
                                      const void *data, size_t len)
{
  const struct usrsock_request_getsockopt_s *req = data;
  struct usrsock_message_datareq_ack_s *ack;
  socklen_t optlen = req->max_valuelen;
  int ret;

  ack = (struct usrsock_message_datareq_ack_s *)usrsock->out;
  ret = usrsock_host_getsockopt(req->usockid,
                                req->level, req->option,
                                ack + 1, &optlen);

  return usrsock_send_dack(usrsock, ack, req->head.xid,
                           ret, optlen, optlen);
}

static int usrsock_getsockname_handler(struct usrsock_s *usrsock,
                                       const void *data, size_t len)
{
  const struct usrsock_request_getsockname_s *req = data;
  struct usrsock_message_datareq_ack_s *ack;
  socklen_t outaddrlen = req->max_addrlen;
  socklen_t inaddrlen = req->max_addrlen;
  int ret;

  ack = (struct usrsock_message_datareq_ack_s *)usrsock->out;
  ret = usrsock_host_getsockname(req->usockid,
          (struct sockaddr *)(ack + 1), &outaddrlen);

  return usrsock_send_dack(usrsock, ack, req->head.xid,
                           ret, inaddrlen, outaddrlen);
}

static int usrsock_getpeername_handler(struct usrsock_s *usrsock,
                                       const void *data, size_t len)
{
  const struct usrsock_request_getpeername_s *req = data;
  struct usrsock_message_datareq_ack_s *ack;
  socklen_t outaddrlen = req->max_addrlen;
  socklen_t inaddrlen = req->max_addrlen;
  int ret;

  ack = (struct usrsock_message_datareq_ack_s *)usrsock->out;
  ret = usrsock_host_getpeername(req->usockid,
          (struct sockaddr *)(ack + 1), &outaddrlen);

  return usrsock_send_dack(usrsock, ack, req->head.xid,
                           ret, inaddrlen, outaddrlen);
}

static int usrsock_bind_handler(struct usrsock_s *usrsock,
                                const void *data, size_t len)
{
  const struct usrsock_request_bind_s *req = data;
  int ret = usrsock_host_bind(req->usockid,
                              (const struct sockaddr *)(req + 1),
                              req->addrlen);

  return usrsock_send_ack(usrsock, req->head.xid, ret);
}

static int usrsock_listen_handler(struct usrsock_s *usrsock,
                                  const void *data, size_t len)
{
  const struct usrsock_request_listen_s *req = data;
  int ret = usrsock_host_listen(req->usockid, req->backlog);

  return usrsock_send_ack(usrsock, req->head.xid, ret);
}

static int usrsock_accept_handler(struct usrsock_s *usrsock,
                                  const void *data, size_t len)
{
  const struct usrsock_request_accept_s *req = data;
  struct usrsock_message_datareq_ack_s *ack;
  socklen_t outaddrlen = req->max_addrlen;
  socklen_t inaddrlen = req->max_addrlen;
  int sockfd;
  int ret;

  ack = (struct usrsock_message_datareq_ack_s *)usrsock->out;
  sockfd = usrsock_host_accept(req->usockid,
                               outaddrlen ?
                               (struct sockaddr *)(ack + 1) : NULL,
                               outaddrlen ? &outaddrlen : NULL);
  if (sockfd >= 0)
    {
      /* Append index as usockid to the payload */

      if (outaddrlen <= inaddrlen)
        {
          *(int16_t *)((void *)(ack + 1) + outaddrlen) = sockfd;
        }
      else
        {
          *(int16_t *)((void *)(ack + 1) + inaddrlen) = sockfd;
        }

      ret = sizeof(int16_t); /* Return usockid size */
    }
  else
    {
      ret = sockfd;
    }

  ret = usrsock_send_dack(usrsock, ack, req->head.xid, ret,
                          inaddrlen, outaddrlen);
  if (ret >= 0 && sockfd >= 0)
    {
      ret = usrsock_send_event(usrsock, sockfd, USRSOCK_EVENT_SENDTO_READY);
    }

  return ret;
}

static int usrsock_ioctl_handler(struct usrsock_s *usrsock,
                                 const void *data, size_t len)
{
  const struct usrsock_request_ioctl_s *req = data;
  struct usrsock_message_datareq_ack_s *ack;
  int ret;

  ack = (struct usrsock_message_datareq_ack_s *)usrsock->out;
  memcpy(ack + 1, req + 1, req->arglen);
  ret = usrsock_host_ioctl(req->usockid, req->cmd,
                           (unsigned long)(ack + 1));

  return usrsock_send_dack(usrsock, ack, req->head.xid, ret,
                           req->arglen, req->arglen);
}

static const usrsock_handler_t g_usrsock_handler[] =
{
  [USRSOCK_REQUEST_SOCKET]      = usrsock_socket_handler,
  [USRSOCK_REQUEST_CLOSE]       = usrsock_close_handler,
  [USRSOCK_REQUEST_CONNECT]     = usrsock_connect_handler,
  [USRSOCK_REQUEST_SENDTO]      = usrsock_sendto_handler,
  [USRSOCK_REQUEST_RECVFROM]    = usrsock_recvfrom_handler,
  [USRSOCK_REQUEST_SETSOCKOPT]  = usrsock_setsockopt_handler,
  [USRSOCK_REQUEST_GETSOCKOPT]  = usrsock_getsockopt_handler,
  [USRSOCK_REQUEST_GETSOCKNAME] = usrsock_getsockname_handler,
  [USRSOCK_REQUEST_GETPEERNAME] = usrsock_getpeername_handler,
  [USRSOCK_REQUEST_BIND]        = usrsock_bind_handler,
  [USRSOCK_REQUEST_LISTEN]      = usrsock_listen_handler,
  [USRSOCK_REQUEST_ACCEPT]      = usrsock_accept_handler,
  [USRSOCK_REQUEST_IOCTL]       = usrsock_ioctl_handler,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int usrsock_event_callback(int16_t usockid, uint16_t events)
{
  return usrsock_send_event(&g_usrsock, usockid, events);
}

int usrsock_init(void)
{
  return file_open(&g_usrsock.usock, "/dev/usrsock", O_RDWR);
}

void usrsock_loop(void)
{
  struct usrsock_request_common_s *common;
  int ret;
  struct pollfd pfd =
    {
      .ptr    = &g_usrsock.usock,
      .events = POLLIN | POLLFILE,
    };

  ret = poll(&pfd, 1, 0);
  if (ret > 0)
    {
      ret = file_read(&g_usrsock.usock, g_usrsock.in, sizeof(g_usrsock.in));
      if (ret > 0)
        {
          common = (struct usrsock_request_common_s *)g_usrsock.in;

          if (common->reqid >= 0 &&
              common->reqid < USRSOCK_REQUEST__MAX)
            {
              ret = g_usrsock_handler[common->reqid](&g_usrsock,
                                                     g_usrsock.in, ret);
              if (ret < 0)
                {
                  syslog(LOG_ERR, "Usrsock request %d failed: %d\n",
                                  common->reqid, ret);
                }
            }
          else
            {
              syslog(LOG_ERR, "Invalid request id: %d\n",
                              common->reqid);
            }
        }
    }

  usrsock_host_loop();
}
