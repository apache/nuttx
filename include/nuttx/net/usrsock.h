/****************************************************************************
 * include/nuttx/net/usrsock.h
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

#ifndef __INCLUDE_NUTTX_NET_USRSOCK_H
#define __INCLUDE_NUTTX_NET_USRSOCK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/net/netconfig.h>
#include <nuttx/compiler.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Event message flags */

#define USRSOCK_EVENT_ABORT          (1 << 1)
#define USRSOCK_EVENT_SENDTO_READY   (1 << 2)
#define USRSOCK_EVENT_RECVFROM_AVAIL (1 << 3)
#define USRSOCK_EVENT_REMOTE_CLOSED  (1 << 4)

/* Response message flags */

#define USRSOCK_MESSAGE_FLAG_REQ_IN_PROGRESS (1 << 0)
#define USRSOCK_MESSAGE_FLAG_EVENT           (1 << 1)

#define USRSOCK_MESSAGE_IS_EVENT(flags) \
                          (!!((flags) & USRSOCK_MESSAGE_FLAG_EVENT))
#define USRSOCK_MESSAGE_IS_REQ_RESPONSE(flags) \
                          (!USRSOCK_MESSAGE_IS_EVENT(flags))

#define USRSOCK_MESSAGE_REQ_IN_PROGRESS(flags) \
                          (!!((flags) & USRSOCK_MESSAGE_FLAG_REQ_IN_PROGRESS))
#define USRSOCK_MESSAGE_REQ_COMPLETED(flags) \
                          (!USRSOCK_MESSAGE_REQ_IN_PROGRESS(flags))

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Request types */

enum usrsock_request_types_e
{
  USRSOCK_REQUEST_SOCKET = 0,
  USRSOCK_REQUEST_CLOSE,
  USRSOCK_REQUEST_CONNECT,
  USRSOCK_REQUEST_SENDTO,
  USRSOCK_REQUEST_RECVFROM,
  USRSOCK_REQUEST_SETSOCKOPT,
  USRSOCK_REQUEST_GETSOCKOPT,
  USRSOCK_REQUEST_GETSOCKNAME,
  USRSOCK_REQUEST_GETPEERNAME,
  USRSOCK_REQUEST_BIND,
  USRSOCK_REQUEST_LISTEN,
  USRSOCK_REQUEST_ACCEPT,
  USRSOCK_REQUEST_IOCTL,
  USRSOCK_REQUEST__MAX
};

/* Response/event message types */

enum usrsock_message_types_e
{
  USRSOCK_MESSAGE_RESPONSE_ACK = 0,
  USRSOCK_MESSAGE_RESPONSE_DATA_ACK,
  USRSOCK_MESSAGE_SOCKET_EVENT,
};

/* Request structures (kernel => /dev/usrsock => daemon) */

begin_packed_struct struct usrsock_request_common_s
{
  uint32_t xid;
  int8_t   reqid;
  int8_t   reserved;
} end_packed_struct;

begin_packed_struct struct usrsock_request_socket_s
{
  struct usrsock_request_common_s head;

  int16_t domain;
  int16_t type;
  int16_t protocol;
} end_packed_struct;

begin_packed_struct struct usrsock_request_close_s
{
  struct usrsock_request_common_s head;

  int16_t usockid;
} end_packed_struct;

begin_packed_struct struct usrsock_request_bind_s
{
  struct usrsock_request_common_s head;

  int16_t usockid;
  uint16_t addrlen;
} end_packed_struct;

begin_packed_struct struct usrsock_request_connect_s
{
  struct usrsock_request_common_s head;

  int16_t usockid;
  uint16_t addrlen;
} end_packed_struct;

begin_packed_struct struct usrsock_request_listen_s
{
  struct usrsock_request_common_s head;

  int16_t usockid;
  uint16_t backlog;
} end_packed_struct;

begin_packed_struct struct usrsock_request_accept_s
{
  struct usrsock_request_common_s head;

  int16_t usockid;
  uint16_t max_addrlen;
} end_packed_struct;

begin_packed_struct struct usrsock_request_sendto_s
{
  struct usrsock_request_common_s head;

  int16_t usockid;
  int32_t flags;
  uint32_t buflen;
  uint16_t addrlen;
} end_packed_struct;

begin_packed_struct struct usrsock_request_recvfrom_s
{
  struct usrsock_request_common_s head;

  int16_t usockid;
  int32_t flags;
  uint32_t max_buflen;
  uint16_t max_addrlen;
} end_packed_struct;

begin_packed_struct struct usrsock_request_setsockopt_s
{
  struct usrsock_request_common_s head;

  int16_t usockid;
  int16_t level;
  int16_t option;
  uint16_t valuelen;
} end_packed_struct;

begin_packed_struct struct usrsock_request_getsockopt_s
{
  struct usrsock_request_common_s head;

  int16_t usockid;
  int16_t level;
  int16_t option;
  uint16_t max_valuelen;
} end_packed_struct;

begin_packed_struct struct usrsock_request_getsockname_s
{
  struct usrsock_request_common_s head;

  int16_t usockid;
  uint16_t max_addrlen;
} end_packed_struct;

begin_packed_struct struct usrsock_request_getpeername_s
{
  struct usrsock_request_common_s head;

  int16_t usockid;
  uint16_t max_addrlen;
} end_packed_struct;

begin_packed_struct struct usrsock_request_ioctl_s
{
  struct usrsock_request_common_s head;

  int16_t usockid;
  int32_t cmd;
  uint16_t arglen;
} end_packed_struct;

/* Response/event message structures (kernel <= /dev/usrsock <= daemon) */

begin_packed_struct struct usrsock_message_common_s
{
  int8_t msgid;
  int8_t flags;
  uint16_t events;
} end_packed_struct;

/* Request acknowledgment/completion message */

begin_packed_struct struct usrsock_message_req_ack_s
{
  struct usrsock_message_common_s head;

  int32_t  result;
  uint32_t xid;
} end_packed_struct;

/* Request acknowledgment/completion message */

begin_packed_struct struct usrsock_message_datareq_ack_s
{
  struct usrsock_message_req_ack_s reqack;

  /* head.result => positive buflen, negative error-code. */

  uint16_t valuelen;          /* length of value returned after buffer */
  uint16_t valuelen_nontrunc; /* actual non-truncated length of value at
                               * daemon-sïde. */
} end_packed_struct;

/* Socket event message */

begin_packed_struct struct usrsock_message_socket_event_s
{
  struct usrsock_message_common_s head;

  int16_t usockid;
} end_packed_struct;

#endif /* __INCLUDE_NUTTX_NET_USRSOCK_H */
