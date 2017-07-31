/****************************************************************************
 * include/nuttx/net/usrsock.h
 *
 *  Copyright (C) 2015, 2017 Haltian Ltd. All rights reserved.
 *  Author: Jussi Kivilinna <jussi.kivilinna@haltian.com>
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
  USRSOCK_REQUEST_BIND,
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
  int8_t reqid;
  uint8_t xid;
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

begin_packed_struct struct usrsock_request_sendto_s
{
  struct usrsock_request_common_s head;

  int16_t usockid;
  uint16_t addrlen;
  uint16_t buflen;
} end_packed_struct;

begin_packed_struct struct usrsock_request_recvfrom_s
{
  struct usrsock_request_common_s head;

  int16_t usockid;
  uint16_t max_buflen;
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

/* Response/event message structures (kernel <= /dev/usrsock <= daemon) */

begin_packed_struct struct usrsock_message_common_s
{
  int8_t msgid;
  int8_t flags;
} end_packed_struct;

/* Request acknowledgment/completion message */

begin_packed_struct struct usrsock_message_req_ack_s
{
  struct usrsock_message_common_s head;

  uint8_t xid;
  int32_t result;
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
  uint16_t events;
} end_packed_struct;

#endif /* __INCLUDE_NUTTX_NET_USRSOCK_H */
