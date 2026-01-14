/****************************************************************************
 * include/nuttx/net/usrsock.h
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

#ifndef __INCLUDE_NUTTX_NET_USRSOCK_H
#define __INCLUDE_NUTTX_NET_USRSOCK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <sys/uio.h>
#include <sys/param.h>

#include <nuttx/net/net.h>
#include <nuttx/net/netconfig.h>
#include <nuttx/compiler.h>
#include <nuttx/mutex.h>

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
  USRSOCK_REQUEST_SHUTDOWN,
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
  uint32_t reqid;
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
  uint16_t addrlen;
  int32_t flags;
  uint32_t buflen;
} end_packed_struct;

begin_packed_struct struct usrsock_request_recvfrom_s
{
  struct usrsock_request_common_s head;

  int16_t usockid;
  uint16_t max_addrlen;
  int32_t flags;
  uint32_t max_buflen;
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
  uint16_t arglen;
  int32_t cmd;
} end_packed_struct;

begin_packed_struct struct usrsock_request_shutdown_s
{
  struct usrsock_request_common_s head;

  int16_t usockid;
  int16_t how;
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

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Global protection lock for usrsock socket */

#ifdef CONFIG_NET_USRSOCK
extern rmutex_t g_usrsock_lock;

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usrsock_lock
 *
 * Description:
 *   Take the global usrsock socket lock
 *
 ****************************************************************************/

static inline_function void usrsock_lock(void)
{
  nxrmutex_lock(&g_usrsock_lock);
}

/****************************************************************************
 * Name: usrsock_unlock
 *
 * Description:
 *   Release the global usrsock socket lock
 *
 ****************************************************************************/

static inline_function void usrsock_unlock(void)
{
  nxrmutex_unlock(&g_usrsock_lock);
}

/****************************************************************************
 * Name: usrsock_sem_timedwait
 *
 * Description:
 *   Wait for sem while temporarily releasing the usrsock lock.
 *
 ****************************************************************************/

static inline_function int
usrsock_sem_timedwait(FAR sem_t *sem, bool interruptible,
                      unsigned int timeout)
{
  return net_sem_timedwait2(sem, interruptible, timeout, &g_usrsock_lock,
                            NULL);
}

/****************************************************************************
 * Name: usrsock_mutex_timedlock
 *
 * Description:
 *   Atomically wait for mutex (or a timeout) while temporarily releasing
 *   the lock on the usrsock.
 *
 *   Caution should be utilized.  Because the usrsock lock is relinquished
 *   during the wait, there could be changes in the usrsock state that occur
 *   before the lock is recovered.  Your design should account for this
 *   possibility.
 *
 * Input Parameters:
 *   mutex    - A reference to the mutex to be taken.
 *   timeout  - The relative time to wait until a timeout is declared.
 *   brkmutex - A reference to the mutex to be temporarily released while
 *              waiting.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

static inline_function int
usrsock_mutex_timedlock(FAR mutex_t *mutex, unsigned int timeout)
{
  unsigned int count;
  int          blresult;
  int          ret;

  /* Release the network lock, remembering my count.  net_breaklock will
   * return a negated value if the caller does not hold the network lock.
   */

  blresult = nxrmutex_breaklock(&g_usrsock_lock, &count);

  /* Now take the mutex, waiting if so requested. */

  if (timeout != UINT_MAX)
    {
      ret = nxmutex_timedlock(mutex, timeout);
    }
  else
    {
      /* Wait as long as necessary to get the lock */

      ret = nxmutex_lock(mutex);
    }

  /* Recover the network lock at the proper count (if we held it before) */

  if (blresult >= 0)
    {
      nxrmutex_restorelock(&g_usrsock_lock, count);
    }

  return ret;
}

/****************************************************************************
 * Name: usrsock_iovec_get() - copy from iovec to buffer.
 ****************************************************************************/

ssize_t usrsock_iovec_get(FAR void *dst, size_t dstlen,
                          FAR const struct iovec *iov, int iovcnt,
                          size_t pos, FAR bool *done);

/****************************************************************************
 * Name: usrsock_iovec_put() - copy to iovec from buffer.
 ****************************************************************************/

ssize_t usrsock_iovec_put(FAR struct iovec *iov, int iovcnt, size_t pos,
                          FAR const void *src, size_t srclen);

/****************************************************************************
 * Name: usrsock_abort() - abort all usrsock's operations
 ****************************************************************************/

void usrsock_abort(void);

/****************************************************************************
 * Name: usrsock_response() - handle usrsock request's ack/response
 ****************************************************************************/

ssize_t usrsock_response(FAR const char *buffer, size_t len,
                         FAR bool *req_done);

/****************************************************************************
 * Name: usrsock_request() - finish usrsock's request
 ****************************************************************************/

int usrsock_request(FAR struct iovec *iov, unsigned int iovcnt);

/****************************************************************************
 * Name: usrsock_register
 *
 * Description:
 *   Register /dev/usrsock
 *
 ****************************************************************************/

void usrsock_register(void);
#endif /* CONFIG_NET_USRSOCK */

#endif /* __INCLUDE_NUTTX_NET_USRSOCK_H */
