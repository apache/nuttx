/****************************************************************************
 * include/nuttx/net/netlink.h
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __INCLUDE_NUTTX_NET_NETLINK_H
#define __INCLUDE_NUTTX_NET_NETLINK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <queue.h>

#include <netpacket/netlink.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This is the form of the obfuscated state structure passed to modules
 * outside of the networking layer.
 */

typedef FAR void *NETLINK_HANDLE;

/* Netlink uses a two step, request-get, referenced by a user managed
 * sequence number.  This means that the requested data must be buffered
 * until it is gotten by the client.  This structure holds that buffered
 * data.
 *
 * REVISIT:  There really should be a time stamp on this so that we can
 * someday weed out un-claimed responses.
 */

struct netlink_response_s
{
  sq_entry_t flink;
  struct nlmsghdr msg;

  /* Message-specific data may follow */
};

#define SIZEOF_NETLINK_RESPONSE_S(n) (sizeof(struct netlink_response_s) + (n))

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: netlink_add_response
 *
 * Description:
 *   Add response data at the tail of the pending response list.
 *
 *   Note:  The network will be momentarily locked to support exclusive
 *   access to the pending response list.
 *
 * Input Parameters:
 *   handle - The handle previously provided to the sendto() implementation
 *            for the protocol.  This is an opaque reference to the Netlink
 *            socket state structure.
 *   resp   - The response to the request.  The memory referenced by 'resp'
 *            must have been allocated via kmm_malloc().  It will be freed
 *            using kmm_free() after it has been consumed.
 *
 ****************************************************************************/

void netlink_add_response(NETLINK_HANDLE handle,
                          FAR struct netlink_response_s *resp);

/****************************************************************************
 * Name: netlink_add_broadcast
 *
 * Description:
 *   Add broadcast data to all interested netlink connections.
 *
 *   Note:  The network will be momentarily locked to support exclusive
 *   access to the pending response list.
 *
 * Input Parameters:
 *   group - The broadcast group index.
 *   data  - The broadcast data.  The memory referenced by 'data'
 *           must have been allocated via kmm_malloc().  It will be freed
 *           using kmm_free() after it has been consumed.
 *
 ****************************************************************************/

void netlink_add_broadcast(int group, FAR struct netlink_response_s *data);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_NET_NETLINK_H */
