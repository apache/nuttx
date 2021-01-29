/****************************************************************************
 * include/nuttx/net/netlink.h
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
