/****************************************************************************
 * net/netlink/netlink.h
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

#ifndef __NET_NETLINK_NETLINK_H
#define __NET_NETLINK_NETLINK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <poll.h>

#include <netpacket/netlink.h>
#include <nuttx/queue.h>
#include <nuttx/net/netlink.h>
#include <nuttx/semaphore.h>
#include <nuttx/wqueue.h>

#include "devif/devif.h"
#include "socket/socket.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_NETLINK_ROUTE
#  define netlink_device_notify(dev)
#  define netlink_device_notify_ipaddr(dev, type, domain)
#endif

#ifdef CONFIG_NET_NETLINK

/**
 * nla_for_each_attr - iterate over a stream of attributes
 * @pos: loop counter, set to current attribute
 * @head: head of attribute stream
 * @len: length of attribute stream
 * @rem: initialized to len, holds bytes currently remaining in stream
 */

#define nla_for_each_attr(pos, head, len, rem)  \
  for (pos = head, rem = len; nla_ok(pos, rem); \
       pos = nla_next(pos, &(rem)))

/* Always use this macro, this allows later putting the
 * message into a separate section or such for things
 * like translation or listing all possible messages.
 * Currently string formatting is not supported (due
 * to the lack of an output buffer.)
 */

#define nl_set_err_msg_attr(extack, attr, msg)         \
  do                                                   \
    {                                                  \
      static const char __msg[] = (msg);               \
      FAR struct netlink_ext_ack *__extack = (extack); \
      if (__extack)                                    \
        {                                              \
          __extack->_msg = __msg;                      \
          __extack->bad_attr = (attr);                 \
        }                                              \
    }                                                  \
  while (0)

/**
 * nla_data - head of payload
 * @nla: netlink attribute
 */

#define nla_data(nla) ((FAR void *)((FAR char *)(nla) + NLA_HDRLEN))

/**
 * nla_len - length of payload
 * @nla: netlink attribute
 */

#define nla_len(nla) ((nla)->nla_len - NLA_HDRLEN)

/**
 * nla_type - attribute type
 * @nla: netlink attribute
 */

#define nla_type(nla) ((nla)->nla_type & NLA_TYPE_MASK)

/**
 * nla_ok - check if the netlink attribute fits into the remaining bytes
 * @nla: netlink attribute
 * @remaining: number of bytes remaining in attribute stream
 */

#define nla_ok(nla, remaining)        \
  ((remaining) >= sizeof(*(nla)) &&   \
  (nla)->nla_len >= sizeof(*(nla)) && \
  (nla)->nla_len <= (remaining))

/**
 * nlmsg_msg_size - length of netlink message not including padding
 * @payload: length of message payload
 */

#define nlmsg_msg_size(payload) (NLMSG_HDRLEN + (payload))

/**
 * nlmsg_len - length of message payload
 * @nlh: netlink message header
 */

#define nlmsg_len(nlh) ((nlh)->nlmsg_len - NLMSG_HDRLEN)

/**
 * nlmsg_attrlen - length of attributes data
 * @nlh: netlink message header
 * @hdrlen: length of family specific header
 */

#define nlmsg_attrlen(nlh, hdrlen) (nlmsg_len(nlh) - NLMSG_ALIGN(hdrlen))

/**
 * nlmsg_data - head of message payload
 * @nlh: netlink message header
 */

#define nlmsg_data(nlh) ((FAR void *)((FAR char *)(nlh) + NLMSG_HDRLEN))

/**
 * nla_get_in_addr - return payload of IPv4 address attribute
 * @nla: IPv4 address netlink attribute
 */

#define nla_get_in_addr(nla) (*(FAR uint32_t *)nla_data(nla))

/**
 * nlmsg_attrdata - head of attributes data
 * @nlh: netlink message header
 * @hdrlen: length of family specific header
 */

#define nlmsg_attrdata(nlh, hdrlen) \
  ((FAR struct nlattr *)((FAR char *)nlmsg_data(nlh) + NLMSG_ALIGN(hdrlen)))

/**
 * nlmsg_parse - parse attributes of a netlink message
 * @nlh: netlink message header
 * @hdrlen: length of family specific header
 * @tb: destination array with maxtype+1 elements
 * @maxtype: maximum attribute type to be expected
 * @policy: validation policy
 * @extack: extended ACK report struct
 *
 * See nla_parse()
 */

#define nlmsg_parse(nlh, hdrlen, tb, maxtype, policy, extack)            \
  ((nlh)->nlmsg_len < nlmsg_msg_size(hdrlen) ? -EINVAL :                 \
                     nla_parse(tb, maxtype, nlmsg_attrdata(nlh, hdrlen), \
                               nlmsg_attrlen(nlh, hdrlen), policy, extack))

/* this can be increased when necessary - don't expose to userland */

#define NETLINK_MAX_COOKIE_LEN  20

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* This connection structure describes the underlying state of the socket. */

struct netlink_conn_s
{
  /* Common prologue of all connection structures. */

  struct socket_conn_s sconn;

  /* NetLink-specific content follows */

  uint32_t pid;                      /* Port ID (if bound) */
  uint32_t groups;                   /* Multicast groups mask (if bound) */
  uint32_t dst_pid;                  /* Destination port ID */
  uint32_t dst_groups;               /* Destination multicast groups mask */
  uint8_t crefs;                     /* Reference counts on this instance */

  /* poll() support */

  int key;                           /* used to cancel notifications */
  FAR struct pollfd *fds;            /* Used to wakeup poll() */

  /* Queued response data */

  sq_queue_t resplist;               /* Singly linked list of responses */
};

/**
 * Standard attribute types to specify validation policy
 */

enum
{
  NLA_UNSPEC,
  NLA_U8,
  NLA_U16,
  NLA_U32,
  NLA_U64,
  NLA_STRING,
  NLA_FLAG,
  NLA_MSECS,
  NLA_NESTED,
  NLA_NESTED_COMPAT,
  NLA_NUL_STRING,
  NLA_BINARY,
  NLA_S8,
  NLA_S16,
  NLA_S32,
  NLA_S64,
  NLA_BITFIELD32,
  NLA_TYPE_MAX = NLA_BITFIELD32,
};

/**
 * struct netlink_ext_ack - netlink extended ACK report struct
 * @_msg: message string to report - don't access directly, use
 *  %nl_set_err_msg_attr
 * @bad_attr: attribute with error
 * @cookie: cookie data to return to userspace (for success)
 * @cookie_len: actual cookie data length
 */

struct netlink_ext_ack
{
  FAR const char *_msg;
  FAR const struct nlattr *bad_attr;
  uint8_t cookie[NETLINK_MAX_COOKIE_LEN];
  uint8_t cookie_len;
};

/**
 * struct nla_policy - attribute validation policy
 * @type: Type of attribute or NLA_UNSPEC
 * @len: Type specific length of payload
 *
 * Policies are defined as arrays of this struct, the array must be
 * accessible by attribute type up to the highest identifier to be expected.
 *
 * Meaning of `len' field:
 *    NLA_STRING           Maximum length of string
 *    NLA_NUL_STRING       Maximum length of string (excluding NUL)
 *    NLA_FLAG             Unused
 *    NLA_BINARY           Maximum length of attribute payload
 *    NLA_NESTED           Don't use `len' field -- length verification is
 *                         done by checking len of nested header (or empty)
 *    NLA_NESTED_COMPAT    Minimum length of structure payload
 *    NLA_U8, NLA_U16,
 *    NLA_U32, NLA_U64,
 *    NLA_S8, NLA_S16,
 *    NLA_S32, NLA_S64,
 *    NLA_MSECS            Leaving the length field zero will verify the
 *                         given type fits, using it verifies minimum length
 *                         just like "All other"
 *    NLA_BITFIELD32      A 32-bit bitmap/bitselector attribute
 *    All other            Minimum length of attribute payload
 *
 * Example:
 * static const struct nla_policy my_policy[ATTR_MAX + 1] = {
 *  [ATTR_FOO] = { .type = NLA_U16 },
 *  [ATTR_BAR] = { .type = NLA_STRING, .len = BARSIZ },
 *  [ATTR_BAZ] = { .len = sizeof(struct mystruct) },
 *  [ATTR_GOO] = { .type = NLA_BITFIELD32, .validation_data =
 *                                                          &myvalidflags },
 * };
 */

struct nla_policy
{
  uint16_t type;
  uint16_t len;
  FAR void *validation_data;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#  define EXTERN extern "C"
extern "C"
{
#else
#  define EXTERN extern
#endif

EXTERN const struct sock_intf_s g_netlink_sockif;

/****************************************************************************
 * Name: netlink_initialize()
 *
 * Description:
 *   Initialize the NetLink connection structures.  Called once and only
 *   from the networking layer.
 *
 ****************************************************************************/

void netlink_initialize(void);

/****************************************************************************
 * Name: netlink_alloc()
 *
 * Description:
 *   Allocate a new, uninitialized NetLink connection structure.  This is
 *   normally something done by the implementation of the socket() API
 *
 ****************************************************************************/

FAR struct netlink_conn_s *netlink_alloc(void);

/****************************************************************************
 * Name: netlink_free()
 *
 * Description:
 *   Free a NetLink connection structure that is no longer in use. This
 *   should be done by the implementation of close().
 *
 ****************************************************************************/

void netlink_free(FAR struct netlink_conn_s *conn);

/****************************************************************************
 * Name: netlink_nextconn()
 *
 * Description:
 *   Traverse the list of allocated NetLink connections
 *
 * Assumptions:
 *   This function is called from NetLink device logic.
 *
 ****************************************************************************/

FAR struct netlink_conn_s *netlink_nextconn(FAR struct netlink_conn_s *conn);

/****************************************************************************
 * Name: netlink_notifier_setup
 *
 * Description:
 *   Set up to perform a callback to the worker function the Netlink
 *   response data is received.  The worker function will execute on the low
 *   priority worker thread.
 *
 * Input Parameters:
 *   worker - The worker function to execute on the low priority work
 *            queue when Netlink response data is available.
 *   conn   - The Netlink connection where the response is expected.
 *   arg    - A user-defined argument that will be available to the worker
 *            function when it runs.
 *
 * Returned Value:
 *   Zero (OK) is returned if the notification was successfully set up.
 *   A negated error value is returned if an unexpected error occurred
 *   and no notification will occur.
 *
 ****************************************************************************/

int netlink_notifier_setup(worker_t worker, FAR struct netlink_conn_s *conn,
                           FAR void *arg);

/****************************************************************************
 * Name: netlink_notifier_teardown
 *
 * Description:
 *   Eliminate a Netlink response notification previously setup by
 *   netlink_notifier_setup().  This function should only be called if the
 *   notification should be aborted prior to the notification.  The
 *   notification will automatically be torn down after the notification.
 *
 * Input Parameters:
 *   conn - Teardown the notification for this Netlink connection.
 *
 ****************************************************************************/

void netlink_notifier_teardown(FAR struct netlink_conn_s *conn);

/****************************************************************************
 * Name: netlink_notifier_signal
 *
 * Description:
 *   New Netlink response data is available.  Execute worker thread
 *   functions for all threads that wait for response data.
 *
 * Input Parameters:
 *   conn - The Netlink connection where the response was just buffered.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void netlink_notifier_signal(FAR struct netlink_conn_s *conn);

/****************************************************************************
 * Name: netlink_tryget_response
 *
 * Description:
 *   Return the next response from the head of the pending response list.
 *   Responses are returned one-at-a-time in FIFO order.
 *
 *   Note:  The network will be momentarily locked to support exclusive
 *   access to the pending response list.
 *
 * Returned Value:
 *   The next response from the head of the pending response list is
 *   returned.  NULL will be returned if the pending response list is
 *   empty
 *
 ****************************************************************************/

FAR struct netlink_response_s *
netlink_tryget_response(FAR struct netlink_conn_s *conn);

/****************************************************************************
 * Name: netlink_get_response
 *
 * Description:
 *   Return the next response from the head of the pending response list.
 *   Responses are returned one-at-a-time in FIFO order.
 *
 *   Note:  The network will be momentarily locked to support exclusive
 *   access to the pending response list.
 *
 * Returned Value:
 *   The next response from the head of the pending response list is
 *   returned.  This function will block until a response is received if
 *   the pending response list is empty.  NULL will be returned only in the
 *   event of a failure.
 *
 ****************************************************************************/

FAR struct netlink_response_s *
netlink_get_response(FAR struct netlink_conn_s *conn);

/****************************************************************************
 * Name: netlink_check_response
 *
 * Description:
 *   Return true is a response is pending now.
 *
 * Returned Value:
 *   True: A response is available; False; No response is available.
 *
 ****************************************************************************/

bool netlink_check_response(FAR struct netlink_conn_s *conn);

/****************************************************************************
 * Name: netlink_route_sendto()
 *
 * Description:
 *   Perform the sendto() operation for the NETLINK_ROUTE protocol.
 *
 ****************************************************************************/

#ifdef CONFIG_NETLINK_ROUTE
ssize_t netlink_route_sendto(NETLINK_HANDLE handle,
                             FAR const struct nlmsghdr *nlmsg,
                             size_t len, int flags,
                             FAR const struct sockaddr_nl *to,
                             socklen_t tolen);

/****************************************************************************
 * Name: netlink_device_notify()
 *
 * Description:
 *   Perform the route broadcast for the NETLINK_ROUTE protocol.
 *
 ****************************************************************************/

void netlink_device_notify(FAR struct net_driver_s *dev);

/****************************************************************************
 * Name: netlink_device_notify_ipaddr()
 *
 * Description:
 *   Perform the route broadcast for the NETLINK_ROUTE protocol.
 *
 ****************************************************************************/

void netlink_device_notify_ipaddr(FAR struct net_driver_s *dev,
                                  int type, int domain);

/****************************************************************************
 * Name: nla_next
 *
 * Description:
 *   Next netlink attribute in attribute stream.
 *
 * Input Parameters:
 *   nla - netlink attribute.
 *   remaining - number of bytes remaining in attribute stream.
 *
 * Returned Value:
 *   Returns the next netlink attribute in the attribute stream and
 *   decrements remaining by the size of the current attribute.
 *
 ****************************************************************************/

FAR struct nlattr *nla_next(FAR const struct nlattr *nla,
                            FAR int *remaining);

/****************************************************************************
 * Name: nla_parse
 *
 * Description:
 *   Parse the nested netlink attribute.
 *
 ****************************************************************************/

int nla_parse(FAR struct nlattr **tb, int maxtype,
              FAR const struct nlattr *head,
              int len, FAR const struct nla_policy *policy,
              FAR struct netlink_ext_ack *extack);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_NET_NETLINK */
#endif /* __NET_NETLINK_NETLINK_H */
