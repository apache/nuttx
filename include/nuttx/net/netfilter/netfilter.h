/****************************************************************************
 * include/nuttx/net/netfilter/netfilter.h
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

#ifndef __INCLUDE_NUTTX_NET_NETFILTER_NETFILTER_H
#define __INCLUDE_NUTTX_NET_NETFILTER_NETFILTER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Responses from hook functions. */

#define NF_DROP   0
#define NF_ACCEPT 1
#define NF_STOLEN 2
#define NF_QUEUE  3
#define NF_REPEAT 4
#define NF_STOP   5 /* Deprecated, for userspace nf_queue compatibility. */
#define NF_MAX_VERDICT NF_STOP

/* IP Hooks */

#define NF_IP_PRE_ROUTING  0 /* After promisc drops, checksum checks. */
#define NF_IP_LOCAL_IN     1 /* If the packet is destined for this box. */
#define NF_IP_FORWARD      2 /* If the packet is destined for another interface. */
#define NF_IP_LOCAL_OUT    3 /* Packets coming from a local process. */
#define NF_IP_POST_ROUTING 4 /* Packets about to hit the wire. */
#define NF_IP_NUMHOOKS     5

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum nf_inet_hooks
{
  NF_INET_PRE_ROUTING,
  NF_INET_LOCAL_IN,
  NF_INET_FORWARD,
  NF_INET_LOCAL_OUT,
  NF_INET_POST_ROUTING,
  NF_INET_NUMHOOKS,
  NF_INET_INGRESS = NF_INET_NUMHOOKS,
};

#endif /* __INCLUDE_NUTTX_NET_NETFILTER_NETFILTER_H */
