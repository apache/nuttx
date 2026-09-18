/****************************************************************************
 * net/lwip/lwipopts.h
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

#ifndef __NET_LWIP_LWIPOPTS_H
#define __NET_LWIP_LWIPOPTS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NO_SYS                          0
#define SYS_LIGHTWEIGHT_PROT            1
#define LWIP_TIMERS                     1
#define LWIP_NETIF_API                  1
#define LWIP_SOCKET                     1
#define LWIP_NETCONN                    1

/* Keep lwIP symbols separate from the native NuttX socket API. */
#define LWIP_COMPAT_SOCKETS             0
#define LWIP_POSIX_SOCKETS_IO_NAMES     0

#define MEM_ALIGNMENT                   CONFIG_NET_LWIP_MEM_ALIGNMENT
#define MEM_LIBC_MALLOC                 1
#define MEMP_MEM_MALLOC                 1
#define MEM_SIZE                        CONFIG_NET_LWIP_MEM_SIZE
#define MEMP_NUM_PBUF                   CONFIG_NET_LWIP_PBUF_NUM
#define MEMP_NUM_NETCONN                CONFIG_NET_LWIP_NETCONN_NUM
#define MEMP_NUM_TCP_PCB                CONFIG_NET_LWIP_TCP_PCB_NUM
#define MEMP_NUM_UDP_PCB                CONFIG_NET_LWIP_UDP_PCB_NUM
#define MEMP_NUM_TCP_SEG                CONFIG_NET_LWIP_TCP_SEG_NUM
#define PBUF_POOL_SIZE                  CONFIG_NET_LWIP_PBUF_POOL_SIZE
#define PBUF_POOL_BUFSIZE               CONFIG_NET_LWIP_PBUF_POOL_BUFSIZE
#define TCP_MSS                         CONFIG_NET_LWIP_TCP_MSS
#define TCP_WND                         CONFIG_NET_LWIP_TCP_WND
#define TCP_SND_BUF                     CONFIG_NET_LWIP_TCP_SND_BUF
#define TCPIP_THREAD_STACKSIZE          CONFIG_NET_LWIP_TCPIP_STACKSIZE
#define TCPIP_THREAD_PRIO               CONFIG_NET_LWIP_TCPIP_PRIORITY
#define TCPIP_MBOX_SIZE                 CONFIG_NET_LWIP_TCPIP_MBOX_SIZE
#define DEFAULT_THREAD_STACKSIZE        CONFIG_NET_LWIP_DEFAULT_STACKSIZE
#define DEFAULT_THREAD_PRIO             CONFIG_NET_LWIP_DEFAULT_PRIORITY
#define DEFAULT_RAW_RECVMBOX_SIZE       8
#define DEFAULT_UDP_RECVMBOX_SIZE       8
#define DEFAULT_TCP_RECVMBOX_SIZE       8
#define DEFAULT_ACCEPTMBOX_SIZE         8

#define LWIP_IPV4                       1
#define LWIP_ICMP                       1
#define LWIP_ARP                        1
#define LWIP_ETHERNET                   1
#define LWIP_NETIF_LINK_CALLBACK        1
#define LWIP_NETIF_STATUS_CALLBACK      1
#define LWIP_NETIF_HOSTNAME             1

#ifdef CONFIG_NET_LWIP_IPV6
#  define LWIP_IPV6                     1
#else
#  define LWIP_IPV6                     0
#endif

#ifdef CONFIG_NET_LWIP_DHCP
#define LWIP_DHCP                       1
#else
#define LWIP_DHCP                       0
#endif
#ifdef CONFIG_NET_LWIP_DNS
#define LWIP_DNS                        1
#else
#define LWIP_DNS                        0
#endif

#ifdef CONFIG_NET_LWIP_RAW
#define LWIP_RAW                        1
#else
#define LWIP_RAW                        0
#endif
#ifdef CONFIG_NET_LWIP_UDP
#define LWIP_UDP                        1
#else
#define LWIP_UDP                        0
#endif
#ifdef CONFIG_NET_LWIP_TCP
#define LWIP_TCP                        1
#else
#define LWIP_TCP                        0
#endif

#define LWIP_SO_RCVTIMEO                1
#define LWIP_SO_SNDTIMEO                1
#define LWIP_SO_REUSE                   1
#define LWIP_TCP_KEEPALIVE              1

#define LWIP_CHECKSUM_CTRL_PER_NETIF    0
#define CHECKSUM_GEN_IP                 1
#define CHECKSUM_GEN_UDP                1
#define CHECKSUM_GEN_TCP                1
#define CHECKSUM_CHECK_IP               1
#define CHECKSUM_CHECK_UDP              1
#define CHECKSUM_CHECK_TCP              1

#define LWIP_DBG_MIN_LEVEL              LWIP_DBG_LEVEL_WARNING
#define LWIP_STATS                      0
#define LWIP_PROVIDE_ERRNO              1

#endif /* __NET_LWIP_LWIPOPTS_H */
