/****************************************************************************
 * include/netinet/ip.h
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

#ifndef __INCLUDE_NETINET_IP_H
#define __INCLUDE_NETINET_IP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/socket.h>
#include <netinet/in.h>

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

#define IPVERSION 4     /* IP version number */
#define IPDEFTTL  64    /* default ttl, from RFC 1340 */

struct iphdr
{
#if __BYTE_ORDER == __LITTLE_ENDIAN
  unsigned int ihl:4;
  unsigned int version:4;
#elif __BYTE_ORDER == __BIG_ENDIAN
  unsigned int version:4;
  unsigned int ihl:4;
#else
# error "unknown endian"
#endif
  uint8_t tos;
  uint16_t tot_len;
  uint16_t id;
  uint16_t frag_off;
  uint8_t ttl;
  uint8_t protocol;
  uint16_t check;
  uint32_t saddr;
  uint32_t daddr;

  /* The options start here. */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __INCLUDE_NETINET_IP_H */
