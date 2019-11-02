/****************************************************************************
 * include/nuttx/net/rpmsg.h
 *
 *   Copyright (C) 2018 Pinecone Inc. All rights reserved.
 *   Author: Jianli Dong <dongjianli@pinecone.net>
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

#ifndef __INCLUDE_NUTTX_NET_RPMSG_H
#define __INCLUDE_NUTTX_NET_RPMSG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NET_RPMSG_EPT_NAME              "rpmsg-%s"

#define NET_RPMSG_IFUP                  0 /* IP-->LINK */
#define NET_RPMSG_IFDOWN                1 /* IP-->LINK */
#define NET_RPMSG_ADDMCAST              2 /* IP-->LINK */
#define NET_RPMSG_RMMCAST               3 /* IP-->LINK */
#define NET_RPMSG_DEVIOCTL              4 /* IP-->LINK */
#define NET_RPMSG_SOCKIOCTL             5 /* IP<--LINK */
#define NET_RPMSG_TRANSFER              6 /* IP<->LINK */

/****************************************************************************
 * Public Types
 ****************************************************************************/

begin_packed_struct struct net_rpmsg_lnkaddr_s
{
  uint32_t length;
  uint8_t  addr[16];
} end_packed_struct;

begin_packed_struct struct net_rpmsg_header_s
{
  uint32_t command;
  int32_t  result;
  uint64_t cookie;
} end_packed_struct;

begin_packed_struct struct net_rpmsg_ifup_s
{
  struct net_rpmsg_header_s  header;
  struct net_rpmsg_lnkaddr_s lnkaddr;

  /* All addresses in the network order */

  uint32_t                   ipaddr;
  uint32_t                   draddr;
  uint32_t                   netmask;
  uint32_t                   dnsaddr;
  uint16_t                   ipv6addr[8];
  uint16_t                   ipv6draddr[8];
  uint16_t                   ipv6netmask[8];
  uint16_t                   ipv6dnsaddr[8];
} end_packed_struct;

#define net_rpmsg_ifdown_s net_rpmsg_header_s

begin_packed_struct struct net_rpmsg_mcast_s
{
  struct net_rpmsg_header_s  header;
  struct net_rpmsg_lnkaddr_s lnkaddr;
} end_packed_struct;

begin_packed_struct struct net_rpmsg_ioctl_s
{
  struct net_rpmsg_header_s header;
  uint32_t                  code;
  uint32_t                  length;
  uint8_t                   arg[0];
} end_packed_struct;

begin_packed_struct struct net_rpmsg_transfer_s
{
  struct net_rpmsg_header_s header;
  uint32_t                  length;
  uint8_t                   data[0];
} end_packed_struct;

#endif /* __INCLUDE_NUTTX_NET_RPMSG_H */
