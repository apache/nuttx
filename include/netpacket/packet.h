/****************************************************************************
 * include/netpacket/packet.h
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

#ifndef __INCLUDE_NETPACKET_PACKET_H
#define __INCLUDE_NETPACKET_PACKET_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct sockaddr_ll
{
  unsigned short sll_family;
  unsigned short sll_protocol;
  int            sll_ifindex;
  unsigned short sll_hatype;
  unsigned char  sll_pkttype;
  unsigned char  sll_halen;
  unsigned char  sll_addr[8];
};

#endif /* __INCLUDE_NETPACKET_PACKET_H */
