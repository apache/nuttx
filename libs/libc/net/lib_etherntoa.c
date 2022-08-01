/****************************************************************************
 * libs/libc/net/lib_etherntoa.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>

#include <net/ethernet.h>
#include <netinet/ether.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ether_ntoa_r
 *
 * Description:
 *   The ether_ntoa_r() function converts the Ethernet host address addr
 *   given in network byte order to a string in standard
 *   hex-digits-and-colons notation.
 *
 ****************************************************************************/

FAR char *ether_ntoa_r(FAR const struct ether_addr *addr, FAR char *buf)
{
  sprintf(buf, "%02x:%02x:%02x:%02x:%02x:%02x",
          addr->ether_addr_octet[0], addr->ether_addr_octet[1],
          addr->ether_addr_octet[2], addr->ether_addr_octet[3],
          addr->ether_addr_octet[4], addr->ether_addr_octet[5]);
  return buf;
}

/****************************************************************************
 * Name: ether_ntoa
 *
 * Description:
 *   The ether_ntoa() function converts the Ethernet host address addr given
 *   in network byte order to a string in standard hex-digits-and-colons
 *   notation. The string is returned in a statically allocated buffer, which
 *   subsequent calls will overwrite.
 *
 ****************************************************************************/

FAR char *ether_ntoa(FAR const struct ether_addr *addr)
{
  static char buffer[20];
  return ether_ntoa_r(addr, buffer);
}
