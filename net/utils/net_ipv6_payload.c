/****************************************************************************
 * net/utils/net_ipv6_payload.c
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

#include <nuttx/net/ip.h>
#include <nuttx/net/ipv6ext.h>

#ifdef CONFIG_NET_IPv6

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: net_ipv6_payload
 *
 * Description:
 *   Given a pointer to the IPv6 header, this function will return a pointer
 *   to the beginning of the L4 payload.
 *
 * Input Parameters:
 *   ipv6  - A pointer to the IPv6 header.
 *   proto - The location to return the protocol number in the IPv6 header.
 *
 * Returned Value:
 *   A pointer to the beginning of the payload.
 *
 ****************************************************************************/

FAR void *net_ipv6_payload(FAR struct ipv6_hdr_s *ipv6, FAR uint8_t *proto)
{
  FAR struct ipv6_extension_s *exthdr;
  FAR uint8_t *payload = (FAR uint8_t *)ipv6 + IPv6_HDRLEN;
  uint8_t nxthdr = ipv6->proto;
  uint16_t extlen;

  while (ipv6_exthdr(nxthdr))
    {
      /* Just skip over the extension header */

      exthdr = (FAR struct ipv6_extension_s *)payload;
      extlen = EXTHDR_LEN(exthdr->len);

      payload += extlen;
      nxthdr   = exthdr->nxthdr;
    }

  *proto = nxthdr;
  return payload;
}

#endif /* CONFIG_NET_IPv6 */
