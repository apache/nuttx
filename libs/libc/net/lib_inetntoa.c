/****************************************************************************
 * libs/libc/net/lib_inetntoa.c
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

#include <arpa/inet.h>
#include <netinet/in.h>

#if defined(CONFIG_NET_IPv4) || defined(CONFIG_LIBC_IPv4_ADDRCONV)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: inet_ntoa_r
 *
 * Description:
 *   The inet_ntoa_r() function converts the Internet host address given in
 *   network byte order to a string in standard numbers-and-dots notation.
 *
 ****************************************************************************/

FAR char *inet_ntoa_r(struct in_addr in, FAR char *buf, size_t bufflen)
{
  FAR unsigned char *ptr = (FAR unsigned char *)&in.s_addr;
  snprintf(buf, bufflen, "%u.%u.%u.%u",
           ptr[0], ptr[1], ptr[2], ptr[3]);
  return buf;
}

/****************************************************************************
 * Name: inet_ntoa
 *
 * Description:
 *   The inet_ntoa() function converts the Internet host address given in
 *   network byte order to a string in standard numbers-and-dots notation.
 *   The string is returned in a statically allocated buffer, which
 *   subsequent calls will overwrite.
 *
 ****************************************************************************/

FAR char *inet_ntoa(struct in_addr in)
{
  static char buffer[INET_ADDRSTRLEN];
  return inet_ntoa_r(in, buffer, sizeof(buffer));
}

#endif /* CONFIG_NET_IPv4 || CONFIG_LIBC_IPv4_ADDRCONV */
