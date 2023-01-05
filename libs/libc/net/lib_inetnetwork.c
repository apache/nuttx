/****************************************************************************
 * libs/libc/net/lib_inetnetwork.c
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

#include <stdio.h>
#include <arpa/inet.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * name inet_network
 *
 * Description:
 *   The inet_network() function converts the string pointed to by cp, in the
 *   standard IPv4 dotted decimal notation, to an integer value suitable for
 *   use as an Internet address.
 *
 *   inet_aton() returns nonzero if the address is valid, zero if not.
 *   The address supplied in cp can have one of the following forms:
 *
 *   a.b.c.d Each of the four numeric parts specifies a byte of the address;
 *           the bytes are assigned in left-to-right order to produce the
 *           binary address.
 *
 *   a.b.c   Parts a and b specify the first two bytes of the binary address.
 *           Part c is interpreted as a 16-bit value that defines the
 *           rightmost two bytes of the binary address. This notation is
 *           suitable for specifying (outmoded) Class B network addresses.
 *
 *   a.b     Part a specifies the first byte of the binary address. Part b is
 *           interpreted as a 24-bit value that defines the rightmost three
 *           bytes of the binary address. This notation is suitable for
 *           specifying (outmoded) Class A network addresses.
 *
 *   a       The value a is interpreted as a 32-bit value that is stored
 *           directly into the binary address without any byte rearrangement.
 *
 * Returned Value:
 *   If input string cannot be recognized as a valid IPv4 number, function
 *   returns zero.
 *
 ****************************************************************************/

in_addr_t inet_network(FAR const char *cp)
{
  in_addr_t t = inet_addr(cp);
  return NTOHL(t);
}
