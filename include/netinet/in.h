/****************************************************************************
 * netinet/in.h
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
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

#ifndef __NETINET_IP_H
#define __NETINET_IP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>

/****************************************************************************
 * Public Macro Definitions
 ****************************************************************************/

/* Values for protocol argument to socket() */

#define IPPROTO_TCP      1
#define IPPROTO_UDP      2

/* Special values of in_addr_t */

#define INADDR_ANY       ((in_addr_t)0x00000000) /* Address to accept any incoming messages */
#define INADDR_BROADCAST ((in_addr_t)0xffffffff) /* Address to send to all hosts */
#define INADDR_NONE      ((in_addr_t)0xffffffff) /* Address indicating an error return */

#define IN6ADDR_ANY_INIT {{{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}}}

/* struct in6_addr union selectors */

#define s6_addr          in6_u.u6_addr8
#define s6_addr16        in6_u.u6_addr16
#define s6_addr32        in6_u.u6_addr32

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* IPv4 Internet address */

typedef uint32 in_addr_t;
struct in_addr
{
  in_addr_t    s_addr;        /* Address (network byte order) */
};

struct sockaddr_in
{
  sa_family_t sin_family;      /* address family: AF_INET */
  uint16      sin_port;        /* port in network byte order */
  struct in_addr sin_addr;   /* internet address */
};

/* IPv6 Internet address */

struct in6_addr
{
  union
  {
    uint8     u6_addr8[16];
    uint16    u6_addr16[8];
    uint32    u6_addr32[4];
  } in6_u;
};

struct sockaddr_in6
{
  sa_family_t sin_family;      /* Address family: AF_INET */
  uint16      sin_port;        /* Port in network byte order */
  struct in6_addr sin6_addr; /* IPv6 internet address */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __NETINET_IP_H */
