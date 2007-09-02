/****************************************************************************
 * arpa/inet.h
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

#ifndef __ARPA_INET_H
#define __ARPA_INET_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <netinet/in.h>

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* This macro to convert a 16/32-bit constant values quantity from host byte
 * order to network byte order.  The 16-bit version of this macro is require
 * for uIP:
 *
 *   Author Adam Dunkels <adam@dunkels.com>
 *   Copyright (c) 2001-2003, Adam Dunkels.
 *   All rights reserved.
 */

#ifdef CONFIG_ENDIAN_BIG
# define HTONS(ns) (ns)
# define HTONL(nl) (nl)
#else
# define HTONS(ns) \
  (uint16)((((uint16) (ns)) << 8) | (((uint16) (ns)) >> 8))
# define HTONL(nl) htonl(nl) \
  ((uint32)HTONS((uint16)((hs) << 16)) | (uint32)HTONS((uint16)((hs) & 0xffff)))
#endif

#define NTOHS(hs) HTONS(hs)
#define NTOHL(hl) NTOHL(hl)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/* Functions to convert between nost and network byte ordering */

EXTERN uint32      ntohl (uint32 nl);
EXTERN uint16      ntohs (uint16 ns);
EXTERN uint32      htonl (uint32 hl);
EXTERN uint16      htons (uint16 hs);

/* Functions to manipulate address representations */

EXTERN int         inet_aton(const char *cp, struct in_addr *inp);
EXTERN in_addr_t   inet_addr(const char *cp);
EXTERN in_addr_t   inet_network(const char *cp);
EXTERN char       *inet_ntoa(struct in_addr in);
EXTERN struct in_addr inet_makeaddr(in_addr_t net, in_addr_t host);
EXTERN in_addr_t   inet_lnaof(struct in_addr in);
EXTERN in_addr_t   inet_netof(struct in_addr in);
EXTERN int         inet_pton(int af, const char *cp, void *buf);
EXTERN const char *inet_ntop(int af, const void *cp, char *buf, socklen_t len);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ARPA_INET_H */
