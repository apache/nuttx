/****************************************************************************
 * include/netinet/tcp.h
 *
 *   Copyright (C) 2017-2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __INCLUDE_NETINET_TCP_H
#define __INCLUDE_NETINET_TCP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/socket.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Per OpenGroup.org:
 *
 *   "The netinet/tcp.h header shall define the following macro for use as a
 *    socket option at the IPPROTO_TCP level:" -- OpenGroup.org
 */

#define TCP_NODELAY   (__SO_PROTOCOL + 0) /* Avoid coalescing of small segments. */

/*   "The macro shall be defined in the header.  The implementation need not
 *    allow the value of the option to be set via setsockopt() or retrieved via
 *    getsockopt()."
 */

/* Additional TCP protocol socket operations not specified at OpenGroup.org */
/* TCP protocol socket operations needed to support TCP Keep-Alive: */

#define TCP_KEEPIDLE  (__SO_PROTOCOL + 1) /* Start keeplives after this IDLE period
                                           * Argument: struct timeval */
#define TCP_KEEPINTVL (__SO_PROTOCOL + 2) /* Interval between keepalives
                                           * Argument: struct timeval */
#define TCP_KEEPCNT   (__SO_PROTOCOL + 3) /* Number of keepalives before death
                                           * Argument: max retry count */
#define TCP_MAXSEG    (__SO_PROTOCOL + 4) /* The maximum segment size */

#endif /* __INCLUDE_NETINET_TCP_H */
