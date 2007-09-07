/****************************************************************************
 * net_internal.h
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

#ifndef __NET_INTERNAL_H
#define __NET_INTERNAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#ifdef CONFIG_NET

#include <nuttx/net.h>

#include "net_internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* These define bit positions for each socket option (see sys/socket.h) */

#define _SO_DEBUG        (1 << SO_DEBUG)
#define _SO_ACCEPTCONN   (1 << SO_ACCEPTCONN)
#define _SO_BROADCAST    (1 << SO_BROADCAST)
#define _SO_REUSEADDR    (1 << SO_REUSEADDR)
#define _SO_KEEPALIVE    (1 << SO_KEEPALIVE)
#define _SO_LINGER       (1 << SO_LINGER)
#define _SO_OOBINLINE    (1 << SO_OOBINLINE)
#define _SO_SNDBUF       (1 << SO_SNDBUF)
#define _SO_RCVBUF       (1 << SO_RCVBUF)
#define _SO_ERROR        (1 << SO_ERROR)
#define _SO_TYPE         (1 << SO_TYPE)
#define _SO_DONTROUTE    (1 << SO_DONTROUTE)
#define _SO_RCVLOWAT     (1 << SO_RCVLOWAT)
#define _SO_RCVTIMEO     (1 << SO_RCVTIMEO)
#define _SO_SNDLOWAT     (1 << SO_SNDLOWAT)
#define _SO_SNDTIMEO     (1 << SO_SNDTIMEO)

/* This idenfies the options that have been implemented.  Someday this
 * should be 0xffff
 */

#define _SO_IMPLEMENTED  0x0000

/* The set of all valid options is a subset of those that are implemented
 * and those that can be supported within the kernel OS configuration.
 */

#ifdef CONFIG_DISABLE_CLOCK
# define _SO_ALLOPTIONS  (_SO_IMPLEMENTED & ~(_SO_RCVTIMEO|_SO_SNDTIMEO)
#else
# define _SO_ALLOPTIONS  (_SO_IMPLEMENTED)
#endif

/* This is the set of options valid for getsockopt and setsockopt */

#define _SO_GETONLY      (_SO_ACCEPTCONN|_SO_ERROR|_SO_TYPE)
#define _SO_SETOPTS      (_SO_ALLOPTIONS & ~_SO_GETONLY)
#define _SO_GETOTPS      _SO_ALLOPTIONS

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Pulblic Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/* net_sockets.c *************************************************************/

EXTERN void weak_function net_initialize(void);
EXTERN int  sockfd_allocate(void);
EXTERN void sockfd_release(int sockfd);
EXTERN FAR struct socket *sockfd_socket(int sockfd);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* CONFIG_NET */
#endif /* __NET_INTERNAL_H */
