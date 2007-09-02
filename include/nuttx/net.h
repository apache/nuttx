/****************************************************************************
 * nuttx/net.h
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

#ifndef __NUTTX_NET_H
#define __NUTTX_NET_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#ifdef CONFIG_NET

#include <semaphore.h>

#include <net/uip/uip.h>
#include <net/uip/psock.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Socket descriptors are the index into the TCB sockets list, offset by the
 * following amount. This offset is used to distinquish file descriptors from
 * socket descriptors
 */

#ifdef CONFIG_NFILE_DESCRIPTORS
# define __SOCKFD_OFFSET CONFIG_NFILE_DESCRIPTORS
#else
# define __SOCKFD_OFFSET 0
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This is the internal representation of a socket reference by a file
 * descriptor.
 */

struct socket
{
  int   s_crefs;          /* Reference count on the socket */
  uint8 s_type;           /* Protocol type: Only SOCK_STREAM or SOCK_DGRAM */
  void *s_conn;           /* Connection: struct uip_conn * or uip_udp_conn * */
};

/* This defines a list of sockets indexed by the socket descriptor */

#if CONFIG_NSOCKET_DESCRIPTORS > 0
struct socketlist
{
  sem_t  sl_sem;          /* Manage access to the socket list */
  sint16 sl_crefs;        /* Reference count */
  struct socket sl_sockets[CONFIG_NSOCKET_DESCRIPTORS];
};
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/* net_sockets.c *************************************************************/

EXTERN FAR struct socketlist *net_alloclist(void);
EXTERN int net_addreflist(FAR struct socketlist *list);
EXTERN int net_releaselist(FAR struct socketlist *list);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_NET */
#endif /* __NUTTX_NET_H */
