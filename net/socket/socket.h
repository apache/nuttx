/****************************************************************************
 * net/socket/socket.h
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

#ifndef __NET_SOCKET_SOCKET_H
#define __NET_SOCKET_SOCKET_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#ifdef CONFIG_NET

#include <sys/types.h>
#include <stdint.h>
#include <time.h>

#include <nuttx/clock.h>
#include <nuttx/net/net.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* This macro converts a socket option value into a bit setting */

#define _SO_BIT(o)       (1 << (o))

/* These define bit positions for each socket option (see sys/socket.h) */

#define _SO_ACCEPTCONN   _SO_BIT(SO_ACCEPTCONN)
#define _SO_BROADCAST    _SO_BIT(SO_BROADCAST)
#define _SO_DEBUG        _SO_BIT(SO_DEBUG)
#define _SO_DONTROUTE    _SO_BIT(SO_DONTROUTE)
#define _SO_ERROR        _SO_BIT(SO_ERROR)
#define _SO_KEEPALIVE    _SO_BIT(SO_KEEPALIVE)
#define _SO_LINGER       _SO_BIT(SO_LINGER)
#define _SO_OOBINLINE    _SO_BIT(SO_OOBINLINE)
#define _SO_RCVBUF       _SO_BIT(SO_RCVBUF)
#define _SO_RCVLOWAT     _SO_BIT(SO_RCVLOWAT)
#define _SO_RCVTIMEO     _SO_BIT(SO_RCVTIMEO)
#define _SO_REUSEADDR    _SO_BIT(SO_REUSEADDR)
#define _SO_SNDBUF       _SO_BIT(SO_SNDBUF)
#define _SO_SNDLOWAT     _SO_BIT(SO_SNDLOWAT)
#define _SO_SNDTIMEO     _SO_BIT(SO_SNDTIMEO)
#define _SO_TYPE         _SO_BIT(SO_TYPE)
#define _SO_TIMESTAMP    _SO_BIT(SO_TIMESTAMP)
#define _SO_BINDTODEVICE _SO_BIT(SO_BINDTODEVICE)

/* This is the largest option value.  REVISIT: belongs in sys/socket.h */

#define _SO_MAXOPT       (18)

/* Macros to set, test, clear options */

#define _SO_SETOPT(s,o)  ((s) |= _SO_BIT(o))
#define _SO_CLROPT(s,o)  ((s) &= ~_SO_BIT(o))
#define _SO_GETOPT(s,o)  (((s) & _SO_BIT(o)) != 0)

/* These are macros that can be used to determine if socket option code is
 * valid (in range) and supported by API.
 */

#define _SO_GETONLYSET   (_SO_ACCEPTCONN|_SO_ERROR|_SO_TYPE)
#define _SO_GETONLY(o)   ((_SO_BIT(o) & _SO_GETONLYSET) != 0)
#define _SO_GETVALID(o)  (((unsigned int)(o)) <= _SO_MAXOPT)
#define _SO_SETVALID(o)  ((((unsigned int)(o)) <= _SO_MAXOPT) && !_SO_GETONLY(o))

/* Macros to convert timeout value */

#ifdef CONFIG_NET_SOCKOPTS
#  define _SO_TIMEOUT(t) ((t) ? (t) * MSEC_PER_DSEC : UINT_MAX)
#else
#  define _SO_TIMEOUT(t) (UINT_MAX)
#endif /* CONFIG_NET_SOCKOPTS */

/* Macro to set socket errors */

#ifdef CONFIG_NET_SOCKOPTS
#  define _SO_CONN_SETERRNO(c,e) \
    do \
      { \
        if ((c) != NULL) \
          { \
            FAR struct socket_conn_s *_conn = \
              (FAR struct socket_conn_s *)(c); \
            _conn->s_error = (int16_t)e; \
          } \
      } \
    while (0)

#  define _SO_SETERRNO(s,e) \
    do \
      { \
        if (s != NULL) \
          { \
            _SO_CONN_SETERRNO((s)->s_conn, e); \
          } \
      } \
    while (0)
#else
#  define _SO_CONN_SETERRNO(c,e)
#  define _SO_SETERRNO(s,e)
#endif /* CONFIG_NET_SOCKOPTS */

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: net_sockif
 *
 * Description:
 *   Return the socket interface associated with this address family.
 *
 * Input Parameters:
 *   family   - Socket address family
 *   type     - Socket type
 *   protocol - Socket protocol
 *
 * Returned Value:
 *   On success, a non-NULL instance of struct sock_intf_s is returned.  NULL
 *   is returned only if the address family is not supported.
 *
 ****************************************************************************/

FAR const struct sock_intf_s *
net_sockif(sa_family_t family, int type, int protocol);

/****************************************************************************
 * Name: net_timeo
 *
 * Description:
 *   Check if a timeout has elapsed.  This can be called from a socket poll
 *   function to determine if a timeout has occurred.
 *
 * Input Parameters:
 *   start_time Timeout start time in system clock ticks
 *   timeout    Timeout value in deciseconds.
 *
 * Returned Value:
 *   0 (FALSE) if not timeout; 1 (TRUE) if timeout
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NET_SOCKOPTS
int net_timeo(clock_t start_time, socktimeo_t timeo);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* CONFIG_NET */
#endif /* __NET_SOCKET_SOCKET_H */
