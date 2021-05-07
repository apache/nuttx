/****************************************************************************
 * include/netinet/tcp.h
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
 *    allow the value of the option to be set via setsockopt() or retrieved
 *    via getsockopt()."
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
