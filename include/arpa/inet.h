/****************************************************************************
 * include/arpa/inet.h
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

#ifndef __INCLUDE_ARPA_INET_H
#define __INCLUDE_ARPA_INET_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <netinet/in.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/* Functions to manipulate address representations */

int         inet_aton(FAR const char *cp, FAR struct in_addr *inp);
in_addr_t   inet_addr(FAR const char *cp);
in_addr_t   inet_network(FAR const char *cp);

FAR char   *inet_ntoa(struct in_addr in);
FAR char   *inet_ntoa_r(struct in_addr in, FAR char *buf, size_t bufflen);
in_addr_t   inet_lnaof(struct in_addr in);
in_addr_t   inet_netof(struct in_addr in);

struct in_addr inet_makeaddr(in_addr_t net, in_addr_t host);

int         inet_pton(int af, FAR const char *src, FAR void *dst);
const char *inet_ntop(int af, FAR const void *src, FAR char *dst,
                      socklen_t size);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_ARPA_INET_H */
