/****************************************************************************
 * include/netinet/in_systm.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __INCLUDE_NETINET_IN_SYSTM_H
#define __INCLUDE_NETINET_IN_SYSTM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* The types of 4.4BSD <netinet/in_systm.h> for the fields of the IP
 * headers, in network byte order.  glibc, musl and the BSDs have them.
 */

typedef uint16_t n_short;  /* A short as received from the network */
typedef uint32_t n_long;   /* A long as received from the network */
typedef uint32_t n_time;   /* Milliseconds since 00:00 UTC, byte order of
                            * the network */

#endif /* __INCLUDE_NETINET_IN_SYSTM_H */
