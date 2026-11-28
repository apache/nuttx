/****************************************************************************
 * arch/risc-v/src/gd32vw55x/gdwifi/config/lwipopts.h
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

/* lwipopts.h - NuttX shim over the GD32VW55x SDK lwIP options.
 *
 * This directory precedes the SDK port dir on the include path, so this
 * wrapper is found first; it chains to the SDK's real lwipopts.h and then
 * reconciles clashes with the NuttX libc headers.
 */

#ifndef _GDWIFI_LWIPOPTS_SHIM_H_
#define _GDWIFI_LWIPOPTS_SHIM_H_

#include_next "lwipopts.h"

/* NuttX <limits.h> defines IOV_MAX as INT_MAX which trips the lwIP
 * sockets.h sanity check; clamp it to what lwIP supports.
 */

#include <limits.h>
#undef IOV_MAX
#define IOV_MAX 0xffff

/* NuttX <sys/types.h> (pulled in by the libc headers) already defines
 * socklen_t, sa_family_t, struct iovec and struct timeval.  Tell lwIP
 * not to redefine those; all remaining socket types (sockaddr etc.) are
 * lwIP-owned in this build - NuttX <sys/socket.h> must NOT be included
 * by SDK translation units.
 */

#include <sys/types.h>
#include <sys/time.h>
#include <sys/uio.h>

#define SOCKLEN_T_DEFINED   1
#define iovec               iovec

/* Keep lwIP from typedefing sa_family_t (NuttX already has it); the
 * 1-byte field layout the prebuilt libs expect is restored by the
 * lwip_sa_family_t patch in the SDK clone's sockets.h.
 */

#define SA_FAMILY_T_DEFINED 1

#undef  LWIP_TIMEVAL_PRIVATE
#define LWIP_TIMEVAL_PRIVATE 0

#endif /* _GDWIFI_LWIPOPTS_SHIM_H_ */
