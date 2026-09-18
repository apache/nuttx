/****************************************************************************
 * net/lwip/arch/cc.h
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

#ifndef __NET_LWIP_ARCH_CC_H
#define __NET_LWIP_ARCH_CC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/uio.h>
#include <sys/time.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* NuttX already provides these POSIX socket-related base types. */

#define SA_FAMILY_T_DEFINED 1
#define LWIP_TIMEVAL_PRIVATE 0

/* lwip/sockets.h uses defined(iovec) to guard struct iovec. */

#define iovec iovec

#define LWIP_NO_STDINT_H 0
#define LWIP_RAND() ((u32_t)rand())

#define U16_F PRIu16
#define S16_F PRId16
#define X16_F PRIx16
#define U32_F PRIu32
#define S32_F PRId32
#define X32_F PRIx32
#define SZT_F "zu"

#ifndef BYTE_ORDER
#  ifdef CONFIG_ENDIAN_BIG
#    define BYTE_ORDER BIG_ENDIAN
#  else
#    define BYTE_ORDER LITTLE_ENDIAN
#  endif
#endif

#define PACK_STRUCT_FIELD(x) x
#define PACK_STRUCT_STRUCT __attribute__((packed))
#define PACK_STRUCT_BEGIN
#define PACK_STRUCT_END

#define LWIP_PLATFORM_ASSERT(msg) \
  do \
    { \
      printf("lwIP assert: %s\n", msg); \
      abort(); \
    } \
  while (0)

#define LWIP_PLATFORM_DIAG(x) \
  do \
    { \
      printf x; \
    } \
  while (0)

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef uint8_t   u8_t;
typedef int8_t    s8_t;
typedef uint16_t  u16_t;
typedef int16_t   s16_t;
typedef uint32_t  u32_t;
typedef int32_t   s32_t;
typedef uintptr_t mem_ptr_t;

#endif /* __NET_LWIP_ARCH_CC_H */
