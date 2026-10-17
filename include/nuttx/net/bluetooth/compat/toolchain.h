/****************************************************************************
 * include/nuttx/net/bluetooth/compat/toolchain.h
 *
 * Copyright (c) ByteDance Ltd. and/or its affiliates.
 *
 * Author: chao an <anchao.archer@bytedance.com>
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

/* SPDX-License-Identifier: Apache-2.0 */

#ifndef __INCLUDE_NUTTX_NET_BLUETOOTH_COMPAT_TOOLCHAIN_H
#define __INCLUDE_NUTTX_NET_BLUETOOTH_COMPAT_TOOLCHAIN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/compiler.h>

#include <stdbool.h>
#include <stdint.h>

#define Z_STRINGIFY(x) #x
#define STRINGIFY(s) Z_STRINGIFY(s)

#define _DO_CONCAT(x, y) x##y
#define _CONCAT(x, y) _DO_CONCAT(x, y)

#if defined(__cplusplus) && (__cplusplus >= 201103L)
#define BUILD_ASSERT(EXPR, MSG...) static_assert(EXPR, "" MSG)
#else
#define BUILD_ASSERT(EXPR, MSG...) _Static_assert((EXPR), "" MSG)
#endif

#ifndef FLEXIBLE_ARRAY_DECLARE
#define FLEXIBLE_ARRAY_DECLARE(type, name, ...) type name[0]
#endif

#define Z_DECL_ALIGN(type) __aligned(__alignof(type)) type

#define CODE_UNREACHABLE compiler_assume(false)

#define UNALIGNED_GET(g)                                                                           \
  __extension__({                                                                                  \
    struct __packed                                                                                \
    {                                                                                              \
      __typeof__(*(g)) __v;                                                                        \
    } *__g = (__typeof__(__g))(g);                                                                 \
    __g->__v;                                                                                      \
  })
#define ___in_section(a, b, c)
#define __in_section(a, b, c) ___in_section(a, b, c)

#ifndef __fallthrough
#if __GNUC__ >= 7
#define __fallthrough __attribute__((fallthrough))
#else
#define __fallthrough
#endif
#endif

#ifndef __packed
#define __packed end_packed_struct
#endif

#ifndef __aligned
#define __aligned(x) aligned_data(x)
#endif

#ifndef __printf_like
#define __printf_like(f, a) printf_like(f, a)
#endif

#define __used used_code
#define __unused unused_code
#define __maybe_unused unused_code

#ifndef __deprecated
#define __deprecated deprecated_function
#endif

#ifndef __must_check
#define __must_check __attribute__((warn_unused_result))
#endif

#define ARG_UNUSED(x) (void)(x)
#define unlikely(x) (predict_false((bool)!!(x)) != 0L)
#define POPCOUNT(x) __builtin_popcount(x)

#ifndef __weak
#define __weak weak_function
#endif

#define __WARN(msg) __WARN1(GCC warning msg)
#define __WARN1(s) _Pragma(#s)

#ifndef __DEPRECATED_MACRO
#define __DEPRECATED_MACRO __WARN("Macro is deprecated")
#endif

#endif /* __INCLUDE_NUTTX_NET_BLUETOOTH_COMPAT_TOOLCHAIN_H */
