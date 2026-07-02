/****************************************************************************
 * include/sanitizer/asan_interface.h
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

/* AddressSanitizer public interface.
 *
 * This is the same header the host compiler-rt runtime provides so that code
 * written against user-space ASan (for example the mbedtls test framework)
 * can include <sanitizer/asan_interface.h> and call
 * __asan_(un)poison_memory_region() unchanged on a NuttX target. NuttX does
 * not link the compiler-rt user-space ASan runtime; on a KASAN target these
 * entry points are provided by mm/kasan and bridge to the in-kernel KASAN
 * shadow. The signatures match the upstream compiler-rt header.
 */

#ifndef __INCLUDE_SANITIZER_ASAN_INTERFACE_H
#define __INCLUDE_SANITIZER_ASAN_INTERFACE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/compiler.h>

#include <stddef.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: __asan_poison_memory_region
 *
 * Description:
 *   Mark the memory region [addr, addr+size) as unaddressable, so that any
 *   instrumented access to it triggers a KASAN report until unpoisoned.
 *
 ****************************************************************************/

void __asan_poison_memory_region(FAR void const volatile *addr, size_t size);

/****************************************************************************
 * Name: __asan_unpoison_memory_region
 *
 * Description:
 *   Mark the memory region [addr, addr+size) as addressable again.
 *
 ****************************************************************************/

void __asan_unpoison_memory_region(FAR void const volatile *addr,
                                   size_t size);

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_SANITIZER_ASAN_INTERFACE_H */
