/****************************************************************************
 * arch/risc-v/src/opensbi/nuttx_sbi_types.h
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

#ifndef __ARCH_RISCV_SRC_OPENSBI_NUTTX_SBI_TYPES_H
#define __ARCH_RISCV_SRC_OPENSBI_NUTTX_SBI_TYPES_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <nuttx/nuttx.h>

#include <stdint.h>
#include <stdbool.h>

#include "riscv_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PRILX               PRIxREG

#define __packed            end_packed_struct
#define __noreturn          noreturn_function
#define __aligned(x)        aligned_data(x)

#define likely(x)           __builtin_expect((x), 1)
#define unlikely(x)         __builtin_expect((x), 0)

#define array_size(x)       (sizeof(x) / sizeof((x)[0]))

#define MAX(a, b)           ((a) > (b) ? (a) : (b))
#define MIN(a, b)           ((a) < (b) ? (a) : (b))
#define CLAMP(a, lo, hi)    MIN(MAX(a, lo), hi)

#define ROUNDUP(a, b)       ((((a) - 1) / (b) + 1) * (b))
#define ROUNDDOWN(a, b)     ((a) / (b) * (b))

/****************************************************************************
 * Type Definitions
 ****************************************************************************/

typedef int8_t              s8;
typedef uint8_t             u8;

typedef int16_t             s16;
typedef uint16_t            u16;

typedef int32_t             s32;
typedef uint32_t            u32;

typedef int64_t             s64;
typedef uint64_t            u64;

typedef uintptr_t           virtual_addr_t;
typedef uintptr_t           virtual_size_t;
typedef uintptr_t           physical_addr_t;
typedef uintptr_t           physical_size_t;

#endif /* __ARCH_RISCV_SRC_OPENSBI_NUTTX_SBI_TYPES_H */
