/****************************************************************************
 * boards/arm64/fvp-v8r/fvp-armv8r/include/board_memorymap.h
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

#ifndef __BOARDS_ARM64_FVP_V8R_FVP_ARMV8R_INCLUDE_BOARD_MEMORYMAP_H
#define __BOARDS_ARM64_FVP_V8R_FVP_ARMV8R_INCLUDE_BOARD_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

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

#ifdef CONFIG_BUILD_PROTECTED
/* Split SRAM between kernel and user spaces */

#  define KTEXT_SIZE  (CONFIG_RAM_SIZE / 4)
#  define KSRAM_SIZE  (CONFIG_RAM_SIZE / 4)
#  define UTEXT_SIZE  (CONFIG_RAM_SIZE / 4)
#  define USRAM_SIZE  (CONFIG_RAM_SIZE / 4)
#else
/* Give All RAM to kernel */

#  define KTEXT_SIZE  (CONFIG_RAM_SIZE / 2)
#  define KSRAM_SIZE  (CONFIG_RAM_SIZE / 2)
#  define UTEXT_SIZE  0
#  define USRAM_SIZE  0
#endif

/* Kernel code memory (RX) */

#define KTEXT_START   CONFIG_RAM_START
#define KTEXT_END     (KTEXT_START + KTEXT_SIZE)

/* Kernel RAM (RW) */

#define KSRAM_START   KTEXT_END
#define KSRAM_END     (KSRAM_START + KSRAM_SIZE)

/* User code memory (RX) */

#define UTEXT_START   KSRAM_END
#define UTEXT_END     (UTEXT_START + UTEXT_SIZE)

/* User RAM (RW) */

#define USRAM_START   UTEXT_END
#define USRAM_END     (USRAM_START + USRAM_SIZE)

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM64_VDK_ARMV8R_BASE_INCLUDE_BOARD_MEMORYMAP_H */
