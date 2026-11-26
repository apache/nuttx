/****************************************************************************
 * arch/tricore/include/arch.h
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

/* This file should never be included directly but, rather,
 * only indirectly through nuttx/arch.h
 */

#ifndef __ARCH_TRICORE_INCLUDE_ARCH_H
#define __ARCH_TRICORE_INCLUDE_ARCH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <stddef.h>
#endif

#include <arch/barriers.h>
#include <arch/csfr.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Address <--> Context Save Areas */

#define tricore_csa2addr(csa) ((uintptr_t *)((((csa) & 0x000F0000) << 12) \
                                             | (((csa) & 0x0000FFFF) << 6)))
#define tricore_addr2csa(addr) ((uintptr_t)(((((uintptr_t)(addr)) & 0xF0000000) >> 12) \
                                            | (((uintptr_t)(addr) & 0x003FFFC0) >> 6)))

/* Read and write global address registers */

#ifdef CONFIG_TRICORE_TOOLCHAIN_TASKING
#  define read_sysreg(reg)                                                 \
    ({                                                                     \
      uint32_t __val;                                                      \
      __asm__ __volatile__("mov.d\t %0, "#reg"" : "=d" (__val)            \
                           :: "memory");                                   \
      __val;                                                               \
    })

#  define write_sysreg(__val, reg)                                         \
    ({                                                                     \
      __asm__ __volatile__("mov.a\t "#reg", %0" :: "d" (__val)             \
                           : "memory");                                    \
    })
#else
#  define read_sysreg(reg)                                                 \
    ({                                                                     \
      uint32_t __val;                                                      \
      __asm__ __volatile__("mov.d\t %0, %%"#reg"" : "=d" (__val)          \
                           :: "memory");                                   \
      __val;                                                               \
    })

#  define write_sysreg(__val, reg)                                         \
    ({                                                                     \
      __asm__ __volatile__("mov.a\t %%"#reg", %0" :: "d" (__val)          \
                           : "memory");                                    \
    })
#endif

/* MTCR and MFCR - move to / from Core Special Function Register */

#define tricore_mtcr(reg, val)                                             \
  ({                                                                       \
    __asm__ __volatile__ ("mtcr %0,%1\n\t"::"i"(reg),"d"(val):"memory");   \
  })

#define tricore_mfcr(reg)                                                  \
  ({                                                                       \
    uint32_t __val;                                                        \
    __asm__ __volatile__ ("mfcr %0,%1": "=d" (__val) :"i"(reg): "memory"); \
    __val;                                                                 \
  })

/* Return from a handler that the trap or interrupt vector entered with
 * jli, to the vector code at A11.
 */

#ifdef CONFIG_TRICORE_TOOLCHAIN_TASKING
#  define tricore_jump_backto_link() \
     __asm__ __volatile__ ("ji a11" : : : "a11")
#else
#  define tricore_jump_backto_link() \
     __asm__ __volatile__ ("ji %a11")
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef __cplusplus
}
#endif

#endif /* __ARCH_TRICORE_INCLUDE_ARCH_H */
