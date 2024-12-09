/****************************************************************************
 * arch/risc-v/include/arch.h
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

#ifndef __ARCH_RISCV_INCLUDE_ARCH_H
#define __ARCH_RISCV_INCLUDE_ARCH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <stddef.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Provide the maximum amount of page table levels per MMU type */

#ifdef CONFIG_ARCH_MMU_TYPE_SV39
#  define ARCH_PGT_MAX_LEVELS (3)
#elif CONFIG_ARCH_MMU_TYPE_SV32
#  define ARCH_PGT_MAX_LEVELS (2)
#endif

/* Amount of static page tables allocated for an address environment */

#ifdef CONFIG_ARCH_ADDRENV
#  define ARCH_SPGTS          (ARCH_PGT_MAX_LEVELS - 1)
#endif

#ifndef __ASSEMBLY__

/* Read the value of a CSR register */

#define READ_CSR(reg) \
  ({ \
     uintreg_t __regval; \
     __asm__ __volatile__("csrr %0, " __STR(reg) : "=r"(__regval)); \
     __regval; \
  })

/* Read the value of a CSR register and set the specified bits */

#define READ_AND_SET_CSR(reg, bits) \
  ({ \
     uintreg_t __regval; \
     __asm__ __volatile__("csrrs %0, " __STR(reg) ", %1": "=r"(__regval) : "rK"(bits)); \
     __regval; \
  })

/* Write a value to a CSR register */

#define WRITE_CSR(reg, val) \
  ({ \
     __asm__ __volatile__("csrw " __STR(reg) ", %0" :: "rK"(val)); \
  })

/* Set the specified bits in a CSR register */

#define SET_CSR(reg, bits) \
  ({ \
     __asm__ __volatile__("csrs " __STR(reg) ", %0" :: "rK"(bits)); \
  })

/* Clear the specified bits in a CSR register */

#define CLEAR_CSR(reg, bits) \
  ({ \
     __asm__ __volatile__("csrc " __STR(reg) ", %0" :: "rK"(bits)); \
  })

/* Swap the value of a CSR register with the specified value */

#define SWAP_CSR(reg, val) \
  ({ \
     uintptr_t regval; \
     __asm__ __volatile__("csrrw %0, " __STR(reg) ", %1" : "=r"(regval) \
                                                         : "rK"(val)); \
     regval; \
  })

/* Write a value to an indirect CSR register */

#define WRITE_INDIRECT_CSR_REG0(reg, val) \
  ({ \
     WRITE_CSR(CSR_ISELECT, reg); \
     WRITE_CSR(CSR_IREG, val); \
  })

/* Read the value of an indirect CSR register */

#define READ_INDIRECT_CSR_REG0(reg, val) \
  ({ \
     WRITE_CSR(CSR_ISELECT, reg); \
     READ_CSR(CSR_IREG, val); \
  })

/* Set the specified bits in an indirect CSR register */

#define SET_INDIRECT_CSR_REG0(reg, val) \
  ({ \
     WRITE_CSR(CSR_ISELECT, reg); \
     SET_CSR(CSR_IREG, val); \
  })

/* Clear the specified bits in an indirect CSR register */

#define CLEAR_INDIRECT_CSR_REG0(reg, val) \
  ({ \
     WRITE_CSR(CSR_ISELECT, reg); \
     CLEAR_CSR(CSR_IREG, val); \
  })

#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifdef CONFIG_ARCH_ADDRENV
#ifndef __ASSEMBLY__

/* A task group must have its L1 table in memory always, and the rest can
 * be dynamically committed to memory (and even swapped).
 *
 * In this implementation level tables except the final level N are always
 * kept in static memory, while the level N tables are always dynamically
 * allocated. There is one static page per level in `spgtables[]`.
 *
 * The implications ? They depend on the MMU type.
 *
 * For Sv32 this means that:
 * - A task can not have more than 4GB of memory allocated.
 * - The minimum amount of memory needed for page tables per task is 8K,
 *   which gives access to 4MB of memory. This is plenty for many tasks.
 *
 * For Sv39 this means that:
 * - A task can not have more than 1GB of memory allocated. This should be
 *   plenty enough...
 * - The minimum amount of memory needed for page tables per task is 12K,
 *   which gives access to 2MB of memory. This is plenty for many tasks.
 */

struct arch_addrenv_s
{
  /* Physical addresses of the static page tables (levels N-1) here, these
   * are allocated when a task is created.
   */

  uintptr_t spgtables[ARCH_SPGTS];

  /* The text, data, heap bases and heap size here */

  uintptr_t textvbase;
  uintptr_t datavbase;
  uintptr_t heapvbase;
  size_t    heapsize;

  /* The page directory root (satp) value */

  uintptr_t satp;
};

typedef struct arch_addrenv_s arch_addrenv_t;
#endif /* __ASSEMBLY__ */
#endif /* CONFIG_ARCH_ADDRENV */

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

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ARCH_RISCV_INCLUDE_ARCH_H */
