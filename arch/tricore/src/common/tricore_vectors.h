/****************************************************************************
 * arch/tricore/src/common/tricore_vectors.h
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

#ifndef __ARCH_TRICORE_SRC_COMMON_TRICORE_VECTORS_H
#define __ARCH_TRICORE_SRC_COMMON_TRICORE_VECTORS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stddef.h>
#include <stdint.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* Bus trap hook function type for safety test modules.
 * Returns OK if the trap was handled, ERROR otherwise.
 */

typedef int (*tricore_bustrap_hook_t)(uint32_t tid, void *context);

/* Arch-native TriCore trap definitions. */

enum tricore_trap_class_e
{
  TRICORE_TRAP_CLASS_MMU       = 0,  /* Memory management */
  TRICORE_TRAP_CLASS_INTPROT   = 1,  /* Internal protection */
  TRICORE_TRAP_CLASS_INSTERR   = 2,  /* Instruction errors */
  TRICORE_TRAP_CLASS_CTXMGMT   = 3,  /* Context management */
  TRICORE_TRAP_CLASS_BUS       = 4,  /* Bus */
  TRICORE_TRAP_CLASS_ASSERT    = 5,  /* Assertion */
  TRICORE_TRAP_CLASS_SYSCALL   = 6,  /* System call */
  TRICORE_TRAP_CLASS_NMI       = 7,  /* Non maskable interrupt */
};

/* Memory management (MMU) trap identification numbers */

enum tricore_trap_mmu_id_e
{
  TRICORE_TRAP_MMU_VAF         = 0,  /* Virtual address fill */
  TRICORE_TRAP_MMU_VAP         = 1,  /* Virtual address protection */
};

/* Internal protection trap identification numbers */

enum tricore_trap_intprot_id_e
{
  TRICORE_TRAP_INTPROT_PRIV    = 1,  /* Privilege violation */
  TRICORE_TRAP_INTPROT_MPR     = 2,  /* Memory protection read */
  TRICORE_TRAP_INTPROT_MPW     = 3,  /* Memory protection write */
  TRICORE_TRAP_INTPROT_MPX     = 4,  /* Memory protection execute */
  TRICORE_TRAP_INTPROT_MPP     = 5,  /* Memory protection peripheral access */
  TRICORE_TRAP_INTPROT_MPN     = 6,  /* Memory protection null address */
  TRICORE_TRAP_INTPROT_GRWP    = 7,  /* Global register write protection */
};

/* Instruction errors trap identification numbers */

enum tricore_trap_insterr_id_e
{
  TRICORE_TRAP_INSTERR_IOPC    = 1,  /* Illegal opcode */
  TRICORE_TRAP_INSTERR_UOPC    = 2,  /* Unimplemented opcode */
  TRICORE_TRAP_INSTERR_OPD     = 3,  /* Invalid operand specification */
  TRICORE_TRAP_INSTERR_ALN     = 4,  /* Data address alignment */
  TRICORE_TRAP_INSTERR_MEM     = 5,  /* Invalid local memory address */
  TRICORE_TRAP_INSTERR_CSE     = 6,  /* Coprocessor trap synchronous error */
};

/* Context management trap identification numbers */

enum tricore_trap_ctxmgmt_id_e
{
  TRICORE_TRAP_CTXMGMT_FCD     = 1,  /* Free context list depletion */
  TRICORE_TRAP_CTXMGMT_CDO     = 2,  /* Call depth overflow */
  TRICORE_TRAP_CTXMGMT_CDU     = 3,  /* Call depth underflow */
  TRICORE_TRAP_CTXMGMT_FCU     = 4,  /* Free context list underflow */
  TRICORE_TRAP_CTXMGMT_CSU     = 5,  /* Call stack underflow */
  TRICORE_TRAP_CTXMGMT_CTYP    = 6,  /* Context type */
  TRICORE_TRAP_CTXMGMT_NEST    = 7,  /* Nesting error */
};

/* Bus trap identification numbers */

enum tricore_trap_bus_id_e
{
  TRICORE_TRAP_BUS_PSE         = 1,  /* Program fetch synchronous error */
  TRICORE_TRAP_BUS_DSE         = 2,  /* Data access synchronous error */
  TRICORE_TRAP_BUS_DAE         = 3,  /* Data access asynchronous error */
  TRICORE_TRAP_BUS_CAE         = 4,  /* Coprocessor trap asynchronous error */
  TRICORE_TRAP_BUS_PIE         = 5,  /* Program memory integrity error */
  TRICORE_TRAP_BUS_DIE         = 6,  /* Data memory integrity error */
  TRICORE_TRAP_BUS_TAE         = 7,  /* Temporal asynchronous error */
};

/* Assertion trap identification numbers */

enum tricore_trap_assert_id_e
{
  TRICORE_TRAP_ASSERT_OVF      = 1,  /* Arithmetic overflow */
  TRICORE_TRAP_ASSERT_SOVF     = 2,  /* Sticky arithmetic overflow */
};

/* Trap information record. */

struct tricore_trap_s
{
  uintptr_t tAddr;
  uint32_t  tId    : 8;
  uint32_t  tClass : 8;
  uint32_t  tCpu   : 3;
  uint32_t  tpstr;    /* Program Synchronous Trap Register */
  uint32_t  tdstr;    /* Data Synchronous Trap Register    */
  uint32_t  tdatr;    /* Data Asynchronous Trap Register   */
  uint32_t  tdeadd;   /* Data Error Address Register       */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Trap Handler */

void tricore_svcall(volatile void *trap);
void tricore_trapcall(volatile void *trap);
void tricore_trapinit(void);
void tricore_bustrap_register_hook(tricore_bustrap_hook_t hook);
void tricore_fcu_register_pool(void *base, size_t num);

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_TRICORE_SRC_COMMON_TRICORE_VECTORS_H */
