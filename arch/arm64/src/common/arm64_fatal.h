/****************************************************************************
 * arch/arm64/src/common/arm64_fatal.h
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

#ifndef __ARCH_ARM64_SRC_COMMON_ARM64_FATAL_H
#define __ARCH_ARM64_SRC_COMMON_ARM64_FATAL_H

#ifndef __ASSEMBLY__

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>
#include <debug.h>
#include <assert.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ESR_ELX_EC_UNKNOWN      (0x00)
#define ESR_ELX_EC_WFX          (0x01)

/* Unallocated EC: 0x02 */

#define ESR_ELX_EC_CP15_32      (0x03)
#define ESR_ELX_EC_CP15_64      (0x04)
#define ESR_ELX_EC_CP14_MR      (0x05)
#define ESR_ELX_EC_CP14_LS      (0x06)
#define ESR_ELX_EC_FP_ASIMD     (0x07)
#define ESR_ELX_EC_CP10_ID      (0x08)  /* EL2 only */
#define ESR_ELX_EC_PAC          (0x09)  /* EL2 and above */

/* Unallocated EC: 0x0A - 0x0B */

#define ESR_ELX_EC_CP14_64      (0x0C)
#define ESR_ELX_EC_BTI          (0x0D)
#define ESR_ELX_EC_ILL          (0x0E)

/* Unallocated EC: 0x0F - 0x10 */

#define ESR_ELX_EC_SVC32        (0x11)
#define ESR_ELX_EC_HVC32        (0x12)  /* EL2 only */
#define ESR_ELX_EC_SMC32        (0x13)  /* EL2 and above */

/* Unallocated EC: 0x14 */

#define ESR_ELX_EC_SVC64        (0x15)
#define ESR_ELX_EC_HVC64        (0x16)  /* EL2 and above */
#define ESR_ELX_EC_SMC64        (0x17)  /* EL2 and above */
#define ESR_ELX_EC_SYS64        (0x18)
#define ESR_ELX_EC_SVE          (0x19)
#define ESR_ELX_EC_ERET         (0x1a)  /* EL2 only */

/* Unallocated EC: 0x1B */

#define ESR_ELX_EC_FPAC         (0x1C)  /* EL1 and above */
#define ESR_ELX_EC_SME          (0x1D)

/* Unallocated EC: 0x1D - 0x1E */

#define ESR_ELX_EC_IMP_DEF      (0x1f)  /* EL3 only */
#define ESR_ELX_EC_IABT_LOW     (0x20)
#define ESR_ELX_EC_IABT_CUR     (0x21)
#define ESR_ELX_EC_PC_ALIGN     (0x22)

/* Unallocated EC: 0x23 */

#define ESR_ELX_EC_DABT_LOW     (0x24)
#define ESR_ELX_EC_DABT_CUR     (0x25)
#define ESR_ELX_EC_SP_ALIGN     (0x26)
#define ESR_ELX_EC_MOPS         (0x27)
#define ESR_ELX_EC_FP_EXC32     (0x28)

/* Unallocated EC: 0x29 - 0x2B */

#define ESR_ELX_EC_FP_EXC64     (0x2C)

/* Unallocated EC: 0x2D - 0x2E */

#define ESR_ELX_EC_SERROR       (0x2F)
#define ESR_ELX_EC_BREAKPT_LOW  (0x30)
#define ESR_ELX_EC_BREAKPT_CUR  (0x31)
#define ESR_ELX_EC_SOFTSTP_LOW  (0x32)
#define ESR_ELX_EC_SOFTSTP_CUR  (0x33)
#define ESR_ELX_EC_WATCHPT_LOW  (0x34)
#define ESR_ELX_EC_WATCHPT_CUR  (0x35)

/* Unallocated EC: 0x36 - 0x37 */

#define ESR_ELX_EC_BKPT32       (0x38)

/* Unallocated EC: 0x39 */

#define ESR_ELX_EC_VECTOR32     (0x3A) /* EL2 only */

/* Unallocated EC: 0x3B */

#define ESR_ELX_EC_BRK64        (0x3C)

/* Unallocated EC: 0x3D - 0x3F */

#define ESR_ELX_EC_MAX          (0x3F)

#define ESR_ELX_EC_SHIFT        (26)
#define ESR_ELX_EC_WIDTH        (6)
#define ESR_ELX_EC_MASK         (0x3F << ESR_ELX_EC_SHIFT)
#define ESR_ELX_EC(esr)         (((esr) & ESR_ELX_EC_MASK) \
                                >> ESR_ELX_EC_SHIFT)

/* Shared ISS fault status code(IFSC/DFSC) for Data/Instruction aborts */

#define ESR_ELX_FSC             (0x3F)
#define ESR_ELX_FSC_TYPE        (0x3C)
#define ESR_ELX_FSC_LEVEL       (0x03)
#define ESR_ELX_FSC_EXTABT      (0x10)
#define ESR_ELX_FSC_MTE         (0x11)
#define ESR_ELX_FSC_SERROR      (0x11)
#define ESR_ELX_FSC_ACCESS      (0x08)
#define ESR_ELX_FSC_FAULT       (0x04)
#define ESR_ELX_FSC_PERM        (0x0C)
#define ESR_ELX_FSC_SEA_TTW0    (0x14)
#define ESR_ELX_FSC_SEA_TTW1    (0x15)
#define ESR_ELX_FSC_SEA_TTW2    (0x16)
#define ESR_ELX_FSC_SEA_TTW3    (0x17)
#define ESR_ELX_FSC_SECC        (0x18)
#define ESR_ELX_FSC_SECC_TTW0   (0x1c)
#define ESR_ELX_FSC_SECC_TTW1   (0x1d)
#define ESR_ELX_FSC_SECC_TTW2   (0x1e)
#define ESR_ELX_FSC_SECC_TTW3   (0x1f)

#define DBG_ESR_EVT(x)          (((x) >> 27) & 0x7)
#define DBG_ESR_EVT_HWBP        (0x0)
#define DBG_ESR_EVT_HWSS        (0x1)
#define DBG_ESR_EVT_HWWP        (0x2)
#define DBG_ESR_EVT_BRK         (0x6)

#define __builtin_unreachable()    \
  do                               \
    {                              \
      serr("Unreachable code\n"); \
      PANIC();                     \
    } while (true)

/****************************************************************************
 * Public Type Declarations
 ****************************************************************************/

typedef int (*fatal_handle_func_t)(uint64_t *regs, uint64_t far,
                                   uint64_t esr);

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: arm64_fatal_handler
 *
 * Description:
 *   Fatal handle for arm64
 * Input Parameters:
 *   reg:    exception stack reg context
 *
 * Returned Value: None
 *   If the function return, the exception has been handled
 *
 ****************************************************************************/

void arm64_fatal_handler(uint64_t *reg);

/****************************************************************************
 * Name: arm64_register_debug_hook
 *
 * Description:
 *   Register a hook function for DEBUG event
 * Input Parameters:
 *   nr:   DEBUG event
 *           DBG_ESR_EVT_HWBP : Hardware BreakPoint
 *           DBG_ESR_EVT_HWSS : Hardware SingleStep
 *           DBG_ESR_EVT_HWWP : Hardware WatchPoint
 *           DBG_ESR_EVT_BRK  : Brk instruction trigger
 *   fn:   hook function
 *
 * Returned Value: none
 *
 ****************************************************************************/

void arm64_register_debug_hook(int nr, fatal_handle_func_t fn);

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM64_SRC_COMMON_ARM64_FATAL_H */
