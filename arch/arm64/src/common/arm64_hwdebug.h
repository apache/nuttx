/****************************************************************************
 * arch/arm64/src/common/arm64_hwdebug.h
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
 *
 ****************************************************************************/

#ifndef __ARCH_ARM64_SRC_COMMON_ARM64_HWDEBUG_H
#define __ARCH_ARM64_SRC_COMMON_ARM64_HWDEBUG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/list.h>
#include <nuttx/mutex.h>
#include <nuttx/sched.h>
#include <sched/sched.h>
#include "arm64_arch.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

enum
{
  BREAKPOINT_LEN_1 = 1,
  BREAKPOINT_LEN_2 = 2,
  BREAKPOINT_LEN_3 = 3,
  BREAKPOINT_LEN_4 = 4,
  BREAKPOINT_LEN_5 = 5,
  BREAKPOINT_LEN_6 = 6,
  BREAKPOINT_LEN_7 = 7,
  BREAKPOINT_LEN_8 = 8,
};

#define DBG_HOOK_HANDLED                0
#define DBG_HOOK_ERROR                  1

/* BRK instruction trap from AArch64 state */

#define ESR_ELX_BRK64_ISS_COMMENT_MASK  0xffff

/* ISS field definitions for System instruction traps */

#define ESR_ELX_SYS64_ISS_RES0_SHIFT    22
#define ESR_ELX_SYS64_ISS_RES0_MASK     \
          (UL(0x7) << ESR_ELX_SYS64_ISS_RES0_SHIFT)
#define ESR_ELX_SYS64_ISS_DIR_MASK      0x1
#define ESR_ELX_SYS64_ISS_DIR_READ      0x1
#define ESR_ELX_SYS64_ISS_DIR_WRITE     0x0

#define ESR_ELX_SYS64_ISS_RT_SHIFT      5
#define ESR_ELX_SYS64_ISS_RT_MASK       \
           (UL(0x1f) << ESR_ELX_SYS64_ISS_RT_SHIFT)
#define ESR_ELX_SYS64_ISS_CRM_SHIFT     1
#define ESR_ELX_SYS64_ISS_CRM_MASK      \
           (UL(0xf) << ESR_ELX_SYS64_ISS_CRM_SHIFT)
#define ESR_ELX_SYS64_ISS_CRN_SHIFT     10
#define ESR_ELX_SYS64_ISS_CRN_MASK      \
           (UL(0xf) << ESR_ELX_SYS64_ISS_CRN_SHIFT)
#define ESR_ELX_SYS64_ISS_OP1_SHIFT     14
#define ESR_ELX_SYS64_ISS_OP1_MASK      \
           (UL(0x7) << ESR_ELX_SYS64_ISS_OP1_SHIFT)
#define ESR_ELX_SYS64_ISS_OP2_SHIFT     17
#define ESR_ELX_SYS64_ISS_OP2_MASK      \
           (UL(0x7) << ESR_ELX_SYS64_ISS_OP2_SHIFT)
#define ESR_ELX_SYS64_ISS_OP0_SHIFT     20
#define ESR_ELX_SYS64_ISS_OP0_MASK      \
           (UL(0x3) << ESR_ELX_SYS64_ISS_OP0_SHIFT)

#define ESR_ELX_SYS64_ISS_SYS_MASK  (ESR_ELX_SYS64_ISS_OP0_MASK | \
                                     ESR_ELX_SYS64_ISS_OP1_MASK | \
                                     ESR_ELX_SYS64_ISS_OP2_MASK | \
                                     ESR_ELX_SYS64_ISS_CRN_MASK | \
                                     ESR_ELX_SYS64_ISS_CRM_MASK)

#define ESR_ELX_SYS64_ISS_SYS_VAL(op0, op1, op2, crn, crm) \
        (((op0) << ESR_ELX_SYS64_ISS_OP0_SHIFT) |          \
         ((op1) << ESR_ELX_SYS64_ISS_OP1_SHIFT) |          \
         ((op2) << ESR_ELX_SYS64_ISS_OP2_SHIFT) |          \
         ((crn) << ESR_ELX_SYS64_ISS_CRN_SHIFT) |          \
         ((crm) << ESR_ELX_SYS64_ISS_CRM_SHIFT))

#define ESR_ELX_SYS64_ISS_SYS_OP_MASK   (ESR_ELX_SYS64_ISS_SYS_MASK | \
                                         ESR_ELX_SYS64_ISS_DIR_MASK)
#define ESR_ELX_SYS64_ISS_RT(esr) (((esr) & ESR_ELX_SYS64_ISS_RT_MASK) \
                                  >> ESR_ELX_SYS64_ISS_RT_SHIFT)

/* Low-level stepping controls. */

#define DBG_MDSCR_SS                    (1 << 0)
#define DBG_SPSR_SS                     (1 << 21)

/* MDSCR_EL1 enabling bits */

#define DBG_MDSCR_KDE                   (1 << 13)
#define DBG_MDSCR_MDE                   (1 << 15)
#define DBG_MDSCR_MASK                  ~(DBG_MDSCR_KDE | DBG_MDSCR_MDE)

/* Privilege Levels */

#define AARCH64_BREAKPOINT_EL1          1
#define AARCH64_BREAKPOINT_EL0          2

/* Breakpoint */

#define ARM_BREAKPOINT_EXECUTE          0

/* Watchpoints */

#define ARM_BREAKPOINT_LOAD             1
#define ARM_BREAKPOINT_STORE            2
#define AARCH64_ESR_ACCESS_MASK         (1 << 6)

/* Lengths */

#define ARM_BREAKPOINT_LEN_1            0x1
#define ARM_BREAKPOINT_LEN_2            0x3
#define ARM_BREAKPOINT_LEN_3            0x7
#define ARM_BREAKPOINT_LEN_4            0xf
#define ARM_BREAKPOINT_LEN_5            0x1f
#define ARM_BREAKPOINT_LEN_6            0x3f
#define ARM_BREAKPOINT_LEN_7            0x7f
#define ARM_BREAKPOINT_LEN_8            0xff

/* Kernel stepping */

#define ARM_KERNEL_STEP_NONE            0
#define ARM_KERNEL_STEP_ACTIVE          1
#define ARM_KERNEL_STEP_SUSPEND         2

/* MDSCR_EL1
 * Monitor Debug System Control Register. It's the
 * main control register for the debug implementation.
 *
 * Initial value for MSDCR_EL1 when starting userspace,
 * which disables all debug exceptions.
 *
 * Instruction Breakpoint Exceptions (software breakpoints)
 * cannot be disabled and MDSCR does not affect
 * single-step behaviour.
 */

#define MSDCR_EL1_INITIAL_VALUE         0

#define ARM64_MDSCR_EL1_SS              (1u << 0)
#define ARM64_MDSCR_EL1_SS_SHIFT        0
#define ARM64_MDSCR_EL1_ERR             (1u << 6)
#define ARM64_MDSCR_EL1_ERR_SHIFT       6
#define ARM64_MDSCR_EL1_TDCC            (1u << 12)
#define ARM64_MDSCR_EL1_TDCC_SHIFT      12
#define ARM64_MDSCR_EL1_KDE             (1u << 13)
#define ARM64_MDSCR_EL1_KDE_SHIFT       13
#define ARM64_MDSCR_EL1_HDE             (1u << 14)
#define ARM64_MDSCR_EL1_HDE_SHIFT       14
#define ARM64_MDSCR_EL1_MDE             (1u << 15)
#define ARM64_MDSCR_EL1_MDE_SHIFT       15
#define ARM64_MDSCR_EL1_RAZ_WI          0x000e0000lu
#define ARM64_MDSCR_EL1_RAZ_WI_SHIFT    16
#define ARM64_MDSCR_EL1_TDA             (1u << 21)
#define ARM64_MDSCR_EL1_TDA_SHIFT       21
#define ARM64_MDSCR_EL1_INTDIS          0x000c0000u
#define ARM64_MDSCR_EL1_INTDIS_SHIFT    22
#define ARM64_MDSCR_EL1_TXU             (1u << 26)
#define ARM64_MDSCR_EL1_TXU_SHIFT       26
#define ARM64_MDSCR_EL1_RXO             (1u << 27)
#define ARM64_MDSCR_EL1_RXO_SHIFT       27
#define ARM64_MDSCR_EL1_TXfull          (1u << 29)
#define ARM64_MDSCR_EL1_TXfull_SHIFT    29
#define ARM64_MDSCR_EL1_RXfull          (1u << 30)
#define ARM64_MDSCR_EL1_RXfull_SHIFT    30

/* ID_AA64DFR0
 * Debug Feature Register 0. This register is used to query the system
 * for the debug capabilities present within the chip.
 */

#define ARM64_ID_AADFR0_EL1_DEBUG_VER   0x0000000000000Flu
#define ARM64_ID_AADFR0_EL1_TRACE_VER   0x000000000000F0lu
#define ARM64_ID_AADFR0_EL1_PMU_VER     0x00000000000F00lu

/* Defines the amount of HW breakpoints. */

#define ARM64_ID_AADFR0_EL1_BRPS        0x0000000000F000lu
#define ARM64_ID_AADFR0_EL1_BRPS_SHIFT  12lu

/* Defines the amount of HW data watchpoints. */

#define ARM64_ID_AADFR0_EL1_WRPS        0x00000000F00000lu
#define ARM64_ID_AADFR0_EL1_WRPS_SHIFT  20lu
#define ARM64_ID_AADFR0_EL1_CTX_CMP     0x000000F0000000lu
#define ARM64_ID_AADFR0_EL1_PMS_VER     0x00000F00000000lu

/* Limits */

#define ARM64_MAX_BRP                   16
#define ARM64_MAX_WRP                   16
#define ARM64_MAX_HBP_SLOTS             (ARM64_MAX_BRP + ARM64_MAX_WRP)

/* Virtual debug register bases. */

#define AARCH64_DBG_REG_BVR             0
#define AARCH64_DBG_REG_BCR             (AARCH64_DBG_REG_BVR + ARM64_MAX_BRP)
#define AARCH64_DBG_REG_WVR             (AARCH64_DBG_REG_BCR + ARM64_MAX_BRP)
#define AARCH64_DBG_REG_WCR             (AARCH64_DBG_REG_WVR + ARM64_MAX_WRP)

/* Debug register names. */

#define AARCH64_DBG_REG_NAME_BVR        bvr
#define AARCH64_DBG_REG_NAME_BCR        bcr
#define AARCH64_DBG_REG_NAME_WVR        wvr
#define AARCH64_DBG_REG_NAME_WCR        wcr

/* Accessor macros for the debug registers. */

#define AARCH64_DBG_READ(N, REG, VAL)            \
  do                                             \
  {                                              \
      VAL = read_sysreg(dbg ## REG ## N ## _el1);\
  } while (0)

#define AARCH64_DBG_WRITE(N, REG, VAL)            \
  do                                              \
  {                                               \
      write_sysreg(VAL, dbg ## REG ## N ## _el1); \
  } while (0)

/****************************************************************************
 * Type Declarations
 ****************************************************************************/

struct arch_hw_breakpoint_ctrl
{
  uint32_t __reserved : 19;
  uint32_t len        : 8;
  uint32_t type       : 2;
  uint32_t privilege  : 2;
  uint32_t enabled    : 1;
};

struct arch_hw_breakpoint
{
  uint64_t address;
  uint64_t trigger;
  struct   arch_hw_breakpoint_ctrl ctrl;
  int      in_used;

  /* callback handler */

  debug_callback_t handle_fn;
  void            *arg;
  int              type;
  size_t           size;
};

struct arm64_breakpoint_context
{
  /* Breakpoint currently in use for each BRP. */

  struct arch_hw_breakpoint on_reg[ARM64_MAX_BRP];

  /* Number of BRP registers on this CPU. */

  int core_num;

  int disabled;
};

struct arm64_debugpoint
{
  void     *addr;
  int       type;
  size_t    size;
  int       hbp_slot;
  int       in_used;
};

struct arm64_debugpoint_slot
{
  struct arm64_debugpoint slot[ARM64_MAX_HBP_SLOTS];
};

typedef int (*break_func_t)(struct regs_context *regs, uint64_t esr);

struct break_inst_hook
{
  break_func_t func;
  uint16_t imm;
  uint16_t mask;
  struct list_node entry;
};

static inline uint32_t encode_ctrl_reg(struct arch_hw_breakpoint_ctrl ctrl)
{
  uint32_t val =
    (ctrl.len << 5) | (ctrl.type << 3) |
    (ctrl.privilege << 1) | ctrl.enabled;

  return val;
}

static inline void decode_ctrl_reg(uint32_t reg,
                                   struct arch_hw_breakpoint_ctrl *ctrl)
{
  ctrl->enabled     = reg & 0x1;
  reg               >>= 1;
  ctrl->privilege   = reg & 0x3;
  reg               >>= 2;
  ctrl->type        = reg & 0x3;
  reg               >>= 2;
  ctrl->len         = reg & 0xff;
}

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

void arm64_hwdebug_init(void);
void arm64_hwdebug_secondary_init(void);

#endif  /* __ARCH_ARM64_SRC_COMMON_ARM64_HWDEBUG_H */
