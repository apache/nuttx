/****************************************************************************
 * arch/arm/src/armv7-a/sm.h
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

/****************************************************************************
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2015-2020, Linaro Limited
 * Copyright (c) 2021-2023, Arm Limited
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_ARMV7_A_SM_H
#define __ARCH_ARM_SRC_ARMV7_A_SM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

#include "arm_cpu_psci.h"
#include "sctlr.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SM_EXIT_TO_SECURE           1

/****************************************************************************
 * Public Type Declarations
 ****************************************************************************/

struct arm_sm_banked_regs
{
  uint32_t usr_sp;
  uint32_t usr_lr;
  uint32_t irq_spsr;
  uint32_t irq_sp;
  uint32_t irq_lr;
  uint32_t fiq_spsr;
  uint32_t fiq_sp;
  uint32_t fiq_lr;

  /* Note that fiq_r{8-12} are not saved here. Instead sm_fiqhandler
   * preserves r{8-12}.
   */

  uint32_t svc_spsr;
  uint32_t svc_sp;
  uint32_t svc_lr;
  uint32_t abt_spsr;
  uint32_t abt_sp;
  uint32_t abt_lr;
  uint32_t und_spsr;
  uint32_t und_sp;
  uint32_t und_lr;
  uint32_t pmcr;
};

struct arm_sm_nsec_ctx
{
  struct arm_sm_banked_regs regs;

  uint32_t r8;
  uint32_t r9;
  uint32_t r10;
  uint32_t r11;
  uint32_t r12;

  uint32_t r0;
  uint32_t r1;
  uint32_t r2;
  uint32_t r3;
  uint32_t r4;
  uint32_t r5;
  uint32_t r6;
  uint32_t r7;

  /* Return state */

  uint32_t mon_lr;
  uint32_t mon_spsr;
};

struct arm_sm_sec_ctx
{
  struct arm_sm_banked_regs regs;

  uint32_t r0;
  uint32_t r1;
  uint32_t r2;
  uint32_t r3;
  uint32_t r4;
  uint32_t r5;
  uint32_t r6;
  uint32_t r7;

  /* Return state */

  uint32_t mon_lr;
  uint32_t mon_spsr;
};

struct arm_sm_ctx
{
  struct arm_sm_sec_ctx sec;
  struct arm_sm_nsec_ctx nsec;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: arm_sm_init
 *
 * Description:
 *   Initializes secure monitor
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void arm_sm_init(void);

/****************************************************************************
 * Name: arm_sm_init_stack
 *
 * Description:
 *   Initializes secure monitor
 *
 * Input Parameters:
 *   stack - the address of the stack that allocated for secure monitor
 *           runtime
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void arm_sm_init_stack(int stack);

/****************************************************************************
 * Name: arm_sm_boot_nsec
 *
 * Description:
 *   Set up the non-secure startup context, the entry will set to the
 *   non-secure context lr register, thus when we boot ap from tee, the
 *   non-secure will start running from the address that defined in lr
 *   register, of which contains the entry
 *
 * Input Parameters:
 *   entry - the entry address of the ap core
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void arm_sm_boot_nsec(uintptr_t entry);

/****************************************************************************
 * Name: arm_sm_get_nsec_ctx
 *
 * Description:
 *   Get the non-secure context that used by secure monitor mode
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The arm_sm_nsec_ctx of which contains the non-secure context
 *
 ****************************************************************************/

struct arm_sm_nsec_ctx *arm_sm_get_nsec_ctx(void);

/****************************************************************************
 * Name: arm_sm_save_banked_regs
 *
 * Description:
 *   Save the banked registers set
 *
 * Input Parameters:
 *   arm_sm_banked_regs - the banked registers set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void arm_sm_save_banked_regs(struct arm_sm_banked_regs *regs);

/****************************************************************************
 * Name: arm_sm_restore_banked_regs
 *
 * Description:
 *   To restore the banked registers that saved before
 *
 * Input Parameters:
 *   arm_sm_banked_regs - the banked registers set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void arm_sm_restore_banked_regs(struct arm_sm_banked_regs *regs);

/****************************************************************************
 * Name: arm_vectorsmc
 *
 * Description:
 *   The arm_vectorsmc internal will make "smc" call to enter into the
 *   monitor mode
 *
 * Input Parameters:
 *   a0 - the r0, stored in register
 *   a1 - the r1, stored in register
 *   a2 - the r2, stored in register
 *   a3 - the r3, stored in register
 *   a4 - the r4, stored in stack
 *   a5 - the r5, stored in stack
 *   a6 - the r6, stored in stack
 *   a7 - the r7, stored in stack
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void arm_vectorsmc(uint32_t a0, uint32_t a1, uint32_t a2, uint32_t a3,
                   uint32_t a4, uint32_t a5, uint32_t a6, uint32_t a7);

/****************************************************************************
 * Name: arm_vectorfiq_entry
 *
 * Description:
 *   This function is called when non-secure world trigger FIQ, and trapped
 *   to monitor world, and then this FIQ is handled in secure world
 *
 * Input Parameters:
 *   a0 - the r0, stored in register
 *   a1 - the r1, stored in register
 *   a2 - the r2, stored in register
 *   a3 - the r3, stored in register
 *   a4 - the r4, stored in stack
 *   a5 - the r5, stored in stack
 *   a6 - the r6, stored in stack
 *   a7 - the r7, stored in stack
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void arm_vectorfiq_entry(uint32_t a0, uint32_t a1, uint32_t a2, uint32_t a3,
                         uint32_t a4, uint32_t a5, uint32_t a6, uint32_t a7);

/****************************************************************************
 * Name: arm_sm_switch_nsec
 *
 * Description:
 *   This function is using to switch from secure to non-secure, is called
 *   from up_idle() function. This function is implemented by make "smc" call
 *   internal.
 *
 * Input Parameters:
 *   None
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void arm_sm_switch_nsec(void);

#endif /* __ARCH_ARM_SRC_ARMV7_A_SM_H */
