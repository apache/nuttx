/****************************************************************************
 * arch/arm/src/armv7-a/arm_sm_runtime.c
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
 * Copyright (c) 2015-2023, Linaro Limited
 * Copyright (c) 2023, Arm Limited
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <string.h>

#include "arm.h"
#include "sm.h"
#include "gic.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SM_STACK_RESERVE_SIZE     sizeof(struct arm_sm_ctx)
#define MONITOR_STACK_OFFS        SM_STACK_RESERVE_SIZE
#define CFG_MONITOR_STACK_EXTRA   0
#define MONITOR_STACK_SIZE        (CONFIG_ARMV7A_MONITOR_STACK_SIZE + \
                                   MONITOR_STACK_OFFS + \
                                   CFG_MONITOR_STACK_EXTRA)
#define STACK_CANARY_SIZE         (4 * sizeof(long))
#define MON_STACK_ALIGNMENT       64 /* (sizeof(long) * U(2)) */

#define ROUNDUP(v, size) (((v) + ((__typeof__(v))(size)-1)) & \
           ~((__typeof__(v))(size)-1))

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* the g_monitor_stack are allocated using to manage the secure context and
 * non-secure context.
 * the "g_monitor_stack" cannot be declared as "const", as the assembly
 * code need to access "g_monitor_stack", if we declared as "const",
 * the arm_sm_init procedure will not work
 */

#ifdef CONFIG_SMP
static uint32_t g_monitor_stack[CONFIG_SMP_NCPUS][ROUNDUP(
           MONITOR_STACK_SIZE + STACK_CANARY_SIZE, MON_STACK_ALIGNMENT) \
           / sizeof(uint32_t)] \
           __attribute__((section(".monitor_stack"), \
           aligned(MON_STACK_ALIGNMENT)));
#else
static uint32_t g_monitor_stack[ROUNDUP(
           MONITOR_STACK_SIZE + STACK_CANARY_SIZE, MON_STACK_ALIGNMENT) \
           / sizeof(uint32_t)] \
           __attribute__((section(".monitor_stack"), \
           aligned(MON_STACK_ALIGNMENT)));
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_sm_init
 *
 * Description:
 *   This method is called at the boot stage, the arm_head.S will call this
 *   method to perform init the secure monitor runtime
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void arm_sm_init(void)
{
#ifdef CONFIG_SMP
  arm_sm_init_stack(((int)&g_monitor_stack[up_cpu_index()]
                + sizeof(g_monitor_stack[up_cpu_index()]))
                - MONITOR_STACK_OFFS);
#else
  arm_sm_init_stack(((int)&g_monitor_stack + sizeof(g_monitor_stack))
                - MONITOR_STACK_OFFS);
#endif
}

/****************************************************************************
 * Name: arm_sm_boot_nsec
 *
 * Description:
 *   As the ap core is loaded by tee core, when the tee core is boot
 *   finished, it need to setup the nsec_ctc mon_lr as the ap core entry
 *   address, then using the smc command to switch to the monitor mode and
 *   switch to the ap core
 *
 * Input Parameters:
 *   entry  - The ap core entry address
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void arm_sm_boot_nsec(uintptr_t entry)
{
  FAR struct arm_sm_nsec_ctx *nsec_ctx;

  nsec_ctx = arm_sm_get_nsec_ctx();
  nsec_ctx->mon_lr = entry;
  nsec_ctx->mon_spsr = PSR_MODE_SVC | PSR_I_BIT;
  if (entry & 1)
    {
      nsec_ctx->mon_spsr |= PSR_T_BIT;
    }
}

/****************************************************************************
 * Name: arm_sm_from_nsec
 *
 * Description:
 *   Returns one of SM_EXIT_TO_* exit monitor in secure or non-secure world
 *
 * Input Parameters:
 *   ctx - The secure monitor context
 *
 * Returned Value:
 *   SM_EXIT_TO_NON_SECURE: return to the non-secure world after this
 *   procedure
 *   SM_EXIT_TO_SECURE: continue running inside the secure world
 *
 ****************************************************************************/

uint32_t arm_sm_from_nsec(FAR struct arm_sm_ctx *ctx)
{
  FAR uint32_t *nsec_r0 = (FAR uint32_t *)(&ctx->nsec.r0);
  FAR struct arm_smccc_res *args = (FAR struct arm_smccc_res *)nsec_r0;

  arm_sm_save_banked_regs(&ctx->nsec.regs);
  arm_sm_restore_banked_regs(&ctx->sec.regs);

  memcpy(&ctx->sec.r0, args, sizeof(*args));

  ctx->sec.mon_lr = (uint32_t)arm_vectorsmc;

  /* the following procedure return "SM_EXIT_TO_SECURE" which indicate
   * that we should still stay at secure world, until the
   * "arm_vectorsmc" using "smc #0" to return to the
   * non-secure world
   */

  return SM_EXIT_TO_SECURE;
}
