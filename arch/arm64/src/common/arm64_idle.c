/****************************************************************************
 * arch/arm64/src/common/arm64_idle.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/arch.h>
#include "arm64_internal.h"
#include "arm64_arch.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_idle
 *
 * Description:
 *   up_idle() is the logic that will be executed when there is no other
 *   ready-to-run task.  This is processor idle time and will continue until
 *   some interrupt occurs to cause a context switch from the idle task.
 *
 *   Processing in this state may be processor-specific. e.g., this is where
 *   power management operations might be performed.
 *
 ****************************************************************************/

void up_idle(void)
{
#if defined(CONFIG_SUPPRESS_INTERRUPTS) || defined(CONFIG_SUPPRESS_TIMER_INTS)
  /* If the system is idle and there are no timer interrupts, then process
   * "fake" timer interrupts. Hopefully, something will wake up.
   */

  nxsched_process_timer();
#elif defined(CONFIG_ARCH_TRUSTZONE_SECURE)

  /* Secure OS idle.
   *
   * Every entry calls ARM64_SMCCC_NUTTX_TEE_IDLE:
   *   - First call: EL3 registers interrupt handlers and boots REE.
   *   - Subsequent calls: EL3 checks if REE was preempted by G1S.
   *     If so, resumes REE. Otherwise returns 0 and TEE does WFI.
   *
   * TEE runs again when:
   *   1. Secure timer fires from REE -> EL3 does world switch ->
   *      TEE resumes here (after SMC/WFI) -> pending G1S IRQ
   *      fires through normal S-EL1 vector table -> threads run ->
   *      all done -> back here -> ARM64_SMCCC_NUTTX_TEE_IDLE -> resume REE
   *   2. TEE was preempted by NS interrupt -> REE idle sends
   *      ARM64_SMCCC_NUTTX_REE_IDLE -> EL3 switches to TEE -> TEE
   *      continues -> eventually idle -> ARM64_SMCCC_NUTTX_TEE_IDLE here
   */

  struct arm64_smccc_res res;

  /* TEE idle: on first call, EL3 registers interrupt handlers
   * and switches to REE. On subsequent calls, checks if REE
   * was preempted by G1S and resumes it, or returns for WFI.
   *
   * Disable IRQs to prevent a lost-wakeup race: if a G1S IRQ
   * were taken between SMC return and WFI, its handler could
   * wake tasks but WFI would still sleep. With IRQs masked,
   * the pending interrupt keeps WFI from blocking.
   */

  irqstate_t flags = irq_save();

  arm64_smccc_smc(ARM64_SMCCC_NUTTX_TEE_IDLE, 0, 0, 0, 0, 0, 0, 0, &res);

  asm("dsb sy");
  asm("wfi");

  irq_restore(flags);

#elif defined(CONFIG_ARCH_TRUSTZONE_NONSECURE)

  /* REE idle: no tasks ready, yield CPU to TEE unconditionally.
   * TEE will WFI. When a NS interrupt fires, EL3 preempts TEE
   * and returns here. When a G1S fires, EL3 switches to TEE
   * and TEE_IDLE will resume us later.
   */

  struct arm64_smccc_res res;

  arm64_smccc_smc(ARM64_SMCCC_NUTTX_REE_IDLE, 0, 0, 0, 0, 0, 0, 0, &res);
#else

  /* Sleep until an interrupt occurs to save power */

  asm("dsb sy");
  asm("wfi");
#endif
}
