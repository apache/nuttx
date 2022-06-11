/****************************************************************************
 * arch/arm/src/armv8-m/arm_fpuconfig.c
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

#include "nvic.h"
#include "arm_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_fpuconfig
 *
 * Description:
 *   Configure the FPU.  Relative bit settings:
 *
 *     CPACR:  Enables access to CP10 and CP11
 *     CONTROL.FPCA: Determines whether the FP extension is active in the
 *       current context:
 *     FPCCR.ASPEN:  Enables automatic FP state preservation, then the
 *       processor sets this bit to 1 on successful completion of any FP
 *       instruction.
 *     FPCCR.LSPEN:  Enables lazy context save of FP state. When this is
 *       done, the processor reserves space on the stack for the FP state,
 *       but does not save that state information to the stack.
 *
 *  Software must not change the value of the ASPEN bit or LSPEN bit while
 *  either:
 *
 *   - the CPACR permits access to CP10 and CP11, that give access to the FP
 *     extension, or
 *   - the CONTROL.FPCA bit is set to 1
 *
 ****************************************************************************/

void arm_fpuconfig(void)
{
  uint32_t regval;

  /* Set CONTROL.FPCA so that we always get the extended context frame
   * with the volatile FP registers stacked above the basic context.
   */

  regval = getcontrol();
  regval |= CONTROL_FPCA;
  setcontrol(regval);

  /* Ensure that FPCCR.LSPEN is disabled, so that we don't have to contend
   * with the lazy FP context save behavior.  Clear FPCCR.ASPEN since we
   * are going to turn on CONTROL.FPCA for all contexts.
   */

  regval = getreg32(NVIC_FPCCR);
  regval &= ~(NVIC_FPCCR_ASPEN | NVIC_FPCCR_LSPEN);
  putreg32(regval, NVIC_FPCCR);

  /* Enable full access to CP10 and CP11 */

  regval = getreg32(NVIC_CPACR);
  regval |= NVIC_CPACR_CP_FULL(10) | NVIC_CPACR_CP_FULL(11);
  putreg32(regval, NVIC_CPACR);
}
