/************************************************************************************
 * arch/arm/include/armv8-m/nvicpri.h
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
 ************************************************************************************/

#ifndef __ARCH_ARM_INCLUDE_ARM8_M_NVICPRI_H
#define __ARCH_ARM_INCLUDE_ARM8_M_NVICPRI_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <arch/chip/chip.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* If CONFIG_ARMV8M_USEBASEPRI is selected, then interrupts will be disabled
 * by setting the BASEPRI register to NVIC_SYSH_DISABLE_PRIORITY so that most
 * interrupts will not have execution priority.  SVCall must have execution
 * priority in all cases.
 *
 * In the normal cases, interrupts are not nest-able and all interrupts run
 * at an execution priority between NVIC_SYSH_PRIORITY_MIN and
 * NVIC_SYSH_PRIORITY_MAX (with NVIC_SYSH_PRIORITY_MAX reserved for SVCall).
 *
 * If, in addition, CONFIG_ARCH_HIPRI_INTERRUPT is defined, then special
 * high priority interrupts are supported.  These are not "nested" in the
 * normal sense of the word.  These high priority interrupts can interrupt
 * normal processing but execute outside of OS (although they can "get back
 * into the game" via a PendSV interrupt).
 *
 * In the normal course of things, interrupts must occasionally be disabled
 * using the up_irq_save() inline function to prevent contention in use of
 * resources that may be shared between interrupt level and non-interrupt
 * level logic.  Now the question arises, if we are using
 * CONFIG_ARCH_HIPRI_INTERRUPT=y, do we disable all interrupts except
 * SVCall (we cannot disable SVCall interrupts).  Or do we only disable the
 * "normal" interrupts?
 *
 * If we are using the BASEPRI register to disable interrupts, then the
 * answer is that we must disable ONLY the "normal interrupts".  That
 * is because we cannot disable SVCALL interrupts and we cannot permit
 * SVCAll interrupts running at a higher priority than the high priority
 * interrupts (otherwise, they will introduce jitter in the high priority
 * interrupt response time.)
 *
 * Hence, if you need to disable the high priority interrupt, you will have
 * to disable the interrupt either at the peripheral that generates the
 * interrupt or at the NVIC.  Disabling global interrupts via the BASEPRI
 * register cannot effect high priority interrupts.
 */

/* The high priority interrupt must be highest priority.  This prevents
 * SVCALL handling from adding jitter to high priority interrupt response.
 * Disabling interrupts will disable all interrupts EXCEPT SVCALL and the
 * high priority interrupts.
 */

#define NVIC_SYSH_MAXNORMAL_PRIORITY  NVIC_SYSH_PRIORITY_DEFAULT
#define NVIC_SYSH_HIGH_PRIORITY       (NVIC_SYSH_PRIORITY_DEFAULT - 2*NVIC_SYSH_PRIORITY_STEP)
#define NVIC_SYSH_DISABLE_PRIORITY    NVIC_SYSH_PRIORITY_DEFAULT
#define NVIC_SYSH_SVCALL_PRIORITY     (NVIC_SYSH_PRIORITY_DEFAULT - 1*NVIC_SYSH_PRIORITY_STEP)

#endif /* __ARCH_ARM_INCLUDE_ARM8_M_NVICPRI_H */
