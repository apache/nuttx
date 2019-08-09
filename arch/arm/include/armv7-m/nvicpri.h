/************************************************************************************
 * arch/arm/include/armv7-m/nvicpri.h
 *
 *   Copyright (C) 2009, 2011-2014, 2017-2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           Dave Marples <dave@marples.net>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

#ifndef __ARCH_ARM_INCLUDE_ARM7_M_NVICPRI_H
#define __ARCH_ARM_INCLUDE_ARM7_M_NVICPRI_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <arch/chip/chip.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* If CONFIG_ARMV7M_USEBASEPRI is selected, then interrupts will be disabled
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

#endif /* __ARCH_ARM_INCLUDE_ARM7_M_NVICPRI_H */
