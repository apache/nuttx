/****************************************************************************
 * arch/arm/src/armv7-m/systick.h
 *
 *   Copyright (C) 2018 Pinecone Inc. All rights reserved.
 *   Author: Xiang Xiao <xiaoxiang@pinecone.net>
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
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_ARMV7_M_SYSTICK_H
#define __ARCH_ARM_SRC_ARMV7_M_SYSTICK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/timers/timer.h>

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

/****************************************************************************
 * Name: systick_initialize
 *
 * Description:
 *   This function initialize SYSTICK hardware module which is part of NVIC
 *   and return an instance of a "lower half" timer interface.
 *
 * Input parameters:
 *   coreclk - false if SYSTICK working clock come from the external
 *     reference clock, otherwise true.
 *   freq - The clock frequency in Hz. If freq is zero, calculate the value
 *     from NVIC_SYSTICK_CALIB register.
 *   minor - The timer driver minor number, skip the registration if minor
 *     < 0.
 *
 * Returned Value:
 *   On success, a non-NULL timer_lowerhalf_s is returned to the caller.
 *   In the event of any failure, a NULL value is returned.
 *
 ****************************************************************************/

#ifdef CONFIG_ARMV7M_SYSTICK
struct timer_lowerhalf_s *systick_initialize(bool coreclk, unsigned int freq,
                                             int minor);
#else
#  define systick_initialize(coreclk, freq, minor) NULL
#endif /* CONFIG_ARMV7M_SYSTICK */

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ARCH_ARM_SRC_ARMV7_M_SYSTICK_H */
