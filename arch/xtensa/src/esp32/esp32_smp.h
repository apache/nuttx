/****************************************************************************
 * arch/xtensa/src/esp32/esp32_smp.h
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>>
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

#ifndef __ARCH_XTENSA_SRC_ESP32_ESP32_SMP_H
#define __ARCH_XTENSA_SRC_ESP32_ESP32_SMP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_SMP

/****************************************************************************
 * Pre-procesor Definitions
 ****************************************************************************/

/* An IDLE thread stack size for CPU0 must be defined */

#if !defined(CONFIG_SMP_IDLETHREAD_STACKSIZE)
#  error CONFIG_SMP_IDLETHREAD_STACKSIZE is not defined
#elif CONFIG_SMP_IDLETHREAD_STACKSIZE < 16
#  error CONFIG_SMP_IDLETHREAD_STACKSIZE is to small
#endif

#define CPU1_IDLETHREAD_STACKSIZE ((CONFIG_SMP_IDLETHREAD_STACKSIZE + 15) & ~15)
#define CPU1_IDLETHREAD_STACKWORDS (CPU1_IDLETHREAD_STACKSIZE >> 2)

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* This is the CPU1 IDLE stack */

extern uint32_t g_cpu1_idlestack[CPU1_IDLETHREAD_STACKWORDS];

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_fromcpu[0,1]_interrupt
 *
 * Description:
 *   Called to handle the from CPU0/1 interrupts.
 *
 ****************************************************************************/

int esp32_fromcpu0_interrupt(int irq, FAR void *context, FAR void *arg);
int esp32_fromcpu1_interrupt(int irq, FAR void *context, FAR void *arg);

#endif /* CONFIG_SMP */
#endif /* __ARCH_XTENSA_SRC_ESP32_ESP32_SMP_H */
