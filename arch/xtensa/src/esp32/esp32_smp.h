/****************************************************************************
 * arch/xtensa/src/esp32/esp32_smp.h
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

#if !defined(CONFIG_IDLETHREAD_STACKSIZE)
#  error CONFIG_IDLETHREAD_STACKSIZE is not defined
#elif CONFIG_IDLETHREAD_STACKSIZE < 16
#  error CONFIG_IDLETHREAD_STACKSIZE is to small
#endif

#define CPU1_IDLETHREAD_STACKSIZE ((CONFIG_IDLETHREAD_STACKSIZE + 15) & ~15)
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

int esp32_fromcpu0_interrupt(int irq, void *context, void *arg);
int esp32_fromcpu1_interrupt(int irq, void *context, void *arg);

#endif /* CONFIG_SMP */
#endif /* __ARCH_XTENSA_SRC_ESP32_ESP32_SMP_H */
