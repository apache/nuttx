/****************************************************************************
 * arch/ceva/src/common/vintc.h
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

#ifndef __ARCH_CEVA_SRC_COMMON_VINTC_H
#define __ARCH_CEVA_SRC_COMMON_VINTC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: up_vintc_initialize
 *
 * Description:
 *   Initialize the VINTC.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_HAVE_VINTC
void up_vintc_initialize(void);
#else
static inline void up_vintc_initialize(void)
{
}
#endif

/****************************************************************************
 * Name: up_vintc_enable_irq
 *
 * Description:
 *   On CEVA architectures, there are four levels of interrupt enabling:
 *   (1) at the global level(up_irq_enable)
 *   (2) at the DSP level(up_enable_irq)
 *   (2) at the VINTC level
 *   (3) at the device level
 *   In order to receive interrupts, they must be enabled at all four levels.
 *
 *   This function implements enabling of the device specified by 'irq'
 *   at the VINTC level if supported by the architecture.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_HAVE_VINTC
void up_vintc_enable_irq(int irq);
#else
static inline void up_vintc_enable_irq(int irq)
{
}
#endif

/****************************************************************************
 * Name: up_vintc_disable_irq
 *
 * Description:
 *   This function implements disabling of the device specified by 'irq'
 *   at the VINTC level if supported by the architecture(up_irq_save()
 *   supports the global level, the device level is hardware specific).
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_HAVE_VINTC
void up_vintc_disable_irq(int irq);
#else
static inline void up_vintc_disable_irq(int irq)
{
}
#endif

/****************************************************************************
 * Name: up_vintc_prioritize_irq
 *
 * Description:
 *   Set the priority of an IRQ.
 *
 ****************************************************************************/

#if defined(CONFIG_ARCH_HAVE_VINTC) && defined(CONFIG_ARCH_IRQPRIO)
int up_vintc_prioritize_irq(int irq, int priority);
#else
static inline int up_vintc_prioritize_irq(int irq, int priority)
{
  return 0; /* Not a critical error */
}
#endif

/****************************************************************************
 * Name: up_vintc_trigger_irq
 *
 * Description:
 *   Trigger an IRQ by software.
 *
 ****************************************************************************/

#if defined(CONFIG_ARCH_HAVE_VINTC) && defined(CONFIG_ARCH_HAVE_IRQTRIGGER)
void up_vintc_trigger_irq(int irq);
#else
static inline void up_vintc_trigger_irq(int irq)
{
}
#endif

/****************************************************************************
 * Name: up_vintc_handler
 *
 * Description:
 *   This function address must be sent from VINTC on VECTOR input in order
 *   to let DSP could jump to the appropriate interrupt handler location.
 *   Note that VINTC may pass this address to the hardware register, but
 *   ARCH specific code is responsible to implement this function.
 *
 ****************************************************************************/

void up_vintc_handler(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ARCH_CEVA_SRC_COMMON_VINTC_H */
