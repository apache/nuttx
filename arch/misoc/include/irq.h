/****************************************************************************
 * arch/misoc/include/irq.h
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

/* This file should never be included directly but, rather, only indirectly
 * through nuttx/irq.h
 */

#ifndef __ARCH_MISOC_INCLUDE_IRQ_H
#define __ARCH_MISOC_INCLUDE_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/irq.h>
#include <arch/chip/irq.h>

#ifdef CONFIG_ARCH_CHIP_LM32
# include <arch/lm32/irq.h>
#endif
#ifdef CONFIG_ARCH_CHIP_MINERVA
# include <arch/minerva/irq.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

irqstate_t up_irq_save(void);
irqstate_t up_irq_enable(void);
void up_irq_restore(irqstate_t flags);

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_MISOC_INCLUDE_IRQ_H */
