/****************************************************************************
 * arch/misoc/src/lm32/chip.h
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

#ifndef __ARCH_MISOC_SRC_LM32_CHIP_H
#define __ARCH_MISOC_SRC_LM32_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "lm32.h"

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

static inline unsigned int irq_getie(void)
{
  unsigned int ie;
  __asm__ __volatile__("rcsr %0, IE" : "=r" (ie));
  return ie;
}

static inline void irq_setie(unsigned int ie)
{
  __asm__ __volatile__("wcsr IE, %0" : : "r" (ie));
}

static inline unsigned int irq_getmask(void)
{
  unsigned int mask;
  __asm__ __volatile__("rcsr %0, IM" : "=r" (mask));
  return mask;
}

static inline void irq_setmask(unsigned int mask)
{
  __asm__ __volatile__("wcsr IM, %0" : : "r" (mask));
}

static inline unsigned int irq_pending(void)
{
  unsigned int pending;
  __asm__ __volatile__("rcsr %0, IP" : "=r" (pending));
  return pending;
}

#ifdef __cplusplus
}
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __ARCH_MISOC_SRC_LM32_CHIP_H */
