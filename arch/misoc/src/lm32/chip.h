/****************************************************************************
 * arch/misoc/src/lm32/chip.h
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Ramtin Amin <keytwo@gmail.com>
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

#ifndef __ARCH_MISOC_SRC_LM32_CHIP_H
#define __ARCH_MISOC_SRC_LM32_CHIP_H 1

/****************************************************************************
 * Inline Functions
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
