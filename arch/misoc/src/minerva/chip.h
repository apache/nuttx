/****************************************************************************
 * arch/misoc/src/minerva/chip.h
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_MISOC_SRC_MINERVA_CHIP_H
#define __ARCH_MISOC_SRC_MINERVA_CHIP_H

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#include "minerva.h"
#include <arch/minerva/csrdefs.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CSR_IRQ_MASK 0x330
#define CSR_IRQ_PENDING 0x360

#define CSR_DCACHE_INFO 0xcc0

#define csrr(reg) \
  ({ \
     unsigned long __tmp; \
     asm volatile ("csrr %0, " #reg : "=r"(__tmp)); \
     __tmp; \
  })

#define csrw(reg, val) \
  ({ \
     if (__builtin_constant_p(val) && (unsigned long)(val) < 32) \
       { \
          asm volatile ("csrw " #reg ", %0" :: "i"(val)); \
       } \
     else \
       { \
          asm volatile ("csrw " #reg ", %0" :: "r"(val)); \
       } \
   })

#define csrs(reg, bit) \
  ({ \
     if (__builtin_constant_p(bit) && (unsigned long)(bit) < 32) \
       { \
        asm volatile ("csrrs x0, " #reg ", %0" :: "i"(bit)); \
       } \
     else \
       { \
        asm volatile ("csrrs x0, " #reg ", %0" :: "r"(bit)); \
       } \
   })

#define csrc(reg, bit) \
  ({ \
     if (__builtin_constant_p(bit) && (unsigned long)(bit) < 32) \
       { \
         asm volatile ("csrrc x0, " #reg ", %0" :: "i"(bit)); \
       } \
     else \
       { \
         asm volatile ("csrrc x0, " #reg ", %0" :: "r"(bit)); \
       } \
   })

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

static inline unsigned int irq_getie(void)
{
  return (csrr(mstatus) & CSR_MSTATUS_MIE) != 0;
}

static inline void irq_setie(unsigned int ie)
{
  if (ie)
    csrs(mstatus, CSR_MSTATUS_MIE);
  else
    csrc(mstatus, CSR_MSTATUS_MIE);
}

static inline unsigned int irq_getmask(void)
{
  unsigned int mask;
  asm volatile ("csrr %0, %1":"=r" (mask):"i"(CSR_IRQ_MASK));
  return mask;
}

static inline void irq_setmask(unsigned int mask)
{
  asm volatile ("csrw %0, %1"::"i" (CSR_IRQ_MASK), "r"(mask));
}

static inline unsigned int irq_pending(void)
{
  unsigned int pending;
  asm volatile ("csrr %0, %1":"=r" (pending):"i"(CSR_IRQ_PENDING));
  return pending;
}

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __ARCH_MISOC_SRC_MINERVA_CHIP_H */
