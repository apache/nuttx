/****************************************************************************
 * arch/misoc/src/minerva/chip.h
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

#ifndef __ARCH_MISOC_SRC_MINERVA_CHIP_H
#define __ARCH_MISOC_SRC_MINERVA_CHIP_H

/****************************************************************************
 * Included Files
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
