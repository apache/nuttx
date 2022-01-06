/****************************************************************************
 * arch/misoc/src/common/hw/common.h
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

#ifndef __ARCH_MISOC_SRC_COMMON_HW_COMMON_H
#define __ARCH_MISOC_SRC_COMMON_HW_COMMON_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* To overwrite CSR accessors, define extern, non-inlined versions
 * of csr_read[bwl]() and csr_write[bwl](), and define
 * CSR_ACCESSORS_DEFINED.
 */

#ifndef CSR_ACCESSORS_DEFINED
#define CSR_ACCESSORS_DEFINED

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#ifdef __ASSEMBLER__
#  define MMPTR(x) x
#else
#  define MMPTR(x) (*((volatile unsigned int *)(x)))

static inline void csr_writeb(uint8_t value, uint32_t addr)
{
  *((volatile uint8_t *)addr) = value;
}

static inline uint8_t csr_readb(uint32_t addr)
{
  return *(volatile uint8_t *)addr;
}

static inline void csr_writew(uint16_t value, uint32_t addr)
{
  *((volatile uint16_t *)addr) = value;
}

static inline uint16_t csr_readw(uint32_t addr)
{
  return *(volatile uint16_t *)addr;
}

static inline void csr_writel(uint32_t value, uint32_t addr)
{
  *((volatile uint32_t *)addr) = value;
}

static inline uint32_t csr_readl(uint32_t addr)
{
  return *(volatile uint32_t *)addr;
}

#endif /* !__ASSEMBLER__ */
#endif /* !CSR_ACCESSORS_DEFINED */
#endif /* __ARCH_MISOC_SRC_COMMON_HW_COMMON_H */
