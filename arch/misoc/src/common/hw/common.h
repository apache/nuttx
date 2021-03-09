/****************************************************************************
 * arch/misoc/src/common/hw/common.h
 *
 *   Copyright (C) 2016, 2019 Gregory Nutt. All rights reserved.
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
