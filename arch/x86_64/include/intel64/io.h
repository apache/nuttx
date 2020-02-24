/****************************************************************************
 * arch/x86_64/include/intel64/io.h
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

/* This file should never be included directed but, rather, only indirectly
 * through arch/io.h
 */

#ifndef __ARCH_X86_64_INCLUDE_INTEL64_IO_H
#define __ARCH_X86_64_INCLUDE_INTEL64_IO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PAGE_SIZE		(4 * 1024ULL)
#define BITS_PER_LONG		64
#define HUGE_PAGE_SIZE		(2 * 1024 * 1024ULL)
#define PAGE_MASK		(~(PAGE_SIZE - 1))
#define HUGE_PAGE_MASK		(~(HUGE_PAGE_SIZE - 1))

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Inline functions
 ****************************************************************************/

/* Standard x86 Port I/O */

static inline void outb(uint8_t regval, uint16_t port)
{
  asm volatile(
    "\toutb %0,%1\n"
    :
    : "a" (regval), "dN" (port)
    );
}

static inline uint8_t inb(uint16_t port)
{
  uint8_t regval;
  asm volatile(
    "\tinb %1,%0\n"
    : "=a" (regval)
    : "dN" (port)
  );
  return regval;
}

static inline void outw(uint16_t regval, uint16_t port)
{
  asm volatile(
    "\toutw %0,%1\n"
    :
    : "a" (regval), "dN" (port)
    );
}

static inline uint16_t inw(uint16_t port)
{
  uint16_t regval;

  asm volatile(
    "\tinw %1,%0\n"
    : "=a" (regval)
    : "dN" (port)
    );
  return regval;
}

static inline void outl(uint32_t regval, uint16_t port)
{
  asm volatile(
    "\toutl %0,%1\n"
    :
    : "a" (regval), "dN" (port)
    );
}

static inline uint32_t inl(uint16_t port)
{
  uint32_t regval;
  asm volatile(
    "\tinl %1,%0\n"
    : "=a" (regval)
    : "dN" (port)
    );
  return regval;
}

/* MMIO */

static inline uint8_t mmio_read8(void *address)
{
	return *(volatile uint8_t *)address;
}

static inline uint16_t mmio_read16(void *address)
{
	return *(volatile uint16_t *)address;
}

static inline uint32_t mmio_read32(void *address)
{
	uint32_t value;

	/* assembly-encoded to match the hypervisor MMIO parser support */
	asm volatile("movl (%1),%0" : "=r" (value) : "r" (address));
	return value;
}

static inline uint64_t mmio_read64(void *address)
{
	return *(volatile uint64_t *)address;
}

static inline void mmio_write8(void *address, uint8_t value)
{
	*(volatile uint8_t *)address = value;
}

static inline void mmio_write16(void *address, uint16_t value)
{
	*(volatile uint16_t *)address = value;
}

static inline void mmio_write32(void *address, uint32_t value)
{
	/* assembly-encoded to match the hypervisor MMIO parser support */
	asm volatile("movl %0,(%1)" : : "r" (value), "r" (address));
}

static inline void mmio_write64(void *address, uint64_t value)
{
	*(volatile uint64_t *)address = value;
}

static inline void up_trash_cpu(void)
{
  asm("mov $0, %%al":::);
  asm("mov $0xfa, %%dx":::);
  asm("out %%al, (%%dx)":::);
}

static inline void up_invalid_TLB(uintptr_t start, uintptr_t end)
{
  start = start & PAGE_MASK;
  end = (end + PAGE_SIZE - 1) & PAGE_MASK;
  for(uintptr_t i = start; i < end; i += PAGE_SIZE)
    {
      asm("invlpg %0;":: "m"(i):"memory");
    }
}

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_X86_64_INCLUDE_INTEL64_IO_H */
