/****************************************************************************
 * arch/x86_64/include/intel64/io.h
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
 * through arch/io.h
 */

#ifndef __ARCH_X86_64_INCLUDE_INTEL64_IO_H
#define __ARCH_X86_64_INCLUDE_INTEL64_IO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <arch/arch.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

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

  /* Assembly-encoded to match the hypervisor MMIO parser support */

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
  /* Assembly-encoded to match the hypervisor MMIO parser support */

  asm volatile("movl %0,(%1)" : : "r" (value), "r" (address));
}

static inline void mmio_write64(void *address, uint64_t value)
{
  *(volatile uint64_t *)address = value;
}

static inline void up_trash_cpu(void)
{
  for (; ; )
    {
      asm volatile ("cli;hlt;");
    }

  asm("ud2":::"memory");
}

static inline void up_invalid_TLB(uintptr_t start, uintptr_t end)
{
  uintptr_t i;

  start = start & PAGE_MASK;
  end = (end + PAGE_SIZE - 1) & PAGE_MASK;

  for (i = start; i < end; i += PAGE_SIZE)
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
