/****************************************************************************
 * arch/x86/include/i486/io.h
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

#ifndef __ARCH_X86_INCLUDE_I486_IO_H
#define __ARCH_X86_INCLUDE_I486_IO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

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
#endif /* __ARCH_X86_INCLUDE_I486_IO_H */
