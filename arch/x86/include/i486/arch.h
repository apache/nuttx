/****************************************************************************
 * arch/x86/include/i486/arch.h
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

/* This file should never be included directed but, rather,
 * only indirectly through nuttx/arch.h
 */

#ifndef __ARCH_X86_INCLUDE_I486_ARCH_H
#define __ARCH_X86_INCLUDE_I486_ARCH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* FLAGS bits */

#define X86_FLAGS_CF         (1 << 0)  /* Bit 0:  Carry Flag */
                                       /* Bit 1:  Reserved */
#define X86_FLAGS_PF         (1 << 2)  /* Bit 2:  Parity Flag */
                                       /* Bit 3:  Reserved */
#define X86_FLAGS_AF         (1 << 4)  /* Bit 4:  Auxillary carry Flag */
                                       /* Bit 5:  Reserved */
#define X86_FLAGS_ZF         (1 << 6)  /* Bit 6:  Zero Flag */
#define X86_FLAGS_SF         (1 << 7)  /* Bit 7:  Sign Flag */
#define X86_FLAGS_TF         (1 << 8)  /* Bit 8:  Trap Flag */
#define X86_FLAGS_IF         (1 << 9)  /* Bit 9:  Interrupt Flag */
#define X86_FLAGS_DF         (1 << 10) /* Bit 10: Direction Flag */
#define X86_FLAGS_OF         (1 << 11) /* Bit 11: Overflow Flag */
#define X86_FLAGS_IOPL_SHIFT (12)      /* Bits 12-13: IOPL mask (286+ only)*/
#define X86_FLAGS_IOPL_MASK  (3 << X86_FLAGS_IOPL_SHIFT)
#define X86_FLAGS_NT         (1 << 14) /* Bit 14: Nested Task */
                                       /* Bit 15: Reserved */

/* EFLAGS bits (Extend the basic FLAGS bit definitions) */

#define X86_EFLAGS_RF        (1 << 16) /* Bit 16: Resume Flag (386+ only) */
#define X86_EFLAGS_VM        (1 << 17) /* Bit 17: Virtual Mode (386+ only) */
#define X86_EFLAGS_AC        (1 << 18) /* Bit 18: Alignment Check (486SX+ only) */
#define X86_EFLAGS_VIF       (1 << 19) /* Bit 19: Virtual Interrupt Flag (Pentium+) */
#define X86_EFLAGS_VIP       (1 << 20) /* Bit 20: Virtual Interrupt Pending (Pentium+) */
#define X86_EFLAGS_ID        (1 << 21) /* Bit 21: CPUID detection flag (Pentium+) */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* GDT data structures
 *
 * The Global Descriptor Table or GDT is a data structure used by Intel x86-
 * family processors starting with the 80286 in order to define the
 * characteristics of the various memory areas used during program execution, 
 * for example the base address, the size and access privileges like
 * executability and writability. These memory areas are called segments in
 * Intel terminology.
 */

/* This structure defines one segment */

struct gdt_entry_s
{
  uint16_t lowlimit;           /* The lower 16 bits of the limit */
  uint16_t lowbase;            /* The lower 16 bits of the base */
  uint8_t  midbase;            /* The next 8 bits of the base */
  uint8_t  access;             /* Access flags, determine ring segment can be used in */
  uint8_t  granularity;
  uint8_t  hibase;             /* The last 8 bits of the base */
} __attribute__((packed));

/* This structure refers to the array of GDT entries, and is in the format
 * required by the lgdt instruction.
 */

struct gdt_ptr_s
{
  uint16_t limit;               /* The upper 16 bits of all selector limits */
  uint32_t base;                /* The address of the first gdt_entry_t struct */
} __attribute__((packed));

/****************************************************************************
 * Inline functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* Return stack pointer */

static inline uint32_t up_getsp()
{
  uint32_t regval;

  asm volatile(
    "\tmovl %%esp, %0\n"
    : "=rm" (regval)
    :
    : "memory");
  return regval;
}

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_X86_INCLUDE_I486_ARCH_H */

