/****************************************************************************
 * arch/xtensa/src/esp32/chip_macros.h
 *
 * Adapted from use in NuttX by:
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Derives from logic originally provided by Cadence Design Systems Inc.
 *
 *   Copyright (c) 2006-2015 Cadence Design Systems Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 ****************************************************************************/

#ifndef __ARCH_XTENSA_SRC_ESP32_CHIP_MACROS_H
#define __ARCH_XTENSA_SRC_ESP32_CHIP_MACROS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* This is the name of the section containing the Xtensa low level handlers
 * that is used by the board linker scripts.
 */

#define HANDLER_SECTION .iram1

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __ASSEMBLY__

#if defined(CONFIG_SMP) && CONFIG_ARCH_INTERRUPTSTACK > 15
  .global	g_cpu_intstack_top
#endif /* CONFIG_SMP && CONFIG_ARCH_INTERRUPTSTACK > 15 */

#endif /* __ASSEMBLY__ */

/****************************************************************************
 * Assembly Language Macros
 ****************************************************************************/

#ifdef __ASSEMBLY__

/* Macro to get the current core ID. Only uses the reg given as an argument.
 * Reading PRID on the ESP32 gives us 0xCDCD on the PRO processor (0)
 * and 0xABAB on the APP CPU (1). We can distinguish between the two by
 * checking bit 13: it's 1 on the APP and 0 on the PRO processor.
 */

    .macro getcoreid reg
    rsr.prid \reg
    extui \reg,\reg,13,1
    .endm

/****************************************************************************
 * Name: setintstack
 *
 * Description:
 *   Set the current stack pointer to the "top" of the correct interrupt
 *   stack for the current CPU.
 *
 ****************************************************************************/

#if defined(CONFIG_SMP) && CONFIG_ARCH_INTERRUPTSTACK > 15
    .macro setintstack tmp1 tmp2
    getcoreid \tmp1                   /* tmp1 = Core ID (0 or 1) */
    movi  \tmp2, g_cpu_intstack_top   /* tmp2 = Array of stack pointers */
    addx4 \tmp2, \tmp1, \tmp2         /* tmp2 = tmp2 + (tmp1 << 2) */
    l32i  a1, \tmp2, 0                /* a1   = *tmp2 */
    .endm
#endif
#endif /* __ASSEMBLY */

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#if defined(CONFIG_SMP) && CONFIG_ARCH_INTERRUPTSTACK > 15
uintptr_t xtensa_intstack_alloc(void);
uintptr_t xtensa_intstack_top(void);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_XTENSA_SRC_ESP32_CHIP_MACROS_H */
