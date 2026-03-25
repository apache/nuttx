/****************************************************************************
 * arch/arm/src/ht32f491x3/arm_vectors.c
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: 2012 Michael Smith. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include "chip.h"
#include "arm_internal.h"
#include "ram_vectors.h"
#include "nvic.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IDLE_STACK (_ebss + CONFIG_IDLETHREAD_STACKSIZE)

#define HT32_VECTOR_RESERVED ((const void *)(uintptr_t)UINT32_MAX)

#if defined(CONFIG_ARCH_CHIP_HT32F49153)
#  define HT32_VECTOR_FWID ((const void *)(uintptr_t)0x00049153u)
#elif defined(CONFIG_ARCH_CHIP_HT32F49163)
#  define HT32_VECTOR_FWID ((const void *)(uintptr_t)0x00049163u)
#else
#  error "Unsupported HT32F491x3 device"
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

extern void __start(void);

static void start(void)
{
  /* Zero lr to mark the end of backtrace */

  asm volatile ("mov lr, #0\n\t"
                "b  __start\n\t");
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

extern void exception_common(void);
extern void exception_direct(void);

/****************************************************************************
 * Public data
 ****************************************************************************/

/* Holtek reserves vector slots 7/9/10/13 and uses slot 8 for a device FWID.
 * Keep the NuttX exception routing semantics for all real exceptions/IRQs.
 */

const void * const _vectors[] locate_data(".vectors")
                              aligned_data(VECTAB_ALIGN) =
{
  IDLE_STACK,
  start,
  [NVIC_IRQ_NMI ... NVIC_IRQ_USAGEFAULT] = &exception_common,
  [7]                                    = HT32_VECTOR_RESERVED,
  [8]                                    = HT32_VECTOR_FWID,
  [9 ... 10]                             = HT32_VECTOR_RESERVED,
  [NVIC_IRQ_SVCALL ... NVIC_IRQ_DBGMONITOR] = &exception_common,
  [13]                                   = HT32_VECTOR_RESERVED,
  [NVIC_IRQ_PENDSV]                      = &exception_common,
  [NVIC_IRQ_SYSTICK ... (15 + ARMV7M_PERIPHERAL_INTERRUPTS)]
                                         = &exception_direct
};
