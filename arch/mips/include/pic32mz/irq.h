/****************************************************************************
 * arch/mips/include/pic32mz/irq.h
 *
 *   Copyright (C) 2015, 2018 Gregory Nutt. All rights reserved.
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

/* This file should never be included directly but, rather, only indirectly
 * through nuttx/irq.h
 */

#ifndef __ARCH_MIPS_INCLUDE_PIC32MZ_IRQ_H
#define __ARCH_MIPS_INCLUDE_PIC32MZ_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/pic32mz/chip.h>

#if defined(CHIP_PIC32MZEC)
#  include <arch/pic32mz/irq_pic32mzxxxec.h>
#elif defined(CHIP_PIC32MZEF)
#  include <arch/pic32mz/irq_pic32mzxxxef.h>
#else
#  error "Unknown PIC32MZ family"
#endif

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

/****************************************************************************
 * Name: cp0_getintctl
 *
 * Description:
 *   Get the CP0 IntCtl register
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline uint32_t cp0_getintctl(void)
{
  register uint32_t intctl;
  __asm__ __volatile__
    (
      "\t.set    push\n"
      "\t.set    noat\n"
      "\t mfc0   %0, $12, 1\n"           /* Get CP0 IntCtl register */
      "\t.set    pop\n"
      : "=r" (intctl)
      :
      : "memory"
    );

  return intctl;
}

/****************************************************************************
 * Name: cp0_putintctl
 *
 * Description:
 *   Write the CP0 IntCtl register
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void cp0_putintctl(uint32_t intctl)
{
  __asm__ __volatile__
    (
      "\t.set    push\n"
      "\t.set    noat\n"
      "\t.set    noreorder\n"
      "\tmtc0   %0, $12, 1\n"             /* Set the IntCtl to the provided value */
      "\t.set    pop\n"
      :
      : "r" (intctl)
      : "memory"
    );
}

/****************************************************************************
 * Name: cp0_getebase
 *
 * Description:
 *   Get the CP0 EBASE register
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline uint32_t cp0_getebase(void)
{
  register uint32_t ebase;
  __asm__ __volatile__
    (
      "\t.set    push\n"
      "\t.set    noat\n"
      "\t mfc0   %0, $15, 1\n"           /* Get CP0 EBASE register */
      "\t.set    pop\n"
      : "=r" (ebase)
      :
      : "memory"
    );

  return ebase;
}

/****************************************************************************
 * Name: cp0_putebase
 *
 * Description:
 *   Write the CP0 EBASE register
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void cp0_putebase(uint32_t ebase)
{
  __asm__ __volatile__
    (
      "\t.set    push\n"
      "\t.set    noat\n"
      "\t.set    noreorder\n"
      "\tmtc0   %0, $15, 1\n"             /* Set the EBASE to the provided value */
      "\t.set    pop\n"
      :
      : "r" (ebase)
      : "memory"
    );
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
#endif /* __ARCH_MIPS_INCLUDE_PIC32MZ_IRQ_H */
