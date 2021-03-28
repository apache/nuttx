/****************************************************************************
 * arch/mips/include/pic32mx/irq.h
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
 * through nuttx/irq.h
 */

#ifndef __ARCH_MIPS_INCLUDE_PIC32MX_IRQ_H
#define __ARCH_MIPS_INCLUDE_PIC32MX_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <arch/pic32mx/chip.h>
#if defined(CHIP_PIC32MX1) || defined(CHIP_PIC32MX2)
#  include <arch/pic32mx/irq_1xx2xx.h>
#elif defined(CHIP_PIC32MX3) || defined(CHIP_PIC32MX4)
#  include <arch/pic32mx/irq_3xx4xx.h>
#elif defined(CHIP_PIC32MX5) || defined(CHIP_PIC32MX6) || defined(CHIP_PIC32MX7)
#  include <arch/pic32mx/irq_5xx6xx7xx.h>
#else
#  error "Unknown PIC32MX family
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
#endif /* __ARCH_MIPS_INCLUDE_PIC32MX_IRQ_H */
