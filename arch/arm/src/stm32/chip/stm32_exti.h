/************************************************************************************
 * arch/arm/src/stm32/chip/stm32_exti.h
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32_CHIP_STM32_EXTI_H
#define __ARCH_ARM_SRC_STM32_CHIP_STM32_EXTI_H

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#ifdef CONFIG_STM32_CONNECTIVITYLINE
#  define STM32_NEXTI            20
#  define STM32_EXTI_MASK        0x000fffff
#else
#  define STM32_NEXTI            19
#  define STM32_EXTI_MASK        0x0007ffff
#endif

#define STM32_EXTI_BIT(n)        (1 << (n))

/* Register Offsets *****************************************************************/

#define STM32_EXTI_IMR_OFFSET    0x0000  /* Interrupt mask register */
#define STM32_EXTI_EMR_OFFSET    0x0004  /* Event mask register */
#define STM32_EXTI_RTSR_OFFSET   0x0008  /* Rising Trigger selection register */
#define STM32_EXTI_FTSR_OFFSET   0x000c  /* Falling Trigger selection register */
#define STM32_EXTI_SWIER_OFFSET  0x0010  /* Software interrupt event register */
#define STM32_EXTI_PR_OFFSET     0x0014  /* Pending register */

/* Register Addresses ***************************************************************/

#define STM32_EXTI_IMR          (STM32_EXTI_BASE+STM32_EXTI_IMR_OFFSET)
#define STM32_EXTI_EMR          (STM32_EXTI_BASE+STM32_EXTI_EMR_OFFSET)
#define STM32_EXTI_RTSR         (STM32_EXTI_BASE+STM32_EXTI_RTSR_OFFSET)
#define STM32_EXTI_FTSR         (STM32_EXTI_BASE+STM32_EXTI_FTSR_OFFSET)
#define STM32_EXTI_SWIER        (STM32_EXTI_BASE+STM32_EXTI_SWIER_OFFSET)
#define STM32_EXTI_PR           (STM32_EXTI_BASE+STM32_EXTI_PR_OFFSET)

/* Register Bitfield Definitions ****************************************************/

/* Interrupt mask register */

#define EXTI_IMR_BIT(n)          STM32_EXTI_BIT(n)
#define EXTI_IMR_SHIFT           (0)     /* Bits 18/19-0: Interrupt Mask on line n */
#define EXTI_IMR_MASK            STM32_EXTI_MASK

/* Event mask register */

#define EXTI_EMR_BIT(n)          STM32_EXTI_BIT(n)
#define EXTI_EMR_SHIFT           (0)     /* Bits 18/19-0:  Event Mask on line n */
#define EXTI_EMR_MASK            STM32_EXTI_MASK

/* Rising Trigger selection register */

#define EXTI_RTSR_BIT(n)         STM32_EXTI_BIT(n)
#define EXTI_RTSR_SHIFT          (0)     /* Bits 18/19-0: Rising trigger event configuration bit of line n */
#define EXTI_RTSR_MASK           STM32_EXTI_MASK

/* Falling Trigger selection register */

#define EXTI_FTSR_BIT(n)         STM32_EXTI_BIT(n)
#define EXTI_FTSR_SHIFT          (0)     /* Bits 18/19-0: Falling trigger event configuration bit of line n */
#define EXTI_FTSR_MASK           STM32_EXTI_MASK

/* Software interrupt event register  */

#define EXTI_SWIER_BIT(n)        STM32_EXTI_BIT(n)
#define EXTI_SWIER_SHIFT         (0)     /*  Bits 18/19-0: Software Interrupt on line n */
#define EXTI_SWIER_MASK          STM32_EXTI_MASK

/* Pending register */

#define EXTI_IMR_BIT(n)          STM32_EXTI_BIT(n)
#define EXTI_IMR_SHIFT           (0)     /* Bits 18/19-0: Pending bit on line x */
#define EXTI_IMR_MASK            STM32_EXTI_MASK

#endif /* __ARCH_ARM_SRC_STM32_CHIP_STM32_EXTI_H */
