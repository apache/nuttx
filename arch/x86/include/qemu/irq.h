/****************************************************************************
 * arch/x86/include/qemu/irq.h
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
 * only indirectly through nuttx/irq.h
 */

#ifndef __ARCH_X86_INCLUDE_QEMU_IRQ_H
#define __ARCH_X86_INCLUDE_QEMU_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Definitions
 ****************************************************************************/

#define ISR0     0
#define ISR1     1
#define ISR2     2
#define ISR3     3
#define ISR4     4
#define ISR5     5
#define ISR6     6
#define ISR7     7
#define ISR8     8
#define ISR9     9
#define ISR10   10
#define ISR11   11
#define ISR12   12
#define ISR13   13
#define ISR14   14
#define ISR15   15
#define ISR16   16
#define ISR17   17
#define ISR18   18
#define ISR19   19
#define ISR20   20
#define ISR21   21
#define ISR22   22
#define ISR23   23
#define ISR24   24
#define ISR25   25
#define ISR26   26
#define ISR27   27
#define ISR28   28
#define ISR29   29
#define ISR30   30
#define ISR31   31

#define IRQ0    32
#define IRQ1    33
#define IRQ2    34
#define IRQ3    35
#define IRQ4    36
#define IRQ5    37
#define IRQ6    38
#define IRQ7    39
#define IRQ8    40
#define IRQ9    41
#define IRQ10   42
#define IRQ11   43
#define IRQ12   44
#define IRQ13   45
#define IRQ14   46
#define IRQ15   47

#define NR_IRQS 48

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Inline functions
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

#endif /* __ARCH_X86_INCLUDE_QEMU_IRQ_H */

