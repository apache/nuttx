/************************************************************************************
 * arch/x86_64/src/intel64/intel64.h
 *
 *   Copyright (C) 2011, 2015-2016 Gregory Nutt,
 *                 2020 Chung-Fan Yang.
 *   All rights reserved.
 *
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           Chung-Fan Yang <sonic.tw.tp@gmail.com>
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

#ifndef __ARCH_X86_64_SRC_INTEL64_INTEL64_H
#define __ARCH_X86_64_SRC_INTEL64_INTEL64_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

#include "up_internal.h"
#include "chip.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Configuration ********************************************************************/

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

#ifndef __ASSEMBLY__

/************************************************************************************
 * Public Data
 ************************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

/************************************************************************************
 * Name: intel64_lowsetup
 *
 * Description:
 *   Called at the very beginning of _start.  Performs low level initialization
 *   including setup of the console UART.  This UART done early so that the serial
 *   console is available for debugging very early in the boot sequence.
 *
 ************************************************************************************/

void intel64_lowsetup(void);

/****************************************************************************
 * Name: vector_*
 *
 * Description:
 *   These are the various ISR/IRQ vector address exported from
 *   intel64_vectors.S.  These addresses need to have global scope so that they
 *   can be known to the interrupt initialization logic in intel64_irq.c.
 *
 ****************************************************************************/

void vector_isr0(void);
void vector_isr1(void);
void vector_isr2(void);
void vector_isr3(void);
void vector_isr4(void);
void vector_isr5(void);
void vector_isr6(void);
void vector_isr7(void);
void vector_isr8(void);
void vector_isr9(void);
void vector_isr10(void);
void vector_isr11(void);
void vector_isr12(void);
void vector_isr13(void);
void vector_isr14(void);
void vector_isr15(void);
void vector_isr16(void);
void vector_isr17(void);
void vector_isr18(void);
void vector_isr19(void);
void vector_isr20(void);
void vector_isr21(void);
void vector_isr22(void);
void vector_isr23(void);
void vector_isr24(void);
void vector_isr25(void);
void vector_isr26(void);
void vector_isr27(void);
void vector_isr28(void);
void vector_isr29(void);
void vector_isr30(void);
void vector_isr31(void);
void vector_irq0(void);
void vector_irq1(void);
void vector_irq2(void);
void vector_irq3(void);
void vector_irq4(void);
void vector_irq5(void);
void vector_irq6(void);
void vector_irq7(void);
void vector_irq8(void);
void vector_irq9(void);
void vector_irq10(void);
void vector_irq11(void);
void vector_irq12(void);
void vector_irq13(void);
void vector_irq14(void);
void vector_irq15(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_X86_64_SRC_INTEL64_INTEL64_H */
