/****************************************************************************
 * arch/risc-v/include/fe310/irq.h
 *
 *   Copyright (C) 2019 Masayuki Ishikawa. All rights reserved.
 *   Author: Masayuki Ishikawa <masayuki.ishikawa@gmail.com>
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

#ifndef __ARCH_RISCV_INCLUDE_FE310_IRQ_H
#define __ARCH_RISCV_INCLUDE_FE310_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <arch/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* In mstatus register */

#define MSTATUS_MIE   (0x1 << 3)  /* Machine Interrupt Enable */
#define MSTATUS_MPIE  (0x1 << 7)  /* Machine Previous Interrupt Enable */
#define MSTATUS_MPPM  (0x3 << 11) /* Machine Previous Privilege (m-mode) */

/* In mie (machine interrupt enable) register */

#define MIE_MTIE      (0x1 << 7)  /* Machine Timer Interrupt Enable */
#define MIE_MEIE      (0x1 << 11) /* Machine External Interrupt Enable */

/* Map RISC-V exception code to NuttX IRQ */

/* IRQ 0-15 : (exception:interrupt=0) */

#define FE310_IRQ_IAMISALIGNED  (0) /* Instruction Address Misaligned */
#define FE310_IRQ_IAFAULT       (1) /* Instruction Address Fault */
#define FE310_IRQ_IINSTRUCTION  (2) /* Illegal Instruction */
#define FE310_IRQ_BPOINT        (3) /* Break Point */
#define FE310_IRQ_LAMISALIGNED  (4) /* Load Address Misaligned */
#define FE310_IRQ_LAFAULT       (5) /* Load Access Fault */
#define FE310_IRQ_SAMISALIGNED  (6) /* Store/AMO Address Misaligned */
#define FE310_IRQ_SAFAULT       (7) /* Store/AMO Access Fault */
#define FE310_IRQ_ECALLU        (8) /* Environment Call from U-mode */
                                    /* 9-10: Reserved */

#define FE310_IRQ_ECALLM       (11) /* Environment Call from M-mode */
                                    /* 12-15: Reserved */

/* IRQ 16- : (async event:interrupt=1) */

#define FE310_IRQ_ASYNC        (16)
#define FE310_IRQ_MSOFT    (FE310_IRQ_ASYNC + 3)  /* Machine Software Int */
#define FE310_IRQ_MTIMER   (FE310_IRQ_ASYNC + 7)  /* Machine Timer Int */
#define FE310_IRQ_MEXT     (FE310_IRQ_ASYNC + 11) /* Machine External Int */

/* Machine Global External Interrupt */

#define FE310_IRQ_UART0    (FE310_IRQ_MEXT + 3)
#define FE310_IRQ_UART1    (FE310_IRQ_MEXT + 4)

#define FE310_IRQ_GPIO0    (FE310_IRQ_MEXT + 8)
#define FE310_IRQ_GPIO1    (FE310_IRQ_MEXT + 9)
#define FE310_IRQ_GPIO2    (FE310_IRQ_MEXT + 10)
#define FE310_IRQ_GPIO3    (FE310_IRQ_MEXT + 11)
#define FE310_IRQ_GPIO4    (FE310_IRQ_MEXT + 12)
#define FE310_IRQ_GPIO5    (FE310_IRQ_MEXT + 13)
#define FE310_IRQ_GPIO6    (FE310_IRQ_MEXT + 14)
#define FE310_IRQ_GPIO7    (FE310_IRQ_MEXT + 15)
#define FE310_IRQ_GPIO8    (FE310_IRQ_MEXT + 16)
#define FE310_IRQ_GPIO9    (FE310_IRQ_MEXT + 17)
#define FE310_IRQ_GPIO10   (FE310_IRQ_MEXT + 18)
#define FE310_IRQ_GPIO11   (FE310_IRQ_MEXT + 19)
#define FE310_IRQ_GPIO12   (FE310_IRQ_MEXT + 20)
#define FE310_IRQ_GPIO13   (FE310_IRQ_MEXT + 21)
#define FE310_IRQ_GPIO14   (FE310_IRQ_MEXT + 22)
#define FE310_IRQ_GPIO15   (FE310_IRQ_MEXT + 23)
#define FE310_IRQ_GPIO16   (FE310_IRQ_MEXT + 24)
#define FE310_IRQ_GPIO17   (FE310_IRQ_MEXT + 25)
#define FE310_IRQ_GPIO18   (FE310_IRQ_MEXT + 26)
#define FE310_IRQ_GPIO19   (FE310_IRQ_MEXT + 27)
#define FE310_IRQ_GPIO20   (FE310_IRQ_MEXT + 28)
#define FE310_IRQ_GPIO21   (FE310_IRQ_MEXT + 29)
#define FE310_IRQ_GPIO22   (FE310_IRQ_MEXT + 30)
#define FE310_IRQ_GPIO23   (FE310_IRQ_MEXT + 31)
#define FE310_IRQ_GPIO24   (FE310_IRQ_MEXT + 32)
#define FE310_IRQ_GPIO25   (FE310_IRQ_MEXT + 33)
#define FE310_IRQ_GPIO26   (FE310_IRQ_MEXT + 34)
#define FE310_IRQ_GPIO27   (FE310_IRQ_MEXT + 35)
#define FE310_IRQ_GPIO28   (FE310_IRQ_MEXT + 36)
#define FE310_IRQ_GPIO29   (FE310_IRQ_MEXT + 37)
#define FE310_IRQ_GPIO30   (FE310_IRQ_MEXT + 38)
#define FE310_IRQ_GPIO31   (FE310_IRQ_MEXT + 39)

/* Total number of IRQs */

#define NR_IRQS            (FE310_IRQ_GPIO31 + 1)

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

EXTERN irqstate_t  up_irq_save(void);
EXTERN void up_irq_restore(irqstate_t);
EXTERN irqstate_t up_irq_enable(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_INCLUDE_FE310_IRQ_H */
