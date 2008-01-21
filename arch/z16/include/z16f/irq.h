/****************************************************************************
 * arch/z16f/irq.h
 * arch/chip/irq.h
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
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
 * only indirectly through nuttx/irq.h (via arch/irq.h)
 */

#ifndef __ARCH_Z16F_IRQ_H
#define __ARCH_Z16F_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Interrupt Vectors */

#define Z16F_IRQ_SYSEXC   ( 0) /* Vector: 0x08 System Exceptions */

#define Z16F_IRQ_IRQ0     ( 1) /* First of 8 IRQs controlled by IRQ0 registers */
#define Z16F_IRQ_ADC      ( 1) /*   Vector: 0x2C IRQ0.0 ADC */
#define Z16F_IRQ_SPI      ( 2) /*   Vector: 0x28 IRQ0.1 SPI */
#define Z16F_IRQ_I2C      ( 3) /*   Vector: 0x24 IRQ0.2 I2C */
#define Z16F_IRQ_UART0TX  ( 4) /*   Vector: 0x20 IRQ0.3 UART0 TX */
#define Z16F_IRQ_UART0RX  ( 5) /*   Vector: 0x1C IRQ0.4 UART0 RX */
#define Z16F_IRQ_TIMER0   ( 6) /*   Vector: 0x18 IRQ0.5 Timer 0 */
#define Z16F_IRQ_TIMER1   ( 7) /*   Vector: 0x14 IRQ0.6 Timer 1 */
#define Z16F_IRQ_TIMER2   ( 8) /*   Vector: 0x10 IRQ0.7 Timer 2 */

#define Z16F_IRQ_IRQ1     ( 9) /* First of 8 IRQs controlled by IRQ1 registers */
#define Z16F_IRQ_P0AD     ( 9) /*   Vector: 0x4C IRQ1.0 Port A/D0, rising/falling edge */
#define Z16F_IRQ_P1AD     (10) /*   Vector: 0x48 IRQ1.1 Port A/D1, rising/falling edge */
#define Z16F_IRQ_P2AD     (11) /*   Vector: 0x44 IRQ1.2 Port A/D2, rising/falling edge */
#define Z16F_IRQ_P3AD     (12) /*   Vector: 0x40 IRQ1.3 Port A/D3, rising/falling edge */
#define Z16F_IRQ_P4AD     (13) /*   Vector: 0x3C IRQ1.4 Port A/D4, rising/falling edge */
#define Z16F_IRQ_P5AD     (14) /*   Vector: 0x38 IRQ1.5 Port A/D5, rising/falling edge */
#define Z16F_IRQ_P6AD     (15) /*   Vector: 0x34 IRQ1.6 Port A/D6, rising/falling edge */
#define Z16F_IRQ_P7AD     (16) /*   Vector: 0x30 IRQ1.7 Port A/D7, rising/falling edge */

#define Z16F_IRQ_IRQ2     (17) /* First of 8 IRQs controlled by IRQ2 registers */
#define Z16F_IRQ_C0       (17) /*   Vector: IRQ2.0 0x6C Port C0, both edges DMA3 */
#define Z16F_IRQ_C1       (18) /*   Vector: IRQ2.1 0x68 Port C1, both edges DMA3 */
#define Z16F_IRQ_C2       (19) /*   Vector: IRQ2.2 0x64 Port C2, both edges DMA3 */
#define Z16F_IRQ_C3       (20) /*   Vector: IRQ2.3 0x60 Port C3, both edges DMA3 */
#define Z16F_IRQ_PWMFAULT (21) /*   Vector: IRQ2.4 0x5C PWM Fault */
#define Z16F_IRQ_UART1TX  (22) /*   Vector: IRQ2.5 0x58 UART1 TX */
#define Z16F_IRQ_UART1RX  (23) /*   Vector: IRQ2.6 0x54 UART1 RX */
#define Z16F_IRQ_PWMTIMER (24) /*   Vector: IRQ2.7 0x50 PWM Timer */

#define Z16F_IRQ_SYSTIMER  Z16F_IRQ_TIMER0
#define NR_IRQS           (25)

/* These macros will map an IRQ to a register bit position */

#define Z16F_IRQ0_BIT(i) (1 << ((i)-Z16F_IRQ_IRQ0))
#define Z16F_IRQ1_BIT(i) (1 << ((i)-Z16F_IRQ_IRQ1))
#define Z16F_IRQ2_BIT(i) (1 << ((i)-Z16F_IRQ_IRQ2))

/* IRQ Stack Frame Format
 *
 * This stack frame is created on each interrupt.  These registers are stored
 * in the TCB to many context switches.
 */

#define REG_R0              ( 0) /* 32-bits: R0 */
#define REG_R1              ( 2) /* 32-bits: R0 */
#define REG_R2              ( 4) /* 32-bits: R0 */
#define REG_R3              ( 6) /* 32-bits: R0 */
#define REG_R4              ( 8) /* 32-bits: R0 */
#define REG_R5              (10) /* 32-bits: R0 */
#define REG_R6              (12) /* 32-bits: R0 */
#define REG_R7              (14) /* 32-bits: R0 */
#define REG_R8              (16) /* 32-bits: R0 */
#define REG_R9              (18) /* 32-bits: R0 */
#define REG_R10             (20) /* 32-bits: R0 */
#define REG_R11             (22) /* 32-bits: R0 */
#define REG_R12             (24) /* 32-bits: R0 */
#define REG_R13             (26) /* 32-bits: R0 */
#define REG_R14             (28) /* 32-bits: R0 */
#define REG_R15             (30) /* 32-bits: R0 */
#define REG_PC              (32) /* 32-bits: Return PC */
#define REG_FLAGS           (34) /* 16-bits: Flags register (with 0x00 padding) */

#define XCPTCONTEXT_REGS    (35)
#define XCPTCONTEXT_SIZE    (2 * XCPTCONTEXT_REGS)

#define REG_FP              REG_R14
#define REG_SP              REG_R15

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* This is the the type of the register save array */

typedef uint16 chipreg_t;

/* This struct defines the way the registers are stored. */

struct xcptcontext
{
  /* Register save area */

  uint16 regs[XCPTCONTEXT_REGS];

  /* The following function pointer is non-zero if there
   * are pending signals to be processed.
   */

#ifndef CONFIG_DISABLE_SIGNALS
  void *sigdeliver; /* Actual type is sig_deliver_t */

  /* The following retains that state during signal execution */

  uint32 saved_pc;	/* Saved return address */
  uint16 saved_i;	/* Saved interrupt state */
#endif
};
#endif

/* The ZDS-II provides built-in operations to test & disable and to restore
 * the interrupt state.
 *
 * irqstate_t irqsave(void);
 * void irqrestore(irqstate_t flags);
 */

#ifdef __ZILOG__
#  define irqsave()     TDI()
#  define irqrestore(f) RI(f)
#endif

/****************************************************************************
 * Inline functions
 ****************************************************************************/

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

#ifndef __ZILOG__
EXTERN irqstate_t irqsave(void);
EXTERN void       irqrestore(irqstate_t flags);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_Z16F_IRQ_H */

