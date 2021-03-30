/****************************************************************************
 * arch/z16/include/z16f/irq.h
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

/* This file should never be included directly but, rather,
 * only indirectly through nuttx/irq.h (via arch/irq.h)
 */

#ifndef __ARCH_Z16_INCLUDE_Z16F_IRQ_H
#define __ARCH_Z16_INCLUDE_Z16F_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Interrupt Vectors
 * (excluding reset and sysexec which are handled differently)
 */

#define Z16F_IRQ_IRQ0     ( 0) /* First of 8 IRQs controlled by IRQ0 registers */
#define Z16F_IRQ_ADC      ( 0) /*   Vector: 0x2C IRQ0.0 ADC */
#define Z16F_IRQ_SPI      ( 1) /*   Vector: 0x28 IRQ0.1 SPI */
#define Z16F_IRQ_I2C      ( 2) /*   Vector: 0x24 IRQ0.2 I2C */
#define Z16F_IRQ_UART0TX  ( 3) /*   Vector: 0x20 IRQ0.3 UART0 TX */
#define Z16F_IRQ_UART0RX  ( 4) /*   Vector: 0x1C IRQ0.4 UART0 RX */
#define Z16F_IRQ_TIMER0   ( 5) /*   Vector: 0x18 IRQ0.5 Timer 0 */
#define Z16F_IRQ_TIMER1   ( 6) /*   Vector: 0x14 IRQ0.6 Timer 1 */
#define Z16F_IRQ_TIMER2   ( 7) /*   Vector: 0x10 IRQ0.7 Timer 2 */

#define Z16F_IRQ_IRQ1     ( 8) /* First of 8 IRQs controlled by IRQ1 registers */
#define Z16F_IRQ_P0AD     ( 8) /*   Vector: 0x4C IRQ1.0 Port A/D0, rising/falling edge */
#define Z16F_IRQ_P1AD     ( 9) /*   Vector: 0x48 IRQ1.1 Port A/D1, rising/falling edge */
#define Z16F_IRQ_P2AD     (10) /*   Vector: 0x44 IRQ1.2 Port A/D2, rising/falling edge */
#define Z16F_IRQ_P3AD     (11) /*   Vector: 0x40 IRQ1.3 Port A/D3, rising/falling edge */
#define Z16F_IRQ_P4AD     (12) /*   Vector: 0x3C IRQ1.4 Port A/D4, rising/falling edge */
#define Z16F_IRQ_P5AD     (13) /*   Vector: 0x38 IRQ1.5 Port A/D5, rising/falling edge */
#define Z16F_IRQ_P6AD     (14) /*   Vector: 0x34 IRQ1.6 Port A/D6, rising/falling edge */
#define Z16F_IRQ_P7AD     (15) /*   Vector: 0x30 IRQ1.7 Port A/D7, rising/falling edge */

#define Z16F_IRQ_IRQ2     (16) /* First of 8 IRQs controlled by IRQ2 registers */
#define Z16F_IRQ_C0       (16) /*   Vector: IRQ2.0 0x6C Port C0, both edges DMA0 */
#define Z16F_IRQ_C1       (17) /*   Vector: IRQ2.1 0x68 Port C1, both edges DMA1 */
#define Z16F_IRQ_C2       (18) /*   Vector: IRQ2.2 0x64 Port C2, both edges DMA2 */
#define Z16F_IRQ_C3       (19) /*   Vector: IRQ2.3 0x60 Port C3, both edges DMA3 */
#define Z16F_IRQ_PWMFAULT (20) /*   Vector: IRQ2.4 0x5C PWM Fault */
#define Z16F_IRQ_UART1TX  (21) /*   Vector: IRQ2.5 0x58 UART1 TX */
#define Z16F_IRQ_UART1RX  (22) /*   Vector: IRQ2.6 0x54 UART1 RX */
#define Z16F_IRQ_PWMTIMER (23) /*   Vector: IRQ2.7 0x50 PWM Timer */

#define Z16F_IRQ_SYSTIMER  Z16F_IRQ_TIMER0
#define NR_IRQS           (24)

/* These macros will map an IRQ to a register bit position */

#define Z16F_IRQ0_BIT(i) (1 << ((i)-Z16F_IRQ_IRQ0))
#define Z16F_IRQ1_BIT(i) (1 << ((i)-Z16F_IRQ_IRQ1))
#define Z16F_IRQ2_BIT(i) (1 << ((i)-Z16F_IRQ_IRQ2))

/* IRQ Stack Frame Format
 *
 * This stack frame is created on each interrupt.  These registers are stored
 * in the TCB to many context switches.
 *
 * The following represent all of the "static" registers r8-r15.  These
 * are registers that whose value must be retained across function calls.
 * These registers must be saved bothby interrupt handling context switch
 * switch logic and also by user-initiated context switches.
 *
 * Registers are saved in the order consistent with pushmho <r8-r15>,
 * that is with r15 pushed first and r8 push last.  Since the z16f has
 * a "push-down" stack, the registers will be "in order" in memory.
 */

#define REG_R8              ( 0) /* 32-bits: R8 */
#define REG_R9              ( 2) /* 32-bits: R9 */
#define REG_R10             ( 4) /* 32-bits: R10 */
#define REG_R11             ( 6) /* 32-bits: R11 */
#define REG_R12             ( 8) /* 32-bits: R12 */
#define REG_R13             (10) /* 32-bits: R13 */

/* The frame pointer and the SP at the point of task resumption must
 * always be saved.
 */

#define REG_R14             (12) /* 32-bits: R14 = fp */
#define REG_FP              REG_R14
#define REG_R15             (14) /* 32-bits: R15 = sp */
#define REG_SP              REG_R15

/* The following represent all of the "volatile" registers r0-r7.  These
 * are registers that whose value need not be retained across function
 * calls.  These registers must be saved by interrupt handling context
 * switch logic but not by user-initiated context switches.
 *
 * Registers are saved in the order consistent with pushmlo <r0-r7>,
 * that is with r7 pushed first and r0 push last.  Since the z16f has
 * a "push-down" stack, the registers will be "in order" in memory.
 */

#define REG_R0              (16) /* 32-bits: R0 */
#define REG_R1              (18) /* 32-bits: R1 */
#define REG_R2              (20) /* 32-bits: R2 */
#define REG_R3              (22) /* 32-bits: R3 */
#define REG_R4              (24) /* 32-bits: R4 */
#define REG_R5              (26) /* 32-bits: R5 */
#define REG_R6              (28) /* 32-bits: R6 */
#define REG_R7              (30) /* 32-bits: R7 */

/* The following two offsets represent the state of the stack on entry
 * into the interrupt handler:
 *
 * TOS[0] = PC[31:24]
 * TOS[1] = PC[23:16]
 * TOS[2] = PC[15:8]
 * TOS[3] = PC[7:0]
 * TOS[4] = 0
 * TOS[5] = flags
 */

#define REG_PC              (32) /* 32-bits: Return PC */
#define REG_FLAGS           (34) /* 16-bits: Flags register (with 0x00 padding) */

#define XCPTCONTEXT_REGS    (35)
#define XCPTCONTEXT_SIZE    (2 * XCPTCONTEXT_REGS)

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* This is the type of the register save array */

typedef uint16_t chipreg_t;

/* This struct defines the way the registers are stored. */

struct xcptcontext
{
  /* Register save area */

  uint16_t regs[XCPTCONTEXT_REGS];

  /* The following function pointer is non-zero if there
   * are pending signals to be processed.
   */

  CODE void *sigdeliver; /* Actual type is sig_deliver_t */

  /* The following retains that state during signal execution.
   *
   * REVISIT:  Because there is only one copy of these save areas,
   * only a single signal handler can be active.  This precludes
   * queuing of signal actions.  As a result, signals received while
   * another signal handler is executing will be ignored!
   */

  uint32_t saved_pc;  /* Saved return address */
  uint16_t saved_i;   /* Saved interrupt state */
};
#endif

/* The ZDS-II provides built-in operations to test & disable and to restore
 * the interrupt state.
 *
 *   irqstate_t up_irq_save(void);
 *   void up_irq_restore(irqstate_t flags);
 *   void up_irq_enable(void);
 *
 * NOTE: These functions should never be called from application code and,
 * as a general rule unless you really know what you are doing, this
 * function should not be called directly from operation system code either:
 * Typically, the wrapper functions, enter_critical_section() and
 * leave_critical section(), are probably what you really want.
 */

#ifdef __ZILOG__
#  define up_irq_save()     TDI()
#  define up_irq_restore(f) RI(f)
#  define up_irq_enable()   EI()
#endif

/****************************************************************************
 * Inline functions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/* ZDS-II intrinsic functions (normally declared in zneo.h) */

intrinsic void EI(void);
intrinsic void DI(void);
intrinsic void RI(unsigned short);
intrinsic void SET_VECTOR(int, void (* func) (void));
intrinsic unsigned short TDI(void);

#ifndef __ZILOG__
irqstate_t up_irq_save(void);
void       up_irq_restore(irqstate_t flags);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_Z16_INCLUDE_Z16F_IRQ_H */
