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
 
#define Z16F_IRQ_RESET    ( 0) /* Vector: 0x04 Reset */
#define Z16F_IRQ_SYSEXC   ( 1) /* Vector: 0x08 Sysexec */
#define Z16F_IRQ_TIMER2   ( 2) /* Vector: 0x10 Timer 2 */
#define Z16F_IRQ_TIMER1   ( 3) /* Vector: 0x14 Timer 1 */
#define Z16F_IRQ_TIMER0   ( 4) /* Vector: 0x18 Timer 0 */
#define Z16F_IRQ_UART0RX  ( 5) /* Vector: 0x1C UART0 RX */
#define Z16F_IRQ_UART0TX  ( 6) /* Vector: 0x20 UART0 TX */
#define Z16F_IRQ_I2C      ( 7) /* Vector: 0x24 I2C */
#define Z16F_IRQ_SPI      ( 8) /* Vector: 0x28 SPI */
#define Z16F_IRQ_ADC      ( 9) /* Vector: 0x2C ADC */
#define Z16F_IRQ_P7AD     (10) /* Vector: 0x30 P7AD */
#define Z16F_IRQ_P6AD     (11) /* Vector: 0x34 P6AD */
#define Z16F_IRQ_P5AD     (12) /* Vector: 0x38 P5AD */
#define Z16F_IRQ_P4AD     (13) /* Vector: 0x3C P4AD */
#define Z16F_IRQ_P3AD     (14) /* Vector: 0x40 P3AD */
#define Z16F_IRQ_P2AD     (15) /* Vector: 0x44 P2AD */
#define Z16F_IRQ_P1AD     (16) /* Vector: 0x48 P1AD */
#define Z16F_IRQ_P0AD     (17) /* Vector: 0x4C P0AD */
#define Z16F_IRQ_PWMTIMER (18) /* Vector: 0x50 PWM Timer */
#define Z16F_IRQ_UART1RX  (19) /* Vector: 0x54 UART1 RX */
#define Z16F_IRQ_UART1TX  (20) /* Vector: 0x58 UART1 TX */
#define Z16F_IRQ_PWMFAULT (21) /* Vector: 0x5C PWM Fault */
#define Z16F_IRQ_C3       (22) /* Vector: 0x60 C3 */
#define Z16F_IRQ_C2       (23) /* Vector: 0x64 C2 */
#define Z16F_IRQ_C1       (24) /* Vector: 0x68 C1 */
#define Z16F_IRQ_C0       (25) /* Vector: 0x6C C0 */

#define Z16F_IRQ_SYSTIMER  Z16F_IRQ_TIMER0
#define NR_IRQS           (26)

/* IRQ Stack Frame Format
 *
 * This stack frame is created on each interrupt.  These registers are stored
 * in the TCB to many context switches.
 */

#define XCPT_I               (0) /* Offset 0: Saved I w/interrupt state in carry */
#define XCPT_BC              (1) /* Offset 1: Saved BC register */
#define XCPT_DE              (2) /* Offset 2: Saved DE register */
#define XCPT_IX              (3) /* Offset 3: Saved IX register */
#define XCPT_IY              (4) /* Offset 4: Saved IY register */
#define XCPT_SP              (5) /* Offset 5: Offset to SP at time of interrupt */
#define XCPT_HL              (6) /* Offset 6: Saved HL register */
#define XCPT_AF              (7) /* Offset 7: Saved AF register */
#define XCPT_PC              (8) /* Offset 8: Offset to PC at time of interrupt */

#define XCPTCONTEXT_REGS     (9)
#define XCPTCONTEXT_SIZE     (2 * XCPTCONTEXT_REGS)

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

  uint16 saved_pc;	/* Saved return address */
  uint16 saved_i;	/* Saved interrupt state */
#endif
};
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

EXTERN irqstate_t irqsave(void);
EXTERN void       irqrestore(irqstate_t flags);

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_Z16F_IRQ_H */

