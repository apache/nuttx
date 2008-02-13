/****************************************************************************
 * arch/z8/include/z8/irq.h
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

#ifndef __ARCH_Z8_IRQ_H
#define __ARCH_Z8_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <ez8.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* ez8 Interrupt Numbers */

#if defined(ENCORE_VECTORS)

#  define  Z8_WDT_IRQ        0
#  define  Z8_TRAP_IRQ       1
#  define  Z8_TIMER2_IRQ     2 /* Only if EZ8_TIMER3 defined */
#  define  Z8_TIMER1_IRQ     3
#  define  Z8_TIMER0_IRQ     4
#  define  Z8_UART0_RX_IRQ   5 /* Only if EZ8_UART0 defined */
#  define  Z8_UART0_TX_IRQ   6 /* Only if EZ8_UART0 defined */
#  define  Z8_I2C_IRQ        7 /* Only if EZ8_I2C defined */
#  define  Z8_SPI_IRQ        8 /* Only if EZ8_SPI defined */
#  define  Z8_ADC_IRQ        9 /* Only if EZ8_ADC defined */
#  define  Z8_P7AD_IRQ      10
#  define  Z8_P6AD_IRQ      11
#  define  Z8_P5AD_IRQ      12
#  define  Z8_P4AD_IRQ      13
#  define  Z8_P3AD_IRQ      14
#  define  Z8_P2AD_IRQ      15
#  define  Z8_P1AD_IRQ      16
#  define  Z8_P0AD_IRQ      17
#  define  Z8_TIMER3_IRQ    18 /* Only if EZ8_TIMER4 defined */
#  define  Z8_UART1_RX_IRQ  19 /* Only if EZ8_UART1 defined */
#  define  Z8_UART1_TX_IRQ  20 /* Only if EZ8_UART1 defined */
#  define  Z8_DMA_IRQ       21 /* Only if EZ8_DMA defined */
#  define  Z8_C3_IRQ        22 /* Only if EZ8_PORT1 defined */
#  define  Z8_C2_IRQ        23 /* Only if EZ8_PORT1 defined */
#  define  Z8_C1_IRQ        24 /* Only if EZ8_PORT1 defined */
#  define  Z8_C0_IRQ        25 /* Only if EZ8_PORT1 defined */

#  define NR_IRQS          (26)

#elif defined(ENCORE_XP_VECTORS)

#  define  Z8_WDT_IRQ        0
#  define  Z8_TRAP_IRQ       1
#  define  Z8_TIMER2_IRQ     2 /* Only if EZ8_TIMER3 defined */
#  define  Z8_TIMER1_IRQ     3
#  define  Z8_TIMER0_IRQ     4
#  define  Z8_UART0_RX_IRQ   5 /* Only if EZ8_UART0 defined */
#  define  Z8_UART0_TX_IRQ   6 /* Only if EZ8_UART0 defined */
#  define  Z8_I2C_IRQ        7 /* Only if EZ8_I2C defined */
#  define  Z8_SPI_IRQ        8 /* Only if EZ8_SPI defined */
#  define  Z8_ADC_IRQ        9 /* Only if EZ8_ADC or EZ8_ADC_NEW defined */
#  define  Z8_P7AD_IRQ      10
#  define  Z8_P6AD_IRQ      11
#  define  Z8_P5AD_IRQ      12
#  define  Z8_P4AD_IRQ      13
#  define  Z8_P3AD_IRQ      14
#  define  Z8_P2AD_IRQ      15
#  define  Z8_P1AD_IRQ      16
#  define  Z8_P0AD_IRQ      17
#  define  Z8_TIMER3_IRQ    18 /* Only if EZ8_TIMER4 defined */
#  define  Z8_UART1_RX_IRQ  19 /* Only if EZ8_UART1 defined */
#  define  Z8_UART1_TX_IRQ  20 /* Only if EZ8_UART1 defined */
#  define  Z8_DMA_IRQ       21 /* Only if EZ8_DMA defined */
#  define  Z8_C3_IRQ        22 /* Only if EZ8_PORT1 defined */
#  define  Z8_C2_IRQ        23 /* Only if EZ8_PORT1 defined */
#  define  Z8_C1_IRQ        24 /* Only if EZ8_PORT1 defined */
#  define  Z8_C0_IRQ        25 /* Only if EZ8_PORT1 defined */
#  define  Z8_POTRAP_IRQ    27
#  define  Z8_WOTRAP_IRQ    28

#  define NR_IRQS          (29)

#elif defined(ENCORE_XP16K_VECTORS)

#  define  Z8_WDT_IRQ        0
#  define  Z8_TRAP_IRQ       1
#  define  Z8_TIMER2_IRQ     2 /* Only if EZ8_TIMER3 defined */
#  define  Z8_TIMER1_IRQ     3
#  define  Z8_TIMER0_IRQ     4
#  define  Z8_UART0_RX_IRQ   5 /* Only if EZ8_UART0 defined */
#  define  Z8_UART0_TX_IRQ   6 /* Only if EZ8_UART0 defined */
#  define  Z8_I2C_IRQ        7 /* Only if EZ8_I2C defined */
#  define  Z8_SPI_IRQ        8  /* Only if EZ8_SPI defined */
#  define  Z8_ADC_IRQ        9 /* Only if EZ8_ADC_NEW defined */
#  define  Z8_P7AD_IRQ      10
#  define  Z8_P6AD_IRQ      11
#  define  Z8_P5AD_IRQ      12
#  define  Z8_P4AD_IRQ      13
#  define  Z8_P3AD_IRQ      14
#  define  Z8_P2AD_IRQ      15
#  define  Z8_P1AD_IRQ      16
#  define  Z8_P0AD_IRQ      17
#  define  Z8_MCT_IRQ       19 /* Only if EZ8_MCT defined */
#  define  Z8_UART1_RX_IRQ  20 /* Only if EZ8_UART1 defined */
#  define  Z8_UART1_TX_IRQ  21 /* Only if EZ8_UART1 defined */
#  define  Z8_C3_IRQ        22
#  define  Z8_C2_IRQ        23
#  define  Z8_C1_IRQ        24
#  define  Z8_C0_IRQ        25
#  define  Z8_POTRAP_IRQ    27
#  define  Z8_WOTRAP_IRQ    28

#  define NR_IRQS          (29)

#elif defined(ENCORE_MC_VECTORS)

#  define  Z8_WDT_IRQ        0
#  define  Z8_TRAP_IRQ       1
#  define  Z8_PWMTIMER_IRQ   2
#  define  Z8_PWMFAULT_IRQ   3
#  define  Z8_ADC_IRQ        4 /* Only if EZ8_ADC_NEW defined */
#  define  Z8_CMP_IRQ        5
#  define  Z8_TIMER0_IRQ     6
#  define  Z8_UART0_RX_IRQ   7 /* Only if EZ8_UART0 defined */
#  define  Z8_UART0_TX_IRQ   8 /* Only if EZ8_UART0 defined */
#  define  Z8_SPI_IRQ        9 /* Only if EZ8_SPI defined */
#  define  Z8_I2C_IRQ       10 /* Only if EZ8_I2C defined */
#  define  Z8_C0_IRQ        12
#  define  Z8_PB_IRQ        13
#  define  Z8_P7A_IRQ       14
#  define  Z8_P3A_IRQ       Z8_P7A_IRQ
#  define  Z8_P6A_IRQ       15
#  define  Z8_P2A_IRQ       Z8_P6A_IRQ
#  define  Z8_P5A_IRQ       16
#  define  Z8_P1A_IRQ       Z8_P5A_IRQ
#  define  Z8_P4A_IRQ       17
#  define  Z8_P0A_IRQ       Z8_P4A_IRQ
#  define  Z8_POTRAP_IRQ    27
#  define  Z8_WOTRAP_IRQ    28

#  define NR_IRQS          (29)

#endif 

#define Z8_IRQ_SYSTIMER Z8_TIMER0_IRQ

/* IRQ Stack Frame Format
 *
 * This stack frame is created on each interrupt.  These registers are stored
 * in the TCB to many context switches.
 */

#define XCPT_I               (0) /* Offset 0: Saved I w/interrupt state in carry */
#define XCPT_BC              (1) /* Offset 1: Saved BC register */
#define XCPT_IX              (3) /* Offset 3: Saved IX register */
#define XCPT_IY              (4) /* Offset 4: Saved IY register */
#define XCPT_SP              (5) /* Offset 5: Offset to SP at time of interrupt */
#define XCPT_HL              (6) /* Offset 6: Saved HL register */
#define XCPT_AF              (7) /* Offset 7: Saved AF register */
#define XCPT_PC              (8) /* Offset 8: Offset to PC at time of interrupt */

#define XCPTCONTEXT_REGS     (9)
#define XCPTCONTEXT_SIZE     (2 * XCPTCONTEXT_REGS)

/* The ZDS-II provides built-in operations to test & disable and to restore
 * the interrupt state.
 *
 * irqstate_t irqsave(void);
 * void irqrestore(irqstate_t flags);
 */

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
  CODE void *sigdeliver; /* Actual type is sig_deliver_t */

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
EXTERN void irqrestore(irqstate_t flags);

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_Z8_IRQ_H */

