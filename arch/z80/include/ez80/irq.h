/****************************************************************************
 * arch/z80/include/ez80/irq.h
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
 * through nuttx/irq.h (via arch/irq.h)
 */

#ifndef __ARCH_Z80_INCLUDE_EZ80_IRQ_H
#define __ARCH_Z80_INCLUDE_EZ80_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ez80 Interrupt Numbers ***************************************************/

#if defined(CONFIG_ARCH_CHIP_EZ80F91)
#  define EZ80_EMACRX_IRQ     (0) /* Vector 0x40 */
#  define EZ80_EMACTX_IRQ     (1) /* Vector 0x44 */
#  define EZ80_EMACSYS_IRQ    (2) /* Vector 0x48 */
#  define EZ80_PLL_IRQ        (3) /* Vector 0x4c */
#  define EZ80_FLASH_IRQ      (4) /* Vector 0x50 */

#  define EZ80_TIMER0_IRQ     (5) /* Vector 0x54 */
#  define EZ80_TIMER1_IRQ     (6) /* Vector 0x58 */
#  define EZ80_TIMER2_IRQ     (7) /* Vector 0x5c */
#  define EZ80_TIMER3_IRQ     (8) /* Vector 0x60 */

#  define EZ80_RTC_IRQ        (9) /* Vector 0x6C */
#  define EZ80_UART0_IRQ     (10) /* Vector 0x70 */
#  define EZ80_UART1_IRQ     (11) /* Vector 0x74 */

#  define EZ80_I2C_IRQ       (12) /* Vector 0x78 */
#  define EZ80_SPI_IRQ       (13) /* Vector 0x7c */

#  define EZ80_PORTA0_IRQ    (14) /* Vector 0x80 */
#  define EZ80_PORTA1_IRQ    (15) /* Vector 0x84 */
#  define EZ80_PORTA2_IRQ    (16) /* Vector 0x88 */
#  define EZ80_PORTA3_IRQ    (17) /* Vector 0x8c */
#  define EZ80_PORTA4_IRQ    (18) /* Vector 0x90 */
#  define EZ80_PORTA5_IRQ    (19) /* Vector 0x94 */
#  define EZ80_PORTA6_IRQ    (20) /* Vector 0x98 */
#  define EZ80_PORTA7_IRQ    (21) /* Vector 0x9c */

#  define EZ80_PORTB0_IRQ    (22) /* Vector 0xa0 */
#  define EZ80_PORTB1_IRQ    (23) /* Vector 0xa4 */
#  define EZ80_PORTB2_IRQ    (24) /* Vector 0xa8 */
#  define EZ80_PORTB3_IRQ    (25) /* Vector 0xac */
#  define EZ80_PORTB4_IRQ    (26) /* Vector 0xb0 */
#  define EZ80_PORTB5_IRQ    (27) /* Vector 0xb4 */
#  define EZ80_PORTB6_IRQ    (28) /* Vector 0xb8 */
#  define EZ80_PORTB7_IRQ    (29) /* Vector 0xbc */

#  define EZ80_PORTC0_IRQ    (30) /* Vector 0xc0 */
#  define EZ80_PORTC1_IRQ    (31) /* Vector 0xc4 */
#  define EZ80_PORTC2_IRQ    (32) /* Vector 0xc8 */
#  define EZ80_PORTC3_IRQ    (33) /* Vector 0xcc */
#  define EZ80_PORTC4_IRQ    (34) /* Vector 0xd0 */
#  define EZ80_PORTC5_IRQ    (35) /* Vector 0xd4 */
#  define EZ80_PORTC6_IRQ    (36) /* Vector 0xd8 */
#  define EZ80_PORTC7_IRQ    (37) /* Vector 0xdc */

#  define EZ80_PORTD0_IRQ    (38) /* Vector 0xe0 */
#  define EZ80_PORTD1_IRQ    (39) /* Vector 0xe4 */
#  define EZ80_PORTD2_IRQ    (40) /* Vector 0xe8 */
#  define EZ80_PORTD3_IRQ    (41) /* Vector 0xec */
#  define EZ80_PORTD4_IRQ    (42) /* Vector 0xf0 */
#  define EZ80_PORTD5_IRQ    (43) /* Vector 0xf4 */
#  define EZ80_PORTD6_IRQ    (44) /* Vector 0xf8 */
#  define EZ80_PORTD7_IRQ    (45) /* Vector 0xfc */

#  define NR_IRQS            (46)
#elif defined(CONFIG_ARCH_CHIP_EZ80F92) || defined(CONFIG_ARCH_CHIP_EZ80F93)

#                                 /* Vectors 0x00-0x06 unused */
#  define EZ80_FLASH_IRQ      (0) /* Vector 0x08 */

#  define EZ80_TIMER0_IRQ     (1) /* Vector 0x0a */
#  define EZ80_TIMER1_IRQ     (2) /* Vector 0x0c */
#  define EZ80_TIMER2_IRQ     (3) /* Vector 0x0e */
#  define EZ80_TIMER3_IRQ     (4) /* Vector 0x10 */
#  define EZ80_TIMER4_IRQ     (5) /* Vector 0x12 */
#  define EZ80_TIMER5_IRQ     (6) /* Vector 0x14 */

#  define EZ80_RTC_IRQ        (7) /* Vector 0x16 */
#  define EZ80_UART0_IRQ      (8) /* Vector 0x18 */
#  define EZ80_UART1_IRQ      (9) /* Vector 0x1a */

#  define EZ80_I2C_IRQ       (10) /* Vector 0x1c */
#  define EZ80_SPI_IRQ       (11) /* Vector 0x1e */
#                                 /* Vectors 0x20-0x2e unused */

#  define EZ80_PORTB0_IRQ    (12) /* Vector 0x30 */
#  define EZ80_PORTB1_IRQ    (13) /* Vector 0x32 */
#  define EZ80_PORTB2_IRQ    (14) /* Vector 0x34 */
#  define EZ80_PORTB3_IRQ    (15) /* Vector 0x36 */
#  define EZ80_PORTB4_IRQ    (16) /* Vector 0x38 */
#  define EZ80_PORTB5_IRQ    (17) /* Vector 0x3a */
#  define EZ80_PORTB6_IRQ    (18) /* Vector 0x3c */
#  define EZ80_PORTB7_IRQ    (19) /* Vector 0x3e */

#  define EZ80_PORTC0_IRQ    (20) /* Vector 0x40 */
#  define EZ80_PORTC1_IRQ    (21) /* Vector 0x42 */
#  define EZ80_PORTC2_IRQ    (22) /* Vector 0x44 */
#  define EZ80_PORTC3_IRQ    (23) /* Vector 0x46 */
#  define EZ80_PORTC4_IRQ    (24) /* Vector 0x48 */
#  define EZ80_PORTC5_IRQ    (25) /* Vector 0x4a */
#  define EZ80_PORTC6_IRQ    (26) /* Vector 0x4c */
#  define EZ80_PORTC7_IRQ    (27) /* Vector 0x4e */

#  define EZ80_PORTD0_IRQ    (28) /* Vector 0x50 */
#  define EZ80_PORTD1_IRQ    (29) /* Vector 0x52 */
#  define EZ80_PORTD2_IRQ    (30) /* Vector 0x54 */
#  define EZ80_PORTD3_IRQ    (31) /* Vector 0x56 */
#  define EZ80_PORTD4_IRQ    (32) /* Vector 0x58 */
#  define EZ80_PORTD5_IRQ    (33) /* Vector 0x5a */
#  define EZ80_PORTD6_IRQ    (34) /* Vector 0x5c */
#  define EZ80_PORTD7_IRQ    (35) /* Vector 0x5e */
#                                 /* Vectors 0x60-0x66 unused */

#  define NR_IRQS            (36)
#endif

#define EZ80_IRQ_SYSTIMER  EZ80_TIMER0_IRQ

/* IRQ Management Macros ****************************************************/

/* IRQ State Save Format ****************************************************
 *
 * These indices describe how the ez8 context is save in the state save array
 *
 * Byte offsets:
 */

/* IRQ Stack Frame Format
 *
 * This stack frame is created on each interrupt.  These registers are stored
 * in the TCB to many context switches.
 */

/* chipreg_t indices */

#define XCPT_I             (0)           /* Index 0: 16-bit interrupt vector register */
#define XCPT_BC            (1)           /* Index 1: Saved 16-bit BC register */
#define XCPT_DE            (2)           /* Index 2: Saved 16-bit DE register */
#define XCPT_IX            (3)           /* Index 3: Saved 16-bit IX register */
#define XCPT_IY            (4)           /* Index 4: Saved 16-bit IY register */
#define XCPT_SP            (5)           /* Index 5: Saved 16-bit SP at time of interrupt */
#define XCPT_HL            (6)           /* Index 6: Saved 16-bit HL register */
#define XCPT_AF            (7)           /* Index 7: Saved 16-bit AF register */
#define XCPT_PC            (8)           /* Index 8: Offset to 16-bit PC at time of interrupt */
#define XCPTCONTEXT_REGS   (9)

#ifdef CONFIG_EZ80_Z80MODE
/* Byte offsets */

#  define XCPT_I_OFFSET    (2*XCPT_I)    /* Offset 0: 16-bit interrupt vector register */
#    define XCPT_IF_OFFSET (2*XCPT_I+0)  /* Offset 1: Saved flags. P set if interrupts enabled */
#    define XCPT_IA_OFFSET (2*XCPT_I+1)  /* Offset 2: Saved lower 8-bits of interrupt vector register */
#  define XCPT_BC_OFFSET   (2*XCPT_BC)   /* Offset 2: Saved 16-bit BC register */
#    define XCPT_C_OFFSET  (2*XCPT_BC+0) /* Offset 2: Saved 8-bit C register */
#    define XCPT_B_OFFSET  (2*XCPT_BC+1) /* Offset 3: Saved 8-bit D register */
#  define XCPT_DE_OFFSET   (2*XCPT_DE)   /* Offset 4: Saved 16-bit DE register */
#    define XCPT_E_OFFSET  (2*XCPT_DE+0) /* Offset 4: Saved 8-bit E register */
#    define XCPT_D_OFFSET  (2*XCPT_DE+1) /* Offset 5: Saved 8-bit D register */
#  define XCPT_IX_OFFSET   (2*XCPT_IX)   /* Offset 6: Saved 16-bit IX register */
#  define XCPT_IY_OFFSET   (2*XCPT_IY)   /* Offset 8: Saved 16-bit IY register */
#  define XCPT_SP_OFFSET   (2*XCPT_SP)   /* Offset 10: Saved 16-bit SP at time of interrupt */
#  define XCPT_HL_OFFSET   (2*XCPT_HL)   /* Offset 12: Saved 16-bit HL register */
#    define XCPT_L_OFFSET  (2*XCPT_HL+0) /* Offset 12: Saved 8-bit L register */
#    define XCPT_H_OFFSET  (2*XCPT_HL+1) /* Offset 13: Saved 8-bit H register */
#  define XCPT_AF_OFFSET   (2*XCPT_AF)   /* Offset 14: Saved 16-bit AF register */
#    define XCPT_F_OFFSET  (2*XCPT_AF+0) /* Offset 14: Saved flags */
#    define XCPT_A_OFFSET  (2*XCPT_AF+1) /* Offset 15: Saved 8-bit A register */
#  define XCPT_PC_OFFSET   (2*XCPT_PC)   /* Offset 16: Offset to 16-bit PC at time of interrupt */
#  define XCPTCONTEXT_SIZE (2*XCPTCONTEXT_REGS)
#else
/* Byte offsets */

#  define XCPT_I_OFFSET  (3*XCPT_I)      /* Offset 0: Saved 24-bit interrupt vector register */
#    define XCPT_IF_OFFSET (3*XCPT_I+0)  /* Offset 0: Saved flags. P set if interrupts enabled */
#    define XCPT_IA_OFFSET (3*XCPT_I+1)  /* Offset 1: Saved lower 8-bits of interrupt vector register */
#  define XCPT_BC_OFFSET   (3*XCPT_BC)   /* Offset 3: Saved 24-bit BC register */
#    define XCPT_C_OFFSET  (3*XCPT_BC+0) /* Offset 3: Saved 8-bit C register */
#    define XCPT_B_OFFSET  (3*XCPT_BC+1) /* Offset 4: Saved 8-bit B register */
#  define XCPT_DE_OFFSET   (3*XCPT_DE)   /* Offset 6: Saved 24-bit DE register */
#    define XCPT_E_OFFSET  (3*XCPT_DE+0) /* Offset 6: Saved 8-bit E register */
#    define XCPT_D_OFFSET  (3*XCPT_DE+1) /* Offset 7: Saved 8-bit D register */
#  define XCPT_IX_OFFSET   (3*XCPT_IX)   /* Offset 9: Saved 24-bit IX register */
#  define XCPT_IY_OFFSET   (3*XCPT_IY)   /* Offset 12: Saved 24-bit IY register */
#  define XCPT_SP_OFFSET   (3*XCPT_SP)   /* Offset 15: Saved 24-bit SP at time of interrupt */
#  define XCPT_HL_OFFSET   (3*XCPT_HL)   /* Offset 18: Saved 24-bit HL register */
#    define XCPT_L_OFFSET  (3*XCPT_HL+0) /* Offset 18: Saved 8-bit L register */
#    define XCPT_H_OFFSET  (3*XCPT_HL+1) /* Offset 19: Saved 8-bit H register */
#  define XCPT_AF_OFFSET   (3*XCPT_AF)   /* Offset 21: Saved AF register */
#    define XCPT_F_OFFSET  (3*XCPT_AF+0) /* Offset 21: Saved AF register */
#    define XCPT_A_OFFSET  (3*XCPT_AF+1) /* Offset 22: Saved 8-bit A register */
#  define XCPT_PC_OFFSET   (3*XCPT_PC)   /* Offset 24: Offset to 24-bit PC at time of interrupt */
#  define XCPTCONTEXT_SIZE (3*XCPTCONTEXT_REGS)
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* This is the type of the register save array */

#ifdef CONFIG_EZ80_Z80MODE
typedef uint16_t chipreg_t;
#else
typedef uint24_t chipreg_t;
#endif

/* This struct defines the way the registers are stored. */

struct xcptcontext
{
  /* Register save area */

  chipreg_t regs[XCPTCONTEXT_REGS];

  /* The following function pointer is non-zero if there
   * are pending signals to be processed.
   */

  CODE void *sigdeliver; /* Actual type is sig_deliver_t */

  /* The following retains that state during signal execution
   *
   * REVISIT:  Because there is only one copy of these save areas,
   * only a single signal handler can be active.  This precludes
   * queuing of signal actions.  As a result, signals received while
   * another signal handler is executing will be ignored!
   */

  chipreg_t saved_pc;    /* Saved return address */
  chipreg_t saved_i;     /* Saved interrupt state */
};
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

/* Name: up_irq_save, up_irq_restore, and friends.
 *
 * NOTE: These functions should never be called from application code and,
 * as a general rule unless you really know what you are doing, this
 * function should not be called directly from operation system code either:
 * Typically, the wrapper functions, enter_critical_section() and
 * leave_critical section(), are probably what you really want.
 */

irqstate_t up_irq_save(void);
void       up_irq_restore(irqstate_t flags);
irqstate_t up_irq_enable(void);

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_Z80_INCLUDE_EZ80_IRQ_H */
