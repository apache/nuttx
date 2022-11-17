/****************************************************************************
 * arch/renesas/src/rx65n/rx65n_vector.S
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

/****************************************************************************
 * Included Files
 ***************************************************************************/

#include <nuttx/config.h>        /* NuttX configuration settings */
#include <arch/board/board.h>   /* Board-specific settings */
#include <arch/irq.h>           /* IRQ definitions */
#include "renesas_internal.h"
#include "arch/rx65n/iodefine.h"
#include "rx65n_definitions.h"

/****************************************************************************
 * Pre-processor Definitions
 ***************************************************************************/

/****************************************************************************
 * External references
 ***************************************************************************/

  .section .text
  .globl  _renesas_doirq               /* C interrupt processing logic */

  .section .bss
  .globl  _up_num_irq
_up_num_irq:
  .long   0
  .size   _up_num_irq, . - _up_num_irq

/****************************************************************************
 * Text
 ***************************************************************************/
  .section .text
  .text

/****************************************************************************
 * Macros
 ***************************************************************************/

/****************************************************************************
 * Macro: trampoline
 *
 * Description:
 *   Enter on exception with:
 *
 *   SP -> PC
 *         PSW
 *
 *   Branch to up_vector with:
 *
 ***************************************************************************/

  .macro  trampoline, irq, label
  .globl  \label
\label:
  PUSH.L  R15
  MOV.L   #_up_num_irq, R15
  MOV.W   #\irq, [R15]
  POP     R15
  BRA.A   _up_vector /* Jump to the common vector handling logic */
  NOP
  .endm

/****************************************************************************
 * Macro: PUSHREGS
 *
 * Description:
 *   push registers to stacks
 *     registers : R15, R14, ..., R1, FPSW,
 *                 ACC1GU, ACC1HI, ACC1LO, ACC0GU, ACC0HI, ACC0LO, USP
 ***************************************************************************/

  .macro  PUSHREGS
  PUSHM   R1 - R15
  PUSHC   FPSW
  MVFACGU #0, A1, R15
  PUSH.L  R15
  MVFACHI #0, A1, R15
  PUSH.L  R15
  MVFACLO #0, A1, R15
  PUSH.L  R15
  MVFACGU #0, A0, R15
  PUSH.L  R15
  MVFACHI #0, A0, R15
  PUSH.L  R15
  MVFACLO #0, A0, R15
  PUSH.L  R15
  PUSHC   USP
  .endm

/****************************************************************************
 * Macro: POPREGS
 *
 * Description:
 *   ppop registers from stacks
 *     registers : USP, ACC0LO, ACC0,HI, ACC0GU, ACC1LO, ACC1,HI, ACC1GU,
 *                 FPSW, R1, R2, ..., R15
 ***************************************************************************/

 .macro  POPREGS
  POPC    USP
  POP     R15
  MVTACLO R15, A0
  POP     R15
  MVTACHI R15, A0
  POP     R15
  MVTACGU R15, A0
  POP     R15
  MVTACLO R15, A1
  POP     R15
  MVTACHI R15, A1
  POP     R15
  MVTACGU R15, A1
  POPC    FPSW
  POPM    R1 - R15
  .endm

/****************************************************************************
 * Macro: MULTI_SWITCHING
 *
 * Description:
 *   interrupt group branch for groupa and groupb
 ***************************************************************************/

  .macro  multi_switching, is_adrs, en_adrs, bit_number, irq_number
  PUSH.L  R1
  MOV.L   #\is_adrs, R1
  MOV.L   [R1], R1
  BTST    #\bit_number, R1
  POP     R1
  BZ      .L\@
  NOP
  PUSH.L  R1
  MOV.L   #\en_adrs, R1
  MOV.L   [R1], R1
  BTST    #\bit_number, R1
  POP     R1
  BZ      .L\@
  NOP
  PUSH.L  R1
  PUSH.L  R2
  MOV.L   #\irq_number, R1
  MOV.L   #_up_num_irq, R2
  MOV.W   R1, [R2]
  POP     R2
  POP     R1
  PUSHC   PSW         /* for RTE of the end of _up_vector */
  BSR.A   _up_vector  /* Jump to the common vector handling logic */
.L\@:
  NOP
  .endm

/****************************************************************************
 * Name: _up_vector
 * R1 = ISP because PC jump from each interrupt handler to _up_vector
 ***************************************************************************/

  .global _up_vector
  .type   _up_vector, #function

_up_vector:
  SETPSW I               /* Re-enable interrupts. */
  PUSHREGS
#ifdef RX65N_INTERRUPT_IPL

  /* Ensure the interrupt mask is set to the syscall priority while the kernel
   * structures are being accessed.
   */

  SETPSW  I          /* Enable Interrupt */
  MVTIPL  RX65N_INTERRUPT_IPL  /* configMAX_SYSCALL_INTERRUPT_PRIORITY */
#endif
  MOV.L   #_up_num_irq, R2
  MOV.W   [R2], R1
  MVFC    ISP, R2   /* stack pointer as 2nd args of _renesas_doirq */
  BSR.A   _renesas_doirq /* remaining R1 register as 1st args of _renesas_doirq */
  NOP
  MVFC    ISP, R2
  CMP     R2, R1
  BEQ     .lint_contextswitch
  NOP
  MOV.L   R1, R2
  MVFC    ISP, R1
  MOV.L   #(XCPTCONTEXT_SIZE), R3 /* R3 = Number of bytes to transfer */
  SMOVF      /* while(R3 > 0) {*R1++ = *R2++; R3--; } */
  NOP
.lint_contextswitch:
#ifdef RX65N_INTERRUPT_IPL

  /* Reset the interrupt mask as no more data structure access is required. */

  CLRPSW  I     /* Disable Interrupt */
  MVTIPL  #15   /* configKERNEL_INTERRUPT_PRIORITY */
#endif
  POPREGS
  RTE
  NOP
  NOP
  .size   _up_vector, . - _up_vector

/****************************************************************************
 * Name: int up_saveusercontext(void *regs)
 *
 * Description:
 *   Save the context of the calling function at the point of the return from
 *   the
 *   call.  This basically a setjmp.
 *
 * Input Parameters:
 *   R1 = Address of register save array
 *
 * Returned Value:
 *   R1 = 0 on normal call; 1 on context switch
 ***************************************************************************/

  .globl  _up_saveusercontext
  .type   _up_saveusercontext, #function

_up_saveusercontext:
  INT #VECT_ICU_SWINT    /* #RX65N_SWINT_VNDX */
  NOP
  NOP
  RTS
  NOP
  NOP
  .size   _up_saveusercontext,  . - _up_saveusercontext

_uprx65_swint_handler:
  PUSHREGS

  /* Then copy the stack content to the register context
   * array pointed to by r1
   */

  MVFC    ISP, R2     /* stack pointer as source context info */
  MOV.L   #XCPTCONTEXT_SIZE, R3   /* R3 = Number of bytes to transfer */
  SMOVF            /* while(R3 > 0) {*R1++ = *R2++; R3--; } */
  NOP
  SUB     #XCPTCONTEXT_SIZE, R1
  ADD     #(REG_R1 * 4), R1
  MOV.L   #1, R2
  MOV.L   R2, [R1]              /* Return 1 on context switch */

  /* Then restore R2, R3 and ISP */

  MVFC    ISP, R1
  MOV.L   (REG_R2 * 4)[R1], R2
  MOV.L   (REG_R3 * 4)[R1], R3
  ADD     #(XCPTCONTEXT_SIZE - 8), R1
  MVTC    R1, ISP

  /* Then return zero to indicate a normal function call return */

  MOV #0, R1    /* And return zero */
  RTE
  NOP
  NOP
  .size   _uprx65_swint_handler,  . - _uprx65_swint_handler

/****************************************************************************
 * Name: void renesas_fullcontextrestore(uint32_t *regs)
 *
 * Description:
 *   Restore the context of the using the provided register save array.
 *
 * Input Parameters:
 *   R1 = Address of register save array
 *
 * Returned Value:
 *   None
 ***************************************************************************/

  .global _renesas_fullcontextrestore
  .type   _renesas_fullcontextrestore, #function

_renesas_fullcontextrestore:
  INT  #VECT_ICU_SWINT2  /* #RX65N_SWINT2_VNDX */
  NOP
  NOP
  RTS
  NOP
  NOP
  .size   _renesas_fullcontextrestore, . - _renesas_fullcontextrestore

_uprx65_swint2_handler:
  MOV.L   R1, R2

/* Then copy the stack content from the register context array pointed */

  MVFC    ISP, R1     /* stack pointer as destination context info */
  ADD     # - (XCPTCONTEXT_SIZE - 8), R1
  MVTC    R1, ISP     /* ISP = address of base of new context info */
  MOV.L   #(XCPTCONTEXT_SIZE), R3 /* R3 = Number of bytes to transfer */
  SMOVF         /* while(R3 > 0) {*R1++ = *R2++; R3--;} */
  POPREGS
  RTE
  NOP
  NOP
  .size   _uprx65_swint2_handler, . - _uprx65_swint2_handler

/****************************************************************************
 * Name: _uprx65_*_handler
 *
 * Description:
 *   Trampoline entry points for each, individual IRQ
 *
 *  R1 :  Points to a the register save structure
 *
 ***************************************************************************/

_uprx65n_start_handler:
  trampoline NR_IRQS, _uprx65_invalid_handler

#ifdef CONFIG_BSC
  trampoline RX65N_BUSERR_IRQ, _uprx65_buserr_handler
#endif
#ifdef CONFIG_RAM
  trampoline RX65N_RAMERR_IRQ, _uprx65_ramerr_handler
#endif
#ifdef CONFIG_FCU
  trampoline RX65N_FIFERR_IRQ, _uprx65_fiferr_handler
#endif
#ifdef CONFIG_FRDYI
  trampoline RX65N_FRDYI_IRQ, _uprx65_frdyi_handler
#endif
  trampoline RX65N_CMI0_IRQ, _uprx65_cmi0_handler
#ifdef CONFIG_RX65N_CMT1
  trampoline RX65N_CMI1_IRQ, _uprx65_cmi1_handler
#endif
#ifdef CONFIG_RX65N_CMTW0
  trampoline  RX65N_CMWI0_IRQ, _uprx65_cmwi0_handler
#endif
#ifdef CONFIG_RX65N_CMTW1
  trampoline RX65N_CMWI0_IRQ, _uprx65_cmwi1_handler
#endif
#ifdef CONFIG_RX65N_USB0
  trampoline RX65N_D0FIFO0_IRQ, _uprx65_d0fifo0_handler
  trampoline RX65N_D1FIFO0_IRQ, _uprx65_d1fifo0_handler
#endif
#ifdef CONFIG_RX65N_RSPI0
  trampoline RX65N_SPRI0_IRQ, _uprx65_spri0_handler
  trampoline RX65N_SPTI0_IRQ, _uprx65_spti0_handler
#endif
#ifdef CONFIG_RX65N_RSPI1
  trampoline RX65N_SPRI1_IRQ, _uprx65_spri1_handler
  trampoline RX65N_SPTI1_IRQ, _uprx65_spti1_handler
#endif
#ifdef CONFIG_RX65N_QSPI
  trampoline RX65N_SPRI_IRQ, _uprx65_spri_handler
  trampoline RX65N_SPTI_IRQ, _uprx65_spti_handler
#endif
#ifdef CONFIG_RX65N_SDHI
  trampoline RX65N_SBFAI_IRQ, _uprx65_sbfai_handler
#endif
#ifdef CONFIG_RX65N_MMCIF
  trampoline RX65N_MBFAI_IRQ, _uprx65_mbfai_handler
#endif
#ifdef CONFIG_RX65N_RIIC1
  trampoline RX65N_RIIC1_RXI1_IRQ, _uprx65_rxii1_handler
  trampoline RX65N_RIIC1_TXI1_IRQ, _uprx65_txii1_handler
#endif
#ifdef CONFIG_RX65N_RIIC0
  trampoline RX65N_RIIC0_RXI0_IRQ, _uprx65_rxii0_handler
  trampoline RX65N_RIIC0_TXI0_IRQ, _uprx65_txii0_handler
#endif
#ifdef CONFIG_RX65N_RIIC2
  trampoline RX65N_RIIC2_RXI2_IRQ, _uprx65_rxii2_handler
  trampoline RX65N_RIIC2_TXI2_IRQ, _uprx65_txii2_handler
#endif
#ifdef CONFIG_RX65N_SCI0
  trampoline  RX65N_RXI0_IRQ, _uprx65_rxi0_handler
  trampoline  RX65N_TXI0_IRQ, _uprx65_txi0_handler
#endif
#ifdef CONFIG_RX65N_SCI1
  trampoline  RX65N_RXI1_IRQ, _uprx65_rxi1_handler
  trampoline  RX65N_TXI1_IRQ, _uprx65_txi1_handler
#endif
#ifdef CONFIG_RX65N_SCI2
  trampoline  RX65N_RXI2_IRQ, _uprx65_rxi2_handler
  trampoline  RX65N_TXI2_IRQ, _uprx65_txi2_handler
#endif
#if defined(CONFIG_RX65N_ICU)
  trampoline RX65N_IRQ0_IRQ,  _uprx65_irq0_handler
  trampoline RX65N_IRQ1_IRQ,  _uprx65_irq1_handler
  trampoline RX65N_IRQ2_IRQ,  _uprx65_irq2_handler
  trampoline RX65N_IRQ3_IRQ,  _uprx65_irq3_handler
  trampoline RX65N_IRQ4_IRQ,  _uprx65_irq4_handler
  trampoline RX65N_IRQ5_IRQ,  _uprx65_irq5_handler
  trampoline RX65N_IRQ6_IRQ,  _uprx65_irq6_handler
  trampoline RX65N_IRQ7_IRQ,  _uprx65_irq7_handler
  trampoline RX65N_IRQ8_IRQ,  _uprx65_irq8_handler
  trampoline RX65N_IRQ9_IRQ,  _uprx65_irq9_handler
  trampoline RX65N_IRQ10_IRQ, _uprx65_irq10_handler
  trampoline RX65N_IRQ11_IRQ, _uprx65_irq11_handler
  trampoline RX65N_IRQ12_IRQ, _uprx65_irq12_handler
  trampoline RX65N_IRQ13_IRQ, _uprx65_irq13_handler
  trampoline RX65N_IRQ14_IRQ, _uprx65_irq14_handler
  trampoline RX65N_IRQ15_IRQ, _uprx65_irq15_handler
#endif
#ifdef CONFIG_RX65N_SCI3
  trampoline RX65N_RXI3_IRQ, _uprx65_rxi3_handler
  trampoline RX65N_TXI3_IRQ, _uprx65_txi3_handler
#endif
#ifdef CONFIG_RX65N_SCI4
  trampoline RX65N_RXI4_IRQ, _uprx65_rxi4_handler
  trampoline RX65N_TXI4_IRQ, _uprx65_txi4_handler
#endif
#ifdef CONFIG_RX65N_SCI5
  trampoline RX65N_RXI5_IRQ, _uprx65_rxi5_handler
  trampoline RX65N_TXI5_IRQ, _uprx65_txi5_handler
#endif
#ifdef CONFIG_RX65N_SCI6
  trampoline RX65N_RXI6_IRQ, _uprx65_rxi6_handler
  trampoline RX65N_TXI6_IRQ, _uprx65_txi6_handler
#endif
#ifdef CONFIG_RX65N_LVD1
  trampoline RX65N_LVD1_IRQ, _uprx65_lvd1_handler
#endif
#ifdef CONFIG_RX65N_LVD2
  trampoline RX65N_LVD2_IRQ, _uprx65_lvd2_handler
#endif
#ifdef CONFIG_RX65N_USB0_R0
  trampoline RX65N_USBR0_IRQ, _uprx65_usbr0_handler
#endif
#ifdef CONFIG_RX65N_RTC
  trampoline RX65N_ALM_IRQ, _uprx65_alm_handler
  trampoline RX65N_PRD_IRQ, _uprx65_prd_handler
#endif
#ifdef CONFIG_RX65N_IWDT
  trampoline RX65N_IWUNI_IRQ, _uprx65_iwuni_handler
#endif
#ifdef CONFIG_RX65N_WDT
  trampoline RX65N_WUNI_IRQ, _uprx65_wuni_handler
#endif
#ifdef CONFIG_RX65N_PDC
  trampoline RX65N_PCDFI_IRQ, _uprx65_pcdfi_handler
#endif
#ifdef CONFIG_RX65N_SCI7
  trampoline RX65N_RXI7_IRQ, _uprx65_rxi7_handler
  trampoline RX65N_TXI7_IRQ, _uprx65_txi7_handler
#endif
#ifdef CONFIG_RX65N_SCI8
  trampoline RX65N_RXI8_IRQ, _uprx65_rxi8_handler
  trampoline RX65N_TXI8_IRQ, _uprx65_txi8_handler
#endif
#ifdef CONFIG_RX65N_SCI9
  trampoline RX65N_RXI9_IRQ, _uprx65_rxi9_handler
  trampoline RX65N_TXI9_IRQ, _uprx65_txi9_handler
#endif
#ifdef CONFIG_RX65N_SCI10
  trampoline RX65N_RXI10_IRQ, _uprx65_rxi10_handler
  trampoline RX65N_TXI10_IRQ, _uprx65_txi10_handler
#endif
#if defined(CONFIG_RX65N_IRQ_GROUP)
_uprx65_groupbe0_handler:

/* TODO: Please write the others with
 * definition RX65N_XXX_IRQ in irq.h as IRQ number
 */

  RTE
  NOP
_uprx65_groupbl2_handler:

/* TODO: Please write the others with
 * definition RX65N_XXX_IRQ in irq.h as IRQ number
 */

  RTE
  NOP
#endif
#ifdef CONFIG_RX65N_RSPI2
  trampoline RX65N_SPRI2_IRQ, _uprx65_spri2_handler
  trampoline RX65N_SPTI2_IRQ, _uprx65_spti2_handler
#endif
#if defined(CONFIG_RX65N_IRQ_GROUP)
_uprx65_groupbl0_handler:
#ifdef CONFIG_RX65N_SCI0
  multi_switching RX65N_GRPBL0_ADDR, RX65N_GENBL0_ADDR, 0, RX65N_TEI0_IRQ
  multi_switching RX65N_GRPBL0_ADDR, RX65N_GENBL0_ADDR, 1, RX65N_ERI0_IRQ
#endif
#ifdef CONFIG_RX65N_SCI1
  multi_switching RX65N_GRPBL0_ADDR, RX65N_GENBL0_ADDR, 2, RX65N_TEI1_IRQ
  multi_switching RX65N_GRPBL0_ADDR, RX65N_GENBL0_ADDR, 3, RX65N_ERI1_IRQ
#endif
#ifdef CONFIG_RX65N_SCI2
  multi_switching RX65N_GRPBL0_ADDR, RX65N_GENBL0_ADDR,  4, RX65N_TEI2_IRQ
  multi_switching RX65N_GRPBL0_ADDR, RX65N_GENBL0_ADDR,  5, RX65N_ERI2_IRQ
#endif
#ifdef CONFIG_RX65N_SCI3
  multi_switching RX65N_GRPBL0_ADDR, RX65N_GENBL0_ADDR,  6, RX65N_TEI3_IRQ
  multi_switching RX65N_GRPBL0_ADDR, RX65N_GENBL0_ADDR,  7, RX65N_ERI3_IRQ
#endif
#ifdef CONFIG_RX65N_SCI4
  multi_switching RX65N_GRPBL0_ADDR, RX65N_GENBL0_ADDR,  8, RX65N_TEI4_IRQ
  multi_switching RX65N_GRPBL0_ADDR, RX65N_GENBL0_ADDR,  9, RX65N_ERI4_IRQ
#endif
#ifdef CONFIG_RX65N_SCI5
  multi_switching RX65N_GRPBL0_ADDR, RX65N_GENBL0_ADDR, 10, RX65N_TEI5_IRQ
  multi_switching RX65N_GRPBL0_ADDR, RX65N_GENBL0_ADDR, 11, RX65N_ERI5_IRQ
#endif
#ifdef CONFIG_RX65N_SCI6
  multi_switching RX65N_GRPBL0_ADDR, RX65N_GENBL0_ADDR, 12, RX65N_TEI6_IRQ
  multi_switching RX65N_GRPBL0_ADDR, RX65N_GENBL0_ADDR, 13, RX65N_ERI6_IRQ
#endif
#ifdef CONFIG_RX65N_SCI7
  multi_switching RX65N_GRPBL0_ADDR, RX65N_GENBL0_ADDR, 14, RX65N_TEI7_IRQ
  multi_switching RX65N_GRPBL0_ADDR, RX65N_GENBL0_ADDR, 15, RX65N_ERI7_IRQ
#endif
#ifdef CONFIG_RX65N_SCI12
  multi_switching RX65N_GRPBL0_ADDR, RX65N_GENBL0_ADDR, 16, RX65N_TEI12_IRQ
  multi_switching RX65N_GRPBL0_ADDR, RX65N_GENBL0_ADDR, 17, RX65N_ERI12_IRQ
#endif

/* TODO: Please write the others with
 * definition RX65N_XXX_IRQ in irq.h as IRQ number
 */

  RTE
  NOP
_uprx65_groupbl1_handler:
#ifdef CONFIG_RX65N_SCI8
  multi_switching RX65N_GRPBL1_ADDR, RX65N_GENBL1_ADDR, 24, RX65N_TEI8_IRQ
  multi_switching RX65N_GRPBL1_ADDR, RX65N_GENBL1_ADDR, 25, RX65N_ERI8_IRQ
#endif /* CONFIG_RX65N_SCI8 */

#ifdef CONFIG_RX65N_SCI9
  multi_switching RX65N_GRPBL1_ADDR, RX65N_GENBL1_ADDR, 26, RX65N_TEI9_IRQ
  multi_switching RX65N_GRPBL1_ADDR, RX65N_GENBL1_ADDR, 27, RX65N_ERI9_IRQ
#endif /* CONFIG_RX65N_SCI9 */

#ifdef CONFIG_RX65N_RIIC0
  multi_switching RX65N_GRPBL1_ADDR, RX65N_GENBL1_ADDR, 13, RX65N_RIIC0_TEI0_IRQ
  multi_switching RX65N_GRPBL1_ADDR, RX65N_GENBL1_ADDR, 14, RX65N_RIIC0_EEI0_IRQ
#endif /* CONFIG_RX65N_RIIC0 */

#ifdef CONFIG_RX65N_RIIC2
  multi_switching RX65N_GRPBL1_ADDR, RX65N_GENBL1_ADDR, 15, RX65N_RIIC2_TEI2_IRQ
  multi_switching RX65N_GRPBL1_ADDR, RX65N_GENBL1_ADDR, 16, RX65N_RIIC2_EEI2_IRQ
#endif /* CONFIG_RX65N_RIIC2 */

#ifdef CONFIG_RX65N_RIIC1
  multi_switching RX65N_GRPBL1_ADDR, RX65N_GENBL1_ADDR, 28, RX65N_RIIC1_TEI1_IRQ
  multi_switching RX65N_GRPBL1_ADDR, RX65N_GENBL1_ADDR, 29, RX65N_RIIC1_EEI1_IRQ
#endif /* CONFIG_RX65N_RIIC1 */

/* TODO: Please write the others with
 * definition RX65N_XXX_IRQ in irq.h as IRQ number
 */

  RTE
  NOP
_uprx65_groupal0_handler:
#ifdef CONFIG_RX65N_SCI10
  multi_switching RX65N_GRPAL0_ADDR, RX65N_GENAL0_ADDR,  8, RX65N_TEI10_IRQ
  multi_switching RX65N_GRPAL0_ADDR, RX65N_GENAL0_ADDR,  9, RX65N_ERI10_IRQ
#endif /* CONFIG_RX65N_SCI10 */

#ifdef CONFIG_RX65N_SCI11
  multi_switching RX65N_GRPAL0_ADDR, RX65N_GENAL0_ADDR, 12, RX65N_TEI11_IRQ
  multi_switching RX65N_GRPAL0_ADDR, RX65N_GENAL0_ADDR, 13, RX65N_ERI11_IRQ
#endif /* CONFIG_RX65N_SCI11 */

#ifdef CONFIG_RX65N_RSPI0
  multi_switching RX65N_GRPAL0_ADDR, RX65N_GENAL0_ADDR, 16, RX65N_SPII0_IRQ
  multi_switching RX65N_GRPAL0_ADDR, RX65N_GENAL0_ADDR, 17, RX65N_SPEI0_IRQ
#endif /* CONFIG_RX65N_RSPI0 */

#ifdef CONFIG_RX65N_RSPI1
  multi_switching RX65N_GRPAL0_ADDR, RX65N_GENAL0_ADDR, 18, RX65N_SPII1_IRQ
  multi_switching RX65N_GRPAL0_ADDR, RX65N_GENAL0_ADDR, 19, RX65N_SPEI1_IRQ
#endif /* CONFIG_RX65N_RSPI1 */

#ifdef CONFIG_RX65N_RSPI2
  multi_switching RX65N_GRPAL0_ADDR, RX65N_GENAL0_ADDR, 20, RX65N_SPII2_IRQ
  multi_switching RX65N_GRPAL0_ADDR, RX65N_GENAL0_ADDR, 21, RX65N_SPEI2_IRQ
#endif /* CONFIG_RX65N_RSPI2 */

/* TODO: Please write the others with
 * definition RX65N_XXX_IRQ in irq.h as IRQ number
 */

  RTE
  NOP
_uprx65_groupal1_handler:
#ifdef CONFIG_RX65N_EMAC0
  multi_switching RX65N_GRPAL1_ADDR, RX65N_GENAL1_ADDR, 4, RX65N_ETH_IRQ
#endif /* CONFIG_RX65N_EMAC0 */

/* TODO: Please write the others with definition RX65N_XXX_IRQ in
 * irq.h as IRQ number */

    RTE
    NOP
#endif /* CONFIG_RX65N_IRQ_GROUP */

#ifdef CONFIG_RX65N_SCI11
  trampoline RX65N_RXI11_IRQ, _uprx65_rxi11_handler
  trampoline RX65N_TXI11_IRQ, _uprx65_txi11_handler
#endif
#ifdef CONFIG_RX65N_SCI12
  trampoline RX65N_RXI12_IRQ, _uprx65_rxi12_handler
  trampoline RX65N_TXI12_IRQ, _uprx65_txi12_handler
#endif
#ifdef CONFIG_RX65N_DMAC
  trampoline RX65N_DMAC0I_IRQ, _uprx65_dmac0i_handler
  trampoline RX65N_DMAC1I_IRQ, _uprx65_dmac1i_handler
  trampoline RX65N_DMAC2I_IRQ, _uprx65_dmac2i_handler
  trampoline RX65N_DMAC3I_IRQ, _uprx65_dmac3i_handler
  trampoline RX65N_DMAC74I_IRQ, _uprx65_dmac74i_handler
#endif
#ifdef CONFIG_RX65N_OST
  trampoline RX65N_OSTD1_IRQ, _uprx65_ostdi_handler
#endif
#ifdef CONFIG_RX65N_EXDMAC
  trampoline RX65N_EXDMAC0I_IRQ, _uprx65_exdmac0i_handler
  trampoline RX65N_EXDMAC1I_IRQ, _uprx65_exdmac1i_handler
#endif

#ifdef CONFIG_RX65N_PERIB
  trampoline RX65N_INTB128_IRQ, _uprx65_intb128_handler
  trampoline RX65N_INTB129_IRQ, _uprx65_intb129_handler
  trampoline RX65N_INTB130_IRQ, _uprx65_intb130_handler
  trampoline RX65N_INTB131_IRQ, _uprx65_intb131_handler
  trampoline RX65N_INTB132_IRQ, _uprx65_intb132_handler
  trampoline RX65N_INTB133_IRQ, _uprx65_intb133_handler
  trampoline RX65N_INTB134_IRQ, _uprx65_intb134_handler
  trampoline RX65N_INTB135_IRQ, _uprx65_intb135_handler
  trampoline RX65N_INTB136_IRQ, _uprx65_intb136_handler
  trampoline RX65N_INTB137_IRQ, _uprx65_intb137_handler
  trampoline RX65N_INTB138_IRQ, _uprx65_intb138_handler
  trampoline RX65N_INTB139_IRQ, _uprx65_intb139_handler
  trampoline RX65N_INTB140_IRQ, _uprx65_intb140_handler
  trampoline RX65N_INTB141_IRQ, _uprx65_intb141_handler
  trampoline RX65N_INTB142_IRQ, _uprx65_intb142_handler
  trampoline RX65N_INTB143_IRQ, _uprx65_intb143_handler
  trampoline RX65N_INTB144_IRQ, _uprx65_intb144_handler
  trampoline RX65N_INTB145_IRQ, _uprx65_intb145_handler
  trampoline RX65N_INTB146_IRQ, _uprx65_intb146_handler
  trampoline RX65N_INTB147_IRQ, _uprx65_intb147_handler
  trampoline RX65N_INTB148_IRQ, _uprx65_intb148_handler
  trampoline RX65N_INTB149_IRQ, _uprx65_intb149_handler
  trampoline RX65N_INTB150_IRQ, _uprx65_intb150_handler
  trampoline RX65N_INTB151_IRQ, _uprx65_intb151_handler
  trampoline RX65N_INTB152_IRQ, _uprx65_intb152_handler
  trampoline RX65N_INTB153_IRQ, _uprx65_intb153_handler
  trampoline RX65N_INTB154_IRQ, _uprx65_intb154_handler
  trampoline RX65N_INTB155_IRQ, _uprx65_intb155_handler
  trampoline RX65N_INTB156_IRQ, _uprx65_intb156_handler
  trampoline RX65N_INTB157_IRQ, _uprx65_intb157_handler
  trampoline RX65N_INTB158_IRQ, _uprx65_intb158_handler
  trampoline RX65N_INTB159_IRQ, _uprx65_intb159_handler
  trampoline RX65N_INTB160_IRQ, _uprx65_intb160_handler
  trampoline RX65N_INTB161_IRQ, _uprx65_intb161_handler
  trampoline RX65N_INTB162_IRQ, _uprx65_intb162_handler
  trampoline RX65N_INTB163_IRQ, _uprx65_intb163_handler
  trampoline RX65N_INTB164_IRQ, _uprx65_intb164_handler
  trampoline RX65N_INTB165_IRQ, _uprx65_intb165_handler
  trampoline RX65N_INTB166_IRQ, _uprx65_intb166_handler
  trampoline RX65N_INTB167_IRQ, _uprx65_intb167_handler
  trampoline RX65N_INTB168_IRQ, _uprx65_intb168_handler
  trampoline RX65N_INTB169_IRQ, _uprx65_intb169_handler
  trampoline RX65N_INTB170_IRQ, _uprx65_intb170_handler
  trampoline RX65N_INTB171_IRQ, _uprx65_intb171_handler
  trampoline RX65N_INTB172_IRQ, _uprx65_intb172_handler
  trampoline RX65N_INTB173_IRQ, _uprx65_intb173_handler
  trampoline RX65N_INTB174_IRQ, _uprx65_intb174_handler
  trampoline RX65N_INTB175_IRQ, _uprx65_intb175_handler
  trampoline RX65N_INTB176_IRQ, _uprx65_intb176_handler
  trampoline RX65N_INTB177_IRQ, _uprx65_intb177_handler
  trampoline RX65N_INTB178_IRQ, _uprx65_intb178_handler
  trampoline RX65N_INTB179_IRQ, _uprx65_intb179_handler
  trampoline RX65N_INTB180_IRQ, _uprx65_intb180_handler
  trampoline RX65N_INTB181_IRQ, _uprx65_intb181_handler
  trampoline RX65N_INTB182_IRQ, _uprx65_intb182_handler
  trampoline RX65N_INTB183_IRQ, _uprx65_intb183_handler
  trampoline RX65N_INTB184_IRQ, _uprx65_intb184_handler
  trampoline RX65N_INTB185_IRQ, _uprx65_intb185_handler
  trampoline RX65N_INTB186_IRQ, _uprx65_intb186_handler
  trampoline RX65N_INTB187_IRQ, _uprx65_intb187_handler
  trampoline RX65N_INTB188_IRQ, _uprx65_intb188_handler
  trampoline RX65N_INTB189_IRQ, _uprx65_intb189_handler
  trampoline RX65N_INTB190_IRQ, _uprx65_intb190_handler
  trampoline RX65N_INTB191_IRQ, _uprx65_intb191_handler
  trampoline RX65N_INTB192_IRQ, _uprx65_intb192_handler
  trampoline RX65N_INTB193_IRQ, _uprx65_intb193_handler
  trampoline RX65N_INTB194_IRQ, _uprx65_intb194_handler
  trampoline RX65N_INTB195_IRQ, _uprx65_intb195_handler
  trampoline RX65N_INTB196_IRQ, _uprx65_intb196_handler
  trampoline RX65N_INTB197_IRQ, _uprx65_intb197_handler
  trampoline RX65N_INTB198_IRQ, _uprx65_intb198_handler
  trampoline RX65N_INTB199_IRQ, _uprx65_intb199_handler
  trampoline RX65N_INTB200_IRQ, _uprx65_intb200_handler
  trampoline RX65N_INTB201_IRQ, _uprx65_intb201_handler
  trampoline RX65N_INTB202_IRQ, _uprx65_intb202_handler
  trampoline RX65N_INTB203_IRQ, _uprx65_intb203_handler
  trampoline RX65N_INTB204_IRQ, _uprx65_intb204_handler
  trampoline RX65N_INTB205_IRQ, _uprx65_intb205_handler
  trampoline RX65N_INTB206_IRQ, _uprx65_intb206_handler
  trampoline RX65N_INTB207_IRQ, _uprx65_intb207_handler
#endif
#ifdef CONFIG_RX65N_PERIA
  trampoline RX65N_INTA208_IRQ, _uprx65_inta208_handler
  trampoline RX65N_INTA209_IRQ, _uprx65_inta209_handler
  trampoline RX65N_INTA210_IRQ, _uprx65_inta210_handler
  trampoline RX65N_INTA211_IRQ, _uprx65_inta211_handler
  trampoline RX65N_INTA212_IRQ, _uprx65_inta212_handler
  trampoline RX65N_INTA213_IRQ, _uprx65_inta213_handler
  trampoline RX65N_INTA214_IRQ, _uprx65_inta214_handler
  trampoline RX65N_INTA215_IRQ, _uprx65_inta215_handler
  trampoline RX65N_INTA216_IRQ, _uprx65_inta216_handler
  trampoline RX65N_INTA217_IRQ, _uprx65_inta217_handler
  trampoline RX65N_INTA218_IRQ, _uprx65_inta218_handler
  trampoline RX65N_INTA219_IRQ, _uprx65_inta219_handler
  trampoline RX65N_INTA220_IRQ, _uprx65_inta220_handler
  trampoline RX65N_INTA221_IRQ, _uprx65_inta221_handler
  trampoline RX65N_INTA222_IRQ, _uprx65_inta222_handler
  trampoline RX65N_INTA223_IRQ, _uprx65_inta223_handler
  trampoline RX65N_INTA224_IRQ, _uprx65_inta224_handler
  trampoline RX65N_INTA225_IRQ, _uprx65_inta225_handler
  trampoline RX65N_INTA226_IRQ, _uprx65_inta226_handler
  trampoline RX65N_INTA227_IRQ, _uprx65_inta227_handler
  trampoline RX65N_INTA228_IRQ, _uprx65_inta228_handler
  trampoline RX65N_INTA229_IRQ, _uprx65_inta229_handler
  trampoline RX65N_INTA230_IRQ, _uprx65_inta230_handler
  trampoline RX65N_INTA231_IRQ, _uprx65_inta231_handler
  trampoline RX65N_INTA232_IRQ, _uprx65_inta232_handler
  trampoline RX65N_INTA233_IRQ, _uprx65_inta233_handler
  trampoline RX65N_INTA234_IRQ, _uprx65_inta234_handler
  trampoline RX65N_INTA235_IRQ, _uprx65_inta235_handler
  trampoline RX65N_INTA236_IRQ, _uprx65_inta236_handler
  trampoline RX65N_INTA237_IRQ, _uprx65_inta237_handler
  trampoline RX65N_INTA238_IRQ, _uprx65_inta238_handler
  trampoline RX65N_INTA239_IRQ, _uprx65_inta239_handler
  trampoline RX65N_INTA240_IRQ, _uprx65_inta240_handler
  trampoline RX65N_INTA241_IRQ, _uprx65_inta241_handler
  trampoline RX65N_INTA242_IRQ, _uprx65_inta242_handler
  trampoline RX65N_INTA243_IRQ, _uprx65_inta243_handler
  trampoline RX65N_INTA244_IRQ, _uprx65_inta244_handler
  trampoline RX65N_INTA245_IRQ, _uprx65_inta245_handler
  trampoline RX65N_INTA246_IRQ, _uprx65_inta246_handler
  trampoline RX65N_INTA247_IRQ, _uprx65_inta247_handler
  trampoline RX65N_INTA248_IRQ, _uprx65_inta248_handler
  trampoline RX65N_INTA249_IRQ, _uprx65_inta249_handler
  trampoline RX65N_INTA250_IRQ, _uprx65_inta250_handler
  trampoline RX65N_INTA251_IRQ, _uprx65_inta251_handler
  trampoline RX65N_INTA252_IRQ, _uprx65_inta252_handler
  trampoline RX65N_INTA253_IRQ, _uprx65_inta253_handler
  trampoline RX65N_INTA254_IRQ, _uprx65_inta254_handler
  trampoline RX65N_INTA255_IRQ, _uprx65_inta255_handler
#endif

/****************************************************************************
 * Data
 ***************************************************************************/

/****************************************************************************
 *  Name: g_intstackalloc/g_intstacktop
 *
 * Description:
 *   Should not happen
 *
 ***************************************************************************/

  .bss
  .align  2

#if CONFIG_ARCH_INTERRUPTSTACK > 3
  .bss
  .align  2
  .global  _g_intstackalloc
  .type   _g_intstackalloc, object
  .global  _g_intstacktop
  .type   _g_intstacktop, object
_g_intstackalloc:
  .skip   (CONFIG_ARCH_INTERRUPTSTACK & ~3)
_g_intstacktop:
  .size   _g_intstacktop, 0
  .size   _g_intstackalloc, (CONFIG_ARCH_INTERRUPTSTACK & ~3)
#endif

/****************************************************************************
 * Interrupt Vectors
 ***************************************************************************/

  .section        .rvectors

/****************************************************************************
 * Name: __int_vector_table
 *
 * Description:
 *   Interrupt vector table
 ***************************************************************************/

  .global __int_vector_table
  .type   __int_vector_table, % object

__int_vector_table:
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
#ifdef CONFIG_BSC
  .long   _uprx65_buserr_handler
#else
  .long   _uprx65_invalid_handler
#endif
  .long   _uprx65_invalid_handler
#ifdef CONFIG_RAM
  .long   _uprx65_ramerr_handler
#else
  .long   _uprx65_invalid_handler
#endif
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
#ifdef CONFIG_FCU
  .long   _uprx65_fiferr_handler
#else
  .long   _uprx65_invalid_handler
#endif
  .long   _uprx65_invalid_handler
#ifdef CONFIG_FRDYI
  .long   _uprx65_frdyi_handler
#else
  .long   _uprx65_invalid_handler
#endif
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_swint2_handler
  .long   _uprx65_swint_handler
  .long   _uprx65_cmi0_handler
#ifdef CONFIG_RX65N_CMT1
  .long   _uprx65_cmi1_handler
#else
  .long   _uprx65_invalid_handler
#endif
#ifdef CONFIG_RX65N_CMTW0
  .long   _uprx65_cmwi0_handler
#else
  .long   _uprx65_invalid_handler
#endif
#ifdef CONFIG_RX65N_CMTW1
  .long   _uprx65_cmwi1_handler
#else
  .long   _uprx65_invalid_handler
#endif
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
#ifdef CONFIG_RX65N_USB0
  .long   _uprx65_d0fifo0_handler
  .long   _uprx65_d1fifo0_handler
#else
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
#endif
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
#ifdef CONFIG_RX65N_RSPI0
  .long   _uprx65_spri0_handler
  .long   _uprx65_spti0_handler
#else
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
#endif
#ifdef CONFIG_RX65N_RSPI1
  .long   _uprx65_spri1_handler
  .long   _uprx65_spti1_handler
#else
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
#endif
#ifdef CONFIG_RX65N_QSPI
  .long   _uprx65_spri_handler
  .long   _uprx65_spti_handler
#else
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
#endif
#ifdef CONFIG_RX65N_SDHI
  .long   _uprx65_sbfai_handler
#else
  .long   _uprx65_invalid_handler
#endif
#ifdef CONFIG_RX65N_MMCIF
  .long   _uprx65_mbfai_handler
#else
  .long   _uprx65_invalid_handler
#endif
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
#ifdef CONFIG_RX65N_RIIC1
  .long   _uprx65_rxii1_handler
  .long   _uprx65_txii1_handler
#else
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
#endif
#ifdef CONFIG_RX65N_RIIC0
  .long   _uprx65_rxii0_handler
  .long   _uprx65_txii0_handler
#else
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
#endif
#ifdef CONFIG_RX65N_RIIC2
  .long   _uprx65_rxii2_handler
  .long   _uprx65_txii2_handler
#else
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
#endif
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
#ifdef CONFIG_RX65N_SCI0
  .long   _uprx65_rxi0_handler
  .long   _uprx65_txi0_handler
#else
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
#endif
#ifdef CONFIG_RX65N_SCI1
  .long   _uprx65_rxi1_handler
  .long   _uprx65_txi1_handler
#else
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
#endif
#ifdef CONFIG_RX65N_SCI2
  .long   _uprx65_rxi2_handler
  .long   _uprx65_txi2_handler
#else
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
#endif
#if defined(CONFIG_RX65N_ICU)
  .long   _uprx65_irq0_handler
  .long   _uprx65_irq1_handler
  .long   _uprx65_irq2_handler
  .long   _uprx65_irq3_handler
  .long   _uprx65_irq4_handler
  .long   _uprx65_irq5_handler
  .long   _uprx65_irq6_handler
  .long   _uprx65_irq7_handler
  .long   _uprx65_irq8_handler
  .long   _uprx65_irq9_handler
  .long   _uprx65_irq10_handler
  .long   _uprx65_irq11_handler
  .long   _uprx65_irq12_handler
  .long   _uprx65_irq13_handler
  .long   _uprx65_irq14_handler
  .long   _uprx65_irq15_handler
#else
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
#endif
#ifdef CONFIG_RX65N_SCI3
  .long   _uprx65_rxi3_handler
  .long   _uprx65_txi3_handler
#else
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
#endif
#ifdef CONFIG_RX65N_SCI4
  .long   _uprx65_rxi4_handler
  .long   _uprx65_txi4_handler
#else
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
#endif
#ifdef CONFIG_RX65N_SCI5
  .long   _uprx65_rxi5_handler
  .long   _uprx65_txi5_handler
#else
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
#endif
#ifdef CONFIG_RX65N_SCI6
  .long   _uprx65_rxi6_handler
  .long   _uprx65_txi6_handler
#else
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
#endif
#ifdef CONFIG_RX65N_LVD1
  .long   _uprx65_lvd1_handler
#else
  .long   _uprx65_invalid_handler
#endif
#ifdef CONFIG_RX65N_LVD2
  .long   _uprx65_lvd2_handler
#else
  .long   _uprx65_invalid_handler
#endif
#ifdef CONFIG_RX65N_USB0_R0
  .long   _uprx65_usbr0_handler
#else
  .long   _uprx65_invalid_handler
#endif
  .long   _uprx65_invalid_handler
#ifdef CONFIG_RX65N_RTC
  .long   _uprx65_alm_handler
  .long   _uprx65_prd_handler
#else
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
#endif
  .long   _uprx65_invalid_handler
#ifdef CONFIG_RX65N_IWDT
  .long   _uprx65_iwuni_handler
#else
  .long   _uprx65_invalid_handler
#endif
#ifdef CONFIG_RX65N_WDT
  .long   _uprx65_wuni_handler
#else
  .long   _uprx65_invalid_handler
#endif
#ifdef CONFIG_RX65N_PDC
  .long   _uprx65_pcdfi_handler
#else
  .long   _uprx65_invalid_handler
#endif
#ifdef CONFIG_RX65N_SCI7
  .long   _uprx65_rxi7_handler
  .long   _uprx65_txi7_handler
#else
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
#endif
#ifdef CONFIG_RX65N_SCI8
  .long   _uprx65_rxi8_handler
  .long   _uprx65_txi8_handler
#else
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
#endif
#ifdef CONFIG_RX65N_SCI9
  .long   _uprx65_rxi9_handler
  .long   _uprx65_txi9_handler
#else
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
#endif
#ifdef CONFIG_RX65N_SCI10
  .long   _uprx65_rxi10_handler
  .long   _uprx65_txi10_handler
#else
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
#endif
#if defined(CONFIG_RX65N_IRQ_GROUP)
  .long   _uprx65_groupbe0_handler
  .long   _uprx65_groupbl2_handler
#else
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
#endif
#ifdef CONFIG_RX65N_RSPI2
  .long   _uprx65_spri2_handler
  .long   _uprx65_spti2_handler
#else
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
#endif
#if defined(CONFIG_RX65N_IRQ_GROUP)
  .long   _uprx65_groupbl0_handler
  .long   _uprx65_groupbl1_handler
  .long   _uprx65_groupal0_handler
  .long   _uprx65_groupal1_handler
#else
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
#endif
#ifdef CONFIG_RX65N_SCI11
  .long   _uprx65_rxi11_handler
  .long   _uprx65_txi11_handler
#else
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
#endif
#ifdef CONFIG_RX65N_SCI12
  .long   _uprx65_rxi12_handler
  .long   _uprx65_txi12_handler
#else
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
#endif
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
#ifdef CONFIG_RX65N_DMAC
  .long   _uprx65_dmac0i_handler
  .long   _uprx65_dmac1i_handler
  .long   _uprx65_dmac2i_handler
  .long   _uprx65_dmac3i_handler
  .long   _uprx65_dmac74i_handler
#else
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
#endif
#ifdef CONFIG_RX65N_OST
  .long   _uprx65_ostdi_handler
#else
  .long   _uprx65_invalid_handler
#endif
#ifdef CONFIG_RX65N_EXDMAC
  .long   _uprx65_exdmac0i_handler
  .long   _uprx65_exdmac1i_handler
#else
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
#endif

#ifdef CONFIG_RX65N_PERIB
  .long   _uprx65_intb128_handler
  .long   _uprx65_intb129_handler
  .long   _uprx65_intb130_handler
  .long   _uprx65_intb131_handler
  .long   _uprx65_intb132_handler
  .long   _uprx65_intb133_handler
  .long   _uprx65_intb134_handler
  .long   _uprx65_intb135_handler
  .long   _uprx65_intb136_handler
  .long   _uprx65_intb137_handler
  .long   _uprx65_intb138_handler
  .long   _uprx65_intb139_handler
  .long   _uprx65_intb140_handler
  .long   _uprx65_intb141_handler
  .long   _uprx65_intb142_handler
  .long   _uprx65_intb143_handler
  .long   _uprx65_intb144_handler
  .long   _uprx65_intb145_handler
  .long   _uprx65_intb146_handler
  .long   _uprx65_intb147_handler
  .long   _uprx65_intb148_handler
  .long   _uprx65_intb149_handler
  .long   _uprx65_intb150_handler
  .long   _uprx65_intb151_handler
  .long   _uprx65_intb152_handler
  .long   _uprx65_intb153_handler
  .long   _uprx65_intb154_handler
  .long   _uprx65_intb155_handler
  .long   _uprx65_intb156_handler
  .long   _uprx65_intb157_handler
  .long   _uprx65_intb158_handler
  .long   _uprx65_intb159_handler
  .long   _uprx65_intb160_handler
  .long   _uprx65_intb161_handler
  .long   _uprx65_intb162_handler
  .long   _uprx65_intb163_handler
  .long   _uprx65_intb164_handler
  .long   _uprx65_intb165_handler
  .long   _uprx65_intb166_handler
  .long   _uprx65_intb167_handler
  .long   _uprx65_intb168_handler
  .long   _uprx65_intb169_handler
  .long   _uprx65_intb170_handler
  .long   _uprx65_intb171_handler
  .long   _uprx65_intb172_handler
  .long   _uprx65_intb173_handler
  .long   _uprx65_intb174_handler
  .long   _uprx65_intb175_handler
  .long   _uprx65_intb176_handler
  .long   _uprx65_intb177_handler
  .long   _uprx65_intb178_handler
  .long   _uprx65_intb179_handler
  .long   _uprx65_intb180_handler
  .long   _uprx65_intb181_handler
  .long   _uprx65_intb182_handler
  .long   _uprx65_intb183_handler
  .long   _uprx65_intb184_handler
  .long   _uprx65_intb185_handler
  .long   _uprx65_intb186_handler
  .long   _uprx65_intb187_handler
  .long   _uprx65_intb188_handler
  .long   _uprx65_intb189_handler
  .long   _uprx65_intb190_handler
  .long   _uprx65_intb191_handler
  .long   _uprx65_intb192_handler
  .long   _uprx65_intb193_handler
  .long   _uprx65_intb194_handler
  .long   _uprx65_intb195_handler
  .long   _uprx65_intb196_handler
  .long   _uprx65_intb197_handler
  .long   _uprx65_intb198_handler
  .long   _uprx65_intb199_handler
  .long   _uprx65_intb200_handler
  .long   _uprx65_intb201_handler
  .long   _uprx65_intb202_handler
  .long   _uprx65_intb203_handler
  .long   _uprx65_intb204_handler
  .long   _uprx65_intb205_handler
  .long   _uprx65_intb206_handler
  .long   _uprx65_intb207_handler
#else
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
#endif
#ifdef CONFIG_RX65N_PERIA
  .long   _uprx65_inta208_handler
  .long   _uprx65_inta209_handler
  .long   _uprx65_inta210_handler
  .long   _uprx65_inta211_handler
  .long   _uprx65_inta212_handler
  .long   _uprx65_inta213_handler
  .long   _uprx65_inta214_handler
  .long   _uprx65_inta215_handler
  .long   _uprx65_inta216_handler
  .long   _uprx65_inta217_handler
  .long   _uprx65_inta218_handler
  .long   _uprx65_inta219_handler
  .long   _uprx65_inta220_handler
  .long   _uprx65_inta221_handler
  .long   _uprx65_inta222_handler
  .long   _uprx65_inta223_handler
  .long   _uprx65_inta224_handler
  .long   _uprx65_inta225_handler
  .long   _uprx65_inta226_handler
  .long   _uprx65_inta227_handler
  .long   _uprx65_inta228_handler
  .long   _uprx65_inta229_handler
  .long   _uprx65_inta230_handler
  .long   _uprx65_inta231_handler
  .long   _uprx65_inta232_handler
  .long   _uprx65_inta233_handler
  .long   _uprx65_inta234_handler
  .long   _uprx65_inta235_handler
  .long   _uprx65_inta236_handler
  .long   _uprx65_inta237_handler
  .long   _uprx65_inta238_handler
  .long   _uprx65_inta239_handler
  .long   _uprx65_inta240_handler
  .long   _uprx65_inta241_handler
  .long   _uprx65_inta242_handler
  .long   _uprx65_inta243_handler
  .long   _uprx65_inta244_handler
  .long   _uprx65_inta245_handler
  .long   _uprx65_inta246_handler
  .long   _uprx65_inta247_handler
  .long   _uprx65_inta248_handler
  .long   _uprx65_inta249_handler
  .long   _uprx65_inta250_handler
  .long   _uprx65_inta251_handler
  .long   _uprx65_inta252_handler
  .long   _uprx65_inta253_handler
  .long   _uprx65_inta254_handler
  .long   _uprx65_inta255_handler
#else
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
  .long   _uprx65_invalid_handler
#endif
  .size   __int_vector_table, . - __int_vector_table
  .end

/****************************************************************************
 * EOF
 ***************************************************************************/
