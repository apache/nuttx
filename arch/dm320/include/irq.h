/************************************************************
 * irq.h
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
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
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
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
 ************************************************************/

/* This file should never be included directed but, rather,
 * only indirectly through nuttx/irq.h
 */

#ifndef __ARCH_DM320_IRQ_H
#define __ARCH_DM320_IRQ_H

/************************************************************
 * Included Files
 ************************************************************/

/************************************************************
 * Definitions
 ************************************************************/

/* IRQ Stack Frame Format:
 *
 * Context is always saved/restored in the same way:
 *
 *   (1) stmia rx, {r0-r14}
 *   (2) then the PC and CPSR
 *
 * This results in the following set of indices that
 * can be used to access individual registers in the
 * xcp.regs array:
 */

#define REG_R0              (0)
#define REG_R1              (1)
#define REG_R2              (2)
#define REG_R3              (3)
#define REG_R4              (4)
#define REG_R5              (5)
#define REG_R6              (6)
#define REG_R7              (7)
#define REG_R8              (8)
#define REG_R9              (9)
#define REG_R10             (10)
#define REG_R11             (11)
#define REG_R12             (12)
#define REG_R13             (13)
#define REG_R14             (14)
#define REG_R15             (15)
#define REG_CPSR            (16)

#define XCPTCONTEXT_REGS    (17)
#define XCPTCONTEXT_SIZE    (4 * XCPTCONTEXT_REGS)

#define REG_A1              REG_R0
#define REG_A2              REG_R1
#define REG_A3              REG_R2
#define REG_A4              REG_R3
#define REG_V1              REG_R4
#define REG_V2              REG_R5
#define REG_V3              REG_R6
#define REG_V4              REG_R7
#define REG_V5              REG_R8
#define REG_V6              REG_R9
#define REG_V7              REG_R10
#define REG_SB              REG_R9
#define REG_SL              REG_R10
#define REG_FP              REG_R11
#define REG_IP              REG_R12
#define REG_SP              REG_R13
#define REG_LR              REG_R14
#define REG_PC              REG_R15

/* DM320 Interrupts */

#define DM320_IRQ_TMR0      0 /* IRQ0:  Timer 0 Interrupt */
#define DM320_IRQ_TMR1      1 /* IRQ1:  Timer 1 Interrupt */
#define DM320_IRQ_TMR2      2 /* IRQ2:  Timer 2 Interrupt (CCD timer 0) */
#define DM320_IRQ_TMR3      3 /* IRQ3:  Timer 3 Interrupt (CCD timer 1) */
#define DM320_IRQ_CCDVD0    4 /* IRQ4:  CCD VD Interrupt #0 */
#define DM320_IRQ_CCDVD1    5 /* IRQ5:  CCD VD Interrupt #1 */
#define DM320_IRQ_CCDWEN    6 /* IRQ6:  CCD WEN Interrupt */
#define DM320_IRQ_VENC      7 /* IRQ7:  Video Encoder Interrupt */
#define DM320_IRQ_SP0       8 /* IRQ8:  Serial Port 0 Interrupt (with DMA) */
#define DM320_IRQ_SP1       9 /* IRQ9:  Serial Port 1 Interrupt */
#define DM320_IRQ_EXTHOST  10 /* IRQ10: External host interrupt */
#define DM320_IRQ_IMGBUF   11 /* IRQ11: Image Buffer */
#define DM320_IRQ_UART0    12 /* IRQ12: UART0 Interrupt */
#define DM320_IRQ_UART1    13 /* IRQ13: UART1 Interrupt */
#define DM320_IRQ_USB0     14 /* IRQ14: USB 0 Interrupt (DMA) */
#define DM320_IRQ_USB1     15 /* IRQ15: USB 1 Interrupt (Core) */
#define DM320_IRQ_VLYNQ    16 /* IRQ16: VLYNQ Interrupt */
#define DM320_IRQ_MTC0     17 /* IRQ17: Memory Traffic Controller 0 (DMA) */
#define DM320_IRQ_MTC1     18 /* IRQ18: Memory Traffic Controller 1 (CFC_RDY) */
#define DM320_IRQ_MMCSD0   19 /* IRQ19: MMC/SD or MS 0 Interrupt */
#define DM320_IRQ_MMCSD1   20 /* IRQ20: MMC/SD or MS 1 Interrupt */
#define DM320_IRQ_EXT0     21 /* IRQ21: External Interrupt #0 (GIO0) */
#define DM320_IRQ_EXT1     22 /* IRQ22: External Interrupt #1 (GIO1) */
#define DM320_IRQ_EXT2     23 /* IRQ23: External Interrupt #2 (GIO2) */
#define DM320_IRQ_EXT3     24 /* IRQ24: External Interrupt #3 (GIO3) */
#define DM320_IRQ_EXT4     25 /* IRQ25: External Interrupt #4 (GIO4) */
#define DM320_IRQ_EXT5     26 /* IRQ26: External Interrupt #5 (GIO5) */
#define DM320_IRQ_EXT6     27 /* IRQ27: External Interrupt #6 (GIO6) */
#define DM320_IRQ_EXT7     28 /* IRQ28: External Interrupt #7 (GIO7) */
#define DM320_IRQ_EXT8     29 /* IRQ29: External Interrupt #8 (GIO8) */
#define DM320_IRQ_EXT9     30 /* IRQ30: External Interrupt #9 (GIO9) */
#define DM320_IRQ_EXT10    31 /* IRQ31: External Interrupt #10 (GIO10) */
#define DM320_IRQ_EXT11    32 /* IRQ32: External Interrupt #11 (GIO11) */
#define DM320_IRQ_EXT12    33 /* IRQ33: External Interrupt #12 (GIO12) */
#define DM320_IRQ_EXT13    34 /* IRQ34: External Interrupt #13 (GIO13) */
#define DM320_IRQ_EXT14    35 /* IRQ35: External Interrupt #14 (GIO14) */
#define DM320_IRQ_EXT15    36 /* IRQ36: External Interrupt #15 (GIO15) */
#define DM320_IRQ_PREV0    37 /* IRQ37: Preview Engine 0 (Preview Over) */
#define DM320_IRQ_PREV1    38 /* IRQ38: Preview Engine 1 (Preview Historgram Over) */
#define DM320_IRQ_WDT      39 /* IRQ39: Watchdog Timer Interrupt */
#define DM320_IRQ_I2C      40 /* IRQ40: I2C Interrupt */
#define DM320_IRQ_CLKC     41 /* IRQ41: Clock controller Interrupt (wake up) */
#define DM320_IRQ_E2ICE    42 /* IRQ42: Embedded ICE Interrupt */
#define DM320_IRQ_ARMCOMRX 43 /* IRQ43: ARMCOMM Receive Interrupt */
#define DM320_IRQ_ARMCOMTX 44 /* IRQ44: ARMCOMM Transmit Interrupt */
#define DM320_IRQ_RSV      45 /* IRQ45: Reserved Interrupt */

#define DM320_IRQ_SYSTIMER DM320_IRQ_TMR0
#define NR_IRQS            (DM320_IRQ_RSV+1)

/************************************************************
 * Public Types
 ************************************************************/

/* This struct defines the way the registers are stored.  We
 * need to save:
 *
 *  1	CPSR
 *  7	Static registers, v1-v7 (aka r4-r10)
 *  1	Frame pointer, fp (aka r11)
 *  1	Stack pointer, sp (aka r13)
 *  1	Return address, lr (aka r14)
 * ---
 * 11	(XCPTCONTEXT_USER_REG)
 *
 * On interrupts, we also need to save:
 *  4	Volatile registers, a1-a4 (aka r0-r3)
 *  1	Scratch Register, ip (aka r12)
 *---
 *  5	(XCPTCONTEXT_IRQ_REGS)
 *
 * For a total of 17 (XCPTCONTEXT_REGS)
 */

#ifndef __ASSEMBLY__
struct xcptcontext
{
  /* The following function pointer is non-zero if there
   * are pending signals to be processed.
   */

  void *sigdeliver; /* Actual type is sig_deliver_t */

  /* These are saved copies of LR and CPSR used during
   * signal processing.
   */

  uint32 saved_pc;
  uint32 saved_cpsr;

  /* Register save area */

  uint32 regs[XCPTCONTEXT_REGS];
};
#endif

/************************************************************
 * Inline functions
 ************************************************************/

#ifndef __ASSEMBLY__

/* Save the current interrupt enable state & disable IRQs */

static inline irqstate_t irqsave(void)
{
  unsigned int flags;
  unsigned int temp;
  __asm__ __volatile__
    (
     "\tmrs    %0, cpsr\n"
     "\torr    %1, %0, #128\n"
     "\tmsr    cpsr_c, %1"
     : "=r" (flags), "=r" (temp)
     :
     : "memory");
  return flags;
}

/* Restore saved IRQ & FIQ state */

static inline void irqrestore(irqstate_t flags)
{
  __asm__ __volatile__
    (
     "msr    cpsr_c, %0"
     :
     : "r" (flags)
     : "memory");
}

static inline void system_call(swint_t func, int parm1,
			       int parm2, int parm3)
{
  __asm__ __volatile__
    (
     "mov\tr0,%0\n\t"
     "mov\tr1,%1\n\t"
     "mov\tr2,%2\n\t"
     "mov\tr3,%3\n\t"
     "swi\t0x900001\n\t"
     :
     : "r" ((long)(func)),  "r" ((long)(parm1)),
       "r" ((long)(parm2)), "r" ((long)(parm3))
     : "r0", "r1", "r2", "r3", "lr");
}
#endif

/************************************************************
 * Public Variables
 ************************************************************/

/************************************************************
 * Public Function Prototypes
 ************************************************************/

#ifndef __ASSEMBLY__
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
#endif

#endif /* __ARCH_DM320_IRQ_H */

