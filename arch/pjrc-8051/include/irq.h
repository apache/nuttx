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

#ifndef __ARCH_IRQ_H
#define __ARCH_IRQ_H

/************************************************************
 * Included Files
 ************************************************************/

/************************************************************
 * Definitions
 ************************************************************/

#define EXT_INT0_IRQ 0
#define TIMER0_IRQ   1
#define EXT_INT1_IRQ 2
#define TIMER1_IRQ   3
#define UART_IRQ     4
#define TIMER2_IRQ   5

#define NR_IRQS      6

/* The stack for all tasks/threads lie at the same position
 * in IRAM.  On context switches, the STACK contents will be
 * copied into the TCB.
 */

#define IRAM_BASE    0x0000
#define IRAM_SIZE    0x0100

#define STACK_BASE   0x0030
#define STACK_SIZE   (IRAM_SIZE - STACK_BASE)

/* This is the form of initial stack frame
 *
 * This initial stack frame will be configured to hold.
 * (1) The 16-bit return address of either:
 *
 *     void task_start(void);
 *     void pthread_start(void)
 *
 *     The return address is stored at the top of stack.
 *     so that the RETI instruction will work:
 *
 *     PC15-8 <- ((SP))
 *     (SP)   <- (SP) -1
 *     PC7-0  <- ((SP))
 *     (SP)   <- (SP) -1
 */

#define FRAME_RETLS 0
#define FRAME_RETMS 1

/* Then a full context save area which can be indexed with
 * the following definitions (relative to the beginning of
 * the initial frame.
 */

#define FRAME_ACC   2
#define FRAME_IE    3
#define FRAME_DPL   4
#define FRAME_DPH   5
#define FRAME_B     6
#define FRAME_R2    7
#define FRAME_R3    8
#define FRAME_R4    9
#define FRAME_R5    10
#define FRAME_R6    11
#define FRAME_R7    12
#define FRAME_R0    13
#define FRAME_R1    14
#define FRAME_PSW   15
#define FRAME_BP    16

#define FRAME_SIZE  17

/************************************************************
 * Public Types
 ************************************************************/

/* This struct defines the way the registers are stored */

#ifndef __ASSEMBLY__
struct xcptcontext
{
   ubyte nbytes;
   ubyte stack[STACK_SIZE];
};
#endif /* __ASSEMBLY */

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

EXTERN irqstate_t irqsave(void) __naked;
EXTERN void       irqrestore(irqstate_t flags) __naked;

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY */
#endif /* __ARCH_IRQ_H */

