/****************************************************************************
 * common/up_internal.h
 *
 *   Copyright (C) 2007, 2008 Gregory Nutt. All rights reserved.
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

#ifndef __UP_INTERNAL_H
#define __UP_INTERNAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <arch/irq.h>
#include "chip/chip.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Bring-up debug configurations.  These are here (vs defconfig)
 * because these should only be controlled during low level
 * board bring-up and not part of normal platform configuration.
 */

#undef  CONFIG_SUPPRESS_INTERRUPTS    /* Do not enable interrupts */
#undef  CONFIG_SUPPRESS_TIMER_INTS    /* No timer */
#undef  CONFIG_SUPPRESS_SERIAL_INTS   /* Console will poll */
#undef  CONFIG_SUPPRESS_UART_CONFIG   /* Do not reconfig UART */
#undef  CONFIG_DUMP_ON_EXIT           /* Dump task state on exit */

/* Macros for portability */

#define IN_INTERRUPT             (current_regs != NULL)
#define SAVE_IRQCONTEXT(tcb)     up_copystate((tcb)->xcp.regs, current_regs)
#define SET_IRQCONTEXT(tcb)      up_copystate(current_regs, (tcb)->xcp.regs)
#define SAVE_USERCONTEXT(tcb)    up_saveusercontext((tcb)->xcp.regs)
#define RESTORE_USERCONTEXT(tcb) up_restoreusercontext((tcb)->xcp.regs)
#define SIGNAL_RETURN(regs)      up_restoreusercontext(regs)

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__
typedef void (*up_vector_t)(void);
#endif

/****************************************************************************
 * Public Variables
 ****************************************************************************/

#ifndef __ASSEMBLY__
/* This holds a references to the current interrupt level
 * register storage structure.  If is non-NULL only during
 * interrupt processing.
 */

extern uint16 *current_regs;
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* Defined in up_copystate.c */

extern void up_copystate(FAR chipreg_t *dest, FAR const chipreg_t *src);

/* Defined in up_saveusercontext.asm */

extern int up_saveusercontext(chipreg_t *regs);

/* Defined in up_restoreusercontext.asm */

extern int up_restoreusercontext(chipreg_t *regs);

/* Supplied by board-specific logic */

extern FAR chipreg_t *up_decodeirq(uint8 rstno, FAR chipreg_t *regs);
extern void up_irqinitialize(void);
extern int  up_timerisr(int irq, FAR chipreg_t *regs);
extern void up_lowputc(char ch) naked_function;
extern char up_lowgetc(void) naked_function;

/* Defined in up_doirq.c */

extern void up_doirq(int irq, FAR chipreg_t *regs);

/* Define in up_sigdeliver */

extern void up_sigdeliver(void);

/* Defined in up_allocateheap.c */

#if CONFIG_MM_REGIONS > 1
void up_addregion(void);
#endif

/* Defined in up_serial.c */

#if CONFIG_NFILE_DESCRIPTORS > 0
extern void up_earlyserialinit(void);
extern void up_serialinit(void);
#else
# define up_earlyserialinit()
# define up_serialinit()
#endif

/* Defined in up_timerisr.c */

extern void up_timerinit(void);

/* Defined in board/up_leds.c */

#ifdef CONFIG_ARCH_LEDS
extern void up_ledinit(void);
extern void up_ledon(int led);
extern void up_ledoff(int led);
#else
# define up_ledinit()
# define up_ledon(led)
# define up_ledoff(led)
#endif

/* Defined in board/up_network.c */

#ifdef CONFIG_NET
extern void up_netinitialize(void);
#else
# define up_netinitialize()
#endif

/* Return the current value of the stack pointer (used in stack dump logic) */

extern uint16 up_getsp(void);

/* Dump stack and registers */

#ifdef CONFIG_ARCH_STACKDUMP
extern void up_stackdump(void);
extern void up_registerdump(void);
#else
# define up_stackdump()
# define up_registerdump()
#endif

#endif /* __ASSEMBLY__ */

#endif  /* __UP_INTERNAL_H */
