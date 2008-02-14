/****************************************************************************
 * arch/z80/src/common/up_internal.h
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
 * Conditional Compilation
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <arch/irq.h>
#include "chip/chip.h"
#include "chip/switch.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/* Supplied by chip- or board-specific logic */

EXTERN void up_irqinitialize(void);
EXTERN int  up_timerisr(int irq, FAR chipreg_t *regs);
EXTERN void up_lowputc(char ch) naked_function;
EXTERN char up_lowgetc(void) naked_function;

/* Defined in up_doirq.c */

EXTERN FAR chipreg_t *up_doirq(ubyte irq, FAR chipreg_t *regs);

/* Define in up_sigdeliver */

EXTERN void up_sigdeliver(void);

/* Defined in up_allocateheap.c */

#if CONFIG_MM_REGIONS > 1
void up_addregion(void);
#endif

/* Defined in up_serial.c */

#if CONFIG_NFILE_DESCRIPTORS > 0
EXTERN void up_earlyserialinit(void);
EXTERN void up_serialinit(void);
#else
# define up_earlyserialinit()
# define up_serialinit()
#endif

/* Defined in up_timerisr.c */

EXTERN void up_timerinit(void);

/* Defined in board/up_leds.c */

#ifdef CONFIG_ARCH_LEDS
EXTERN void up_ledinit(void);
EXTERN void up_ledon(int led);
EXTERN void up_ledoff(int led);
#else
# define up_ledinit()
# define up_ledon(led)
# define up_ledoff(led)
#endif

/* Defined in board/up_network.c */

#ifdef CONFIG_NET
EXTERN void up_netinitialize(void);
#else
# define up_netinitialize()
#endif

/* Return the current value of the stack pointer (used in stack dump logic) */

EXTERN uint16 up_getsp(void);

/* Dump stack and registers */

#ifdef CONFIG_ARCH_STACKDUMP
EXTERN void up_stackdump(void);
# define REGISTER_DUMP() _REGISTER_DUMP()
#else
# define up_stackdump()
# define REGISTER_DUMP()
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif  /* __UP_INTERNAL_H */
