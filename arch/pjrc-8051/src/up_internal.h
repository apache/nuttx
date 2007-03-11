/**************************************************************************
 * up_internal.h
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
 **************************************************************************/

#ifndef __ARCH_UP_INTERNAL_H
#define __ARCH_UP_INTERNAL_H

/**************************************************************************
 * Included Files
 **************************************************************************/

#include <nuttx/config.h>
#ifdef CONFIG_ARCH_PJRC
# include "pjrc.h"
#endif

/**************************************************************************
 * Public Definitions
 **************************************************************************/

/* Bring-up debug configurations */

#define CONFIG_FRAME_DUMP 1             /* Enabled stack/frame dumping logic */
#define CONFIG_SUPPRESS_INTERRUPTS 1    /* Do not enable interrupts */
#define CONFIG_SWITCH_FRAME_DUMP 1      /* Dump frames from normal switches */
#undef  CONFIG_INTERRUPT_FRAME_DUMP     /* Dump frames from interrupt switches */

/**************************************************************************
 * Public Types
 **************************************************************************/

/**************************************************************************
 * Public Variables
 **************************************************************************/

#ifndef __ASSEMBLY__

/* This is the top of the stack containing the interrupt
 * stack frame.  It is set when processing an interrupt.  It
 * is also cleared when the interrupt returns so this can
 * also be used like a boolean indication that we are in an
 * interrupt.
 */

extern ubyte g_irqtos;

/* If during execution of an interrup handler, a context
 * switch must be performed, the follwing will be set to
 * to that address of the relevant context structure.  The
 * actual switch will be deferred until the time that the
 * the interrupt exits.
 */

extern FAR struct xcptcontext *g_irqcontext;

#endif /* __ASSEMBLY */

/**************************************************************************
 * Public Function Prototypes
 **************************************************************************/

#ifndef __ASSEMBLY__

#if CONFIG_MM_REGIONS > 1
extern void  up_addregion(void);
#endif
extern void up_delay(ubyte milliseconds) __naked;
extern void  up_irqinitialize(void);
extern void  up_restorecontext(FAR struct xcptcontext *context) _naked;
extern ubyte up_savecontext(FAR struct xcptcontext *context) __naked;
extern void  up_savestack(FAR struct xcptcontext *context, ubyte tos);
extern void  up_timerinit(void);

/* Defined in up_assert.c */

#ifdef CONFIG_FRAME_DUMP
extern void up_dumpstack(void);
extern void up_dumpframe(FAR struct xcptcontext *context);
#else
# define up_dumpstack()
# define up_dumpframe(x)
#endif

/* Defined in up_leds.c */

#ifdef CONFIG_8051_LEDS
extern void up_ledinit(void);
extern void up_ledon(int led);
extern void up_ledoff(int led);
#else
# define up_ledinit()
# define up_ledon(led)
# define up_ledoff(led)
#endif

#endif /* __ASSEMBLY */
#endif /* __ARCH_UP_INTERNAL_H */
