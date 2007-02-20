/************************************************************
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
 ************************************************************/

#ifndef __UP_INTERNAL_H
#define __UP_INTERNAL_H

/************************************************************
 * Included Files
 ************************************************************/

/************************************************************
 * Definitions
 ************************************************************/

/************************************************************
 * Public Types
 ************************************************************/

#ifndef __ASSEMBLY__
typedef void (*up_vector_t)(void);
#endif

/************************************************************
 * Public Variables
 ************************************************************/

#ifndef __ASSEMBLY__
extern uint32 *current_regs;
#endif

/************************************************************
 * Inline Functions
 ************************************************************/


/************************************************************
 * Public Functions
 ************************************************************/

#ifndef __ASSEMBLY__

/* Defined in files with the same name as the function */

extern void up_copystate(uint32 *dest, uint32 *src);
extern void up_dataabort(uint32 *regs);
extern void up_doirq(int irq, uint32* regs);
extern void up_fullcontextrestore(uint32 *regs) __attribute__ ((noreturn));
extern void up_irqinitialize(void);
extern void up_prefetchabort(uint32 *regs);
extern int  up_saveusercontext(uint32 *regs);
extern void up_sigdeliver(void);
extern void up_syscall(uint32 *regs);
extern int  up_timerisr(int irq, uint32 *regs);
extern void up_undefinedinsn(uint32 *regs);

#ifdef CONFIG_DEBUG
extern void up_lowputc(char ch);
#else
# define up_lowputc(ch)
#endif

/* Defined in up_vectors.S */

extern void up_vectorundefinsn(void);
extern void up_vectorswi(void);
extern void up_vectorprefetch(void);
extern void up_vectordata(void);
extern void up_vectoraddrexcptn(void);
extern void up_vectorirq(void);
extern void up_vectorfiq(void);

/* Defined in up_serial.c */

extern void up_earlyserialinit(void);
extern void up_serialinit(void);

/* Defined in up_timerisr.c */

extern void up_timerinit(void);

/* Defined in up_irq.c */

extern void up_maskack_irq(int irq);

#endif /* __ASSEMBLY__ */

#endif  /* __UP_INTERNAL_H */
