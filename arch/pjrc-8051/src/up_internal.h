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

/**************************************************************************
 * Public Definitions
 **************************************************************************/

/* Memory Map
 *
 * BEGIN  END     DESCRIPTION
 * 0x0000 0x1fff  CODE:   ROM containg PAULMON2
 *                DATA:   RAM for program variables
 * 0x2000 0x7fff  COMMON: RAM for program code or
 *                        variables
 * 0x8000 0xf7ff  COMMON: FLASH for program code
 * 0xf800 0xfeff  COMMON: Peripherals
 * 0xff00 0xffff  COMMON: unused
 *
 * Program code may be loaded at the RAM location 0x2000-0x7fff
 * for testing.  If loaded into the FLASH location at
 * 0x8000-0xf7ff, PAULMON2 will automatically write the program
 * into flash.  The program is configured in the RAM-based test
 * configuration:
 */

#define RAM_BLOCK_START  IRAM_SIZE
#define RAM_BLOCK_END    0x1fff

#define PROGRAM_BASE     0x2000
#define PROGRAM_END      0x7fff

#define FLASH_BASE       0x8000
#define FLASH_END        0xf7ff

/* Well-known entry points to access PAULMON2's built-in functions */

#define PM2_ENTRY_PHEX1   0x002e
#define PM2_ENTRY_COUT    0x0030
#define PM2_ENTRY_CIN     0x0032
#define PM2_ENTRY_PHEX    0x0034
#define PM2_ENTRY_PHEX16  0x0036
#define PM2_ENTRY_PSTR    0x0038
#define PM2_ENTRY_ESC     0x003e
#define PM2_ENTRY_UPPER   0x0040
#define PM2_ENTRY_PINT8U  0x004D
#define PM2_ENTRY_PINT8   0x0050
#define PM2_ENTRY_PINT16U 0x0053
#define PM2_ENTRY_NEWLINE 0x0048
#define PM2_ENTRY_PRGM    0x0059
#define PM2_ENTRY_ERBLOCK 0x0067

#define PM2_VECTOR_BASE    PROGRAM_BASE
#define PM2_VECTOR_EXTINT0 (PM2_VECTOR_BASE + 3)
#define PM2_VECTOR_TIMER0  (PM2_VECTOR_BASE + 11)
#define PM2_VECTOR_EXTINT1 (PM2_VECTOR_BASE + 19)
#define PM2_VECTOR_TIMER1  (PM2_VECTOR_BASE + 27)
#define PM2_VECTOR_UART    (PM2_VECTOR_BASE + 35)
#define PM2_VECTOR_TIMER2  (PM2_VECTOR_BASE + 43)

/**************************************************************************
 * Public Types
 **************************************************************************/

/**************************************************************************
 * Public Variables
 **************************************************************************/

#ifndef __ASSEMBLY__

/* This is the top of the stack containing the interrupt stack frame.  It
 * is set when processing an interrupt.  It is also cleared when the
 * interrupt returns so this can also be used like a boolean indication that
 * we are in an interrupt.
 */

extern ubyte g_irqtos;

#endif /* __ASSEMBLY */

/**************************************************************************
 * Public Function Prototypes
 **************************************************************************/

#ifndef __ASSEMBLY__

extern void  up_irqinitialize(void);
extern void  up_restorecontext(FAR struct xcptcontext *context);
extern void up_restorestack(FAR struct xcptcontext *context);
extern ubyte up_savecontext(FAR struct xcptcontext *context);
extern void  up_savestack(FAR struct xcptcontext *context);
extern void  up_timerinit(void);

#endif /* __ASSEMBLY */
#endif /* __ARCH_UP_INTERNAL_H */
