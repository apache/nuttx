/****************************************************************************
 * arch/avr/src/common/up_internal.h
 *
 *   Copyright (C) 2010-2011, 2015, 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

#ifdef CONFIG_ARCH_FAMILY_AVR32
# include "avr32.h"
#else
# include "avr.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Check if an interrupt stack size is configured */

#ifndef CONFIG_ARCH_INTERRUPTSTACK
# define CONFIG_ARCH_INTERRUPTSTACK 0
#endif

/* This is the value used to mark the stack for subsequent stack monitoring
 * logic.
 */

#define STACK_COLOR    's'
#define INTSTACK_COLOR 's'
#define HEAP_COLOR     'h'

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__
typedef void (*up_vector_t)(void);
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__
/* Address of the saved user stack pointer */

#if CONFIG_ARCH_INTERRUPTSTACK > 3
extern void g_intstackbase;
#endif

/* These 'addresses' of these values are setup by the linker script.  They
 * are not actual uint32_t storage locations! They are only used meaningfully
 * in the following way:
 *
 *  - The linker script defines, for example, the symbol_sdata.
 *  - The declareion extern uint32_t _sdata; makes C happy.  C will believe
 *    that the value _sdata is the address of a uint32_t variable _data
 *    (it is not!).
 *  - We can recoved the linker value then by simply taking the address of
 *    of _data.  like:  uint32_t *pdata = &_sdata;
 */

extern uint32_t _stext;           /* Start of .text */
extern uint32_t _etext;           /* End_1 of .text + .rodata */
extern const uint32_t _eronly;    /* End+1 of read only section (.text + .rodata) */
extern uint32_t _sdata;           /* Start of .data */
extern uint32_t _edata;           /* End+1 of .data */
extern uint32_t _sbss;            /* Start of .bss */
extern uint32_t _ebss;            /* End+1 of .bss */

#endif /* __ASSEMBLY__ */

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* Defined in files with the same name as the function */

#ifdef CONFIG_ARCH_DMA
void weak_function up_dma_initialize(void);
#endif
void up_sigdeliver(void);
void up_lowputc(char ch);
void up_puts(const char *str);
void up_lowputs(const char *str);
void up_dumpstate(void);

/* Defined in common/up_allocateheap.c or chip/xxx_allocateheap.c */

#if CONFIG_MM_REGIONS > 1
void up_addregion(void);
#else
# define up_addregion()
#endif

/* Defined in chip/xxx_lowinit.c.  This function is called from the
 * head.S file just before jumping to nx_start().  This function
 * performs whatever very low level initialization that is needed
 * before the OS gets started (clocks, console, LEDs, etc.)
 */

void up_lowinit(void);

/* Defined in chip/xxx_serial.c */

#ifdef CONFIG_DEV_CONSOLE
void up_earlyserialinit(void);
void up_serialinit(void);
#endif

#ifdef CONFIG_RPMSG_UART
void rpmsg_serialinit(void);
#endif

/* Defined in chip/xxx_ethernet.c */

#if defined(CONFIG_NET) && !defined(CONFIG_NETDEV_LATEINIT)
void up_netinitialize(void);
#else
# define up_netinitialize()
#endif

/* Defined in chip/xxx_usbdev.c */

#ifdef CONFIG_USBDEV
void up_usbinitialize(void);
void up_usbuninitialize(void);
#else
# define up_usbinitialize()
# define up_usbuninitialize()
#endif

#endif /* __ASSEMBLY__ */
#endif /* __UP_INTERNAL_H */
