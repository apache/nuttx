/*****************************************************************************
 * arch/sh/src/sh1/sh1_head.S
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
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
 *****************************************************************************/

/*****************************************************************************
 * Included Files
 *****************************************************************************/

#include <nuttx/config.h>	/* NuttX configuration settings */
#include <arch/board/board.h>	/* Board-specific settings */
#include <arch/irq.h>		/* IRQ definitons */

#include "chip.h"		/* Chip-specific settings */
#include "up_internal.h"
#include "up_arch.h"

/*****************************************************************************
 * Definitions
 *****************************************************************************/

/* This file holds the NuttX start logic that runs when the SH-1/US7032EVB1
 * is reset.  This logic must be located in SRAM at 0x0a00:2000.  On that
 * platform, the entire PROM and the first 8Kb of SRAM are reserved for CMON.
 */

/*****************************************************************************
 * External references
 *****************************************************************************/

/* Called functions */

	.globl	_up_lowsetup		/* Early initialization of UART */
#ifdef CONFIG_USE_EARLYSERIALINIT
	.globl	_up_earlyserialinit	/* Early initialization of serial driver */
#endif
#ifdef CONFIG_ARCH_LEDS
	.globl	_up_ledinit		/* Boot LED setup */
#endif
#ifdef CONFIG_DEBUG
	.globl	_up_lowputc		/* Low-level debug output */
#endif
	.globl	_os_start		/* NuttX entry point */

/* Variables set up by the linker script */

	.globl	_sbss			/* Start of BSS */
	.globl	_ebss			/* End of BSS */
	.globl	_svect			/* Start of the new vector location */

#ifdef CONFIG_BOOT_FROM_FLASH
	.globl	_eronly			/* Where .data defaults are stored in FLASH */
	.global	_sdata			/* Start of .data in RAM */
	.globl	_edata			/* End of .data in RAM */
#endif

/*****************************************************************************
 * Macros
 *****************************************************************************/

/*****************************************************************************
 * Name: showprogress
 *
 * Description:
 *   Print a character on the UART to show boot status. This macro will
 *   modify r0, r1, r2 and r14
 *
 *****************************************************************************/

	.macro	showprogress, code
#ifdef CONFIG_DEBUG
	mov	r0, #\code
	bl	up_lowputc
	mov.l   .Llowputc, r0		/* Address of up_earlyserialinit */
	jsr     @r0			/* Call it */
	or      r0, r0			/* Delay slot */
#endif
	.endm

/*****************************************************************************
 * Vectors
 *****************************************************************************/

	.section	.vect

/*****************************************************************************
 * Name: __vector_table
 *
 * Description:
 *   Interrupt vector table.  The actual vectors are managed by CMON.  For
 *   any non-zero settings in the following table, CMON will redirect interrupt
 *   handling to that function.
 *
 *****************************************************************************/

	.globl		__vector_table
	.type		__vector_table, %object
__vector_table:
	/* Resets */

	.long	__start		/* 0-1: Power-on reset (hard, NMI high) PC & SP */
	.long	_ebss+CONFIG_PROC_STACK_SIZE-4
	.long	__start		/* 2-3: Manual reset (soft, NMI low) PC & SP */
	.long	_ebss+CONFIG_PROC_STACK_SIZE-4

	.rept 252
	.long	0
	.endr

	.size	__vector_table, . - __vector_table

/*****************************************************************************
 * Text
 *****************************************************************************/

	.section	.text

/*****************************************************************************
 * Name: __start
 *
 * Description:
 *   Reset entry point.  This is the first function to execute when the
 *   processor is reset.  It initializes hardware and then gives control to
 *   NuttX.  Nearly all SH-1 resources have already been setup by CMON so all
 *   that is necessary for us to do here is setup the stack pointer and BSS.
 *
 *****************************************************************************/

	.global __start
	.type	__start, #function

__start:
	/* Initialize stack pointer to the preallocated stack */
	mov.l	.Lstack, r15
	
	/* set up the bus controller for the EVB */

	mov.l	.Lwcr1, r0
	sub	r1,r1
	mov.w	r1, @r0

	/* Configure the BSR to use /LBS, /HBS, /WR */

	mov.l   .Lbcr, r0
	mov.w   .Lbas, r1
	bra     __start0
	mov.w   r1, @r0
	
	.align 2
.Lstack:
	.long	_ebss+CONFIG_PROC_STACK_SIZE-4
.Lwcr1:
	.long	0x5ffffa2
.Lbcr:
	.long  0x5ffffa0
.Lbas:
	.word  0x0800

__start0:
	/* Copy the monitor vectors to a002000-a00211f */

	mov	#0, r0			/* R0: Monitor vector table at address 0 in PROM */
	mov.l	.Lsvect, r1		/* R1: Redirected vector table in SRAM */
	mov.l	.Lvectend, r3		/* R3: Copy only up to external interrupts */
1:
	mov.l	@r0, r2			/* R2: Value from mnitor monitor vector table */
	mov.l	r2, @r1			/* Write into SRAM vector table */
	add	#4, r0			/* R0: Address of next vector to read from monitor vector table */
	add	#4, r1			/* R1: Address of next vector to write to SRAM vector table */
	cmp/gt	r0, r3			/* Copy only only up to external interrupts at */
	bt	1b			/* Continue looping until all copied */
	nop				/* Delay slot */

	/* Update the VBR to show new adddress of vector table */

	mov.l	.Lsvect, r0		/* R0: Address of SRAM vector table */
	ldc	r0, vbr			/* Set VBR to start of SRAM vector table */

	/* Initialize data segement */

#ifdef CONFIG_BOOT_FROM_FLASH
	mov.l	.Lsdata, r0		/* R0: Start of .data segment */
	mov.l	.Ledata, r1		/* R1: End+1 of .data segment */
	mov.l	.Leronly, r2		/* R2: Start of FLASH .data segment copy */
2:
	mov.l	@r2, r3			/* R3: Next byte from FLASH copy */
	mov.l	r3, @r0			/* Copy to .data */
	add	#4, r2			/* R2: Address of next byte to read from FLASH */
	add	#4, r0			/* R0: Address to write next byte to .data */
	cmp/gt	r0, r1			/* End of .data? */
	bt	2b			/* Loop until end of data */
	nop				/* Delay slot */
#endif

	/* Clear BSS */

	mov.l   .Lsbss, r0		/* R0: Start of BSS segment */
	mov.l   .Lebss, r1		/* R1: End+1 of BSS segment */
	mov	#0, r2			/* R2: Value = 0 */
3:
	mov.l	r2, @r0			/* Clear the next word in BSS */
	add	#4, r0			/* R0: Address of next byte to clear in BSS */
	cmp/ge	r0, r1			/* End of BSS? */
	bt	3b			/* Loop until the end of BSS */
	nop				/* Delay slot

	/* Configure the uart so that we can get debug output as soon
	 * as possible.
	 */

	mov.l   .Llowsetup, r0		/* Address of up_lowsetup */
	jsr     @r0			/* Call it */
	or      r0, r0			/* Delay slot */

	showprogress 'A'

	/* Perform early serial initialization */

#ifdef CONFIG_USE_EARLYSERIALINIT
	mov.l   .Learlyser, r0		/* Address of up_earlyserialinit */
	jsr     @r0			/* Call it */
	or      r0, r0			/* Delay slot */
#endif

	showprogress 'B'

	/* Call C++ constructors */

#ifdef CONFIG_CPLUSPLUS
#  warning "No C++ support yet"
	showprogress 'C'
#endif
	showprogress '\n'

	/* Initialize onboard LEDs */

#ifdef CONFIG_ARCH_LEDS
	mov.l   .Lledinit, r0		/* Address of up_ledinit */
	jsr     @r0			/* Call it */
	or      r0, r0			/* Delay slot */
#endif

	/* Then jump to NuttX entry */

	mov.l   .Losstart,r0
	jsr     @r0
	or      r0, r0

	/* Shouldn't get here */

	/* Call destructors  -- never get here */

#ifdef CONFIG_CPLUSPLUS
#  warning "No C++ support yet"
#endif

4:  nop
	bra 4b
	nop

	.align 2
#ifdef CONFIG_BOOT_FROM_FLASH
.Leronly:
	.long   _eronly
.Lsdata:
	.long   _sdata
.Ledata:
	.long   _edata
#endif
.Lsbss:  
	.long   _sbss
.Lebss:
	.long   _ebss
#ifdef CONFIG_USE_EARLYSERIALINIT
.Learlyser:
	.long	_up_earlyserialinit
#endif
.Llowsetup:
	.long	_up_lowsetup
#ifdef CONFIG_DEBUG
.Llowputc:
	.long	_up_lowputc
#endif
.Lledinit:
	.long	_up_ledinit
.Losstart:
	.long   _os_start
.Lsvect:
	.long   _svect
.Lvectend:
	.long   (SH1_IRQ7_VECOFFSET+3)

/*****************************************************************************
 * DATA
 *****************************************************************************/

	.section	.data

	/* This global variable is unsigned long g_heapbase and is
	 * exported from here only because of its coupling to the stack
	 * above.
	 */

	.data
	.align	4
	.globl	_g_heapbase
	.type	_g_heapbase, object
_g_heapbase:
	.long	_ebss+CONFIG_PROC_STACK_SIZE
	.size	_g_heapbase, .-_g_heapbase

	.end

