/****************************************************************************
 * arch/hc/src/mc9s12ne64/mc9s12ne64_start.S
 * arch/hc/src/chip/mc9s12ne64_start.S
 *
 *	 Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *	 Author: Gregory Nutt <spudmonkey@racsa.co.cr>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *	  notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *	  notice, this list of conditions and the following disclaimer in
 *	  the documentation and/or other materials provided with the
 *	  distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *	  used to endorse or promote products derived from this software
 *	  without specific prior written permission.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

#include "mc9s12ne64_mmc.h"
#include "mc9s12ne64_crg.h"

/****************************************************************************
 * Private Definitions
 ****************************************************************************/

/************************************************************************************
 * Global Symbols
 ************************************************************************************/

	.globl		__start
	.globl		os_start
	.file		"mc9s12ne64_start.S"

/****************************************************************************
 * Macros
 ****************************************************************************/

/* Memory map initialization */

	.macro	MMCINIT
	clr		HC12_MMC_INITRG			/* Set the register map position to 0x0000*/
	nop

	ldab	#0x09
	stab	*HC12_MMC_INITEE		/* Set EEPROM position to 0x0800 */

#ifdef CONFIG_HC12_SERIALMON
	ldab	#0x39					/* Set RAM position to 0x3800 */
#else
	ldab	#0x20					/* Set RAM position to 0x2000*/
#endif
	stab	*HC12_MMC_INITRM
	ldaa	#MMC_MISC_ROMON			/* MISC: EXSTR1=0 EXSTR0=0 ROMHM=0 ROMON=1 */
	staa	*HC12_MMC_MISC		
	.endm

/* System clock initialization */

	.macro	PLLINIT 

	/* Select the clock source from crystal */

	clr 	HC12_CRG_CLKSEL

	/* Set the multipler and divider and enable the PLL */

	bclr	*HC12_CRG_PLLCTL #CRG_PLLCTL_PLLON
	ldab	#15
	stab	HC12_CRG_SYNR
	stab	HC12_CRG_REFDV
	bset	*HC12_CRG_PLLCTL #CRG_PLLCTL_PLLON

	/* Wait for the PLL to lock on */

.Lpll_lock:
	brclr	*HC12_CRG_CRGFLG #CRG_CRGFLG_LOCK .Lpll_lock

	/* Then select the PLL clock source */

	bset	*HC12_CRG_CLKSEL #CRG_CLKSEL_PLLSEL
	.endm


/****************************************************************************
 * .text
 ****************************************************************************/

	.section	nonbanked, "x"

/****************************************************************************
 * Name: __start
 *
 * Description:
 *	 Power-up reset entry point
 *
 ****************************************************************************/

__start:
	/* Hardware setup */

	MMCINIT							/* Initialize the MMC */
	PLLINIT							/* Initialize the PLL */

	/* Setup the stack pointer */

	lds		.Lstackbase

	/* Clear BSS */

	ldx 	.Lsbss					/* Start of .BSS */
	ldd 	.Lebss					/* End+1 of .BSS */

.Lclearbss:
	pshd
	cpx 	2,sp+					/* Check if all BSS has been cleared */
	beq 	.Lbsscleared			/* If so, exit the loop */

	clr 	0,x						/* Clear this byte */
	inx								/* Address the next byte */
	bra 	.Lclearbss				/* And loop until all cleared */
.Lbsscleared:

	/* Initialize the data section */

	ldx		.Lsdata					/* Start of .DATA (destination) */
	movw	.Ledata, 0, sp			/* End of .DATA (destination) */
	ldy 	.Leronly				/* Start of .DATA (source) */

.Linitdata:
	cpx 	0, sp					/* Check if all .DATA has been initialized */
	beq 	.Ldatainitialized		/* If so, exit the loop */
	ldab	0, y					/* Fetch the next byte from the source */
	iny								/* Increment the source address */
	stab	0, x					/* Store the byte to the destination */
	inx								/* Increment the destination address */
	bra 	.Linitdata				/* And loop until all of .DATA is initialized */
.Ldatainitialized:

	/* Now, start the OS */

	call	os_start
	bra		__start

	/* Variables:
	 * _sbss is the start of the BSS region (see ld.script)
	 * _ebss is the end of the BSS regsion (see ld.script)
	 * The idle task stack starts at the end of BSS and is
	 * of size CONFIG_IDLETHREAD_STACKSIZE.  The heap continues
	 * from there until the end of memory.	See g_heapbase
	 * below.
	 */

.Lsbss:
	.long	_sbss
.Lebss:
	.long	_ebss
.Lstackbase:
	.hword	_ebss+CONFIG_IDLETHREAD_STACKSIZE-4
.Leronly:
	.long	_eronly /* Where .data defaults are stored in FLASH */
.Lsdata:
	.long	_sdata	/* Where .data needs to reside in SDRAM */
.Ledata:
	.long	_edata
	.size	__start, .-__start

	/* This global variable is unsigned long g_heapbase and is
	 * exported from here only because of its coupling to LCO
	 * above.
	 */

	.data
	.align	4
	.globl	g_heapbase
	.type	g_heapbase, object
g_heapbase:
	.long	_ebss+CONFIG_IDLETHREAD_STACKSIZE
	.size	g_heapbase, .-g_heapbase

	.end

	.end
