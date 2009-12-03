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
	.file		"mc9s12ne64_start.S"

/****************************************************************************
 * Macros
 ****************************************************************************/

/* Memory map initialization */

	.macro	MMCINIT
	movb	#0, HC12_MMC_INITRG 	/* Set the register map position */
	nop
	movb	#32, HC12_MMC_INITRM	/* Set the RAM map position */
	movb	#1, HC12_MMC_MISC		/* MISC: EXSTR1=0 EXSTR0=0 ROMHM=0 ROMON=1 */
	.endm

/* System clock initialization */

	.macro	PLLINIT 

	/* Select the clock source from crystal */

	movb	#0, HC12_CRG_CLKSEL
	movb	#CRG_CRGSEL_PLLSEL, HC12_CRG_CLKSEL

	/* Set the multipler and divider and enable the PLL */

	movb	#0, HC12_CRG_PLLCTL
	movb	#15, HC12_CRG_SYNR
	movb	#15, HC12_CRG_REFDV 
	movb	#(CRG_PLLCTL_CME|CRG_PLLCTL_PLLON), HC12_CRG_PLLCTL

	/* Wait for the PLL to lock on */

.Lpll_lock:
	ldab	HC12_CRG_CRGFLG
	clra
	andb	#CRG_CRGFLG_LOCK
	ldx		#0
	bne		.Lpll_lock
	tbne	d,.Lpll_lock

	/* The select the PLL clock source */

	bset	HC12_CRG_CLKSEL_OFFSET, #CRG_CLKSEL_PLLSEL
	.endm

/****************************************************************************
 * .text
 ****************************************************************************/

/****************************************************************************
 * Name: __start
 *
 * Description:
 *	 Power-up reset entry point
 *
 ****************************************************************************/

__start:
	MMCINIT		/* Initialize the MMC */
	PLLINIT		/* Initialize the PLL */
	.end
