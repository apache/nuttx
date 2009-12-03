/************************************************************************************
 * arch/hc/src/mc9s12ne64/mc9s12ne64_vectors.S
 * arch/hc/src/chip/mc9s12ne64_vectors.S
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <arch/irq.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/************************************************************************************
 * Global Symbols
 ************************************************************************************/

	.globl		__start
	.file		"mc9shc12ne64_vectors.S"

/************************************************************************************
 * Macros
 ************************************************************************************/

	.macro	HANDLER, label, irqno
\label:
	.endm

/************************************************************************************
 * Vectors
 ************************************************************************************/

	.section	.vectors, "x"
	.align		2
	.globl		hc12_vectors
	.type		hc12_vectors, function

hc12_vectors:
											/* ff80-ff9f: Reserved */
	.hword		villegal					/* ff80: Reserved*/
	.hword		villegal					/* ff82: Reserved */
	.hword		villegal					/* ff84: Reserved */
	.hword		villegal					/* ff86: Reserved */
	.hword		villegal					/* ff88: Reserved */
	.hword		villegal					/* ff8a: Reserved */
	.hword		villegal					/* ff8c: Reserved */
	.hword		villegal					/* ff9e: Reserved */
	.hword		villegal					/* ff90: Reserved */
	.hword		villegal					/* ff92: Reserved */
	.hword		villegal					/* ff94: Reserved */
	.hword		villegal					/* ff96: Reserved */
	.hword		villegal					/* ff98: Reserved */
	.hword		villegal					/* ff9a: Reserved */
	.hword		villegal					/* ff9c: Reserved */
	.hword		villegal					/* ff9e: Reserved */
	.hword		vemacec 					/* ffa0: EMAC excessive collision */
	.hword		vemaclc 					/* ffa2: EMAC late collision*/
	.hword		vemacbrxerr 				/* ffa4: MAC babbling receive error*/
	.hword		vemacrxbbo					/* ffa6: EMAC receive buffer B overrun */
	.hword		vemacrxbao					/* ffa8: EMAC receive buffer A overrun */
	.hword		vemacrxerr					/* ffaa: EMAC receive error */
	.hword		vemacmii					/* ffac: EMAC MII management transfer complete */
	.hword		vemacrxfc					/* ffae: EMAC receive flow control */
	.hword		vemactxc					/* ffb0: EMAC frame transmission complete */
	.hword		vemaccrxbbc 				/* ffb2: EMAC receive buffer B complete */
	.hword		vemaccrxbac 				/* ffb4: EMAC receive buffer A complete */
	.hword		vephy						/* ffb6: EPHY interrupt */
	.hword		vflash						/* ffb8: FLASH */
	.hword		villegal					/* ffba: Reserved */
	.hword		villegal					/* ffbc: Reserved */
	.hword		villegal					/* ffbe: Reserved */
	.hword		viic						/* ffc0: IIC bus */
	.hword		villegal					/* ffc2: Reserved */
	.hword		vcrgscm 					/* ffc4: CRG self clock mode */
	.hword		vcrgplllck					/* ffc6: CRG PLL lock */
	.hword		villegal					/* ffc8: Reserved */
	.hword		vportg						/* ffca: Port G */
	.hword		vporth						/* ffcc: Port H */
	.hword		vportj						/* ffcd: Port J */
	.hword		villegal					/* ffd0: Reserved */
	.hword		vatd						/* ffd2: ATD */
	.hword		vsci1						/* ffd4: SCI1 */
	.hword		vsci0						/* ffd6: SCI0 */
	.hword		vspi						/* ffd8: SPI */
	.hword		vtimpaie					/* ffda: Pulse accumulator input edge */
	.hword		vtimpaovf					/* ffdc: Pulse accumulator overflow */
	.hword		vtimovf 					/* ffde: Standard timer overflow */
	.hword		vtimch7 					/* ffe0: Standard timer channel 7 */
	.hword		vtimch6 					/* ffe2: Standard timer channel 6 */
	.hword		vtimch5 					/* ffe4: Standard timer channel 5 */
	.hword		vtimch4 					/* ffe6: Standard timer channel 4 */
	.hword		villegal					/* ffe8: Reserved */
	.hword		villegal					/* ffea: Reserved */
	.hword		villegal					/* ffec: Reserved */
	.hword		villegal					/* ffee: Reserved */
	.hword		vrti						/* fff0: Real-time interrupt */
	.hword		virq						/* fff2: IRQ */
	.hword		vxirq						/* fff4: XIRQ */
	.hword		vswi						/* fff6: SWI */
	.hword		vtrap						/* fff8: Unimplemented instruction trap */
	.hword		vcop						/* fffa: COP failure reset */
	.hword		vclkmon 					/* fffc: Clock monitor fail reset */
	.hword		__start 					/* fffe: Reset vector */

	.size	hc12_vectors, .-hc12_vectors

/************************************************************************************
 * .text
 ************************************************************************************/

	.text
	.type	handlers, function
handlers:

	HANDLER vemacec, HC12_IRQ_VEMACEC			/* EMAC excessive collision */
	HANDLER vemaclc, HC12_IRQ_VEMACLC			/* EMAC late collision */
	HANDLER vemacbrxerr, HC12_IRQ_VEMACBRXERR	/* EMAC babbling receive error */
	HANDLER vemacrxbbo, HC12_IRQ_VEMACRXBBO 	/* EMAC receive buffer B overrun */
	HANDLER vemacrxbao, HC12_IRQ_VEMACRXBAO 	/* EMAC receive buffer A overrun */
	HANDLER vemacrxerr, HC12_IRQ_VEMACRXERR 	/* EMAC receive error */
	HANDLER vemacmii, HC12_IRQ_VEMACMII 		/* EMAC MII management transfer complete */
	HANDLER vemacrxfc, HC12_IRQ_VEMACRXFC		/* EMAC receive flow control */
	HANDLER vemactxc, HC12_IRQ_VEMACTXC 		/* EMAC frame transmission complete */
	HANDLER vemaccrxbbc, HC12_IRQ_VEMACCRXBBC	/* EMAC receive buffer B complete */
	HANDLER vemaccrxbac, HC12_IRQ_VEMACCRXBAC	/* EMAC receive buffer A complete */
	HANDLER vephy, HC12_IRQ_VEPHY				/* EPHY interrupt */
	HANDLER vflash, HC12_IRQ_VFLASH 			/* FLASH */
	HANDLER viic, HC12_IRQ_VIIC 				/* IIC bus */
	HANDLER vcrgscm, HC12_IRQ_VCRGSCM			/* CRG self clock mode */
	HANDLER vcrgplllck, HC12_IRQ_VCRGPLLLCK 	/* CRG PLL lock */
	HANDLER vportg, HC12_IRQ_VPORTG 			/* Port G */
	HANDLER vporth, HC12_IRQ_VPORTH 			/* Port H */
	HANDLER vportj, HC12_IRQ_VPORTJ 			/* Port J */
	HANDLER vatd, HC12_IRQ_VATD 				/* ATD */
	HANDLER vsci1, HC12_IRQ_VSCI1				/* SCI1 */
	HANDLER vsci0, HC12_IRQ_VSCI0				/* SCI0 */
	HANDLER vspi, HC12_IRQ_VSPI 				/* SPI */
	HANDLER vtimpaie, HC12_IRQ_VTIMPAIE 		/* Pulse accumulator input edge */
	HANDLER vtimpaovf, HC12_IRQ_VTIMPAOVF		/* Pulse accumulator overflow */
	HANDLER vtimovf, HC12_IRQ_VTIMOVF			/* Standard timer overflow */
	HANDLER vtimch7, HC12_IRQ_VTIMCH7			/* Standard timer channel 7 */
	HANDLER vtimch6, HC12_IRQ_VTIMCH6			/* Standard timer channel 6 */
	HANDLER vtimch5, HC12_IRQ_VTIMCH5			/* Standard timer channel 5 */
	HANDLER vtimch4, HC12_IRQ_VTIMCH4			/* Standard timer channel 4 */
	HANDLER vrti, HC12_IRQ_VRTI 				/* Real-time interrupt */
	HANDLER virq, HC12_IRQ_VIRQ 				/* IRQ */
	HANDLER vxirq, HC12_IRQ_VXIRQ				/* XIRQ */
	HANDLER vswi, HC12_IRQ_VSWI 				/* SWI */
	HANDLER vtrap, HC12_IRQ_VTRAP				/* Unimplemented instruction trap */
	HANDLER vcop, HC12_IRQ_VCOP 				/* COP failure reset*/
	HANDLER vclkmon, HC12_IRQ_VCLKMON			/* Clock monitor fail reset */
	HANDLER villegal, HC12_IRQ_VILLEGAL 		/* Any reserved vector */

/************************************************************************************
 *	Common IRQ handling logic
 *
 * Description:
 *	 To be provided
 *
 ************************************************************************************/

vcommon:
	.size	handlers, .-handlers

/************************************************************************************
 *	Name: up_interruptstack/g_userstack
 *
 * Description:
 *	 Shouldn't happen
 *
 ************************************************************************************/

#if CONFIG_ARCH_INTERRUPTSTACK > 1
		.bss
		.align	2
up_interruptstack:
		.skip	(CONFIG_ARCH_INTERRUPTSTACK & ~1)
up_interruptstack_base:
		.size	up_interruptstack, .-up_interruptstack
#endif

/************************************************************************************
 * .rodata
 ************************************************************************************/

	.section	.rodata, "a"

/* Variables: _sbss is the start of the BSS region (see ld.script) _ebss is the end
 * of the BSS regsion (see ld.script). The idle task stack starts at the end of BSS
 * and is of size CONFIG_IDLETHREAD_STACKSIZE.	The IDLE thread is the thread that
 * the system boots on and, eventually, becomes the idle, do nothing task that runs
 * only when there is nothing else to run.	The heap continues from there until the
 * end of memory.  See g_heapbase below.
 */

		.globl	g_heapbase
		.type	g_heapbase, object
g_heapbase:
		.long	_ebss+CONFIG_IDLETHREAD_STACKSIZE
		.size	g_heapbase, .-g_heapbase

		.end

