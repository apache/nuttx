/************************************************************************************
 * arch/sh/src/m16c/chip.h
 *
 *	 Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *	 Author: Gregory Nutt <spudmonkey@racsa.co.cr>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *		notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *		notice, this list of conditions and the following disclaimer in
 *		the documentation and/or other materials provided with the
 *		distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *		used to endorse or promote products derived from this software
 *		without specific prior written permission.
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
#include "chip.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/************************************************************************************
 * Macro Definitions
 ************************************************************************************/

/* Zero a block of memory */

 	.macro	m16c_bzero, start, end
	mov.b	#0x00, R0L
	mov.w	#(\start & 0x0ffff), A1
	mov.w	#(\end & 0x0ffff), R3
	sub.w	A1, R3
	sstr.b	
	.endm

/* Copy a block of memory */

	.macro	m16c_memcpy, dest, src, end
	mov.w	#(\dest & 0x0ffff),A0
	mov.b	#(\dest >> 16), R1H
	mov.w	#\src, A1
	mov.w	#(\end & 0x0ffff), R3
	sub.w	A1, R3
	smovf.b
	.endm

/* Special page vectors. This macro puts the jump address of	
 * functions defined as special page into the special page vector table.
 * See example calls below and see the M16C Software Manual or NC30
 * manual for more information on special page vectors.
 */
#if 0
 	.macro	m16c_special, num
	.org	0x0ffffe-(\num*2)
	.globl	__SPECIAL_\num
	.word	__SPECIAL_\num	& 0x0ffff
	.endm
#enif

/************************************************************************************
 * Data
 ************************************************************************************/

/* The near RAM memory map is as follows:
 * 
 * 0x00400 - DATA		Size: Determined by linker
 *           BSS		Size: Determined by linker
 *           Idle stack		Size: CONFIG_IDLETHREAD_STACKSIZE
 *           Interrupt stack	Size: CONFIG_ARCH_INTERRUPTSTACK
 *           Heap		Size: Everything remaining
 * 0x00bff - (end+1)
 */

	.data
#if 0
	.globl	_g_stackbase
	.type	_g_stackbase, object
_g_stackbase:
	.word	_enbss
	.size	_g_heapbase, .-_g_heapbase
	.size	_g_istackbase, .-_g_istackbase

	.globl	_g_istackbase
	.type	_g_istackbase, object
_g_istackbase:
	.word	_enbss+CONFIG_IDLETHREAD_STACKSIZE
	.size	_g_istackbase, .-_g_istackbase
#endif
	.globl	_g_heapbase
	.type	_g_heapbase, object
_g_heapbase:
	.word	_enbss+CONFIG_IDLETHREAD_STACKSIZE+CONFIG_ARCH_INTERRUPTSTACK
	.size	_g_heapbase, .-_g_heapbase

/************************************************************************************
 * Interrupt Vectors
 ************************************************************************************/

/* Variable vector section */

	.section varvects			/* Variable vector table */
	.globl	_m16c_unexpected_isr
	.globl	_m16c_brk_isr
	.long	_m16c_brk_isr			/* ffd00: BRK instruction */
	.long	0xffffffff			/* ffd04 */
	.long	0xffffffff			/* ffd08 */
	.long	0xffffffff			/* ffd0c */
	.globl	_m16c_int3_isr
	.long	_m16c_int3_isr			/* ffd10: INT3 */
#ifdef CONFIG_M16C_SWINTS
	.globl	_m16c_swint5_isr
	.long	_m16c_swint5_isr		/* ffd14: S/W interrupt 5 */
	.globl	_m16c_swint6_isr
	.long	_m16c_swint6_isr		/* ffd18: S/W interrupt 6 */
	.globl	_m16c_swint7_isr
	.long	_m16c_swint7_isr		/* ffd1c: S/W interrupt 7 */
#else
	.long	_m16c_unexpected_isr		/* ffd14: Reserved */
	.long	_m16c_unexpected_isr		/* ffd18: Reserved */
	.long	_m16c_unexpected_isr		/* ffd1c: Reserved */
#endif
	.globl	_m16c_int5_isr
	.long	_m16c_int5_isr			/* ffd20: INT5 */
	.globl	_m16c_int4_isr
	.long	_m16c_int4_isr			/* ffd24: INT4 */
	.globl	_m16c_uart2bcd_isr
	.long	_m16c_uart2bcd_isr		/* ffd28: UART2 bus collision detection */
	.globl	_m16c_dma0_isr
	.long	_m16c_dma0_isr			/* ffd2c: DMA0 */
	.globl	_m16c_dma1_isr
	.long	_m16c_dma1_isr			/* ffd30: DMA1 */
	.globl	_m16c_keyinp_isr
	.long	_m16c_keyinp_isr		/* ffd34: Key input interrupt */
	.globl	_m16c_adc_isr
	.long	_m16c_adc_isr			/* ffd38: A-D */
	.globl	_m16c_uart2xmitnack2_isr
	.long	_m16c_uart2xmitnack2_isr	/* ffd3c UART2 transmit/NACK2 */
	.globl	_m16c_uart2rcvack2_isr
	.long	_m16c_uart2rcvack2_isr		/* ffd40: UART2 receive/ACK2 */
	.globl	_m16c_uart0xmit_isr
	.long	_m16c_uart0xmit_isr		/* ffd44: UART0 transmit */
	.globl	_m16c_uart0rcv_isr
	.long	_m16c_uart0rcv_isr		/* ffd48: UART0 receive */
	.globl	_m16c_uart1xmit_isr
	.long	_m16c_uart1xmit_isr		/* ffd4c: UART1 transmit */
	.globl	_m16c_uart1rcv_isr
	.long	_m16c_uart1rcv_isr		/* ffd50: UART1 receive */
	.globl	_m16c_tmra0_isr
	.long	_m16c_tmra0_isr			/* ffd54: Timer A0 */
	.globl	_m16c_tmra1_isr
	.long	_m16c_tmra1_isr			/* ffd58: Timer A1 */
	.globl	_m16c_tmra2_isr
	.long	_m16c_tmra2_isr			/* ffd5c: Timer A2 */
	.globl	_m16c_tmra3_isr
	.long	_m16c_tmra3_isr			/* ffd60: Timer A3 */
	.globl	_m16c_tmra4_isr
	.long	_m16c_tmra4_isr			/* ffd64: Timer A4 */
	.globl	_m16c_tmrb0_isr
	.long	_m16c_tmrb0_isr			/* ffd68: Timer B0 */
	.globl	_m16c_tmrb1_isr
	.long	_m16c_tmrb1_isr			/* ffd6c: Timer B1 */
	.globl	_m16c_tmrb2_isr
	.long	_m16c_tmrb2_isr			/* ffd70: Timer B2 */
	.globl	_m16c_int0_isr
	.long	_m16c_int0_isr			/* ffd74: INT0 */
	.globl	_m16c_int1_isr
	.long	_m16c_int1_isr			/* ffd78: INT1 */
#ifdef CONFIG_M16C_SWINTS
	.globl	_m16c_swint31_isr
	.long	_m16c_swint31_isr		/* ffd7c: S/W interrupt 31 */
	.globl	_m16c_swint32_isr
	.long	_m16c_swint32_isr		/* ffd80: S/W interrupt 32 */
	.globl	_m16c_swint33_isr
	.long	_m16c_swint33_isr		/* ffd84: S/W interrupt 33 */
	.globl	_m16c_swint34_isr
	.long	_m16c_swint34_isr		/* ffd88: S/W interrupt 34 */
	.globl	_m16c_swint35_isr
	.long	_m16c_swint35_isr		/* ffd8c: S/W interrupt 35 */
	.globl	_m16c_swint36_isr
	.long	_m16c_swint36_isr		/* ffd90: S/W interrupt 36 */
	.globl	_m16c_swint37_isr
	.long	_m16c_swint37_isr		/* ffd94: S/W interrupt 37 */
	.globl	_m16c_swint38_isr
	.long	_m16c_swint38_isr		/* ffd98: S/W interrupt 38 */
	.globl	_m16c_swint39_isr
	.long	_m16c_swint39_isr		/* ffd9c: S/W interrupt 39 */
	.globl	_m16c_swint40_isr
	.long	_m16c_swint40_isr		/* ffda0: S/W interrupt 40 */
	.globl	_m16c_swint41_isr
	.long	_m16c_swint41_isr		/* ffda4: S/W interrupt 41 */
	.globl	_m16c_swint42_isr
	.long	_m16c_swint42_isr		/* ffda8: S/W interrupt 42 */
	.globl	_m16c_swint43_isr
	.long	_m16c_swint43_isr		/* ffdac: S/W interrupt 43 */
	.globl	_m16c_swint44_isr
	.long	_m16c_swint44_isr		/* ffdb0: S/W interrupt 44 */
	.globl	_m16c_swint45_isr
	.long	_m16c_swint45_isr		/* ffdb4: S/W interrupt 45 */
	.globl	_m16c_swint46_isr
	.long	_m16c_swint46_isr		/* ffdb8: S/W interrupt 46 */
	.globl	_m16c_swint47_isr
	.long	_m16c_swint47_isr		/* ffdbc: S/W interrupt 47 */
	.globl	_m16c_swint48_isr
	.long	_m16c_swint48_isr		/* ffdc0: S/W interrupt 48 */
	.globl	_m16c_swint49_isr
	.long	_m16c_swint49_isr		/* ffdc4: S/W interrupt 49 */
	.globl	_m16c_swint50_isr
	.long	_m16c_swint50_isr		/* ffdc8: S/W interrupt 50 */
	.globl	_m16c_swint51_isr
	.long	_m16c_swint51_isr		/* ffdcc: S/W interrupt 51 */
	.globl	_m16c_swint52_isr
	.long	_m16c_swint52_isr		/* ffdd0: S/W interrupt 52 */
	.globl	_m16c_swint53_isr
	.long	_m16c_swint53_isr		/* ffdd4: S/W interrupt 53 */
	.globl	_m16c_swint54_isr
	.long	_m16c_swint54_isr		/* ffdd8: S/W interrupt 54 */
	.globl	_m16c_swint55_isr
	.long	_m16c_swint55_isr		/* ffddc: S/W interrupt 55 */
	.globl	_m16c_swint56_isr
	.long	_m16c_swint56_isr		/* ffde0: S/W interrupt 56 */
	.globl	_m16c_swint57_isr
	.long	_m16c_swint57_isr		/* ffde4: S/W interrupt 57 */
	.globl	_m16c_swint58_isr
	.long	_m16c_swint58_isr		/* ffde8: S/W interrupt 58 */
	.globl	_m16c_swint59_isr
	.long	_m16c_swint59_isr		/* ffdec: S/W interrupt 59 */
	.globl	_m16c_swint60_isr
	.long	_m16c_swint60_isr		/* ffdf0: S/W interrupt 60 */
	.globl	_m16c_swint61_isr
	.long	_m16c_swint61_isr		/* ffdf4: S/W interrupt 61 */
	.globl	_m16c_swint62_isr
	.long	_m16c_swint62_isr		/* ffdf8: S/W interrupt 62 */
	.globl	_m16c_swint63_isr
	.long	_m16c_swint63_isr		/* ffdfc: S/W interrupt 63 */
#else
	.long	_m16c_unexpected_isr		/* ffd7c: Reserved */
	.long	_m16c_unexpected_isr		/* ffd80: Not supported */
	.long	_m16c_unexpected_isr		/* ffd84: Not supported */
	.long	_m16c_unexpected_isr		/* ffd88: Not supported */
	.long	_m16c_unexpected_isr		/* ffd8c: Not supported */
	.long	_m16c_unexpected_isr		/* ffd90: Not supported */
	.long	_m16c_unexpected_isr		/* ffd94: Not supported */
	.long	_m16c_unexpected_isr		/* ffd98: Not supported */
	.long	_m16c_unexpected_isr		/* ffd9c: Not supported */
	.long	_m16c_unexpected_isr		/* ffda0: Not supported */
	.long	_m16c_unexpected_isr		/* ffda4: Not supported1 */
	.long	_m16c_unexpected_isr		/* ffda8: Not supported */
	.long	_m16c_unexpected_isr		/* ffdac: Not supported */
	.long	_m16c_unexpected_isr		/* ffdb0: Not supported */
	.long	_m16c_unexpected_isr		/* ffdb4: Not supported */
	.long	_m16c_unexpected_isr		/* ffdb8: Not supported */
	.long	_m16c_unexpected_isr		/* ffdbc: Not supported */
	.long	_m16c_unexpected_isr		/* ffdc0: Not supported */
	.long	_m16c_unexpected_isr		/* ffdc4: Not supported */
	.long	_m16c_unexpected_isr		/* ffdc8: Not supported */
	.long	_m16c_unexpected_isr		/* ffdcc: Not supported1 */
	.long	_m16c_unexpected_isr		/* ffdd0: Not supported */
	.long	_m16c_unexpected_isr		/* ffdd4: Not supported3 */
	.long	_m16c_unexpected_isr		/* ffdd8: Not supported */
	.long	_m16c_unexpected_isr		/* ffddc: Not supported */
	.long	_m16c_unexpected_isr		/* ffde0: Not supported */
	.long	_m16c_unexpected_isr		/* ffde4: Not supported7 */
	.long	_m16c_unexpected_isr		/* ffde8: Not supported */
\	.long	_m16c_unexpected_isr		/* ffdec: Not supported */
	.long	_m16c_unexpected_isr		/* ffdf0: Not supported0 */
	.long	_m16c_unexpected_isr		/* ffdf4: Not supported */
	.long	_m16c_unexpected_isr		/* ffdf8: Not supported */
	.long	_m16c_unexpected_isr		/* ffdfc: Not supported */
#endif

/* Fixed vector section
 *
 * The fixed vector table begins at address ffe00.  The firt portion
 * of the fixed vector table is the special page table.  This table
 * is not currently used.
 */
 	.section specpg				/* Special page table */
 
	.section fixvects			/* Fixed vector table */
	.globl	_m16c_undefinst_irq
	.long	_m16c_undefinst_irq		/* fffdc: Undefined instruction */
	.globl	_m16c_overflow_irq
	.long	_m16c_overflow_irq		/* fffe0: Overflow */
	.globl	_m16c_brkinst_irq
	.long	_m16c_brkinst_irq		/* fffe4: BRK instruction */
	.globl	_m16c_addrmatch_irq
	.long	_m16c_addrmatch_irq		/* fffe8: Address match */
#ifdef CONFIG_M16C_DEBUGGER
	.globl	_m16c_sstep_irq
	.long	_m16c_sstep_irq			/* fffec: Single step */
#else
	.long	_m16c_unexpected_isr		/* fffec: Not supported */
#endif
	.globl	_m16c_wdog_irq
	.long	_m16c_wdog_irq			/* ffff0: Watchdog timer */
#ifdef CONFIG_M16C_DEBUGGER
	.globl	_m16c_dbc_irq
	.long	_m16c_dbc_irq			/* ffff4: DBC */
#else
	.long	_m16c_unexpected_isr		/* ffff4: Not supported */
#endif
	.globl	_m16c_nmi_irq
	.long	_m16c_nmi_irq			/* ffff8: NMI */
	.long	__start				/* ffffc: Reset */

/************************************************************************************
 * Code
 ************************************************************************************/
/************************************************************************************
 * Name: _start
 *
 * Description:
 *   After reset, program execution starts here.
 *
 ************************************************************************************/

 	.text
	.global __start
	.type	__start, #function
__start:

/* Set the interrupt and user stack pointers */

	mov.w	#_enbss, R0
	ldc	R0, usp			/* Set the user stack pointer */
	add.w	#CONFIG_IDLETHREAD_STACKSIZE, R0
	ldc	R0, isp			/* Set interrupt thread stack pointer to the end of BSS */
	fset	U			/* Set bit 7 (U) to select the user stack pointer */

/* Set BCLK speed. At reset, the processor clock  (BLCK) defaults to a divisor of 8.
 * This sets clock to F1 (divide by 1) on XIN: BCLK = XIN frequency.
 */

 	mov.b	#0x01h, M16C_PRCR	/* Unprotect CM0 to change clock setting */
	mov.b	#0x08h, M16C_CM0	/* enable CM17 and CM16 to set BCLK to F1
					 * CM17 & CM16 defaults to 0 after reset and
					 * so we only need to reset CM06 to 0 */
	mov.b	#0x00,M16C_PRCR		/* protect CM0 */

/* The two MS bits of the interrupt cause select register must be set to
 * enable the use of INT4 and INT5
 */

 	mov.b	#0xc0, M16C_IFSR	/* Set b7 & b6 if application will use INT4 & INT5 */
	ldc	#M16C_IRAM_BASE, sb	/* Set sb register (to what?) */
	ldintb	#_svarvect

/* Clear .bss sections */

	m16c_bzero	_snbss, _enbss
#ifdef CONFIG_M16C_HAVEFARRAM
	m16c_bzero	_sfbss, _efbss
#endif

/* Initialize .data sections */

	m16c_memcpy	_sndata, _enronly, _endata
#ifdef CONFIG_M16C_HAVEFARRAM
	m16c_memcpy	_sfdata, _efronly, _efdata
#endif

/* Pass control to NuttX */

	.glbl	_os_start
	jsr.a	_os_start

/* NuttX will not return, but just in case... */

.Lreturned:
	jmp	.Lreturned
