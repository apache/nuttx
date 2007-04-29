/********************************************************************
 * lpc214x/lpc214x_head.S
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
 ********************************************************************/

/********************************************************************
 * Included Files
 ********************************************************************/

#include <nuttx/config.h>
#include "up_internal.h"
#include "up_arch.h"

/********************************************************************
 * Definitions
 ********************************************************************/

/* This file holds the NuttX start logic that runs when the LPC2148
 * is reset.  This logic must be located at address 0x0000:0000 in
 * flash but may be linked to run at different locations based on
 * the selected mode:
 *
 * default: Executes from 0x0000:0000.  In non-default modes, the
 *   MEMAP register is set override the settings of the CPU configuration
 *   pins.
 *
 *  CONFIG_EXTMEM_MODE: Code executes from external memory starting at
 *    address 0x8000:0000.
 *
 *  CONFIG_RAM_MODE: Code executes from on-chip RAM at address
 *     0x4000:0000.
 *
 * Starupt Code must be linked to run at the correct address
 * corresponding to the selected mode.
 */

#if defined(CONFIG_EXTMEM_MODE)
#  if CONFIG_CODE_BASE != LPC214X_EXTMEM_BASE
#    error "CONFIG_CODE_BASE must be 0x80000000 in EXTMEM mode"
#  endif
#elif defined(CONFIG_RAM_MODE)
#  if CONFIG_CODE_BASE != LPC214X_ONCHIP_RAM_BASE
#    error "CONFIG_CODE_BASE must be 0x40000000 in EXTMEM mode"
#  endif
#else
#  if CONFIG_CODE_BASE != LPC214X_FLASH_BASE
#    error "CONFIG_CODE_BASE must be 0x00000000 in default mode"
#  endif
#endif

/* Phase Locked Loop (PLL) initialization values
 *
 * <e> PLL Setup
 * <i> 
 * <i> CCLK - Processor Clock
 * <i> Fcco - PLL Oscillator
 *   <o1.0..4>   MSEL: PLL Multiplier Selection
 *               <1-32><#-1>
 *               <i> PLL Multiplier "M" Value
 *               <i> CCLK = M * Fosc
 *   <o1.5..6>   PSEL: PLL Divider Selection
 *               <0=> 1   <1=> 2   <2=> 4   <3=> 8
 *               <i> PLL Divider "P" Value
 *               <i> Fcco = CCLK * 2 * P
 *               <i> 156MHz <= Fcco <= 320MHz
 * </e>
*/
#defin LPC214X_PLLCFG_VALUE 0x00000024

/* Memory Accelerator Module (MAM) initialization values
 *
 * MAM Control Register
 *   BIT 0:1 Mode
 *           0 = Disabled
 *           1 = Partially Enabled
 *           2 = Fully Enabled
 * MAM Timing Register
 *   BIT 0:2 Fetch Cycles
 *           0 = Reserved
 *           1 = 1
 *           2 = 2
 *           3 = 3
 *           4 = 4
 *           5 = 5
 *           6 = 6
 *           7 = 7
 */

#define LPC214X_MAMCR_VALUE  0x00000002
#define LPC214x_MAMTIM_VALUE 0x00000004

/* VPBDIV initialization values
 *
 * BITS 0:1 VPB Peripheral Bus Clock Rate
 *          0 = VPB Clock = CPU Clock / 4
 *          1 = VPB Clock = CPU Clock
 *          2 = VPB Clock = CPU Clock / 2
 * BITS 4:5 XCLKDIV: XCLK Pin
 *          0 = XCLK Pin = CPU Clock / 4
 *          1 = XCLK Pin = CPU Clock
 *          2 = XCLK Pin = CPU Clock / 2
 */

#define LPC214X_VPBDIV_VALUE 0x00000001

/* External Memory Pins definitions
 *
 * CS0..3, OE, WE, BLS0..3,  D0..31, A2..23, JTAG Pins
 */

#define LPC214X_PINSEL2_VALUE 0x0e6149e4

/* External Memory Controller (EMC) initialization values
 *
 * Bank Configuration n (BCFG0..3)
 *    BIT 0:3   IDCY: Idle Cycles (0-15)
 *    BIT 5:9   WST1: Wait States 1 (0-31)
 *    BIT 11:15 WST2: Wait States 2 (0-31)
 *    BIT 10    RBLE: Read Byte Lane Enable
 *    BIT 26    WP: Write Protect
 *    BIT 27    BM: Burst ROM
 *    BIT 28:29 MW: Memory Width  (0=8-bit 1=16-bit 2=32-bit 3=Reserved)
 */

#define LPC214X_BCFG0_VALUE   0x0000fbef
#define LPC214X_BCFG1_VALUE   0x0000fbef
#define LPC214X_BCFG2_VALUE   0x0000fbef
#define LPC214X_BCFG3_VALUE   0x0000fbef

/********************************************************************
 * Macros
 ********************************************************************/

/* Print a character on the UART to show boot status. This macro will
 * modify r0, r1, r2 and r14
 */

	.macro	showprogress, code
#ifdef CONFIG_DEBUG
	mov	r0, #\code
	bl	up_lowputc
#endif
	.endm

/* Configured the PINSEL2 register if EXTMEM mode is selected */

	.macro	configpinsel2, base, val
#ifdef CONFIG_EXTMEM_MODE
	ldr	\base, =LPC214X_PINSEL2
	ldr	\val, =LPC214X_PINSEL2_VALUE
	str	\val, [\base]
#endif
	.endm

/* Configure the external memory controller */

	.macro	configemc, base, val
#ifdef CONFIG_EMC_SETUP
	ldr	\base, =LPC214X_EMC_BASE

#ifdef CONFIG_BCFG0_SETUP
	ldr	\val, =LPC214X_BCFG0_VALUE
	str	\val, [\base, #LPC214X_BCFG0_OFFSET]
#endif

#ifdef CONFIG_BCFG1_SETUP
	ldr	\val, =LPC214X_BCFG1_VALUE
	str	\val, [\base, #LPC214X_BCFG1_OFFSET]
#endif

#ifdef CONFIG_BCFG2_SETUP
	ldr	\val, =LPC214X_BCFG2_VALUE
	str	\val, [\base, #LPC214X_BCFG2_OFFSET]
#endif

#ifdef CONFIG_BCFG3_SETUP
	ldr	\val, =LPC214X_BCFG3_VALUE
	str	\val, [\base, #LPC214X_BCFG3_OFFSET]
#endif
#endif
	.endm

/* Configure VPBDIV */

	.macro	configvpbdiv, base, val
#ifdef CONFIG_VPBDIV_SETUP
	ldr	\base, =LPC214X_VPBDIV
	ldr	\val, =LPC214X_VPBDIV_VALUE
	str	\val, [\base]
#endif
	.endm

/* Configure the PLL */

	.macro	configpll, base, val1, val2, val3
#ifdef LPC214X_PLL_SETUP
	ldr	\base, =LPC214X_PLL_BASE
	mov	\val1, #0xaa
	mov	\val2, #0x55

	/* Configure and Enable PLL */

	mov	\val3, #LPC214X_PLLCFG_VALUE
	str	\val3, [\base, #LPC214X_PLLCFG_OFFSET] 
	mov	\val3, #LPC214X_PLLCON_PLLE
	str	\val3, [\base, #LPC214X_PLLCON_OFFSET]
	str	\val1, [\base, #LPC214X_PLLFEED_OFFSET]
	str	\val2, [\base, #LPC214X_PLLFEED_OFFSET]

	/* Wait until PLL Locked */
1:
	ldr	\val3, [\base, #LPC214X_PLLSTAT_OFFSET]
	ands	\val3, \val3, #LPC214X_PLLSTAT_PLOCK
	beq	1b

	/* Switch to PLL Clock */

	mov	\val3, #(LPC214X_PLLCON_PLLE | LPC214X_PLLCON_PLLC)
	str	\val3, [\base, #LPC214X_PLLCON__OFFSET]
	str	\val1, [\base, #LPC214X_PLLFEED_OFFSET]
	str	\val2, [\base, #LPC214X_PLLFEED_OFFSET]
#endif
	.endm

/* Configure the Memory Accelerator Module (MAM) */

	.macro	configmam, base, val
#ifdef CONFIG_MAM_SETUP
	ldr	\base, =LPC214X_MAM_BASE
	mov	\val, #LPC214x_MAMTIM_VALUE
	str	\val, [\base, #LPC214x_MAMTIM_OFFSET] 
	mov	\val, #LPC214X_MAMCR_VALUE
	str	\val, [\base, #LPC214X_MAMCR_OFFSET] 
#endif
	.endm

/* Setup MEMMAP for the selected mode of operation */

	.macro	configmemmap, base, val
	ldr	r0, =LPC214X_MEMMAP
#if defined(CONFIG_EXTMEM_MODE)
	mov	r1, #3
#elif defined(CONFIG_RAM_MODE)
	mov	r1, #2
#else /* Setting the default should not be necessary */
	mov	r1, #1
#endif
	str	r1, [r0]
	.endm

/********************************************************************
 * Text
 ********************************************************************/

	.text

/********************************************************************
 * Name: _vector_table
 *
 * Description:
 *   Interrrupt vector table.  This must be located at the beginning
 *   of the memory space (at CONFIG_CODE_BASE).  The first entry in
 *   the vector table is the reset vector and this is the code that
 *   will execute whn the processor is reset.
 *
 ********************************************************************/

	.globl	_vector_table
	.type	_vector_table, %function
_vector_table:
	ldr	pc, .Lresethandler		/* 0x00: Reset */
	ldr	pc, .Lundefinedhandler		/* 0x04: Undefined instruction */
	ldr	pc, .Lswihandler		/* 0x08: Software interrupt */
	ldr	pc, .Lprefetchaborthandler	/* 0x0c: Prefetch abort */
	ldr	pc, .Ldataaborthandler		/* 0x10: Data abort */
	ldr	pc, .Laddrexcptnhandler		/* 0x14: Address exception */
	ldr	pc, .Lirqhandler		/* 0x18: IRQ */
	ldr	pc, .Lfiqhandler		/* 0x1c: FIQ */

	.globl   __start
	.globl	up_vectorundefinsn
	.globl	up_vectorswi
	.globl	up_vectorprefetch
	.globl	up_vectordata
	.globl	up_vectoraddrexcptn
	.globl	up_vectorirq
	.globl	up_vectorfiq

.Lresethandler:
	.long   __start
.Lundefinedhandler:
	.long	up_vectorundefinsn
.Lswihandler:
	.long	up_vectorswi
.Lprefetchaborthandler:
	.long	up_vectorprefetch
.Ldataaborthandler:
	.long	up_vectordata
.Laddrexcptnhandler:
	.long	up_vectoraddrexcptn
.Lirqhandler:
	.long	up_vectorirq
.Lfiqhandler:
	.long	up_vectorfiq
	.size	_vector_table, . - _vector_table
	.end

/********************************************************************
 * Name: __start
 *
 * Description:
 *   Reset entry point.  This is the first function to execute when
 *   the processor is reset.  It initializes hardware and then gives
 *   control to NuttX.
 *
 ********************************************************************/

	.global __start
	.type	__start, #function

__start:
	/* Setup the initial processor mode */

	mov	r0, #(SVC_MODE | PSR_I_BIT | PSR_F_BIT )
	msr	cpsr, r0

	/* Set up external memory mode (if so selected) */

	configpinsel2	r0, r1

	/* Setup the External Memory Controllor (EMC) as configured */

	configemc	r0, r1

	/* Configure VPBDIV */

	configvpbdiv	r0, r1
	
	/* Configure the PLL */

	configpll	r0, r1, r2, r3

	/* Configure the Memory Accelerator Module (MAM) */

	configmam	r0, r1

	/* Setup MEMMAP for the selected mode of operation */

	configmemmap	r0, r1
	
	showprogress 'A'

	/* Setup system stack (and get the BSS range) */

	adr	r0, LC0
	ldmia   r0, {r4, r5, sp}

	/* Clear system BSS section */

	mov	r0, #0
1:	cmp	r4, r5
	strcc	r0, [r4], #4
	bcc	1b

	showprogress 'B'

	/* Copy system .data sections to new home in RAM. */

#ifdef CONFIG_BOOT_FROM_FLASH

	adr	r3, LC2
	ldmia	r3, {r0, r1, r2}

1:	ldmia	r0!, {r3 - r10}
	stmia	r1!, {r3 - r10}
	cmp	r1, r2
	blt	1b

#endif
	/* Perform early serial initialization */

	mov	fp, #0
	bl	up_earlyserialinit

#ifdef CONFIG_DEBUG
	mov	r0, #'C'
	bl	up_putc
	mov	r0, #'\n'
	bl	up_putc
#endif
	/* Initialize onboard LEDs */

#ifdef CONFIG_ARCH_LEDS
	bl	up_ledinit
#endif

	/* Then jump to OS entry */

	b	os_start

	/* Variables:
	 * _sbss is the start of the BSS region (see ld.script)
	 * _ebss is the end of the BSS regsion (see ld.script)
	 * The idle task stack starts at the end of BSS and is
	 * of size CONFIG_PROC_STACK_SIZE.  The heap continues
	 * from there until the end of memory.  See g_heapbase
	 * below.
	 */

LC0:	.long	_sbss
	.long	_ebss
	.long	_ebss+CONFIG_PROC_STACK_SIZE-4

#ifdef CONFIG_BOOT_FROM_FLASH
LC2:	.long	_eronly	/* Where .data defaults are stored in FLASH */
	.long	_sdata	/* Where .data needs to reside in SDRAM */
	.long	_edata
#endif
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
	.long	_ebss+CONFIG_PROC_STACK_SIZE
	.size	g_heapbase, .-g_heapbase

	.end

