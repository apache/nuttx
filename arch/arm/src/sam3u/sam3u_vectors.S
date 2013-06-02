/************************************************************************************************
 * arch/arm/src/sam3u/sam3u_vectors.S
 * arch/arm/src/chip/sam3u_vectors.S
 *
 *   Copyright (C) 2009-2010, 2013 Gregory Nutt. All rights reserved.
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
 ************************************************************************************************/

/************************************************************************************************
 * Included Files
 ************************************************************************************************/

#include <nuttx/config.h>

#include <arch/irq.h>

#include "exc_return.h"

/************************************************************************************************
 * Preprocessor Definitions
 ************************************************************************************************/

/* Memory Map:
 *
 * 0x0800:0000 - Beginning of FLASH. Address of vectors.  Mapped to address 0x0000:0000 at boot
 *               time.
 * 0x0803:ffff - End of flash
 * 0x2000:0000 - Start of SRAM and start of .data (_sdata)
 *             - End of .data (_edata) and start of .bss (_sbss)
 *             - End of .bss (_ebss) and bottom of idle stack
 *             - _ebss + CONFIG_IDLETHREAD_STACKSIZE = end of idle stack, start of heap. NOTE
 *               that the ARM uses a decrement before store stack so that the correct initial
 *               value is the end of the stack + 4;
 * 0x2000:bfff - End of SRAM and end of heap
 */

#define IDLE_STACK		(_ebss+CONFIG_IDLETHREAD_STACKSIZE)
#define HEAP_BASE		(_ebss+CONFIG_IDLETHREAD_STACKSIZE)

/************************************************************************************************
 * Global Symbols
 ************************************************************************************************/

	.globl		__start

	.syntax		unified
	.thumb
	.file		"sam3u_vectors.S"

/************************************************************************************************
 * Macros
 ************************************************************************************************/

/* On entry into an IRQ, the hardware automatically saves the xPSR, PC, LR, R12, R0-R3
 * registers on the stack, then branches to an instantantiation of the following
 * macro.  This macro simply loads the IRQ number into R0, then jumps to the common
 * IRQ handling logic.
 */

	.macro	HANDLER, label, irqno
	.thumb_func
\label:
	mov		r0, #\irqno
	b		sam3u_common
	.endm

/************************************************************************************************
 * Vectors
 ************************************************************************************************/

	.section	.vectors, "ax"
	.code		16
	.align		2
	.globl		sam3u_vectors
	.type		sam3u_vectors, function

sam3u_vectors:

/* Processor Exceptions */

	.word	IDLE_STACK		/* Vector  0: Reset stack pointer */
	.word	__start			/* Vector  1: Reset vector */
	.word	sam_nmi			/* Vector  2: Non-Maskable Interrupt (NMI) */
	.word	sam_hardfault	/* Vector  3: Hard fault */
	.word	sam_mpu			/* Vector  4: Memory management (MPU) */
	.word	sam_busfault	/* Vector  5: Bus fault */
	.word	sam_usagefault	/* Vector  6: Usage fault */
	.word	sam_reserved	/* Vector  7: Reserved */
	.word	sam_reserved	/* Vector  8: Reserved */
	.word	sam_reserved	/* Vector  9: Reserved */
	.word	sam_reserved	/* Vector 10: Reserved */
	.word	sam_svcall		/* Vector 11: SVC call */
	.word	sam_dbgmonitor	/* Vector 12: Debug monitor */
	.word	sam_reserved	/* Vector 13: Reserved */
	.word	sam_pendsv		/* Vector 14: Pendable system service request */
	.word	sam_systick		/* Vector 15: System tick */

/* External Interrupts */

	.word	sam_supc		/* Vector 16+0:  Supply Controller */
	.word	sam_rstc		/* Vector 16+1:  Reset Controller */
	.word	sam_rtc			/* Vector 16+2:  Real Time Clock */
	.word	sam_rtt			/* Vector 16+3:  Real Time Timer */
	.word	sam_wdt			/* Vector 16+4:  Watchdog Timer */
	.word	sam_pmc			/* Vector 16+5:  Power Management Controller */
	.word	sam_eefc0		/* Vector 16+6:  Enhanced Embedded Flash Controller 0 */
	.word	sam_eefc1		/* Vector 16+7:  Enhanced Embedded Flash Controller 1 */
	.word	sam_uart		/* Vector 16+8:  Universal Asynchronous Receiver Transmitter */
	.word	sam_smc			/* Vector 16+9:  Static Memory Controller */
	.word	sam_pioa		/* Vector 16+10: Parallel I/O Controller A */
	.word	sam_piob		/* Vector 16+11: Parallel I/O Controller B */
	.word	sam_pioc		/* Vector 16+12: Parallel I/O Controller C */
	.word	sam_usart0		/* Vector 16+13: USART 0 */
	.word	sam_usart1		/* Vector 16+14: USART 1 */
	.word	sam_usart2		/* Vector 16+15: USART 2 */
	.word	sam_usart3		/* Vector 16+16: USART 3 */
	.word	sam_hsmci		/* Vector 16+17: High Speed Multimedia Card Interface */
	.word	sam_twi0		/* Vector 16+18: Two-Wire Interface 0 */
	.word	sam_twi1		/* Vector 16+19: Two-Wire Interface 1 */
	.word	sam_spi			/* Vector 16+20: Serial Peripheral Interface */
	.word	sam_ssc			/* Vector 16+21: Synchronous Serial Controller */
	.word	sam_tc0			/* Vector 16+22: Timer Counter 0 */
	.word	sam_tc1			/* Vector 16+23: Timer Counter 1 */
	.word	sam_tc2			/* Vector 16+24: Timer Counter 2 */
	.word	sam_pwm			/* Vector 16+25: Pulse Width Modulation Controller */
	.word	sam_adc12b		/* Vector 16+26: 12-bit ADC Controller */
	.word	sam_adc			/* Vector 16+27: 10-bit ADC Controller */
	.word	sam_dmac		/* Vector 16+28: DMA Controller */
	.word	sam_udphs		/* Vector 16+29: USB Device High Speed */
	.size	sam_vectors, .-sam_vectors

/************************************************************************************************
 * .text
 ************************************************************************************************/

	.text
	.type	handlers, function
	.thumb_func
handlers:
	HANDLER	sam_reserved, SAM_IRQ_RESERVED		/* Unexpected/reserved vector */
	HANDLER	sam_nmi, SAM_IRQ_NMI				/* Vector  2: Non-Maskable Interrupt (NMI) */
	HANDLER	sam_hardfault, SAM_IRQ_HARDFAULT	/* Vector  3: Hard fault */
	HANDLER	sam_mpu, SAM_IRQ_MEMFAULT			/* Vector  4: Memory management (MPU) */
	HANDLER	sam_busfault, SAM_IRQ_BUSFAULT		/* Vector  5: Bus fault */
	HANDLER	sam_usagefault, SAM_IRQ_USAGEFAULT	/* Vector  6: Usage fault */
	HANDLER	sam_svcall, SAM_IRQ_SVCALL			/* Vector 11: SVC call */
	HANDLER	sam_dbgmonitor, SAM_IRQ_DBGMONITOR	/* Vector 12: Debug Monitor */
	HANDLER	sam_pendsv, SAM_IRQ_PENDSV			/* Vector 14: Penable system service request */
	HANDLER	sam_systick, SAM_IRQ_SYSTICK		/* Vector 15: System tick */

	HANDLER	sam_supc, SAM_IRQ_SUPC				/* Vector 16+0:  Supply Controller */
	HANDLER	sam_rstc, SAM_IRQ_RSTC				/* Vector 16+1:  Reset Controller */
	HANDLER	sam_rtc, SAM_IRQ_RTC				/* Vector 16+2:  Real Time Clock */
	HANDLER	sam_rtt, SAM_IRQ_RTT				/* Vector 16+3:  Real Time Timer */
	HANDLER	sam_wdt, SAM_IRQ_WDT				/* Vector 16+4:  Watchdog Timer */
	HANDLER	sam_pmc, SAM_IRQ_PMC				/* Vector 16+5:  Power Management Controller */
	HANDLER	sam_eefc0, SAM_IRQ_EEFC0			/* Vector 16+6:  Enhanced Embedded Flash Controller 0 */
	HANDLER	sam_eefc1, SAM_IRQ_EEFC1			/* Vector 16+7:  Enhanced Embedded Flash Controller 1 */
	HANDLER	sam_uart, SAM_IRQ_UART				/* Vector 16+8:  Universal Asynchronous Receiver Transmitter */
	HANDLER	sam_smc, SAM_IRQ_SMC				/* Vector 16+9:  Static Memory Controller */
	HANDLER	sam_pioa, SAM_IRQ_PIOA				/* Vector 16+10: Parallel I/O Controller A */
	HANDLER	sam_piob, SAM_IRQ_PIOB				/* Vector 16+11: Parallel I/O Controller B */
	HANDLER	sam_pioc, SAM_IRQ_PIOC				/* Vector 16+12: Parallel I/O Controller C */
	HANDLER	sam_usart0, SAM_IRQ_USART0			/* Vector 16+13: USART 0 */
	HANDLER	sam_usart1, SAM_IRQ_USART1			/* Vector 16+14: USART 1 */
	HANDLER	sam_usart2, SAM_IRQ_USART2			/* Vector 16+15: USART 2 */
	HANDLER	sam_usart3, SAM_IRQ_USART3			/* Vector 16+16: USART 3 */
	HANDLER	sam_hsmci, SAM_IRQ_HSMCI			/* Vector 16+17: High Speed Multimedia Card Interface */
	HANDLER	sam_twi0, SAM_IRQ_TWI0				/* Vector 16+18: Two-Wire Interface 0 */
	HANDLER	sam_twi1, SAM_IRQ_TWI1				/* Vector 16+19: Two-Wire Interface 1 */
	HANDLER	sam_spi, SAM_IRQ_SPI				/* Vector 16+20: Serial Peripheral Interface */
	HANDLER	sam_ssc, SAM_IRQ_SSC				/* Vector 16+21: Synchronous Serial Controller */
	HANDLER	sam_tc0, SAM_IRQ_TC0				/* Vector 16+22: Timer Counter 0 */
	HANDLER	sam_tc1, SAM_IRQ_TC1				/* Vector 16+23: Timer Counter 1 */
	HANDLER	sam_tc2, SAM_IRQ_TC2				/* Vector 16+24: Timer Counter 2 */
	HANDLER	sam_pwm, SAM_IRQ_PWM				/* Vector 16+25: Pulse Width Modulation Controller */
	HANDLER	sam_adc12b, SAM_IRQ_ADC12B			/* Vector 16+26: 12-bit ADC Controller */
	HANDLER	sam_adc, SAM_IRQ_ADC				/* Vector 16+27: 10-bit ADC Controller */
	HANDLER	sam_dmac, SAM_IRQ_DMAC				/* Vector 16+28: DMA Controller */
	HANDLER	sam_udphs, SAM_IRQ_UDPHS			/* Vector 16+29: USB Device High Speed */

/* Common IRQ handling logic.  On entry here, the return stack is on either
 * the PSP or the MSP and looks like the following:
 *
 *      REG_XPSR
 *      REG_R15
 *      REG_R14
 *      REG_R12
 *      REG_R3
 *      REG_R2
 *      REG_R1
 * MSP->REG_R0
 *
 * And
 *      R0 contains the IRQ number
 *      R14 Contains the EXC_RETURN value
 *      We are in handler mode and the current SP is the MSP
 */

sam3u_common:

	/* Complete the context save */

#ifdef CONFIG_NUTTX_KERNEL
	/* The EXC_RETURN value will be 0xfffffff9 (privileged thread) or 0xfffffff1
	 * (handler mode) if the stack is on the MSP.  It can only be on the PSP if
	 * EXC_RETURN is 0xfffffffd (unprivileged thread)
	 */

	adds	r2, r14, #3				/* If R14=0xfffffffd, then r2 == 0 */
	ite		ne						/* Next two instructions are conditional */
	mrsne	r1, msp					/* R1=The main stack pointer */
	mrseq	r1, psp					/* R1=The process stack pointer */
#else
	mrs		r1, msp					/* R1=The main stack pointer */
#endif

	/* r1 holds the value of the stack pointer AFTER the excption handling logic
	 * pushed the various registers onto the stack.  Get r2 = the value of the
	 * stack pointer BEFORE the interrupt modified it.
	 */

	mov		r2, r1					/* R2=Copy of the main/process stack pointer */
	add		r2, #HW_XCPT_SIZE		/* R2=MSP/PSP before the interrupt was taken */
#ifdef CONFIG_ARMV7M_USEBASEPRI
	mrs		r3, basepri				/* R3=Current BASEPRI setting */
#else
	mrs		r3, primask				/* R3=Current PRIMASK setting */
#endif

#ifdef CONFIG_ARCH_FPU
	/* Skip over the block of memory reserved for floating pointer register save.
	 * Lazy FPU register saving is used.  FPU registers will be saved in this
	 * block only if a context switch occurs (this means, of course, that the FPU
	 * cannot be used in interrupt processing).
	 */

	sub		r1, #(4*SW_FPU_REGS)
#endif

	/* Save the the remaining registers on the stack after the registers pushed
	 * by the exception handling logic. r2=SP and r3=primask or basepri, r4-r11,
	 * r14=register values.
	 */

#ifdef CONFIG_NUTTX_KERNEL
	stmdb	r1!, {r2-r11,r14}		/* Save the remaining registers plus the SP value */
#else
	stmdb	r1!, {r2-r11}			/* Save the remaining registers plus the SP value */
#endif

	/* Disable interrupts, select the stack to use for interrupt handling
	 * and call up_doirq to handle the interrupt
	 */

	cpsid	i						/* Disable further interrupts */

	/* If CONFIG_ARCH_INTERRUPTSTACK is defined, we will use a special interrupt
	 * stack pointer.  The way that this is done here prohibits nested interrupts!
	 * Otherwise, we will re-use the main stack for interrupt level processing.
	 */

#if CONFIG_ARCH_INTERRUPTSTACK > 3
	ldr		sp, =g_intstackbase
	str		r1, [sp, #-4]!			/* Save the MSP on the interrupt stack */
	bl		up_doirq				/* R0=IRQ, R1=register save (msp) */
	ldr		r1, [sp, #+4]!			/* Recover R1=main stack pointer */
#else
	mov		sp, r1					/* We are using the main stack pointer */
	bl		up_doirq				/* R0=IRQ, R1=register save (msp) */
	mov		r1, sp					/* Recover R1=main stack pointer */
#endif

	/* On return from up_doirq, R0 will hold a pointer to register context
	 * array to use for the interrupt return.  If that return value is the same
	 * as current stack pointer, then things are relatively easy.
	 */

	cmp		r0, r1					/* Context switch? */
	beq		1f						/* Branch if no context switch */

	/* We are returning with a pending context switch.
	 *
	 * If the FPU is enabled, then we will need to restore FPU registers.
	 * This is not done in normal interrupt save/restore because the cost
	 * is prohibitive.  This is only done when switching contexts.  A
	 * consequence of this is that floating point operations may not be
	 * performed in interrupt handling logic.
	 *
	 * Here:
	 *   r0 = Address of the register save area
	
	 * NOTE: It is a requirement that up_restorefpu() preserve the value of
	 * r0!
	 */

#ifdef CONFIG_ARCH_FPU
	bl		up_restorefpu			/* Restore the FPU registers */
#endif

	/* We are returning with a pending context switch.  This case is different
	 * because in this case, the register save structure does not lie on the
	 * stack but, rather, are within a TCB structure.  We'll have to copy some
	 * values to the stack.
	 */

	add		r1, r0, #SW_XCPT_SIZE	/* R1=Address of HW save area in reg array */
	ldmia	r1, {r4-r11}			/* Fetch eight registers in HW save area */
	ldr		r1, [r0, #(4*REG_SP)]	/* R1=Value of SP before interrupt */
	stmdb	r1!, {r4-r11}			/* Store eight registers in HW save area */
#ifdef CONFIG_NUTTX_KERNEL
	ldmia	r0, {r2-r11,r14}		/* Recover R4-R11, r14 + 2 temp values */
#else
	ldmia	r0, {r2-r11}			/* Recover R4-R11 + 2 temp values */
#endif
	b		2f						/* Re-join common logic */

	/* We are returning with no context switch.  We simply need to "unwind"
	 * the same stack frame that we created
	 *
	 * Here:
	 *   r1 = Address of the return stack (same as r0)
	 */
1:
#ifdef CONFIG_NUTTX_KERNEL
	ldmia	r1!, {r2-r11,r14}		/* Recover R4-R11, r14 + 2 temp values */
#else
	ldmia	r1!, {r2-r11}			/* Recover R4-R11 + 2 temp values */
#endif

#ifdef CONFIG_ARCH_FPU
	/* Skip over the block of memory reserved for floating pointer register
	 * save. Then R1 is the address of the HW save area
	 */

	add		r1, #(4*SW_FPU_REGS)
#endif

	/* Set up to return from the exception
	 *
	 * Here:
	 *   r1 = Address on the target thread's stack position at the start of
	 *        the registers saved by hardware
	 *   r3 = primask or basepri
	 *   r4-r11 = restored register values
	 */
2:

#ifdef CONFIG_NUTTX_KERNEL
	/* The EXC_RETURN value will be 0xfffffff9 (privileged thread) or 0xfffffff1
	 * (handler mode) if the stack is on the MSP.  It can only be on the PSP if
	 * EXC_RETURN is 0xfffffffd (unprivileged thread)
	 */

	mrs		r2, control				/* R2=Contents of the control register */
	tst		r14, #EXC_RETURN_PROCESS_STACK /* nonzero if context on process stack */
	beq		3f						/* Branch if privileged */

	orr		r2, r2, #1				/* Unprivileged mode */
	msr		psp, r1					/* R1=The process stack pointer */
	b		4f
3:
	bic		r2, r2, #1				/* Privileged mode */
	msr		msp, r1					/* R1=The main stack pointer */
4:
	msr		control, r2				/* Save the updated control register */
#else
	msr		msp, r1					/* Recover the return MSP value */

	/* Preload r14 with the special return value first (so that the return
	 * actually occurs with interrupts still disabled).
	 */

	ldr		r14, =EXC_RETURN_PRIVTHR	/* Load the special value */
#endif

    /* Restore the interrupt state */

#ifdef CONFIG_ARMV7M_USEBASEPRI
	msr		basepri, r3				/* Restore interrupts priority masking*/
	cpsie	i						/* Re-enable interrupts */
#else
	msr		primask, r3				/* Restore interrupts */
#endif

	/* Always return with R14 containing the special value that will: (1)
	 * return to thread mode, and (2) continue to use the MSP
	 */

	bx		r14						/* And return */
	.size	handlers, .-handlers

/************************************************************************************************
 *  Name: up_interruptstack/g_intstackbase
 *
 * Description:
 *   Shouldn't happen
 *
 ************************************************************************************************/

#if CONFIG_ARCH_INTERRUPTSTACK > 3
	.bss
	.global	g_intstackbase
	.align	4
up_interruptstack:
	.skip	(CONFIG_ARCH_INTERRUPTSTACK & ~3)
g_intstackbase:
	.size	up_interruptstack, .-up_interruptstack
#endif

/************************************************************************************************
 * .rodata
 ************************************************************************************************/

	.section	.rodata, "a"

/* Variables: _sbss is the start of the BSS region (see ld.script) _ebss is the end
 * of the BSS regsion (see ld.script). The idle task stack starts at the end of BSS
 * and is of size CONFIG_IDLETHREAD_STACKSIZE.  The IDLE thread is the thread that
 * the system boots on and, eventually, becomes the idle, do nothing task that runs
 * only when there is nothing else to run.  The heap continues from there until the
 * end of memory.  See g_idle_topstack below.
 */

	.globl	g_idle_topstack
	.type	g_idle_topstack, object
g_idle_topstack:
	.word	HEAP_BASE
	.size	g_idle_topstack, .-g_idle_topstack

	.end
