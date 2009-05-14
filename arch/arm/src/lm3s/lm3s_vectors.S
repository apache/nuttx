/************************************************************************************
 * arch/arm/src/lm3s/lm3s_vectors.S
 * arch/arm/src/chip/lm3s_vectors.S
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <arch/irq.h>

/************************************************************************************
 * Preprocessor Definitions
 ************************************************************************************/

/* Memory Map:
 *
 * 0x0000:0000 - Beginning of FLASH. Address of vectors (if not using bootloader)
 * 0x0002:0000 - Address of vectors if using bootloader
 * 0x0003:ffff - End of flash
 * 0x2000:0000 - Start of SRAM and start of .data (_sdata)
 *             - End of .data (_edata) abd start of .bss (_sbss)
 *             - End of .bss (_ebss) and bottom of idle stack
 *             - _ebss + CONFIG_IDLETHREAD_STACKSIZE = end of idle stack, start of heap
 * 0x2000:ffff - End of SRAM and end of heap
 */

#define IDLE_STACK      (_ebss+CONFIG_IDLETHREAD_STACKSIZE-4)
#define HEAP_BASE       (_ebss+CONFIG_IDLETHREAD_STACKSIZE-4)

/************************************************************************************
 * Global Symbols
 ************************************************************************************/

	.globl		__start
	.globl		dispach_irq

	.syntax		unified
	.thumb
	.file		"lm3s_vectors.S"

/************************************************************************************
 * Macros
 ************************************************************************************/

/* On entry into an IRQ, the hardware automatically saves the xPSR, PC, LR, R12, R0-R3
 * registers on the stack, then branches to an instantantiation of the following
 * macro.  This macro simply loads the IRQ number into R0, then jumps to the common
 * IRQ handling logic.
 */

	.macro	HANDLER, label, irqno
\label:
	mov	r0, #\irqno
	b	lm3s_irqcommon
	.endm

/************************************************************************************
 * Vectors
 ************************************************************************************/

	.section	.vectors, "ax"
	.code		16
	.align		2
        .globl		lm3s_vectors
        .type		lm3s_vectors, function

lm3s_vectors:
/* Processor Exceptions */
	.word	IDLE_STACK		/* Vector  0: Reset stack pointer */
	.word	__start			/* Vector  1: Reset vector */
	.word	lm3s_nmi		/* Vector  2: Non-Maskable Interrupt (NMI) */
	.word	lm3s_hardfault		/* Vector  3: Hard fault */
	.word	lm3s_mpu		/* Vector  4: Memory management (MPU) */
	.word	lm3s_busfault		/* Vector  5: Bus fault */
	.word	lm3s_usagefault		/* Vector  6: Usage fault */
	.word	lm3s_reserved		/* Vector  7: Reserved */
	.word	lm3s_reserved		/* Vector  8: Reserved */
	.word	lm3s_reserved		/* Vector  9: Reserved */
	.word	lm3s_reserved		/* Vector 10: Reserved */
	.word	lm3s_svcall		/* Vector 11: SVC call */
	.word	lm3s_dbgmonitor		/* Vector 12: Debug monitor */
	.word	lm3s_reserved		/* Vector 13: Reserved */
	.word	lm3s_pendsv		/* Vector 14: Pendable system service request */
	.word	lm3s_systick		/* Vector 15: System tick */

/* External Interrupts */

#ifdef CONFIG_ARCH_CHIP_LM3S6918
	.word	lm3s_gpioa		/* Vector 16: GPIO Port A */
	.word	lm3s_gpiob		/* Vector 17: GPIO Port B */
	.word	lm3s_gpiod		/* Vector 18: GPIO Port C */
	.word	lm3s_gpioe		/* Vector 19: GPIO Port D */
	.word	lm3s_gpioe		/* Vector 20: GPIO Port E */
	.word	lm3s_uart0		/* Vector 21: UART 0 */
	.word	lm3s_uart1		/* Vector 22: UART 1 */
	.word	lm3s_ssi0		/* Vector 23: SSI 0 */
	.word	lm3s_i2c0		/* Vector 24: I2C 0 */
	.word	lm3s_reserved		/* Vector 25: Reserved */
	.word	lm3s_reserved		/* Vector 26: Reserved */
	.word	lm3s_reserved		/* Vector 27: Reserved */
	.word	lm3s_reserved		/* Vector 28: Reserved */
	.word	lm3s_reserved		/* Vector 29: Reserved */
	.word	lm3s_adc0		/* Vector 30: ADC Sequence 0 */
	.word	lm3s_adc1		/* Vector 31: ADC Sequence 1 */
	.word	lm3s_adc2		/* Vector 32: ADC Sequence 2 */
	.word	lm3s_adc3		/* Vector 33: ADC Sequence 3 */
	.word	lm3s_wdog		/* Vector 34: Watchdog Timer */
	.word	lm3s_tmr0a		/* Vector 35: Timer 0 A */
	.word	lm3s_tmr0b		/* Vector 36: Timer 0 B */
	.word	lm3s_tmr1a		/* Vector 37: Timer 1 A */
	.word	lm3s_tmr1b		/* Vector 38: Timer 1 B */
	.word	lm3s_tmr2a		/* Vector 39: Timer 2 A */
	.word	lm3s_tmr2b		/* Vector 40: Timer 3 B */
	.word	lm3s_cmp0		/* Vector 41: Analog Comparator 0 */
	.word	lm3s_dmp1		/* Vector 42: Analog Comparator 1 */
	.word	lm3s_reserved		/* Vector 43: Reserved */
	.word	lm3s_syscon		/* Vector 44: System Control */
	.word	lm3s_flashcon		/* Vector 45: FLASH Control */
	.word	lm3s_gpiof		/* Vector 46: GPIO Port F */
	.word	lm3s_gpiog		/* Vector 47: GPIO Port G */
	.word	lm3s_gpioh		/* Vector 48: GPIO Port H */
	.word	lm3s_reserved		/* Vector 49: Reserved */
	.word	lm3s_ssi1		/* Vector 50: SSI 1 */
	.word	lm3s_tmr3a		/* Vector 51: Timer 3 A */
	.word	lm3s_tmr3b		/* Vector 52: Timer 3 B */
	.word	lm3s_i2c1		/* Vector 53: I2C 1 */
	.word	lm3s_reserved		/* Vector 54: Reserved */
	.word	lm3s_reserved		/* Vector 55: Reserved */
	.word	lm3s_reserved		/* Vector 56: Reserved */
	.word	lm3s_reserved		/* Vector 57: Reserved */
 	.word	lm3s_eth		/* Vector 58: Ethernet Controller */
	.word	lm3s_hib		/* Vector 59: Hibernation Module */
	.word	lm3s_reserved		/* Vector 60: Reserved */
	.word	lm3s_reserved		/* Vector 61: Reserved */
	.word	lm3s_reserved		/* Vector 62: Reserved */
	.word	lm3s_reserved		/* Vector 63: Reserved */
	.word	lm3s_reserved		/* Vector 64: Reserved */
	.word	lm3s_reserved		/* Vector 65: Reserved */
	.word	lm3s_reserved		/* Vector 66: Reserved */
	.word	lm3s_reserved		/* Vector 67: Reserved */
	.word	lm3s_reserved		/* Vector 68: Reserved */
	.word	lm3s_reserved		/* Vector 69: Reserved */
	.word	lm3s_reserved		/* Vector 70: Reserved */
#else
#  error "Vectors not specified for this LM3S chip"
#endif
	.size	lm3s_vectors, .-lm3s_vectors

/************************************************************************************
 * .text
 ************************************************************************************/

	.text
        .type   handlers, function
handlers:
	HANDLER	lm3s_reserved, LMSB_IRQ_RESERVED	/* Unexpected/reserved vector */
	HANDLER lm3s_nmi, LMSB_IRQ_NMI                  /* Vector  2: Non-Maskable Interrupt (NMI) */
	HANDLER lm3s_hardfault, LMSB_IRQ_HARDFAULT      /* Vector  3: Hard fault */
	HANDLER lm3s_mpu, LMSB_IRQ_MPU                  /* Vector  4: Memory management (MPU) */
	HANDLER lm3s_busfault, LMSB_IRQ_BUSFAULT        /* Vector  5: Bus fault */
	HANDLER lm3s_usagefault, LMSB_IRQ_USAGEFAULT    /* Vector  6: Usage fault */
	HANDLER lm3s_svcall, LMSB_IRQ_SVCALL            /* Vector 11: SVC call */
	HANDLER lm3s_dbgmonitor, LMSB_IRQ_DBGMONITOR    /* Vector 12: Debug Monitor */
	HANDLER lm3s_pendsv, LMSB_IRQ_PENDSV            /* Vector 14: Penable system service request */
	HANDLER lm3s_systick, LMSB_IRQ_SYSTICK          /* Vector 15: System tick */

#ifdef CONFIG_ARCH_CHIP_LM3S6918
	HANDLER	lm3s_gpioa, LM3S_IRQ_GPIOA		/* Vector 16: GPIO Port A */
	HANDLER	lm3s_gpiob, LM3S_IRQ_GPIOB		/* Vector 17: GPIO Port B */
	HANDLER	lm3s_gpioc, LM3S_IRQ_GPIOC		/* Vector 18: GPIO Port C */
	HANDLER	lm3s_gpiod, LM3S_IRQ_GPIOD		/* Vector 19: GPIO Port D */
	HANDLER	lm3s_gpioe, LM3S_IRQ_GPIOE		/* Vector 20: GPIO Port E */
	HANDLER	lm3s_uart0, LM3S_IRQ_UART0		/* Vector 21: UART 0 */
	HANDLER	lm3s_uart1, LM3S_IRQ_UART1		/* Vector 22: UART 1 */
	HANDLER	lm3s_ssi0, LM3S_IRQ_SSI0		/* Vector 23: SSI 0 */
	HANDLER	lm3s_i2c0, LM3S_IRQ_I2C0		/* Vector 24: I2C 0 */
	HANDLER	lm3s_adc0, LM3S_IRQ_ADC0		/* Vector 30: ADC Sequence 0 */
	HANDLER	lm3s_adc1, LM3S_IRQ_ADC1		/* Vector 31: ADC Sequence 1 */
	HANDLER	lm3s_adc2, LM3S_IRQ_ADC2		/* Vector 32: ADC Sequence 2 */
	HANDLER	lm3s_adc3, LM3S_IRQ_ADC3		/* Vector 33: ADC Sequence 3 */
	HANDLER	lm3s_wdog, LM3S_IRQ_WDOG		/* Vector 34: Watchdog Timer */
	HANDLER	lm3s_tmr0a, LM3S_IRQ_TIMER0A		/* Vector 35: Timer 0 A */
	HANDLER	lm3s_tmr0b, LM3S_IRQ_TIMER0B		/* Vector 36: Timer 0 B */
	HANDLER	lm3s_tmr1a, LM3S_IRQ_TIMER1A		/* Vector 37: Timer 1 A */
	HANDLER	lm3s_tmr1b, LM3S_IRQ_TIMER1B		/* Vector 38: Timer 1 B */
	HANDLER	lm3s_tmr2a, LM3S_IRQ_TIMER2A		/* Vector 39: Timer 2 A */
	HANDLER	lm3s_tmr2b, LM3S_IRQ_TIMER2B		/* Vector 40: Timer 3 B */
	HANDLER	lm3s_cmp0, LM3S_IRQ_COMPARE0		/* Vector 41: Analog Comparator 0 */
	HANDLER	lm3s_dmp1, LM3S_IRQ_COMPARE1		/* Vector 42: Analog Comparator 1 */
	HANDLER	lm3s_syscon, LM3S_IRQ_SYSCON		/* Vector 44: System Control */
	HANDLER	lm3s_flashcon, LM3S_IRQ_FLASHCON	/* Vector 45: FLASH Control */
	HANDLER	lm3s_gpiof, LM3S_IRQ_GPIOF		/* Vector 46: GPIO Port F */
	HANDLER	lm3s_gpiog, LM3S_IRQ_GPIOG		/* Vector 47: GPIO Port G */
	HANDLER	lm3s_gpioh, LM3S_IRQ_GPIOH		/* Vector 48: GPIO Port H */
	HANDLER	lm3s_ssi1, LM3S_IRQ_SSI1		/* Vector 50: SSI 1 */
	HANDLER	lm3s_tmr3a, LM3S_IRQ_TIMER3A		/* Vector 51: Timer 3 A */
	HANDLER	lm3s_tmr3b, LM3S_IRQ_TIMER3B		/* Vector 52: Timer 3 B */
	HANDLER	lm3s_i2c1, LM3S_IRQ_I2C1		/* Vector 53: I2C 1 */
 	HANDLER	lm3s_eth, LM3S_IRQ_ETHCON		/* Vector 58: Ethernet Controller */
	HANDLER	lm3s_hib, LM3S_IRQ_HIBERNATE		/* Vector 59: Hibernation Module */
#else
#  error "Vectors not specified for this LM3S chip"
#endif


/* Common IRQ handling logic.  On entry here, the stack is like the following:
 *
 *      REG_XPSR
 *      REG_R15
 *      REG_R14
 *      REG_R12
 *      REG_R3
 *      REG_R2
 *      REG_R1
 * PSP->REG_R0
 *
 * and R0 contains the IRQ number
 */

lm3s_irqcommon:
	/* Complete the context save */

	mrs	r1, psp			/* R1=The process stack pointer */
	mov	r2, r1			/* R2=Copy of the process stack pointer */
	add	r2, #HW_XCPT_SIZE	/* R2=PSP before the interrupt was taken */
	mrs	r3, primask		/* R3=Current PRIMASK setting */
	stmdb	r1!, {r2-r11}		/* Save the remaining registers plus the SP value */

	/* Disable interrupts, select the stack to use for interrupt handling
	 * and call up_doirq to handle the interrupt
	 */

	cpsid	i			/* Disable further interrupts */

	/* If CONFIG_ARCH_INTERRUPTSTACK is defined, we will use a special interrupt
	 * stack pointer.  The way that this is done here prohibits nested interrupts!
	 * Otherwise, we will re-use the process stack for interrupt level processing.
	 */

#ifdef CONFIG_ARCH_INTERRUPTSTACK
	ld	sp, #up_interruptstack_base
#else
	mov	sp, r1			/* Use the process stack pointer */
#endif
	stmdb	sp!, {r1, r14}		/* Save the process SP and the return LR */
	bl	up_doirq		/* R0=IRQ, R1=register save */
	ldmia	sp!, {r1, r14}		/* Recover the process SP and the return LR */

	/* On return from up_doirq, R0 will hold a pointer to register context
	 * array to use for the interrupt return.  If that return value is the same
	 * as current stack pointer, then things are relatively easy.
	 */

	cmp	r0, r1
	beq	1f			/* Branch if no context switch */

	/* We are returning from a context switch.  This is different because in this
	 * case, the register save structure does not lie on the stack but, rather,
	 * is within a TCB structure.  We'll have to copy some values to the stack.
	 */

        add	r1, r0, #(XCPTCONTEXT_SIZE-4)	/* r2=offset HW save area */
	ldmia	r1, {r4-r11}		/* Eight registers in HW save area */
	ldr	r1, [r0, #(4*REG_SP)]	/* R1=Value of SP before interrupt */
	stmdb	r1!, {r4-r11}		/* Eight registers in HW save area */
	msr	psp, r1			/* New PSP */

	/* We simply need to "unwind" the same stack frame that we created */
1:
	stmdb	r0!, {r2-r11}		/* Recover R4-R11 + 2 temp values */

	/* Do we need to restore interrupts? */

	tst	r3, #1			/* PRIMASK it 1=1 means that interrupts are masked */
	bne	2f
	cpsie	i			/* Restore interrupts */
2:
	bx	r14			/* And return */
	.size	handler, .-handlers

/************************************************************************************
 *  Name: up_interruptstack/g_userstack
 *
 * Description:
 *   Shouldn't happen
 *
 ************************************************************************************/

#if CONFIG_ARCH_INTERRUPTSTACK > 3
        .bss
        .align  4
up_interruptstack:
        .skip   ((CONFIG_ARCH_INTERRUPTSTACK & ~3) - 4)
up_interruptstack_base:
        .skip   4
        .size   up_interruptstack, .-up_interruptstack
#endif

/************************************************************************************
 * .rodata
 ************************************************************************************/

	.section	.rodata, "a"

/* Variables: _sbss is the start of the BSS region (see ld.script) _ebss is the end
 * of the BSS regsion (see ld.script). The idle task stack starts at the end of BSS
 * and is of size CONFIG_IDLETHREAD_STACKSIZE.  The IDLE thread is the thread that
 * the system boots on and, eventually, becomes the idle, do nothing task that runs
 * only when there is nothing else to run.  The heap continues from there until the
 * end of memory.  See g_heapbase below.
 */

        .globl  g_heapbase
        .type   g_heapbase, object
g_heapbase:
        .long   _ebss+CONFIG_IDLETHREAD_STACKSIZE
        .size   g_heapbase, .-g_heapbase

        .end
