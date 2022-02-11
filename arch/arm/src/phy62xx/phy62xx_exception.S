
/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <arch/irq.h>
#include "exc_return.h"

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

	.globl		exception_common
	.globl		exception_common_inline
	.file		"arm_exception.S"

/****************************************************************************
 * .text
 ****************************************************************************/

/* Common exception handling logic.  On entry here, the return stack is on either
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
 *      IPSR contains the IRQ number
 *      R14 Contains the EXC_RETURN value
 *      We are in handler mode and the current SP is the MSP
 */

	.text
	.align	2
	.code	16
	.thumb_func
	.type	exception_common, function
exception_common:

	/* Complete the context save */

	pop     {r4}
	mrs		r1, msp
	add		r1, #4
	msr		msp, r1
	/* Get the current stack pointer.  The EXC_RETURN value tells us whether
	 * the context is on the MSP or PSP.
	 */
	mrs		r1, msp					/* R1=The main stack pointer */

	/* R1 is the current stack pointer.  HW_XCPT_REGS were pushed onto the stack
	 * when the interrupt was taken so (R1)+HW_XCPT_SIZE is the value of the
	 * stack pointer before the interrupt.  The total size of the context save
	 * area is XCPTCONTEXT_SIZE = SW_XCPT_SIZE + HW_XCPT_SIZE so (R1)-SW_XCPT_SIZE
	 * is the address of the beginning of the context save area.
	 */

	/*stack layout:(*for SP position)*/
	/*init:           *Jump(2W) + Exception(2W)*/
	/*before doIrq:   *SW_XCpt(10W) + Exception(2W) + Jump(2W) + Exception(2W)*/
	/*after doIrq:    *SW_XCpt(10W) + Jump(2W) + Exception(2W)*/
	/*stack layout:*/

2:
	/* Save SP, PRIMASK, and R4-R7 in the context array */
	sub		r1, #SW_XCPT_SIZE			/* R1=Beginning of context array on the stack */
	//sub		r1, #XCPTCONTEXT_SIZE			//new data should duplicate hw exception stack
	mov		r2, #XCPTCONTEXT_SIZE			/* R2=Size of the context array */
	add		r2, r1					/* R2=MSP/PSP before the interrupt was taken */
								/* (ignoring the xPSR[9] alignment bit) */
	mrs		r3, primask				/* R3=Current PRIMASK setting */
	mov		r0, r1					/* Copy the context array pointer */
	stmia	r0!, {r2-r7}		    /* Save the SP, PRIMASK, and R4-R7 in the context array */

	/* Save R8-R11 and the EXEC_RETURN value in the context array */

	mov		r2, r8					/* Copy high registers to low */
	mov		r3, r9
	mov		r4, r10
	mov		r5, r11
	stmia	r0!, {r2-r5}				/* Save the high registers r8-r11 */

	/*load hw interrupt stack*/
	//mov		r3, r0
	//add 	r3, #40				/*total 10 word, 2 word is for jump table*/
	//ldmia	r3!, {r4-r7}			/* Fetch four registers from the HW save area */
	//stmia	r0!, {r4-r7}
	//ldmia	r3!, {r4-r7}			/* Fetch four registers from the HW save area */
	//stmia	r0!, {r4-r7}

	/* Get the exception number in R0=IRQ, R1=register save area on stack */

	mrs		r0, ipsr				/* R0=exception number */

	/* If CONFIG_ARCH_INTERRUPTSTACK is defined, we will use a special interrupt
	 * stack pointer.  The way that this is done here prohibits nested interrupts!
	 * Otherwise, we will use the stack that was current when the interrupt was taken.
	 */

	msr		msp, r1					/* We are using the main stack pointer */
	bl		arm_doirq				/* R0=IRQ, R1=register save area on stack */
	mrs		r1, msp					/* Recover R1=main stack pointer */

	/* On return from arm_doirq, R0 will hold a pointer to register context
	 * array to use for the interrupt return.  If that return value is the same
	 * as current stack pointer, then things are relatively easy.
	 */

	cmp		r0, r1					/* Context switch? */
	beq		3f					/* Branch if no context switch */

	/* We are returning with a pending context switch.  This case is different
	 * because in this case, the register save structure does not lie on the
	 * stack but, rather within a TCB structure.  We'll have to copy some
	 * values to the stack.
	 */

	/* Copy the hardware-saved context to the new stack */

	mov		r2, #SW_XCPT_SIZE		/* R2=Size of software-saved portion of the context array */
	add		r1, r0, r2			/* R1=Address of HW save area in reg array */
	//add		r1, #8				/* skip dummy */
	ldr		r2, [r0, #(4*REG_SP)]		/* R2=Value of SP before the interrupt */
	//sub		r2, #(HW_XCPT_SIZE-8)		/* R2=Address of HW save area on the return stack and skip dummy */
	sub		r2, #(HW_XCPT_SIZE)		/* R2=Address of HW save area on the return stack and skip dummy */	
	ldmia		r1!, {r4-r7}			/* Fetch four registers from the HW save area */
	stmia		r2!, {r4-r7}			/* Copy four registers to the return stack */
	ldmia		r1!, {r4-r7}			/* Fetch four registers from the HW save area */
	stmia		r2!, {r4-r7}			/* Copy four registers to the return stack */

	/* Restore the register contents */

	mov		r1, r0

3:
	/* We are returning with no context switch.  We simply need to "unwind"
	 * the same stack frame that we created at entry.
	 */

	/* Recover R8-R11 and EXEC_RETURN (5 registers) */

	mov		r2, #(4*REG_R8)				/* R2=Offset to R8 storage */
	add		r0, r1, r2				/* R0=Address of R8 storage */

	ldmia		r0!, {r2-r5}				/* Recover R8-R11 and R14 (5 registers)*/
	mov		r8, r2					/* Move to position in high registers */
	mov		r9, r3
	mov		r10, r4
	mov		r11, r5


	/* Recover SP (R2), PRIMASK (R3), and R4-R7. Determine the value of
	 * the stack pointer as it was on entry to the exception handler.
	 */

	ldmia		r1!, {r2-r7}				/* Recover R4-R7 + 2 temp values */
	//mov		r1, #(HW_XCPT_SIZE-8)			/* R1=Size of hardware-saved portion of the context array */
	mov		r1, #HW_XCPT_SIZE			/* R1=Size of hardware-saved portion of the context array */
	sub		r1, r2, r1				/* R1=Value of MSP/PSP on exception entry */

	/* Restore the stack pointer.  The EXC_RETURN value tells us whether the
	 * context is on the MSP or PSP.
	 */

	msr		msp, r1					/* R1=The main stack pointer */
	ldr		r0, =EXC_RETURN_PRIVTHR			/* R0=EXC_RETURN to privileged mode */
	mov		r14, r0					/* R14=EXC_RETURN to privileged mode */

	/* Restore the interrupt state */

	msr		primask, r3				/* Restore interrupts priority masking*/

	/* Always return with R14 containing the special value that will: (1)
	 * return to thread mode, and (2) select the correct stack.
	 */

	bx		r14					/* And return */

	.size	exception_common, .-exception_common

/****************************************************************************
 *  Name: g_intstackalloc/g_intstacktop
 *
 * Description:
 *   Shouldn't happen
 *
 ****************************************************************************/


	.text
	.align	2
	.code	16
	.thumb_func
	.type	exception_common_inline, function
exception_common_inline:

	/* Complete the context save */

	/* Get the current stack pointer.  The EXC_RETURN value tells us whether
	 * the context is on the MSP or PSP.
	 */
	mrs		r1, msp					/* R1=The main stack pointer */
	//sub		r1, #8					/* align to normal jumpfunction mode */

	/* R1 is the current stack pointer.  HW_XCPT_REGS were pushed onto the stack
	 * when the interrupt was taken so (R1)+HW_XCPT_SIZE is the value of the
	 * stack pointer before the interrupt.  The total size of the context save
	 * area is XCPTCONTEXT_SIZE = SW_XCPT_SIZE + HW_XCPT_SIZE so (R1)-SW_XCPT_SIZE
	 * is the address of the beginning of the context save area.
	 */

	/*stack layout:(*for SP position)*/
	/*init:           *Jump(2W) + Exception(2W)*/
	/*before doIrq:   *SW_XCpt(10W) + Exception(2W) + Jump(2W) + Exception(2W)*/
	/*after doIrq:    *SW_XCpt(10W) + Jump(2W) + Exception(2W)*/
	/*stack layout:*/

2:
	/* Save SP, PRIMASK, and R4-R7 in the context array */
	sub		r1, #SW_XCPT_SIZE			/* R1=Beginning of context array on the stack */
	//sub		r1, #XCPTCONTEXT_SIZE			//new data should duplicate hw exception stack
	mov		r2, #XCPTCONTEXT_SIZE			/* R2=Size of the context array */
	add		r2, r1					/* R2=MSP/PSP before the interrupt was taken */
								/* (ignoring the xPSR[9] alignment bit) */
	mrs		r3, primask				/* R3=Current PRIMASK setting */
	mov		r0, r1					/* Copy the context array pointer */
	stmia	r0!, {r2-r7}		    /* Save the SP, PRIMASK, and R4-R7 in the context array */

	/* Save R8-R11 and the EXEC_RETURN value in the context array */

	mov		r2, r8					/* Copy high registers to low */
	mov		r3, r9
	mov		r4, r10
	mov		r5, r11
	stmia	r0!, {r2-r5}				/* Save the high registers r8-r11 */

	/*load hw interrupt stack*/
	//mov		r3, r0
	//add 	r3, #40				/*total 10 word, 2 word is for jump table*/
	//ldmia	r3!, {r4-r7}			/* Fetch four registers from the HW save area */
	//stmia	r0!, {r4-r7}
	//ldmia	r3!, {r4-r7}			/* Fetch four registers from the HW save area */
	//stmia	r0!, {r4-r7}

	/* Get the exception number in R0=IRQ, R1=register save area on stack */

	mrs		r0, ipsr				/* R0=exception number */

	/* If CONFIG_ARCH_INTERRUPTSTACK is defined, we will use a special interrupt
	 * stack pointer.  The way that this is done here prohibits nested interrupts!
	 * Otherwise, we will use the stack that was current when the interrupt was taken.
	 */

	msr		msp, r1					/* We are using the main stack pointer */
	bl		arm_doirq				/* R0=IRQ, R1=register save area on stack */
	mrs		r1, msp					/* Recover R1=main stack pointer */

	/* On return from arm_doirq, R0 will hold a pointer to register context
	 * array to use for the interrupt return.  If that return value is the same
	 * as current stack pointer, then things are relatively easy.
	 */

	cmp		r0, r1					/* Context switch? */
	beq		3f					/* Branch if no context switch */

	/* We are returning with a pending context switch.  This case is different
	 * because in this case, the register save structure does not lie on the
	 * stack but, rather within a TCB structure.  We'll have to copy some
	 * values to the stack.
	 */

	/* Copy the hardware-saved context to the new stack */

	mov		r2, #SW_XCPT_SIZE		/* R2=Size of software-saved portion of the context array */
	add		r1, r0, r2			/* R1=Address of HW save area in reg array */
	//add		r1, #8			/* skip dummy */
	ldr		r2, [r0, #(4*REG_SP)]		/* R2=Value of SP before the interrupt */
	//sub		r2, #(HW_XCPT_SIZE-8)		/* R2=Address of HW save area on the return stack and skip dummy */
	sub		r2, #(HW_XCPT_SIZE)		/* R2=Address of HW save area on the return stack and skip dummy */
	ldmia		r1!, {r4-r7}			/* Fetch four registers from the HW save area */
	stmia		r2!, {r4-r7}			/* Copy four registers to the return stack */
	ldmia		r1!, {r4-r7}			/* Fetch four registers from the HW save area */
	stmia		r2!, {r4-r7}			/* Copy four registers to the return stack */

	/* Restore the register contents */

	mov		r1, r0

3:
	/* We are returning with no context switch.  We simply need to "unwind"
	 * the same stack frame that we created at entry.
	 */

	/* Recover R8-R11 and EXEC_RETURN (5 registers) */

	mov		r2, #(4*REG_R8)				/* R2=Offset to R8 storage */
	add		r0, r1, r2				/* R0=Address of R8 storage */

	ldmia		r0!, {r2-r5}				/* Recover R8-R11 and R14 (5 registers)*/
	mov		r8, r2					/* Move to position in high registers */
	mov		r9, r3
	mov		r10, r4
	mov		r11, r5


	/* Recover SP (R2), PRIMASK (R3), and R4-R7. Determine the value of
	 * the stack pointer as it was on entry to the exception handler.
	 */

	ldmia		r1!, {r2-r7}				/* Recover R4-R7 + 2 temp values */
	//mov		r1, #(HW_XCPT_SIZE-8)			/* R1=Size of hardware-saved portion of the context array */
	mov		r1, #(HW_XCPT_SIZE)			/* R1=Size of hardware-saved portion of the context array */
	sub		r1, r2, r1				/* R1=Value of MSP/PSP on exception entry */

	/* Restore the stack pointer.  The EXC_RETURN value tells us whether the
	 * context is on the MSP or PSP.
	 */

	msr		msp, r1					/* R1=The main stack pointer */
	ldr		r0, =EXC_RETURN_PRIVTHR			/* R0=EXC_RETURN to privileged mode */
	mov		r14, r0					/* R14=EXC_RETURN to privileged mode */

	/* Restore the interrupt state */

	msr		primask, r3				/* Restore interrupts priority masking*/

	/* Always return with R14 containing the special value that will: (1)
	 * return to thread mode, and (2) select the correct stack.
	 */

	bx		r14					/* And return */

	.size	exception_common_inline, .-exception_common_inline

/****************************************************************************
 *  Name: g_intstackalloc/g_intstacktop
 *
 * Description:
 *   Shouldn't happen
 *
 ****************************************************************************/

#if CONFIG_ARCH_INTERRUPTSTACK > 3
	.bss
	.global	g_intstackalloc
	.global	g_intstacktop
	.balign	4
g_intstackalloc:
	.skip	(CONFIG_ARCH_INTERRUPTSTACK & ~3)
g_intstacktop:
	.size	g_intstackalloc, .-g_intstackalloc
#endif

	.end
