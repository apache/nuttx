/****************************************************************************
 * arch/xtensa/src/common/xtensa_int_handlers.S
 *
 * Adapted from use in NuttX by:
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Derives from logic originally provided by Cadence Design Systems Inc.
 *
 *   Copyright (c) 2006-2015 Cadence Design Systems Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 ****************************************************************************/

	.file	"xtensa_int_handlers.S"

/* NOTES on the use of 'call0' for long jumps instead of 'j':
 *
 *  1. This file should be assembled with the -mlongcalls option to xt-xcc.
 *
 *  2. The -mlongcalls compiler option causes 'call0 dest' to be expanded to
 *     a sequence 'l32r a0, dest' 'callx0 a0' which works regardless of the
 *     distance from the call to the destination. The linker then relaxes
 *     it back to 'call0 dest' if it determines that dest is within range.
 *     This allows more flexibility in locating code without the performance
 *     overhead of the 'l32r' literal data load in cases where the destination
 *     is in range of 'call0'. There is an additional benefit in that 'call0'
 *     has a longer range than 'j' due to the target being word-aligned, so
 *     the 'l32r' sequence is less likely needed.
 *
 *  3. The use of 'call0' with -mlongcalls requires that register a0 not be
 *     live at the time of the call, which is always the case for a function
 *     call but needs to be ensured if 'call0' is used as a jump in lieu of 'j'.
 *
 *  4. This use of 'call0' is independent of the C function call ABI.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <arch/irq.h>
#include <arch/chip/core-isa.h>
#include <arch/xtensa/xtensa_abi.h>
#include <arch/xtensa/xtensa_specregs.h>

#include "xtensa_macros.S"

#include "chip.h"
#include "xtensa.h"
#include "xtensa_timer.h"

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

#if !defined(CONFIG_SMP) && CONFIG_ARCH_INTERRUPTSTACK > 15
 .data
 .align     16
 .global    g_intstackalloc
 .global    g_intstacktop
 .type      g_intstackalloc, @object
 .type      g_intstacktop, @object
g_intstackalloc:
 .skip      INTSTACK_SIZE
g_intstacktop:
 .size      g_intstackalloc, .-g_intstackalloc
#endif

/****************************************************************************
 * Assembly Language Macros
 ****************************************************************************/

/****************************************************************************
 * Macro extract_msb - return the input with only the highest bit set.
 *
 * Entry Conditions/Side Effects:
 *   Input  : "ain"  - Input value, clobbered.
 *   Output : "aout" - Output value, has only one bit set, MSB of "ain".
 *
 * The two arguments must be different AR registers.
 *
 ****************************************************************************/

	.macro	extract_msb	aout ain
1:
	addi	\aout, \ain, -1				/* aout = ain - 1 */
	and		\ain, \ain, \aout			/* ain  = ain & aout */
	bnez	\ain, 1b					    /* Repeat until ain == 0 */
	addi	\aout, \aout, 1				/* Return aout + 1 */
	.endm

/****************************************************************************
 * Macro dispatch_c_isr level mask tmp
 *
 * Description:
 *
 * This macro will dispatch the set of pending and enabled interrupts to
 * their handlers. It just does the necessary configuration and ends up
 * calling xtensa_int_decode.
 *
 * Assumptions:
 *   - PS.INTLEVEL is set to "level" at entry
 *   - PS.EXCM = 0, C calling enabled
 *
 * Entry Conditions/Side Effects:
 *   level - interrupt level
 *   mask  - interrupt bitmask for this level
 *   a12   - register save area
 *
 * Exit Conditions:
 *   This macro will use registers a0 and a2-a5 and a12.
 *   a1  - May point to the new thread's SP
 *   a12 - Points to the register save area (which may not be on the stack).
 *
 ****************************************************************************/

	.macro	dispatch_c_isr	level mask tmp

	/* If the interrupt stack is disabled, reserve xcpcontext to ensure
	 * that signal processing can have a separate xcpcontext to handle
	 * signal context (ref: xtensa_schedulesigaction.c):
	 */

#if CONFIG_ARCH_INTERRUPTSTACK < 15
	addi	sp, sp, -XCPTCONTEXT_SIZE
#endif

	/* Set up PS for C, enable interrupts above this level and clear EXCM. */

	ps_setup	\level \tmp

#ifdef __XTENSA_CALL0_ABI__
	/* Get mask of pending, enabled interrupts at this level into a2. */

	rsr		a2, INTENABLE
	rsr		a3, INTERRUPT
	movi	a4, \mask
	and		a2, a2, a3
	and		a2, a2, a4				/* a2 = Set of pending, enabled interrupts for this level */
	beqz	a2, 1f						/* Nothing to do */

	/* Call xtensa_int_decode passing the address of the register save area
	 * as a parameter (A3).
	 */

		          						  /* Argument 1: Set of CPU interrupt to dispatch */
	mov		a3, a12					    /* Argument 2: Top of stack = register save area */
	call0	xtensa_int_decode	  /* Call xtensa_int_decode */

	/* On return from xtensa_int_decode, a2 will contain the address of the new
	 * register save area.  Usually this would be the same as the current SP.
	 * But in the event of a context switch, a2 will instead refer to the TCB
	 * register save area.  This may or may not reside on a stack.
	 */

	mov		a12, a2						/* Switch to the save area of the new thread */

#else
	/* Get mask of pending, enabled interrupts at this level into a6. */

	rsr		a6, INTENABLE
	rsr		a2, INTERRUPT
	movi	a3, \mask
	and		a6, a6, a2
	and		a6, a6, a3				/* a6 = Set of pending, enabled interrupts for this level */
	beqz	a6, 1f						/* Nothing to do */

	/* At this point, the exception frame should have been allocated and filled,
	 * and current sp points to the interrupt stack (if enabled). Copy the
	 * pre-exception's base save area below the current SP. We saved the SP in A12
	 * before getting here.
	 */

#ifdef CONFIG_XTENSA_INTBACKTRACE
	l32i a3, a12, (4 * REG_A0)        /* Copy pre-exception a0 (return address) */
	s32e a3, sp, -16
	l32i a3, a12, (4 * REG_A1)        /* Copy pre-exception a1 (stack pointer) */
	s32e a3, sp, -12

	/* Backtracing only needs a0 and a1, no need to create full base save area.
	 * Also need to change current frame's return address to point to pre-exception's
	 * last run instruction.
	 */

	rsr a0, EPC_1 + \level - 1  /* return address */
	movi a4, 0xc0000000         /* constant with top 2 bits set (call size) */
	or a0, a0, a4               /* set top 2 bits */
	addx2 a0, a4, a0            /* clear top bit -- simulating call4 size   */
#endif

	/* Call xtensa_int_decode passing the address of the register save area
	 * as a parameter (A7).
	 */

										        /* Argument 1: Set of CPU interrupt to dispatch */
	mov		a7, a12					  	/* Argument 2: Top of stack = register save area */
	call4	xtensa_int_decode		/* Call xtensa_int_decode */

	/* On return from xtensa_int_decode, a6 will contain the address of the new
	 * register save area.  Usually this would be the same as the current SP.
	 * But in the event of a context switch, a6 will instead refer to the TCB
	 * register save area.  This may or may not reside on a stack.
	 */

	mov		a12, a6						/* Switch to the save area of the new thread */
#endif

#if CONFIG_ARCH_INTERRUPTSTACK < 15
	addi	sp, sp, XCPTCONTEXT_SIZE
#endif

	/* Done */

1:
	.endm

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * LEVEL 1 INTERRUPT HANDLER
 ****************************************************************************/
/* The level1 interrupt vector is invoked via the User exception vector. */

	.section HANDLER_SECTION, "ax"
	.type	_xtensa_level1_handler, @function
	.global	_xtensa_level1_handler
	.align	4

_xtensa_level1_handler:

	mov		a0, sp							              /* Save SP in A0 */
	addi	sp, sp, -XCPTCONTEXT_SIZE   /* Allocate interrupt stack frame */
	s32i	a0, sp, (4 * REG_A1)			        /* Save pre-interrupt SP */
	rsr		a0, PS						                /* Save interruptee's PS */
	s32i	a0, sp, (4 * REG_PS)
	rsr		a0, EPC_1						              /* Save interruptee's PC */
	s32i	a0, sp, (4 * REG_PC)
	rsr		a0, EXCSAVE_1					            /* Save interruptee's a0 */
	s32i	a0, sp, (4 * REG_A0)
	s32i	a2, sp, (4 * REG_A2)

	/* Save rest of interrupt context. */

	call0	_xtensa_context_save

  /* Save current SP before (possibly) overwriting it, it's the register save
   * area. This value will be used later by dispatch_c_isr to retrieve the
   * register save area.
   */

  mov  a12, sp

  /* Switch to an interrupt stack if we have one */

#if CONFIG_ARCH_INTERRUPTSTACK > 15
  setintstack a13 a14
#endif

	/* Decode and dispatch the interrupt.  In the event of an interrupt
	 * level context dispatch_c_isr() will (1) switch stacks to the new
	 * thread's and (2) provide the address of the register state save
	 * area in a12.  NOTE that the state save area may or may not lie
	 * in the new thread's stack.
	 */

	dispatch_c_isr	1 XCHAL_INTLEVEL1_MASK a0

	/* Restore registers in preparation to return from interrupt */

	mov		a2, a12							      /* a2 = address of new state save area */
	call0	_xtensa_context_restore		/* (preserves a2) */

	/* Restore only level-specific regs (the rest were already restored) */

	l32i	a0, a2, (4 * REG_PS)      /* Retrieve interruptee's PS */
	wsr		a0, PS
	l32i	a0, a2, (4 * REG_PC)      /* Retrieve interruptee's PC */
	wsr		a0, EPC_1
	l32i	a0, a2, (4 * REG_A0)      /* Retrieve interruptee's A0 */
	l32i	sp, a2, (4 * REG_A1)      /* Retrieve interrupt stack frame */
	l32i	a2, a2, (4 * REG_A2)      /* Retrieve interruptee's A2 */
	rsync								          	/* Ensure PS and EPC written */

	/* Return from exception. RFE returns from either the UserExceptionVector
	 * or the KernelExceptionVector.  RFE sets PS.EXCM back to 0, and then
	 * jumps to the address in EPC[1]. PS.UM and PS.WOE are left unchanged.
	 */

	rfe										          /* And return from "exception" */

/****************************************************************************
 * MEDIUM PRIORITY (LEVEL 2+) INTERRUPT LOW LEVEL HANDLERS.
 *
 * C Prototype:
 *   void _xtensa_levelN_handler(void)
 *
 * Description:
 *   Medium priority interrupts are by definition those with priority greater
 *   than 1 and not greater than XCHAL_EXCM_LEVEL. These are disabled
 *   by setting PS.EXCM and therefore can easily support a C environment for
 *   handlers in C, and interact safely with NuttX.
 *
 *   Each vector goes at a predetermined location according to the Xtensa
 *   hardware configuration, which is ensured by its placement in a special
 *   section known to the NuttX linker script.  The vector logic performs
 *   the minimum necessary operations before jumping to the handler via
 *   a CALL0 instruction.  See "NOTES on the use of call0 ..." above.
 *
 *   The corresponding handler sets up the appropriate stack frame, saves
 *   a few vector-specific registers and calls _xtensa_context_save()
 *   to save the rest of the interrupted context.  It then calls the C
 *   logic to decode the specific interrupt source and dispatch to the
 *   appropriate C interrupt handler.
 *
 ****************************************************************************/

#if XCHAL_EXCM_LEVEL >= 2
	.section HANDLER_SECTION, "ax"
	.type	_xtensa_level2_handler, @function
	.global	_xtensa_level2_handler
	.align	4

_xtensa_level2_handler:

	mov		a0, sp							/* Save SP in A0 */
	addi	sp, sp, -XCPTCONTEXT_SIZE	/* Allocate interrupt stack frame */
	s32i	a0, sp, (4 * REG_A1)			/* Save pre-interrupt SP */
	rsr		a0, EPS_2						/* Save interruptee's PS */
	s32i	a0, sp, (4 * REG_PS)
	rsr		a0, EPC_2						/* Save interruptee's PC */
	s32i	a0, sp, (4 * REG_PC)
	rsr		a0, EXCSAVE_2					/* Save interruptee's a0 */
	s32i	a0, sp, (4 * REG_A0)
	s32i	a2, sp, (4 * REG_A2)

	/* Save rest of interrupt context. */

	call0	_xtensa_context_save

	/* Save current SP before (possibly) overwriting it, it's the register save
	 * area. This value will be used later by dispatch_c_isr to retrieve the
	 * register save area.
	 */

	mov  a12, sp

	/* Switch to an interrupt stack if we have one */

#if CONFIG_ARCH_INTERRUPTSTACK > 15
	setintstack a13 a14
#endif

	/* Decode and dispatch the interrupt.  In the event of an interrupt
	 * level context dispatch_c_isr() will (1) switch stacks to the new
	 * thread's and (2) provide the address of the register state save
	 * area in a12.  NOTE that the state save area may or may not lie
	 * in the new thread's stack.
	 */

	dispatch_c_isr	2 XCHAL_INTLEVEL2_MASK a0

	/* Restore registers in preparation to return from interrupt */

	mov		a2, a12							/* a2 = address of new state save area */
	call0	_xtensa_context_restore			/* (preserves a2) */

	/* Restore only level-specific regs (the rest were already restored) */

	l32i	a0, a2, (4 * REG_PS)      /* Retrieve interruptee's PS */
	wsr		a0, EPS_2
	l32i	a0, a2, (4 * REG_PC)      /* Retrieve interruptee's PC */
	wsr		a0, EPC_2
	l32i	a0, a2, (4 * REG_A0)      /* Retrieve interruptee's A0 */
	l32i	sp, a2, (4 * REG_A1)      /* Retrieve interrupt stack frame */
	l32i	a2, a2, (4 * REG_A2)      /* Retrieve interruptee's A2 */
	rsync									/* Ensure EPS and EPC written */

	/* Return from interrupt.  RFI  restores the PS from EPS_2 and jumps to
	 * the address in EPC_2.
	 */

	rfi		2

#endif /* XCHAL_EXCM_LEVEL >= 2 */

#if XCHAL_EXCM_LEVEL >= 3
	.section HANDLER_SECTION, "ax"
	.type	_xtensa_level3_handler, @function
	.global	_xtensa_level3_handler
	.align	4

_xtensa_level3_handler:

	mov		a0, sp							/* Save SP in A0 */
	addi	sp, sp, -XCPTCONTEXT_SIZE	/* Allocate interrupt stack frame */
	s32i	a0, sp, (4 * REG_A1)			/* Save pre-interrupt SP */
	rsr		a0, EPS_3						/* Save interruptee's PS */
	s32i	a0, sp, (4 * REG_PS)
	rsr		a0, EPC_3						/* Save interruptee's PC */
	s32i	a0, sp, (4 * REG_PC)
	rsr		a0, EXCSAVE_3					/* Save interruptee's a0 */
	s32i	a0, sp, (4 * REG_A0)
	s32i	a2, sp, (4 * REG_A2)

	/* Save rest of interrupt context. */

	call0	_xtensa_context_save

	/* Save current SP before (possibly) overwriting it, it's the register save
	 * area. This value will be used later by dispatch_c_isr to retrieve the
	 * register save area.
	 */

	mov  a12, sp

	/* Switch to an interrupt stack if we have one */

#if CONFIG_ARCH_INTERRUPTSTACK > 15
	setintstack a13 a14
#endif

	/* Decode and dispatch the interrupt.  In the event of an interrupt
	 * level context dispatch_c_isr() will (1) switch stacks to the new
	 * thread's and (2) provide the address of the register state save
	 * area in a12.  NOTE that the state save area may or may not lie
	 * in the new thread's stack.
	 */

	dispatch_c_isr	3 XCHAL_INTLEVEL3_MASK a0

	/* Restore registers in preparation to return from interrupt */

	mov		a2, a12							/* a2 = address of new state save area */
	call0	_xtensa_context_restore			/* (preserves a2) */

	/* Restore only level-specific regs (the rest were already restored) */

	l32i	a0, a2, (4 * REG_PS)      /* Retrieve interruptee's PS */
	wsr		a0, EPS_3
	l32i	a0, a2, (4 * REG_PC)      /* Retrieve interruptee's PC */
	wsr		a0, EPC_3
	l32i	a0, a2, (4 * REG_A0)      /* Retrieve interruptee's A0 */
	l32i	sp, a2, (4 * REG_A1)      /* Retrieve interrupt stack frame */
	l32i	a2, a2, (4 * REG_A2)      /* Retrieve interruptee's A2 */
	rsync									/* Ensure EPS and EPC written */

	/* Return from interrupt.  RFI  restores the PS from EPS_3 and jumps to
	 * the address in EPC_3.
	 */

	rfi		3

#endif /* XCHAL_EXCM_LEVEL >= 3 */

#if XCHAL_EXCM_LEVEL >= 4
	.section HANDLER_SECTION, "ax"
	.type	_xtensa_level4_handler, @function
	.global	_xtensa_level4_handler
	.align	4

_xtensa_level4_handler:

	mov		a0, sp							/* Save SP in A0 */
	addi	sp, sp, -XCPTCONTEXT_SIZE	/* Allocate interrupt stack frame */
	s32i	a0, sp, (4 * REG_A1)			/* Save pre-interrupt SP */
	rsr		a0, EPS_4						/* Save interruptee's PS */
	s32i	a0, sp, (4 * REG_PS)
	rsr		a0, EPC_4						/* Save interruptee's PC */
	s32i	a0, sp, (4 * REG_PC)
	rsr		a0, EXCSAVE_4					/* Save interruptee's a0 */
	s32i	a0, sp, (4 * REG_A0)
	s32i	a2, sp, (4 * REG_A2)

	/* Save rest of interrupt context. */

	call0	_xtensa_context_save

	/* Save current SP before (possibly) overwriting it, it's the register save
	 * area. This value will be used later by dispatch_c_isr to retrieve the
	 * register save area.
	 */

	mov  a12, sp

	/* Switch to an interrupt stack if we have one */

#if CONFIG_ARCH_INTERRUPTSTACK > 15
	setintstack a13 a14
#endif

	/* Decode and dispatch the interrupt.  In the event of an interrupt
	 * level context dispatch_c_isr() will (1) switch stacks to the new
	 * thread's and (2) provide the address of the register state save
	 * area in a12.  NOTE that the state save area may or may not lie
	 * in the new thread's stack.
	 */

	dispatch_c_isr	4 XCHAL_INTLEVEL4_MASK a0

	/* Restore registers in preparation to return from interrupt */

	mov		a2, a12							/* a2 = address of new state save area */
	call0	_xtensa_context_restore			/* (presevers a2) */

	/* Restore only level-specific regs (the rest were already restored) */

	l32i	a0, a2, (4 * REG_PS)      /* Retrieve interruptee's PS */
	wsr		a0, EPS_4
	l32i	a0, a2, (4 * REG_PC)      /* Retrieve interruptee's PC */
	wsr		a0, EPC_4
	l32i	a0, a2, (4 * REG_A0)      /* Retrieve interruptee's A0 */
	l32i	sp, a2, (4 * REG_A1)      /* Retrieve interrupt stack frame */
	l32i	a2, a2, (4 * REG_A2)      /* Retrieve interruptee's A2 */
	rsync									/* Ensure EPS and EPC written */

	/* Return from interrupt.  RFI  restores the PS from EPS_4 and jumps to
	 * the address in EPC_4.
	 */

	rfi		4

#endif /* XCHAL_EXCM_LEVEL >= 4 */

#if XCHAL_EXCM_LEVEL >= 5
	.section HANDLER_SECTION, "ax"
	.type	_xtensa_level5_handler, @function
	.global	_xtensa_level5_handler
	.align	4

_xtensa_level5_handler:

	mov		a0, sp							/* Save SP in A0 */
	addi	sp, sp, -XCPTCONTEXT_SIZE	/* Allocate interrupt stack frame */
	s32i	a0, sp, (4 * REG_A1)			/* Save pre-interrupt SP */
	rsr		a0, EPS_5						/* Save interruptee's PS */
	s32i	a0, sp, (4 * REG_PS)
	rsr		a0, EPC_5						/* Save interruptee's PC */
	s32i	a0, sp, (4 * REG_PC)
	rsr		a0, EXCSAVE_5					/* Save interruptee's a0 */
	s32i	a0, sp, (4 * REG_A0)
	s32i	a2, sp, (4 * REG_A2)

	/* Save rest of interrupt context. */

	call0	_xtensa_context_save

	/* Save current SP before (possibly) overwriting it, it's the register save
	 * area. This value will be used later by dispatch_c_isr to retrieve the
	 * register save area.
	 */

	mov  a12, sp

	/* Switch to an interrupt stack if we have one */

#if CONFIG_ARCH_INTERRUPTSTACK > 15
	setintstack a13 a14
#endif

	/* Decode and dispatch the interrupt.  In the event of an interrupt
	 * level context dispatch_c_isr() will (1) switch stacks to the new
	 * thread's and (2) provide the address of the register state save
	 * area in a12.  NOTE that the state save area may or may not lie
	 * in the new thread's stack.
	 */

	dispatch_c_isr	5 XCHAL_INTLEVEL5_MASK a0

	/* Restore registers in preparation to return from interrupt */

	mov		a2, a12							/* a2 = address of new state save area */
	call0	_xtensa_context_restore			/* (preserves a2) */

	/* Restore only level-specific regs (the rest were already restored) */

	l32i	a0, a2, (4 * REG_PS)      /* Retrieve interruptee's PS */
	wsr		a0, EPS_5
	l32i	a0, a2, (4 * REG_PC)      /* Retrieve interruptee's PC */
	wsr		a0, EPC_5
	l32i	a0, a2, (4 * REG_A0)      /* Retrieve interruptee's A0 */
	l32i	sp, a2, (4 * REG_A1)      /* Retrieve interrupt stack frame */
	l32i	a2, a2, (4 * REG_A2)      /* Retrieve interruptee's A2 */
	rsync									/* Ensure EPS and EPC written */

	/* Return from interrupt.  RFI  restores the PS from EPS_5 and jumps to
	 * the address in EPC_5.
	 */

	rfi		5

#endif /* XCHAL_EXCM_LEVEL >= 5 */

#if XCHAL_EXCM_LEVEL >= 6
	.section HANDLER_SECTION, "ax"
	.type	_xtensa_level6_handler, @function
	.global	_xtensa_level6_handler
	.align	4

_xtensa_level6_handler:

	mov		a0, sp							/* Save SP in A0 */
	addi	sp, sp, -XCPTCONTEXT_SIZE	/* Allocate interrupt stack frame */
	s32i	a0, sp, (4 * REG_A1)			/* Save pre-interrupt SP */
	rsr		a0, EPS_6						/* Save interruptee's PS */
	s32i	a0, sp, (4 * REG_PS)
	rsr		a0, EPC_6						/* Save interruptee's PC */
	s32i	a0, sp, (4 * REG_PC)
	rsr		a0, EXCSAVE_6					/* Save interruptee's a0 */
	s32i	a0, sp, (4 * REG_A0)
	s32i	a2, sp, (4 * REG_A2)

	/* Save rest of interrupt context. */

	call0	_xtensa_context_save			/* Save full register state */

	/* Save current SP before (possibly) overwriting it, it's the register save
	 * area. This value will be used later by dispatch_c_isr to retrieve the
	 * register save area.
	 */

	mov  a12, sp

	/* Switch to an interrupt stack if we have one */

#if CONFIG_ARCH_INTERRUPTSTACK > 15
	setintstack a13 a14
#endif

	/* Decode and dispatch the interrupt.  In the event of an interrupt
	 * level context dispatch_c_isr() will (1) switch stacks to the new
	 * thread's and (2) provide the address of the register state save
	 * area in a12.  NOTE that the state save area may or may not lie
	 * in the new thread's stack.
	 */

	dispatch_c_isr	6 XCHAL_INTLEVEL6_MASK a0

	/* Restore registers in preparation to return from interrupt */

	mov		a2, a12							/* a2 = address of new state save area */
	call0	_xtensa_context_restore			/* (preserves a2) */

	/* Restore only level-specific regs (the rest were already restored) */

	l32i	a0, a2, (4 * REG_PS)			/* Retrieve interruptee's PS */
	wsr		a0, EPS_6
	l32i	a0, a2, (4 * REG_PC)			/* Retrieve interruptee's PC */
	wsr		a0, EPC_6
	l32i	a0, a2, (4 * REG_A0)			/* Retrieve interruptee's A0 */
	l32i	sp, a2, (4 * REG_A1)			/* Retrieve interrupt stack frame */
	l32i	a2, a2, (4 * REG_A2)			/* Retrieve interruptee's A2 */
	rsync									/* Ensure EPS and EPC written */

	/* Return from interrupt.  RFI  restores the PS from EPS_6 and jumps to
	 * the address in EPC_6.
	 */

	rfi		6

#endif /* XCHAL_EXCM_LEVEL >= 6 */

/****************************************************************************
 * HIGH PRIORITY (LEVEL > XCHAL_EXCM_LEVEL) LOW-LEVEL HANDLERS
 *
 * High priority interrupts are by definition those with priorities greater
 * than XCHAL_EXCM_LEVEL. This includes non-maskable (NMI). High priority
 * interrupts cannot interact with NuttX, that is they must save all regs
 * they use and not call any NuttX function.
 *
 * A further restriction imposed by the Xtensa windowed architecture is that
 * high priority interrupts must not modify the stack area even logically
 * "above" the top of the interrupted stack (they need to provide their
 * own stack or static save area).
 *
 * Cadence Design Systems recommends high priority interrupt handlers be
 * coded in assembly and used for purposes requiring very short service
 * times.
 *
 * Here are templates for high priority (level 2+) interrupt vectors.
 * They assume only one interrupt per level to avoid the burden of
 * identifying which interrupts at this level are pending and enabled. This
 * allows for  minimum latency and avoids having to save/restore a2 in
 * addition to a0.  If more than one interrupt per high priority level is
 * configured, this burden is on the handler which in any case must provide
 * a way to save and restore registers it uses without touching the
 * interrupted stack.
 *
 * A template and example can be found in the Cadence Design Systems tools
 * documentation: "Microprocessor Programmer's Guide".
 *
 * Each vector goes at a predetermined location according to the Xtensa
 * hardware configuration, which is ensured by its placement in a special
 * section known to the Xtensa linker support package (LSP). It performs
 * the minimum necessary before jumping to the handler in the .text section.
 *
 ****************************************************************************/

#if XCHAL_NUM_INTLEVELS >= 2 && XCHAL_EXCM_LEVEL < 2 && XCHAL_DEBUGLEVEL != 2
	.section	HANDLER_SECTION, "ax"
	.type		_xtensa_level2_handler, @function
	.global		_xtensa_level2_handler
	.align		4

_xtensa_level2_handler:

#if 1
	/* For now, just panic */

	mov		a0, sp							/* Save SP in A0 */
	addi	sp, sp, -XCPTCONTEXT_SIZE	/* Allocate interrupt stack frame */
	s32i	a0, sp, (4 * REG_A1)			/* Save pre-interrupt SP */
	rsr		a0, EPS_2						/* Save interruptee's PS */
	s32i	a0, sp, (4 * REG_PS)
	rsr		a0, EPC_2						/* Save interruptee's PC */
	s32i	a0, sp, (4 * REG_PC)
	rsr		a0, EXCSAVE_2					/* Save interruptee's a0 */
	s32i	a0, sp, (4 * REG_A0)

	s32i	a2, sp, (4 * REG_A2)
	movi	a2, XTENSA_LEVEL2_EXCEPTION	/* Address of state save on stack */
	call0	_xtensa_panic				/* Does not return */

#else
	/* Add high priority level 2 interrupt handler code here. */

	rsr		a0, EXCSAVE_2				/* Restore a0 */
	rfi		2

#endif
#endif /* XCHAL_NUM_INTLEVELS >= 2 && XCHAL_EXCM_LEVEL < 2 && XCHAL_DEBUGLEVEL != 2 */

#if XCHAL_NUM_INTLEVELS >= 3 && XCHAL_EXCM_LEVEL < 3 && XCHAL_DEBUGLEVEL != 3
	.section	HANDLER_SECTION, "ax"
	.type		_xtensa_level3_handler, @function
	.global		_xtensa_level3_handler
	.align		4

_xtensa_level3_handler:

#if 1
	/* For now, just panic */

	mov		a0, sp							/* Save SP in A0 */
	addi	sp, sp, -XCPTCONTEXT_SIZE	/* Allocate interrupt stack frame */
	s32i	a0, sp, (4 * REG_A1)			/* Save pre-interrupt SP */
	rsr		a0, EPS_3						/* Save interruptee's PS */
	s32i	a0, sp, (4 * REG_PS)
	rsr		a0, EPC_3						/* Save interruptee's PC */
	s32i	a0, sp, (4 * REG_PC)
	rsr		a0, EXCSAVE_3					/* Save interruptee's a0 */
	s32i	a0, sp, (4 * REG_A0)

	s32i	a2, sp, (4 * REG_A2)
	movi	a2, XTENSA_LEVEL3_EXCEPTION	/* Address of state save on stack */
	call0	_xtensa_panic				/* Does not return */

#else
	wsr		a0, EXCSAVE_3				/* Save a0 */

	/* Add high priority level 3 interrupt handler code here. */

	rsr		a0, EXCSAVE_3				/* Restore a0 */
	rfi		3

#endif
#endif /* XCHAL_NUM_INTLEVELS >= 3 && XCHAL_EXCM_LEVEL < 3 && XCHAL_DEBUGLEVEL != 3 */

#if XCHAL_NUM_INTLEVELS >= 4 && XCHAL_EXCM_LEVEL < 4 && XCHAL_DEBUGLEVEL != 4
	.section	HANDLER_SECTION, "ax"
	.type		_xtensa_level4_handler, @function
	.global		_xtensa_level4_handler
	.align		4

_xtensa_level4_handler:

#if 1
	/* For now, just panic */

	mov		a0, sp							/* Save SP in A0 */
	addi	sp, sp, -XCPTCONTEXT_SIZE	/* Allocate interrupt stack frame */
	s32i	a0, sp, (4 * REG_A1)			/* Save pre-interrupt SP */
	rsr		a0, EPS_4						/* Save interruptee's PS */
	s32i	a0, sp, (4 * REG_PS)
	rsr		a0, EPC_4						/* Save interruptee's PC */
	s32i	a0, sp, (4 * REG_PC)
	rsr		a0, EXCSAVE_4					/* Save interruptee's a0 */
	s32i	a0, sp, (4 * REG_A0)

	s32i	a2, sp, (4 * REG_A2)
	movi	a2, XTENSA_LEVEL4_EXCEPTION	/* Address of state save on stack */
	call0	_xtensa_panic				/* Does not return */

#else
	wsr		a0, EXCSAVE_4				/* Save a0 */

	/* Add high priority level 3 interrupt handler code here. */

	rsr		a0, EXCSAVE_4				/* Restore a0 */
	rfi		4

#endif
#endif /* XCHAL_NUM_INTLEVELS >= 4 && XCHAL_EXCM_LEVEL < 4 && XCHAL_DEBUGLEVEL != 4 */

#if XCHAL_NUM_INTLEVELS >= 5 && XCHAL_EXCM_LEVEL < 5 && XCHAL_DEBUGLEVEL != 5
	.section	HANDLER_SECTION, "ax"
	.type		_xtensa_level5_handler, @function
	.global		_xtensa_level5_handler
	.align		4

_xtensa_level5_handler:

#if 1
	/* For now, just panic */

	mov		a0, sp							/* Save SP in A0 */
	addi	sp, sp, -XCPTCONTEXT_SIZE	/* Allocate interrupt stack frame */
	s32i	a0, sp, (4 * REG_A1)			/* Save pre-interrupt SP */
	rsr		a0, EPS_5						/* Save interruptee's PS */
	s32i	a0, sp, (4 * REG_PS)
	rsr		a0, EPC_5						/* Save interruptee's PC */
	s32i	a0, sp, (4 * REG_PC)
	rsr		a0, EXCSAVE_5					/* Save interruptee's a0 */
	s32i	a0, sp, (4 * REG_A0)

	s32i	a2, sp, (4 * REG_A2)
	movi	a2, XTENSA_LEVEL5_EXCEPTION	/* Address of state save on stack */
	call0	_xtensa_panic				/* Does not return */

#else
	wsr		a0, EXCSAVE_5				/* Save a0 */

	/* Add high priority level 5 interrupt handler code here. */

	rsr		a0, EXCSAVE_5				/* Restore a0 */
	rfi		5

#endif
#endif /* XCHAL_NUM_INTLEVELS >= 5 && XCHAL_EXCM_LEVEL < 5 && XCHAL_DEBUGLEVEL != 5 */

#if XCHAL_NUM_INTLEVELS >= 6 && XCHAL_EXCM_LEVEL < 6 && XCHAL_DEBUGLEVEL != 6
	.section	HANDLER_SECTION, "ax"
	.type		_xtensa_level6_handler, @function
	.global		_xtensa_level6_handler
	.align		4

_xtensa_level6_handler:

#if 1
	/* For now, just panic */

	mov		a0, sp							/* Save SP in A0 */
	addi	sp, sp, -XCPTCONTEXT_SIZE	/* Allocate interrupt stack frame */
	s32i	a0, sp, (4 * REG_A1)			/* Save pre-interrupt SP */
	rsr		a0, EPS_6						/* Save interruptee's PS */
	s32i	a0, sp, (4 * REG_PS)
	rsr		a0, EPC_6						/* Save interruptee's PC */
	s32i	a0, sp, (4 * REG_PC)
	rsr		a0, EXCSAVE_6					/* Save interruptee's a0 */
	s32i	a0, sp, (4 * REG_A0)

	s32i	a2, sp, (4 * REG_A2)
	movi	a2, XTENSA_LEVEL6_EXCEPTION	/* Address of state save on stack */
	call0	_xtensa_panic				/* Does not return */

#else
	wsr		a0, EXCSAVE_6				/* Save a0 */

	/* Add high priority level 6 interrupt handler code here. */

	rsr		a0, EXCSAVE_6				/* Restore a0 */
	rfi		6

#endif
#endif /* XCHAL_NUM_INTLEVELS >= 6 && XCHAL_EXCM_LEVEL < 6 && XCHAL_DEBUGLEVEL != 6 */
