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
 *   This macro will use registers a2, a3 and a4.
 *   a2 - Points to the, possbily, new register save area.
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

	/* Get the mask of pending, enabled interrupts at this level. */

	rsr    ARG1, INTENABLE
	rsr    a3, INTERRUPT
	movi   a4, \mask
	and    ARG1, ARG1, a3
	and    ARG1, ARG1, a4           /* Set of pending, enabled interrupts for this level */
	beqz   ARG1, 1f                 /* Nothing to do */

	/* Link the pre-exception frame for debugging. At this point, a12 points to the
	 * allocated and filled exception stack frame (old value of SP in case of
	 * an interrupt stack).
	 */

	exception_backtrace a12 \level

		          					    /* Argument 1: Set of CPU interrupt to dispatch */
	mov     ARG2, a12					/* Argument 2: Top of stack = register save area */
	CALL    xtensa_int_decode

	/* xtensa_int_decode returns the address of the new register save area.
	 * Usually this would be the same as the current SP. But in the event of
	 * a context switch, it will instead refer to the TCB register save area.
	 */

	mov		a2, RETVAL

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

	/* Create an interrupt frame and save minimal context. */

	exception_entry 1

	/* Save the rest of the state context. */

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
	 * area in a2.
	 */

	dispatch_c_isr	1 XCHAL_INTLEVEL1_MASK a0

	/* Restore registers in preparation to return from interrupt */

	call0   _xtensa_context_restore     /* Preserves a2 */

	/* Restore only level-specific regs (the rest were already restored) */

	exception_exit 1

	/* Return from exception. RFE returns from either the UserExceptionVector
	 * or the KernelExceptionVector.  RFE sets PS.EXCM back to 0, and then
	 * jumps to the address in EPC[1]. PS.UM and PS.WOE are left unchanged.
	 */

	rfe

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

	/* Create an interrupt frame and save minimal context. */

	exception_entry 2

	/* Save the rest of the state context. */

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
	 * area in a2.
	 */

	dispatch_c_isr	2 XCHAL_INTLEVEL2_MASK a0

	/* Restore registers in preparation to return from interrupt */

	call0	_xtensa_context_restore			/* Preserves a2 */

	/* Restore only level-specific regs (the rest were already restored) */

	exception_exit 2

	/* Return from interrupt.  RFI restores the PS from EPS_2 and jumps to
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

	/* Create an interrupt frame and save minimal context. */

	exception_entry 3

	/* Save the rest of the state context. */

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
	 * area in a2.
	 */

	dispatch_c_isr	3 XCHAL_INTLEVEL3_MASK a0

	/* Restore registers in preparation to return from interrupt */

	call0	_xtensa_context_restore			/* Preserves a2 */

	/* Restore only level-specific regs (the rest were already restored) */

	exception_exit 3

	/* Return from interrupt.  RFI restores the PS from EPS_3 and jumps to
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

	/* Create an interrupt frame and save minimal context. */

	exception_entry 4

	/* Save the rest of the state context. */

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
	 * area in a2.
	 */

	dispatch_c_isr	4 XCHAL_INTLEVEL4_MASK a0

	/* Restore registers in preparation to return from interrupt */

	call0	_xtensa_context_restore			/* Preserves a2 */

	/* Restore only level-specific regs (the rest were already restored) */

	exception_exit 4

	/* Return from interrupt.  RFI restores the PS from EPS_4 and jumps to
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

	/* Create an interrupt frame and save minimal context. */

	exception_entry 5

	/* Save the rest of the state context. */

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
	 * area in a2.
	 */

	dispatch_c_isr	5 XCHAL_INTLEVEL5_MASK a0

	/* Restore registers in preparation to return from interrupt */

	call0	_xtensa_context_restore			/* Preserves a2 */

	/* Restore only level-specific regs (the rest were already restored) */

	exception_exit 5

	/* Return from interrupt.  RFI restores the PS from EPS_5 and jumps to
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

	/* Create an interrupt frame and save minimal context. */

	exception_entry 6

	/* Save the rest of the state context. */

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
	 * area in a2.
	 */

	dispatch_c_isr	6 XCHAL_INTLEVEL6_MASK a0

	/* Restore registers in preparation to return from interrupt */

	call0	_xtensa_context_restore			/* Preserves a2 */

	/* Restore only level-specific regs (the rest were already restored) */

	exception_exit 6

	/* Return from interrupt.  RFI restores the PS from EPS_6 and jumps to
	 * the address in EPC_6.
	 */

	rfi		6

#endif /* XCHAL_EXCM_LEVEL >= 6 */

#if XCHAL_HAVE_NMI
	.section HANDLER_SECTION, "ax"
	.type	_xtensa_nmi_handler, @function
	.global	_xtensa_nmi_handler
	.align	4

_xtensa_nmi_handler:
	/* For now, just panic */

	exception_entry XCHAL_NMILEVEL

	movi	a2, XTENSA_NMI_EXCEPTION		/* Argument 1: Error code */
	call0	_xtensa_panic					/* Does not return */

	.size	_xtensa_nmi_handler, . -_xtensa_nmi_handler

#endif /* XCHAL_HAVE_NMI */

#if XCHAL_HAVE_DEBUG
	.section HANDLER_SECTION, "ax"
	.type	_xtensa_debug_handler, @function
	.global	_xtensa_debug_handler
	.align	4

_xtensa_debug_handler:

	exception_entry XCHAL_DEBUGLEVEL

	movi	a2, XTENSA_DEBUG_EXCEPTION		/* Argument 1: Error code */
	call0	_xtensa_panic					/* Does not return */

	.size	_xtensa_debug_handler, . -_xtensa_debug_handler

#endif /* XCHAL_HAVE_DEBUG */

	.section HANDLER_SECTION, "ax"
	.type	_xtensa_kernel_handler, @function
	.global	_xtensa_kernel_handler
	.align	4

_xtensa_kernel_handler:

	exception_entry 1

	movi	a2, XTENSA_KERNEL_EXCEPTION		/* Argument 1: Error code */
	call0	_xtensa_panic					/* Does not return */

	.size	_xtensa_kernel_handler, . -_xtensa_kernel_handler

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

	/* Create an interrupt frame and save minimal context. */

	exception_entry 2

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

	/* Create an interrupt frame and save minimal context. */

	exception_entry 3

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

	/* Create an interrupt frame and save minimal context. */

	exception_entry 4

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

	/* Create an interrupt frame and save minimal context. */

	exception_entry 5

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

	/* Create an interrupt frame and save minimal context. */

	exception_entry 6

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
