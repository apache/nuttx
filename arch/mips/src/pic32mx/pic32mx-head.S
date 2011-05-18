/****************************************************************************
 * arch/mips/src/pic32mx/pic32mx-head.S
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <arch/mips32/registers.h>
#include <arch/mips32/cp0.h>
#include <arch/pic32mx/cp0.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Data memory is organized as follows:
 *
 * 1) Possible space reserved for debug data
 * 2) Ram functions: (.data):
 *    Start:   _sramfunc
 *    End(+1): _eramfunc
 * 3) Initialized data (.data):
 *    Start:   _sdata
 *    End(+1): _edata
 * 4) Uninitialized data (.bss):
 *    Start:   _sbss
 *    End(+1): _ebss
 * 5) Idle thread stack:
 *    Start:   _ebss
 *    End(+1): _ebss+CONFIG_IDLETHREAD_STACKSIZE
 * 6) Heap                       Range  
 *    Start:   _ebss+CONFIG_IDLETHREAD_STACKSIZE
 *    End(+1): to the end of memory
 */

#define PIC32MX_STACK_BASE _ebss
#define PIC32MX_STACK_TOP  _ebss+CONFIG_IDLETHREAD_STACKSIZE-4
#define PIC32MX_HEAP_BASE  _ebss+CONFIG_IDLETHREAD_STACKSIZE

/****************************************************************************
 * Global Symbols
 ****************************************************************************/

	.file	"pic32mx-head.S"
	.global __start
	.global halt

	.global	__nmi_handler
	.global os_start

/****************************************************************************
 * Name: __reset
 *
 * Description:
 *   Reset entry point.  This function is positioned at the beginning of
 *   the boot FLASH by the linker in KSEG1.  Simply jumps to the __start
 *   logic in KSEG0 (also in the boot FLASH).
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Does not return
 *
 ****************************************************************************/

	.globl	__reset
	.section .reset, "ax", @progbits
	.set	noreorder
	.ent	__reset
__reset:
	la		k0, __start	/* Just jump to the startup initialization code */
	jr		k0
	nop
	.end	__reset

/****************************************************************************
 * Name: __start
 *
 * Description:
 *   This is the KSEG0 startup code.  It receives control from the reset
 *   entry point.  This lgic This prepares the processor to execute
 *   C code, performs some very low-level initialization, then starts NuttX
 *   (via __start_nuttx
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Does not return
 *
 ****************************************************************************/

	.section .start, "ax", @progbits
	.set	noreorder
	.ent	__start
__start:

	/* If this function was entered because of an NMI, then turn processing
	 * over to the NMI handler.
	 */

	mfc0	k0, $12				/* Load CP0 status register */
	ext		k0, k0, 19, 1		/* Extract NMI bit */
	beqz	k0, .Lnotnmi
	nop
	la		k0, nmi_handler
	jr		k0
	nop

	/* This is not an NMI */

.Lnotnmi:

	/* Initialize the stack pointer */

	la		sp, PIC32MX_STACK_TOP

	/* Initialize the globl pointer (gp).  _gp is initialized by the linker
	 * script to point to the "middle" of the small variables region.
	 */

	la		gp, _gp

	/* Initialize Global Pointer in Shadow Set.  The SRSCtl PSS field must
	 * be set to the shadow set in which to initialize the global pointer.
	 * Since we only have a single shadow set, we will initialize
	 * SRSCtl PSS to SRSCtl HSS.  We then write the global pointer to the
	 * previous shadow set to ensure that on interrupt, the global pointer
	 * has been initialized.
	 */

	mfc0	t1, PIC32MX_CP0_SRSCTL	/* Read SRSCtl register */
	add		t3, t1, zero			/* Save off current SRSCtl */
	ext		t2, t1, 26, 4			/* to obtain HSS field */
	ins		t1, t2, 6, 4			/* Put HSS field */
	mtc0	t1, PIC32MX_CP0_SRSCTL	/* into SRSCtl PSS  */
	wrpgpr	gp, gp					/* Set global pointer in PSS */
	mtc0	t3, PIC32MX_CP0_SRSCTL	/* Restore SRSCtl */

	/* Clear uninitialized data sections */

	la		t0, _sbss
	la		t1, _ebss
	b		.Lbsscheck
	nop

.Lbssloop:	
	sw		zero, 0x0(t0)
	sw		zero, 0x4(t0)
	sw		zero, 0x8(t0)
	sw		zero, 0xc(t0)
	addu	t0, 16

.Lbsscheck:
	bltu	t0, t1, .Lbssloop
	nop

	/* Copy initialized data from program flash to data memory */

	la		t0, _data_loaddr
	la		t1, _sdata
	la		t2, _edata
	b		.Ldatacheck
	nop

.Ldataloop:	
	lw		t3, (t0)
	sw		t3, (t1)
	addu	t0, 4
	addu	t1, 4

.Ldatacheck:
	bltu	t1, t2, .Ldataloop
	nop

	/* If there are no RAM functions, skip the next two sections --
	 * copying RAM functions from program flash to data memory and
	 * initializing bus matrix registers.
	 */

#ifdef CONFIG_PIC32MX_RAMFUNCS
	la		t1, _ramfunc_sizeof
	beqz	t1, .Lnoramfuncs
	nop

	/* Copy RAM functions from program flash to data memory */

	la		t0, _ramfunc_loadaddr
	la		t1, _sramfunc
	la		t2, _eramfunc

.Lramfuncloop:  
	lw		t3,(t0)
	sw		t3,(t1)
	addu	t0,4
	addu	t1,4

	bltu	t1, t2, .Lramfuncloop
	nop

	/* Initialize bus matrix registers if RAM functions exist in the
	 * application
	 */

	la		t1, _bmxdkpba_address
	la		t2, BMXDKPBA
	sw		t1, 0(t2)
	la		t1, _bmxdudba_address
	la		t2, BMXDUDBA
	sw		t1, 0(t2)
	la		t1, _bmxdupba_address
	la		t2, BMXDUPBA
	sw		t1, 0(t2)
.Lnoramfuncs:
#endif

	/* Initialize CP0 Count register */

	mtc0	zero, PIC32MX_CP0_COUNT

	/* Initialize Compare register */

	li		t2, -1
	mtc0	t2, PIC32MX_CP0_COMPARE

	/* Initialize EBase register */

	la		t1, _ebase_address
	mtc0	t1, PIC32MX_CP0_EBASE

	/* Initialize IntCtl register */

	la		t1, _vector_spacing
	li		t2, 0				/* Clear t2 */
	ins		t2, t1, 5, 5		/* Shift value to VS field */
	mtc0	t2, PIC32MX_CP0_INTCTL

	/* Initialize CAUSE registers
	 * - Enable counting of Count register (DC = 0)
	 * - Use special exception vector (IV = 1)
	 * - Clear pending software interrupts (IP1:IP0 = 0)
	 */

	li		t1, CP0_CAUSE_IV
	mtc0	t1, PIC32MX_CP0_CAUSE

	/* Initialize STATUS register
	 * - Access to Coprocessor 0 not allowed in user mode (CU0 = 0)
	 * - User mode uses configured endianness (RE = 0)
	 * - Preserve Bootstrap Exception vectors (BEV)
	 * - Preserve soft reset (SR) and non-maskable interrupt (NMI)
	 * - CorExtend enabled based on whether CorExtend User Defined
	 *   Instructions have been implemented (CEE = Config(UDI))
	 * - Disable any pending interrups (IM7..IM2 = 0, IM1..IM0 = 0)
	 * - Disable hardware interrupts (IPL7:IPL2 = 0)
	 * - Base mode is Kernel mode (UM = 0)
	 * - Error level is normal (ERL = 0)
	 * - Exception level is normal (EXL = 0)
	 * - Interrupts are disabled (IE = 0)
	 */

	mfc0	t0, PIC32MX_CP0_CONFIG
	ext		t1, t0, 22,1		/* Extract UDI from Config register */
	sll		t1, t1, 17			/* Move UDI to Status.CEE location */
	mfc0	t0, PIC32MX_CP0_STATUS
	and		t0, t0, 0x00580000	/* Preserve SR, NMI, and BEV */
	or		t0, t1, t0			/* Include Status.CEE (from UDI) */
	mtc0	t0, PIC32MX_CP0_STATUS

	/* Initialize Status BEV for normal exception vectors */

	mfc0	t0, PIC32MX_CP0_STATUS
	and		t0, t0, 0xffbfffff	# Clear BEV
	mtc0	t0, PIC32MX_CP0_STATUS

	/* Start NuttX. We do this via a thunk in the text section so that
	 * a normal jump and link can be used, enabling the startup code
	 * to work properly whether main is written in MIPS16 or MIPS32
	 * code. I.e., the linker will correctly adjust the JAL to JALX if
	 * necessary
	 */

	and		a0, a0, 0
	and		a1, a1, 0
	la		t0, __start_nuttx
	jr		t0
	nop
	.end __start

/****************************************************************************
 * Name: _bev_exception
 *
 * Description:
 *   Boot Exception Vector Handler.  Jumps to _bootstrap_exception_handler
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Does not return
 *
 ****************************************************************************/

	.section .bev_handler,"ax",@progbits
	.set	noreorder
	.ent	_bev_exception
_bev_exception:
	la		k0, _bootstrap_exception_handler
	jr		k0
	nop
	.end _bev_exception
			
/****************************************************************************
 * Name: _bev_exception
 *
 * Description:
 *   General Exception Vector Handler.  Jumps to _general_exception_handler
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Does not return
 *
 ****************************************************************************/

	.section .gen_handler,"ax",@progbits
	.set	noreorder
	.ent	_gen_exception
_gen_exception:
	la		k0, _general_exception_context
	jr		k0
	nop
	.end _gen_exception

/****************************************************************************
 * Name: __start_nuttx
 *
 * Description:
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Does not return
 *
 ****************************************************************************/

	.text
	.ent __start_nuttx
__start_nuttx:
	/* Perform low level initialization */

	jal	up_lowinit
	nop

	/* Perform early serial initialization */

#ifdef CONFIG_USE_EARLYSERIALINIT
	jal	up_earlyserialinit
	nop
#endif

	/* Call os_start */

	jal os_start
	nop

	/* Just in case main returns, go into infinite loop */

halt:
1:
	b	 1b
	nop
	.end __start_nuttx

	/* This global variable is unsigned long g_heapbase and is exported
	 * here only because of its coupling to idle thread stack.
	 */

	.data
	.align	4
	.globl	g_heapbase
	.type	g_heapbase, object
g_heapbase:
	.long	_ebss+CONFIG_IDLETHREAD_STACKSIZE
	.size	g_heapbase, .-g_heapbase

