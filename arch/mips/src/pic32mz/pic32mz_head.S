/****************************************************************************
 * arch/mips/src/pic32mz/pic32mz_head.S
 *
 *   Copyright (C) 2015, 2018 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <arch/mips32/registers.h>
#include <arch/pic32mz/cp0.h>

#include "pic32mz_config.h"
#include "hardware/pic32mz_features.h"
#include "pic32mz_excptmacros.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifdef CONFIG_PIC32MZ_MVEC
#  error "Multi-vectors not supported"
#  ifndef CONFIG_PIC32MZ_EBASE
#    error "EBASE address provided"  /* Should come from the linker script */
#  endif
#  ifndef CONFIG_PIC32MZ_VECTORSPACING
#    error "No vector spacing provided"
#  endif
#endif

/* Linker memory organization ***********************************************/
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
 *
 * The following are placed outside of the "normal" memory segments -- mostly
 * so that they do not have to be cleared on power up.
 *
 * 5) Idle thread stack:
 *    Start:   _ebss
 *    End(+1): _ebss + CONFIG_IDLETHREAD_STACKSIZE
 * 6) Optional interrupt stack
 *    Start:   _ebss + CONFIG_IDLETHREAD_STACKSIZE
 *    End(+1): _ebss + CONFIG_IDLETHREAD_STACKSIZE + 
               (CONFIG_ARCH_INTERRUPTSTACK & ~3)
 * 6a) Heap (without interrupt stack)
 *    Start:   _ebss + CONFIG_IDLETHREAD_STACKSIZE
 *    End(+1): to the end of memory
 * 6b) Heap (with interrupt stack)
 *    Start: _ebss + CONFIG_IDLETHREAD_STACKSIZE + 
             (CONFIG_ARCH_INTERRUPTSTACK & ~3)
 *    End(+1): to the end of memory
 */

#define PIC32MZ_STACK_BASE      _ebss
#define PIC32MZ_STACK_TOP       _ebss + CONFIG_IDLETHREAD_STACKSIZE - 4

#if CONFIG_ARCH_INTERRUPTSTACK > 3
#  define PIC32MZ_INTSTACK_BASE PIC32MZ_STACK_TOP
#  define PIC32MZ_INTSTACK_SIZE (CONFIG_ARCH_INTERRUPTSTACK & ~3)
#  define PIC32MZ_INTSTACK_TOP  PIC32MZ_STACK_TOP + PIC32MZ_INTSTACK_SIZE
#  define PIC32MZ_HEAP_BASE     PIC32MZ_INTSTACK_TOP
#else
#  define PIC32MZ_HEAP_BASE     PIC32MZ_STACK_TOP
#endif

#if defined (CONFIG_MIPS32_ICACHE) || defined (CONFIG_MIPS32_DCACHE)
#  define K0_CACHE_ALGORITHM CP0_CONFIG_K0_CACHEABLE
#else
#  define K0_CACHE_ALGORITHM CP0_CONFIG_K0_UNCACHED
#endif

#ifdef CONFIG_MIPS32_ICACHE
#  ifdef CONFIG_MIPS32_ICACHE_SIZE
#    define PIC32MZ_ICACHE_SIZE CONFIG_MIPS32_ICACHE_SIZE
#  else
#    define PIC32MZ_ICACHE_SIZE 16384
#  endif

#  ifdef CONFIG_MIPS32_ILINE_SIZE
#    define PIC32MZ_ILINE_SIZE CONFIG_MIPS32_ILINE_SIZE
#  else
#    define PIC32MZ_ILINE_SIZE 16
#  endif

#  define PIC32MZ_KSEG0_IBASE CONFIG_MIPS32_KSEG0_IBASE
#  define PIC32MZ_INDEXSTORETAG_I 8
#endif

#ifdef CONFIG_MIPS32_DCACHE
#  ifdef CONFIG_MIPS32_DCACHE_SIZE
#    define PIC32MZ_DCACHE_SIZE CONFIG_MIPS32_DCACHE_SIZE
#  else
#    define PIC32MZ_DCACHE_SIZE 4096
#  endif

#  ifdef CONFIG_MIPS32_DLINE_SIZE
#    define PIC32MZ_DLINE_SIZE CONFIG_MIPS32_DLINE_SIZE
#  else
#    define PIC32MZ_DLINE_SIZE 16
#  endif

#  define PIC32MZ_KSEG0_DBASE CONFIG_MIPS32_KSEG0_DBASE
#  define PIC32MZ_INDEXSTORETAG_D 9
#endif

/****************************************************************************
 * Assembly Language Macros
 ****************************************************************************/

/****************************************************************************
 * Name: showprogress
 *
 * Description:
 *   Print a character on the UART to show boot status. This macro will
 *   modify t0 and a0.
 *
 ****************************************************************************/

	.macro	showprogress, code
#ifdef CONFIG_DEBUG_FEATURES
  la   t0, up_lowputc
  la   a0, \code
	jalr ra, t0
	nop
#endif
	.endm

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

	.file	"pic32mz_head.S"

	/* Exported symbols */

	.global	__reset
	.global __start
	.global halt
	.global	devcfg
#if CONFIG_ARCH_INTERRUPTSTACK > 3
	.global g_intstackbase
#ifdef CONFIG_PIC32MZ_NESTED_INTERRUPTS
	.global	g_nestlevel
#endif
#endif
	.global	g_idle_topstack

	/* Imported symbols */

	.global nx_start
	.global	pic32mz_exception
	.global pic32mz_decodeirq
  .global pic32mz_lowinit
#ifdef CONFIG_PIC32MZ_NMIHANDLER
	.global	pic32mz_donmi
#endif

#ifdef CONFIG_DEBUG_FEATURES
  .global pic32mz_consoleinit
  .global up_lowputc
#endif

	/* This file contains 32-bit assembly code */

	.set nomips16

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

	.section .reset, "ax", @progbits
	.align	2
	.set	noreorder
	.set	nomips16
	.ent	__reset

__reset:

#ifdef CONFIG_MIPS_MICROMIPS
	.set	micromips

	.word	0x10000004		/* 0x0000 */
							/*   MIPS32:    Branch forward 0x14 bytes   */
							/*   MicroMIPS: ADDI32 $0, $0, 0x0007 (NOP) */
	.word	0x00000000		/* 0x0004 */
							/*   MIPS32:    NOP   */
							/*   MicroMIPS: NOP */

	/* If we get here then we are in microMIPS mode.  That is because the
	 * preceding instructions are all NOP in that case and we fall through
	 * to here.  Otherwise, we branched to __reset_switch_isa
	 */

__reset_micromips:

	/* Just jump to the startup initialization code */

	la		k0, __start		/* 0x0008 */
	jr		k0				/* 0x0010 */
	nop						/* 0x0012 */

	/* Device not in proper ISA mode.  If we are not in microMIPS mode then
	 * we get here from the first instruction at __reset which will get
	 * interpreted as a branch to this location.
	 */

	.align	2
	.set	nomicromips

__reset_halt:
	b		__reset_halt	/* 0x0014 <- Branch target */
	nop
#else
	/* Just jump to the startup initialization code */

	.set nomicromips
	la		k0, __start
	jr		k0
	nop
#endif
	.end	__reset

/****************************************************************************
 * Name: _gen_exception
 *
 * Description:
 *   General Exception Vector Handler.  Jumps to _exception_handler. This
 *   vector will be positioned at 0xbfc00180 by the linker script. NOTE:
 *   If we set the BEV bit in the status register so all interrupt vectors
 *   should go through the _bev_exception.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Does not return
 *
 ****************************************************************************/

	.section .gen_excpt,"ax",@progbits
	.set	noreorder
#ifdef CONFIG_MIPS_MICROMIPS
	.set	micromips
#endif
	.ent	_gen_exception

_gen_exception:
	la		k0, _exception_handler
	jr		k0
	nop
	.end _gen_exception

/****************************************************************************
 * Name: _ebase_exception
 *
 * Description:
 *   Interrupt Exception Vector Handler.  Jumps to _int_handler.  This
 *   vector will be positioned at 0xbfc00200 by the linker script. NOTE:
 *   Several vectors (JTAG, TLB fills, etc.) could come through this vector.
 *   However, this is intended to serve vectors in PIC32MZ single vector
 *   mode:  The EBASE register will be set to 0xbfc00000 and the vector
 *   should go to EBASE + 0x0200.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Does not return
 *
 ****************************************************************************/

	.section .ebase_excpt,"ax",@progbits
	.set	noreorder
#ifdef CONFIG_MIPS_MICROMIPS
	.set	micromips
#endif
	.ent	_ebase_exception

_ebase_exception:
	la		k0, _int_handler
	jr		k0
	nop
	.end _ebase_exception

/****************************************************************************
 * Name: _bev_exception
 *
 * Description:
 *   Boot Exception Vector Handler.  Jumps to _exception_handler.  This
 *   vector will be positioned at 0xbfc00380 by the linker script.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Does not return
 *
 ****************************************************************************/

	.section .bev_excpt,"ax",@progbits
	.set	noreorder
#ifdef CONFIG_MIPS_MICROMIPS
	.set	micromips
#endif
	.ent	_bev_exception

_bev_exception:
	la		k0, _exception_handler
	jr		k0
	nop
	.end _bev_exception

/****************************************************************************
 * Name: _int_exception
 *
 * Description:
 *   Interrupt Exception Vector Handler.  Jumps to _int_handler.  This
 *   vector will be positioned at 0xbfc00400 by the linker script.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Does not return
 *
 ****************************************************************************/

	.section .int_excpt,"ax",@progbits
	.set	noreorder
#ifdef CONFIG_MIPS_MICROMIPS
	.set	micromips
#endif
	.ent	_int_exception

_int_exception:
	la		k0, _int_handler
	jr		k0
	nop
	.end _int_exception

/****************************************************************************
 * Name: __start
 *
 * Description:
 *   This is the KSEG0 startup code.  It receives control from the reset
 *   entry point.  This logic prepares the processor to execute
 *   C code, performs some very low-level initialization, then starts NuttX
 *   (via __start_nuttx)
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
#ifdef CONFIG_MIPS_MICROMIPS
	.set	micromips
#endif
	.ent	__start

__start:

	/* If this function was entered because of an NMI, then turn processing
	 * over to the NMI handler.
	 */

#ifdef CONFIG_PIC32MZ_NMIHANDLER
	mfc0	k0, $12				/* Load CP0 status register */
	ext		k0, k0, 19, 1		/* Extract NMI bit */
	beqz	k0, .Lnotnmi
	nop
	la		k0, _nmi_handler
	jr		k0
	nop

	/* This is not an NMI */

.Lnotnmi:
#endif

	/* Initialize the stack pointer */

	la		sp, PIC32MZ_STACK_TOP

	/* Initialize the global pointer (gp).  _gp is initialized by the linker
	 * script to point to the "middle" of the small variables region.
	 */

	la		gp, _gp

	/* Initialize Global Pointer in Shadow Sets. The SRSCtl PSS field must
	 * be set to the shadow set in which to initialize the global pointer.
	 * We initialize PSS to the number of register sets and work down to
	 * set zero. We then write the global pointer to the
	 * previous shadow set to ensure that on interrupt, the global pointer
	 * has been initialized.
	 */

	mfc0	t1, PIC32MZ_CP0_SRSCTL	/* Read SRSCtl register */
	add		t3, t1, zero			/* Save off current SRSCtl */
	ext 	t2, t1, 26, 4			/* Extract the HSS field */

.Lsrsloop:
	ins		t1, t2, 6, 4			/* Put HSS field */
	mtc0	t1, PIC32MZ_CP0_SRSCTL	/* into SRSCtl PSS  */
	ehb 							/* Clear hazard before using new SRSCTL */
	wrpgpr	gp, gp					/* Set global pointer in PSS */
	addiu	t2, t2, -1				/* Move to the next shadow set */
	bne 	t2, $0, .Lsrsloop

	mtc0	t3, PIC32MZ_CP0_SRSCTL	/* Restore SRSCtl */
	ehb

  la   t0, pic32mz_consoleinit
  jalr ra, t0 
  nop

  showprogress 'A'

#if defined (CONFIG_MIPS32_ICACHE) || defined (CONFIG_MIPS32_DCACHE)
  /* Initialize K0 Cache. The cache resets in an indeterminate state.
   * We need to clear the tags and invalidate any data.
   * It's done as follows:
   *  1 - Clear the ErrCtl register to use the TagLo(1) register.
   *  2 - Clear the TagLo register.
   *  3 - Perform an IndexStoreTag for each line to copy the content of TagLo.
   */

  /* Clear ErrCtl and TagLo */

  mtc0 zero, PIC32MZ_CP0_ERRCTL
  mtc0 zero, PIC32MZ_CP0_TAGLO
  ehb

#  ifdef CONFIG_MIPS32_ICACHE
  /* Init I-Cache (Copy content of TagLo) */

  li t0, PIC32MZ_KSEG0_IBASE
  addu t1, t0, PIC32MZ_ICACHE_SIZE
.icacheloop:
  addu t0, t0, PIC32MZ_ILINE_SIZE
  bne t0, t1, .icacheloop
  cache PIC32MZ_INDEXSTORETAG_I, -4(t0)
#  endif

#  ifdef CONFIG_MIPS32_DCACHE
  /* Init D-Cache (Copy content of TagLo) */

  li t0, PIC32MZ_KSEG0_DBASE
  addu t1, t0, PIC32MZ_DCACHE_SIZE
.dcacheloop:
  addu t0, t0, PIC32MZ_DLINE_SIZE
  bne t0, t1, .dcacheloop
  cache PIC32MZ_INDEXSTORETAG_D, -4(t0)
#  endif

  /* Force memory synchronization */

  sync

  showprogress 'B'

#endif /* CONFIG_MIPS32_ICACHE || CONFIG_MIPS32_DCACHE */

  /* Set the cache algorithm.
   * If the cache was enable, then it has already been initialized and
   * the cache algorithm will be set to write-back with write allocation.
   * if not, just set the algorithm to uncached.
   */

  mfc0 t0, PIC32MZ_CP0_CONFIG
  ori  t0, CP0_CONFIG_K0_MASK
  xori t0, CP0_CONFIG_K0_MASK
  ori  t0, K0_CACHE_ALGORITHM
  mtc0 t0, PIC32MZ_CP0_CONFIG

  showprogress 'C'

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

  showprogress 'D'

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

  showprogress 'E'

	/* If there are no RAM functions, skip the next two sections --
	 * copying RAM functions from program flash to data memory and
	 * initializing bus matrix registers.
	 */

#ifdef CONFIG_ARCH_RAMFUNCS
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
	la		t2, PIC32MZ_BMX_DKPBA
	sw		t1, 0(t2)
	la		t1, _bmxdudba_address
	la		t2, PIC32MZ_BMX_DUDBA
	sw		t1, 0(t2)
	la		t1, _bmxdupba_address
	la		t2, PIC32MZ_BMX_DUPBA
	sw		t1, 0(t2)

  showprogress 'F'

.Lnoramfuncs:
#endif

	/* Initialize CP0 Count register */

	mtc0	zero, PIC32MZ_CP0_COUNT

	/* Initialize Compare register */

	li		t2, -1
	mtc0	t2, PIC32MZ_CP0_COMPARE

	/* Initialize EBase register */

#ifdef CONFIG_PIC32MZ_MVEC
	la		t1, CONFIG_PIC32MZ_EBASE
	mtc0	t1, PIC32MZ_CP0_EBASE

	/* Initialize IntCtl register */

	li		t1, CONFIG_PIC32MZ_VECTORSPACING
	li		t2, 0
	ins		t2, t1, CP0_INTCTL_VS_SHIFT, 5
	mtc0	t2, PIC32MZ_CP0_INTCTL
#endif

	/* Initialize CAUSE registers
	 * - Enable counting of Count register (DC = 0)
	 * - Use special exception vector (IV = 1)
	 * - Clear pending software interrupts (IP1:IP0 = 0)
	 */

	li		t1, CP0_CAUSE_IV
	mtc0	t1, PIC32MZ_CP0_CAUSE

	/* Initialize STATUS register
	 * - Access to Coprocessor 0 not allowed in user mode (CU0 = 0)
	 * - User mode uses configured endianness (RE = 0)
	 * - Preserve Bootstrap Exception vectors (BEV)
	 * - Preserve soft reset (SR) and non-maskable interrupt (NMI)
	 * - CorExtend enabled based on whether CorExtend User Defined
	 *   Instructions have been implemented (CEE = Config(UDI))
	 * - Disable any pending interrupts (IM7..IM2 = 0, IM1..IM0 = 0)
	 * - Disable hardware interrupts (IPL7:IPL2 = 0)
	 * - Base mode is Kernel mode (UM = 0)
	 * - Error level is normal (ERL = 0)
	 * - Exception level is normal (EXL = 0)
	 * - Interrupts are disabled (IE = 0)
	 */

	mfc0	t0, PIC32MZ_CP0_CONFIG
	ext		t1, t0, 22,1			/* Extract UDI from Config register */
	sll		t1, t1, 17				/* Move UDI to Status.CEE location */
	mfc0	t0, PIC32MZ_CP0_STATUS
	and		t0, t0, 0x00580000		/* Preserve SR, NMI, and BEV */
	or		t0, t1, t0				/* Include Status.CEE (from UDI) */
	mtc0	t0, PIC32MZ_CP0_STATUS

	/* Initialize Status BEV for normal exception vectors */

	mfc0	t0, PIC32MZ_CP0_STATUS
	and		t0, t0, ~CP0_STATUS_BEV	/* Clear BEV */
	mtc0	t0, PIC32MZ_CP0_STATUS

#ifdef CONFIG_MIPS_MICROMIPS
	/* Exception handlers are also being compiled for microMIPS, so enable
	 * config3ISAOnExc now that exception vectors have been [re]located.
	 */

	mfc0	t0, MIPS32_CP0_CONFIG3		/* Load CONFIG3 register */
	or		t0, t0, CP0_CONFIG3_ISAONEXC	/* Specify microMIPS mode on exceptions */
	mtc0	t0, MIPS32_CP0_CONFIG3		/* Update CONFIG3 register */
#endif
  showprogress 'G'

  showprogress '\n'

	/* Start NuttX. We do this via a thunk in the text section so that
	 * a normal jump and link can be used, enabling the startup code
	 * to work properly whether main is written in MIPS16 or MIPS32
	 * code. I.e., the linker will correctly adjust the JAL to JALX if
	 * necessary
	 */

	la		t0, __start_nuttx
	jr		t0
	nop
	.end __start

/****************************************************************************
 * Name: _exception_handler
 *
 * Description:
 *   BEV/General exception handler.  Calls pic32mz_exception()
 *
 ****************************************************************************/

	.section .bev_handler, "ax", @progbits
	.set	noreorder
#ifdef CONFIG_MIPS_MICROMIPS
	.set	micromips
#endif
	.ent	_exception_handler

_exception_handler:
	EXCPT_PROLOGUE t0				/* Save registers on stack, enable nested interrupts */
	move	a0, sp					/* Pass register save structure as the parameter 1 */
	USE_INTSTACK t0, t1, t2, t3		/* Switch to the interrupt stack */
	la		t0, pic32mz_exception	/* Call pic32mz_exception(regs) */
	jalr	ra, t0
	nop
#ifdef CONFIG_PIC32MZ_NESTED_INTERRUPTS
	di								/* Prohibit nested interrupts from here */
#endif
	RESTORE_STACK t0, t1			/* Undo the operations of USE_STACK */
	EXCPT_EPILOGUE v0				/* Return to the context returned by pic32mz_exception() */
	.end	_exception_handler

/****************************************************************************
 * Name: _int_handler
 *
 * Description:
 *   Interrupt exception handler.  Calls up_decodeirq()
 *
 ****************************************************************************/

	.section .int_handler, "ax", @progbits
	.set	noreorder
#ifdef CONFIG_MIPS_MICROMIPS
	.set	micromips
#endif
	.ent	_int_handler

_int_handler:
	EXCPT_PROLOGUE t0				/* Save registers on stack, enable nested interrupts */
	move	a0, sp					/* Pass register save structure as the parameter 1 */
	USE_INTSTACK t0, t1, t2, t3		/* Switch to the interrupt stack */
	la		t0, pic32mz_decodeirq	/* Call pic32mz_decodeirq(regs) */
	jalr	ra, t0
	nop
#ifdef CONFIG_PIC32MZ_NESTED_INTERRUPTS
	di								/* Prohibit nested interrupts from here */
#endif
	RESTORE_STACK t0, t1			/* Undo the operations of USE_STACK */
	EXCPT_EPILOGUE v0				/* Return to the context returned by pic32mz_decodeirq() */
	.end	_int_handler

/****************************************************************************
 * Name: _nmi_handler
 *
 * Description:
 *   NMI exception handler.  Calls pic32mz_donmi
 *
 ****************************************************************************/

#ifdef CONFIG_PIC32MZ_NMIHANDLER
	.section .nmi_handler, "ax", @progbits
	.set	noreorder
#ifdef CONFIG_MIPS_MICROMIPS
	.set	micromips
#endif
	.ent	_nmi_handler

_nmi_handler:
	EXCPT_PROLOGUE t0				/* Save registers on stack, enable nested interrupts */
	move	a0, sp					/* Pass register save structure as the parameter 1 */
	USE_INTSTACK t0, t1, t2, t3		/* Switch to the interrupt stack */
	la		t0, pic32mz_donmi		/* Call up_donmi(regs) */
	jalr	ra, t0
	nop
#ifdef CONFIG_PIC32MZ_NESTED_INTERRUPTS
	di								/* Prohibit nested interrupts from here */
#endif
	RESTORE_STACK t0, t1			/* Undo the operations of USE_STACK */
	EXCPT_EPILOGUE v0				/* Return to the context returned by pic32mz_donmi() */
	.end	_nmi_handler
#endif

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
#ifdef CONFIG_MIPS_MICROMIPS
	.set	micromips
#endif
	.ent	__start_nuttx

__start_nuttx:
	/* Perform low level initialization */

	la		t0, pic32mz_lowinit
	jalr  ra, t0
	nop

	/* Call nx_start */

	la		t0, nx_start
	jalr	ra, t0
	nop

	/* Just in case main returns, go into an infinite loop */

halt:
1:
	b	 1b
	nop
	.end __start_nuttx

/****************************************************************************
 * Device Configuration
 ****************************************************************************/

	.section .devcfg, "a"
	.type	devcfg, object
devcfg:
devcfg3:
	.long	CONFIG_PIC32MZ_USERID   << DEVCFG3_USERID_SHIFT    | \
			CONFIG_PIC32MZ_FMIIEN   << DEVCFG3_FMIIEN_SHIFT    | \
			CONFIG_PIC32MZ_FETHIO   << DEVCFG3_FETHIO_SHIFT    | \
			CONFIG_PIC32MZ_PGL1WAY  << DEVCFG3_PGL1WAY_SHIFT   | \
			CONFIG_PIC32MZ_PMDL1WAY << DEVCFG3_PMDL1WAY_SHIFT  | \
			CONFIG_PIC32MZ_IOL1WAY  << DEVCFG3_IOL1WAY_SHIFT   | \
			CONFIG_PIC32MZ_FUSBIDIO << DEVCFG3_FUSBIDIO_SHIFT  | \
			DEVCFG3_RWO

devcfg2:
	.long	CONFIG_PIC32MZ_PLLIDIV   | CONFIG_PIC32MZ_FPLLRNG   | \
			CONFIG_PIC32MZ_FPLLICLK  | CONFIG_PIC32MZ_PLLMULT   | \
			CONFIG_PIC32MZ_PLLODIV   | CONFIG_PIC32MZ_UPLLFSEL  | \
			DEVCFG2_RWO

devcfg1:
	.long	CONFIG_PIC32MZ_FNOSC     | CONFIG_PIC32MZ_DMTINV    |\
			CONFIG_PIC32MZ_FSOSCEN   | CONFIG_PIC32MZ_IESO      | \
			CONFIG_PIC32MZ_POSCMOD   | CONFIG_PIC32MZ_OSCIOFNC  | \
			CONFIG_PIC32MZ_FCKSM     | CONFIG_PIC32MZ_WDTPS     | \
			CONFIG_PIC32MZ_WDTSPGM   | CONFIG_PIC32MZ_WINDIS    | \
			CONFIG_PIC32MZ_FWDTEN    | CONFIG_PIC32MZ_FWDTWINSZ | \
			CONFIG_PIC32MZ_DMTCNT    | CONFIG_PIC32MZ_FSOSCEN   | \
			CONFIG_PIC32MZ_FSOSCEN   | CONFIG_PIC32MZ_FDMTEN    | \
			DEVCFG1_RWO

devcfg0:
	.long	CONFIG_PIC32MZ_DEBUGGER  | CONFIG_PIC32MZ_JTAGEN    | \
			CONFIG_PIC32MZ_ICESEL    | CONFIG_PIC32MZ_TRCEN     | \
			CONFIG_PIC32MZ_BOOTISA   | CONFIG_PIC32MZ_FECCCON   | \
			CONFIG_PIC32MZ_FSLEEP    | CONFIG_PIC32MZ_DBGPER    | \
			CONFIG_PIC32MX_SMCLR     | CONFIG_PIC32MX_SOSCGAIN  | \
			CONFIG_PIC32MX_SOSCBOOST | CONFIG_PIC32MX_POSCGAIN  | \
			CONFIG_PIC32MX_POSCBOOST | CONFIG_PIC32MZ_EJTAGBEN  | \
			DEVCFG0_RW0
	.size	devcfg, .-devcfg

/* Every word in the configuration space and sequence space has an
 * associated alternate word. During device start-up, primary words are
 * read and if uncorrectable ECC errors are found, the BCFGERR flag is set
 * and alternate words are used.
 */

	.section .adevcfg, "a"
	.type	adevcfg, object
adevcfg:
adevcfg3:
	.long	ADEVCFG3_USERID         << DEVCFG3_USERID_SHIFT    | \
			CONFIG_PIC32MZ_FMIIEN   << DEVCFG3_FMIIEN_SHIFT    | \
			CONFIG_PIC32MZ_FETHIO   << DEVCFG3_FETHIO_SHIFT    | \
			CONFIG_PIC32MZ_PGL1WAY  << DEVCFG3_PGL1WAY_SHIFT   | \
			CONFIG_PIC32MZ_PMDL1WAY << DEVCFG3_PMDL1WAY_SHIFT  | \
			CONFIG_PIC32MZ_IOL1WAY  << DEVCFG3_IOL1WAY_SHIFT   | \
			CONFIG_PIC32MZ_FUSBIDIO << DEVCFG3_FUSBIDIO_SHIFT  | \
			DEVCFG3_RWO

adevcfg2:
	.long	CONFIG_PIC32MZ_PLLIDIV  | CONFIG_PIC32MZ_FPLLRNG   | \
			CONFIG_PIC32MZ_FPLLICLK | CONFIG_PIC32MZ_PLLMULT   | \
			CONFIG_PIC32MZ_PLLODIV  | CONFIG_PIC32MZ_UPLLFSEL  | \
			DEVCFG2_RWO

adevcfg1:
	.long	CONFIG_PIC32MZ_FNOSC    | CONFIG_PIC32MZ_DMTINV    |\
			CONFIG_PIC32MZ_FSOSCEN  | CONFIG_PIC32MZ_IESO      | \
			CONFIG_PIC32MZ_POSCMOD  | CONFIG_PIC32MZ_OSCIOFNC  | \
			CONFIG_PIC32MZ_FCKSM    | CONFIG_PIC32MZ_WDTPS     | \
			CONFIG_PIC32MZ_WDTSPGM  | CONFIG_PIC32MZ_WINDIS    | \
			ADEVCFG1_FWDTEN         | CONFIG_PIC32MZ_FWDTWINSZ | \
			CONFIG_PIC32MZ_DMTCNT   | CONFIG_PIC32MZ_FSOSCEN   | \
			CONFIG_PIC32MZ_FSOSCEN  | CONFIG_PIC32MZ_FDMTEN    | \
			DEVCFG1_RWO

adevcfg0:
	.long	CONFIG_PIC32MZ_DEBUGGER | CONFIG_PIC32MZ_JTAGEN    | \
			CONFIG_PIC32MZ_ICESEL   | CONFIG_PIC32MZ_TRCEN     | \
			CONFIG_PIC32MZ_BOOTISA  | CONFIG_PIC32MZ_FECCCON   | \
			CONFIG_PIC32MZ_FSLEEP   | CONFIG_PIC32MZ_DBGPER    | \
			CONFIG_PIC32MZ_EJTAGBEN | DEVCFG0_RW0

	.size	adevcfg, .-adevcfg

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Interrupt stack variables */

#if CONFIG_ARCH_INTERRUPTSTACK > 3

/* g_instackbase is a pointer to the final, aligned word of the interrupt
 * stack.
 */

	.sdata
	.type	g_intstackbase, object
g_intstackbase:
	.long	PIC32MZ_INTSTACK_TOP-4
	.size	g_intstackbase, .-g_intstackbase

/* g_nextlevel is the exception nesting level... the interrupt stack is not
 * available to nested exceptions.
 */

#ifdef CONFIG_PIC32MZ_NESTED_INTERRUPTS
	.sbss
	.type	g_nestlevel, object
g_nestlevel:
	.skip		4
#endif
#endif

/* This global variable is unsigned int g_idle_topstack and is exported here only
 * because of its coupling to idle thread stack.
 */

	.sdata
	.type	g_idle_topstack, object
g_idle_topstack:
	.long	PIC32MZ_HEAP_BASE
	.size	g_idle_topstack, .-g_idle_topstack
