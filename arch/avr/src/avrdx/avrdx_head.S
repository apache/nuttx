/****************************************************************************
 * arch/avr/src/avrdx/avrdx_head.S
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <avr/io.h>
#include <avr/common.h>
#include <avr/sfr_defs.h>

#include <arch/irq.h>

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

/* Stack is allocated just after uninitialized data and just before the heap */

#define STACKBASE (_enoinit+CONFIG_IDLETHREAD_STACKSIZE)

/* The RAMPZ register should be available for all MCUs in this family
 *
 * - Support for the EPLMX instructions is assumed if RAMPZ is present
 * - If RAMPZ is not present, support for LPMX is assumed
 */

#define HAVE_RAMPZ 1

/****************************************************************************
 * External Symbols
 ****************************************************************************/

	.file	"up_nommuhead.S"
	.global	__start				/* Entry point */
	.global	_sbss				/* Start of .bss.  Defined by ld.script */
	.global	_ebss				/* End of .bss.  Defined by ld.script */
	.global	_sdata				/* Start of .data section in RAM */
	.global	_edata				/* End of .data section in RAM */
	.global	_eronly				/* Start of .data section in FLASH */
	.global	_enoinit			/* End of uninitilized data.  Defined by ld.script */
	.global	avr_lowinit			/* Perform low level initialization */
	.global	nx_start			/* NuttX entry point */

	.global	vectortab
#if defined(CONFIG_ARCH_CHIP_AVR128DA28)
	.global avrdx_nmi
	.global avrdx_bod_vlm
	.global avrdx_rtc_cnt
	.global avrdx_rtc_pit
	.global avrdx_ccl_ccl
	.global avrdx_porta_port
	.global avrdx_tca0_ovf
	.global avrdx_tca0_hunf
	.global avrdx_tca0_cmp0
	.global avrdx_tca0_cmp1
	.global avrdx_tca0_cmp2
	.global avrdx_tcb0_int
	.global avrdx_tcb1_int
	.global avrdx_tcd0_ovf
	.global avrdx_tcd0_trig
	.global avrdx_twi0_twis
	.global avrdx_twi0_twim
	.global avrdx_spi0_int
	.global avrdx_usart0_rxc
	.global avrdx_usart0_dre
	.global avrdx_usart0_txc
	.global avrdx_portd_port
	.global avrdx_ac0_ac
	.global avrdx_adc0_resrdy
	.global avrdx_adc0_wcmp
	.global avrdx_zcd0_zcd
	.global avrdx_ptc_ptc
	.global avrdx_ac1_ac
	.global avrdx_portc_port
	.global avrdx_tcb2_int
	.global avrdx_usart1_rxc
	.global avrdx_usart1_dre
	.global avrdx_usart1_txc
	.global avrdx_portf_port
	.global avrdx_nvmctrl_ee
	.global avrdx_spi1_int
	.global avrdx_usart2_rxc
	.global avrdx_usart2_dre
	.global avrdx_usart2_txc
	.global avrdx_ac2_ac
#elif defined(CONFIG_ARCH_CHIP_AVR128DA64)
	.global avrdx_nmi
	.global avrdx_bod_vlm
	.global avrdx_rtc_cnt
	.global avrdx_rtc_pit
	.global avrdx_ccl_ccl
	.global avrdx_porta_port
	.global avrdx_tca0_lunf
	.global avrdx_tca0_ovf
	.global avrdx_tca0_hunf
	.global avrdx_tca0_cmp0
	.global avrdx_tca0_lcmp0
	.global avrdx_tca0_cmp1
	.global avrdx_tca0_lcmp1
	.global avrdx_tca0_cmp2
	.global avrdx_tca0_lcmp2
	.global avrdx_tcb0_int
	.global avrdx_tcb1_int
	.global avrdx_tcd0_ovf
	.global avrdx_tcd0_trig
	.global avrdx_twi0_twis
	.global avrdx_twi0_twim
	.global avrdx_spi0_int
	.global avrdx_usart0_rxc
	.global avrdx_usart0_dre
	.global avrdx_usart0_txc
	.global avrdx_portd_port
	.global avrdx_ac0_ac
	.global avrdx_adc0_resrdy
	.global avrdx_adc0_wcmp
	.global avrdx_zcd0_zcd
	.global avrdx_ptc_ptc
	.global avrdx_ac1_ac
	.global avrdx_portc_port
	.global avrdx_tcb2_int
	.global avrdx_usart1_rxc
	.global avrdx_usart1_dre
	.global avrdx_usart1_txc
	.global avrdx_portf_port
	.global avrdx_nvmctrl_ee
	.global avrdx_spi1_int
	.global avrdx_usart2_rxc
	.global avrdx_usart2_dre
	.global avrdx_usart2_txc
	.global avrdx_ac2_ac
	.global avrdx_tcb3_int
	.global avrdx_twi1_twis
	.global avrdx_twi1_twim
	.global avrdx_portb_port
	.global avrdx_porte_port
	.global avrdx_tca1_lunf
	.global avrdx_tca1_ovf
	.global avrdx_tca1_hunf
	.global avrdx_tca1_cmp0
	.global avrdx_tca1_lcmp0
	.global avrdx_tca1_cmp1
	.global avrdx_tca1_lcmp1
	.global avrdx_tca1_cmp2
	.global avrdx_tca1_lcmp2
	.global avrdx_zcd1_zcd
	.global avrdx_usart3_rxc
	.global avrdx_usart3_dre
	.global avrdx_usart3_txc
	.global avrdx_usart4_rxc
	.global avrdx_usart4_dre
	.global avrdx_usart4_txc
	.global avrdx_portg_port
	.global avrdx_zcd2_zcd
	.global avrdx_tcb4_int
	.global avrdx_usart5_rxc
	.global avrdx_usart5_dre
	.global avrdx_usart5_txc
#elif defined(CONFIG_ARCH_CHIP_AVR128DB64)
	.global avrdx_nmi
	.global avrdx_bod_vlm
	.global avrdx_clkctrl_cfd
	.global avrdx_mvio_mvio
	.global avrdx_rtc_cnt
	.global avrdx_rtc_pit
	.global avrdx_ccl_ccl
	.global avrdx_porta_port
	.global avrdx_tca0_lunf
	.global avrdx_tca0_ovf
	.global avrdx_tca0_hunf
	.global avrdx_tca0_cmp0
	.global avrdx_tca0_lcmp0
	.global avrdx_tca0_cmp1
	.global avrdx_tca0_lcmp1
	.global avrdx_tca0_cmp2
	.global avrdx_tca0_lcmp2
	.global avrdx_tcb0_int
	.global avrdx_tcb1_int
	.global avrdx_tcd0_ovf
	.global avrdx_tcd0_trig
	.global avrdx_twi0_twis
	.global avrdx_twi0_twim
	.global avrdx_spi0_int
	.global avrdx_usart0_rxc
	.global avrdx_usart0_dre
	.global avrdx_usart0_txc
	.global avrdx_portd_port
	.global avrdx_ac0_ac
	.global avrdx_adc0_resrdy
	.global avrdx_adc0_wcmp
	.global avrdx_zcd0_zcd
	.global avrdx_ac1_ac
	.global avrdx_portc_port
	.global avrdx_tcb2_int
	.global avrdx_usart1_rxc
	.global avrdx_usart1_dre
	.global avrdx_usart1_txc
	.global avrdx_portf_port
	.global avrdx_nvmctrl_ee
	.global avrdx_spi1_int
	.global avrdx_usart2_rxc
	.global avrdx_usart2_dre
	.global avrdx_usart2_txc
	.global avrdx_ac2_ac
	.global avrdx_twi1_twis
	.global avrdx_twi1_twim
	.global avrdx_tcb3_int
	.global avrdx_portb_port
	.global avrdx_porte_port
	.global avrdx_tca1_lunf
	.global avrdx_tca1_ovf
	.global avrdx_tca1_hunf
	.global avrdx_tca1_cmp0
	.global avrdx_tca1_lcmp0
	.global avrdx_tca1_cmp1
	.global avrdx_tca1_lcmp1
	.global avrdx_tca1_cmp2
	.global avrdx_tca1_lcmp2
	.global avrdx_zcd1_zcd
	.global avrdx_usart3_rxc
	.global avrdx_usart3_dre
	.global avrdx_usart3_txc
	.global avrdx_usart4_rxc
	.global avrdx_usart4_dre
	.global avrdx_usart4_txc
	.global avrdx_portg_port
	.global avrdx_zcd2_zcd
	.global avrdx_tcb4_int
	.global avrdx_usart5_rxc
	.global avrdx_usart5_dre
	.global avrdx_usart5_txc
#else
#error "Unrecognized chip"
#endif

/****************************************************************************
 * Macros
 ****************************************************************************/

	.macro	vector name
	jmp		\name
	.endm

/****************************************************************************
 * Vector Table
 ****************************************************************************/

/* The AVR128DA28 has 41 interrupt vectors including vector 0, the reset
 * vector.
 */

	.section .vectors, "ax", @progbits
	.func	vectortab
vectortab:
	jmp		__start				/*  0: Vector 0 is the reset vector */
#if defined(CONFIG_ARCH_CHIP_AVR128DA28)
	vector avrdx_nmi
	vector avrdx_bod_vlm
	vector avrdx_rtc_cnt
	vector avrdx_rtc_pit
	vector avrdx_ccl_ccl
	vector avrdx_porta_port
	vector avrdx_tca0_ovf
	vector avrdx_tca0_hunf
	vector avrdx_tca0_cmp0
	vector avrdx_tca0_cmp1
	vector avrdx_tca0_cmp2
	vector avrdx_tcb0_int
	vector avrdx_tcb1_int
	vector avrdx_tcd0_ovf
	vector avrdx_tcd0_trig
	vector avrdx_twi0_twis
	vector avrdx_twi0_twim
	vector avrdx_spi0_int
	vector avrdx_usart0_rxc
	vector avrdx_usart0_dre
	vector avrdx_usart0_txc
	vector avrdx_portd_port
	vector avrdx_ac0_ac
	vector avrdx_adc0_resrdy
	vector avrdx_adc0_wcmp
	vector avrdx_zcd0_zcd
	vector avrdx_ptc_ptc
	vector avrdx_ac1_ac
	vector avrdx_portc_port
	vector avrdx_tcb2_int
	vector avrdx_usart1_rxc
	vector avrdx_usart1_dre
	vector avrdx_usart1_txc
	vector avrdx_portf_port
	vector avrdx_nvmctrl_ee
	vector avrdx_spi1_int
	vector avrdx_usart2_rxc
	vector avrdx_usart2_dre
	vector avrdx_usart2_txc
	vector avrdx_ac2_ac
#elif defined(CONFIG_ARCH_CHIP_AVR128DA64)
	vector avrdx_nmi
	vector avrdx_bod_vlm
	vector avrdx_rtc_cnt
	vector avrdx_rtc_pit
	vector avrdx_ccl_ccl
	vector avrdx_porta_port
	vector avrdx_tca0_lunf
	vector avrdx_tca0_ovf
	vector avrdx_tca0_hunf
	vector avrdx_tca0_cmp0
	vector avrdx_tca0_lcmp0
	vector avrdx_tca0_cmp1
	vector avrdx_tca0_lcmp1
	vector avrdx_tca0_cmp2
	vector avrdx_tca0_lcmp2
	vector avrdx_tcb0_int
	vector avrdx_tcb1_int
	vector avrdx_tcd0_ovf
	vector avrdx_tcd0_trig
	vector avrdx_twi0_twis
	vector avrdx_twi0_twim
	vector avrdx_spi0_int
	vector avrdx_usart0_rxc
	vector avrdx_usart0_dre
	vector avrdx_usart0_txc
	vector avrdx_portd_port
	vector avrdx_ac0_ac
	vector avrdx_adc0_resrdy
	vector avrdx_adc0_wcmp
	vector avrdx_zcd0_zcd
	vector avrdx_ptc_ptc
	vector avrdx_ac1_ac
	vector avrdx_portc_port
	vector avrdx_tcb2_int
	vector avrdx_usart1_rxc
	vector avrdx_usart1_dre
	vector avrdx_usart1_txc
	vector avrdx_portf_port
	vector avrdx_nvmctrl_ee
	vector avrdx_spi1_int
	vector avrdx_usart2_rxc
	vector avrdx_usart2_dre
	vector avrdx_usart2_txc
	vector avrdx_ac2_ac
	vector avrdx_tcb3_int
	vector avrdx_twi1_twis
	vector avrdx_twi1_twim
	vector avrdx_portb_port
	vector avrdx_porte_port
	vector avrdx_tca1_lunf
	vector avrdx_tca1_ovf
	vector avrdx_tca1_hunf
	vector avrdx_tca1_cmp0
	vector avrdx_tca1_lcmp0
	vector avrdx_tca1_cmp1
	vector avrdx_tca1_lcmp1
	vector avrdx_tca1_cmp2
	vector avrdx_tca1_lcmp2
	vector avrdx_zcd1_zcd
	vector avrdx_usart3_rxc
	vector avrdx_usart3_dre
	vector avrdx_usart3_txc
	vector avrdx_usart4_rxc
	vector avrdx_usart4_dre
	vector avrdx_usart4_txc
	vector avrdx_portg_port
	vector avrdx_zcd2_zcd
	vector avrdx_tcb4_int
	vector avrdx_usart5_rxc
	vector avrdx_usart5_dre
	vector avrdx_usart5_txc
#elif defined(CONFIG_ARCH_CHIP_AVR128DB64)
	vector avrdx_nmi
	vector avrdx_bod_vlm
	vector avrdx_clkctrl_cfd
	vector avrdx_mvio_mvio
	vector avrdx_rtc_cnt
	vector avrdx_rtc_pit
	vector avrdx_ccl_ccl
	vector avrdx_porta_port
	vector avrdx_tca0_lunf
	vector avrdx_tca0_ovf
	vector avrdx_tca0_hunf
	vector avrdx_tca0_cmp0
	vector avrdx_tca0_lcmp0
	vector avrdx_tca0_cmp1
	vector avrdx_tca0_lcmp1
	vector avrdx_tca0_cmp2
	vector avrdx_tca0_lcmp2
	vector avrdx_tcb0_int
	vector avrdx_tcb1_int
	vector avrdx_tcd0_ovf
	vector avrdx_tcd0_trig
	vector avrdx_twi0_twis
	vector avrdx_twi0_twim
	vector avrdx_spi0_int
	vector avrdx_usart0_rxc
	vector avrdx_usart0_dre
	vector avrdx_usart0_txc
	vector avrdx_portd_port
	vector avrdx_ac0_ac
	vector avrdx_adc0_resrdy
	vector avrdx_adc0_wcmp
	vector avrdx_zcd0_zcd
	vector avrdx_ac1_ac
	vector avrdx_portc_port
	vector avrdx_tcb2_int
	vector avrdx_usart1_rxc
	vector avrdx_usart1_dre
	vector avrdx_usart1_txc
	vector avrdx_portf_port
	vector avrdx_nvmctrl_ee
	vector avrdx_spi1_int
	vector avrdx_usart2_rxc
	vector avrdx_usart2_dre
	vector avrdx_usart2_txc
	vector avrdx_ac2_ac
	vector avrdx_twi1_twis
	vector avrdx_twi1_twim
	vector avrdx_tcb3_int
	vector avrdx_portb_port
	vector avrdx_porte_port
	vector avrdx_tca1_lunf
	vector avrdx_tca1_ovf
	vector avrdx_tca1_hunf
	vector avrdx_tca1_cmp0
	vector avrdx_tca1_lcmp0
	vector avrdx_tca1_cmp1
	vector avrdx_tca1_lcmp1
	vector avrdx_tca1_cmp2
	vector avrdx_tca1_lcmp2
	vector avrdx_zcd1_zcd
	vector avrdx_usart3_rxc
	vector avrdx_usart3_dre
	vector avrdx_usart3_txc
	vector avrdx_usart4_rxc
	vector avrdx_usart4_dre
	vector avrdx_usart4_txc
	vector avrdx_portg_port
	vector avrdx_zcd2_zcd
	vector avrdx_tcb4_int
	vector avrdx_usart5_rxc
	vector avrdx_usart5_dre
	vector avrdx_usart5_txc
#else
#error "Unrecognized chip"
#endif
	.endfunc

/****************************************************************************
 * Reset Entry Point
 ****************************************************************************/

	.section .init, "ax", @progbits
	.func	__start
__start:

	/* no interrupts! Useful for software reset by jumping to address 0 */
	cli

#if defined(EIND)
	/* set EIND to 0, just to be sure we are sane */
	out		_SFR_IO_ADDR(EIND), 0 // EIND = 0x3c
#endif /* EIND */

	/* Clear the zero register, clear the status register and initialize the
	 * IDLE thread stack
	 */

	clr		r1
	out		_SFR_IO_ADDR(SREG), r1
	ldi		r28, lo8(STACKBASE)
	ldi		r29, hi8(STACKBASE)
	out		_SFR_IO_ADDR(SPH), r29
	out		_SFR_IO_ADDR(SPL), r28


#ifdef CONFIG_AVR_HAVE_FLMAP

	/* Configure FLMAP bits in NVMCTRL.CTRLB in case this was software
	 * reset and they are not in default state.
	 *
	 * Don't care if the application actually changes the register. These
	 * bits are not under configuration protection (CCP) and runaway
	 * application can possibly change them.
	 *
	 * This is executed if the chip supports it, regardless of if the board
	 * declared it is using the feature by setting AVR_HAVE_BOARD_FLMAP
	 * in Kconfig (it may be using it without setting that option.)
	 */

	ldi		r26, lo8(NVMCTRL_CTRLB)
	ldi		r27, hi8(NVMCTRL_CTRLB)
	ld		r16, X

	/* Default value (on current chips anyway) uses all bits set to one. */

#  if !defined(NVMCTRL_FLMAP_1_bm)

	/* Might be there won't be such (supported) device ever. */

#    error Chips without FLMAP bit 1 are not supported

#  elif !defined(NVMCTRL_FLMAP_2_bm)

	/* Expected case */

	ldi		r17, NVMCTRL_FLMAP_0_bm | NVMCTRL_FLMAP_1_bm;

#  else

	/* Future device with more than 128kB RAM. The default is expected
	 * to be all bits set to one but it needs to be verified when
	 * support for such chip is added. Issue a warning.
	 */

#    warning Support for more than 128kB flash was done blindly here

	ldi		r17, NVMCTRL_FLMAP_0_bm | NVMCTRL_FLMAP_1_bm | NVMCTRL_FLMAP_2_bm;

#  endif

	/* As long as we are always setting bits to one, we don't need
	 * to clear them in the original value
	 */

	or		r16, r17
	st		X, r16

#endif

	/* Copy initial global data values from FLASH into RAM */

	.global	__do_copy_data;			/* Required to suppress dragging in logic from libgcc */
__do_copy_data:

#ifdef HAVE_RAMPZ
	ldi		r17, hi8(_edata)
	ldi		r26, lo8(_sdata)
	ldi		r27, hi8(_sdata)
	ldi		r30, lo8(_eronly)
	ldi		r31, hi8(_eronly)
	ldi		r16, hh8(_eronly)
	out		_SFR_IO_ADDR(RAMPZ), r16
	rjmp	.Lcopystart

.Lcopyloop:
	elpm	r0, Z+
	st		X+, r0

.Lcopystart:
	cpi		r26, lo8(_edata)
	cpc		r27, r17
	brne	.Lcopyloop
#else
	ldi		r17, hi8(_edata)
	ldi		r26, lo8(_sdata)
	ldi		r27, hi8(_sdata)
	ldi		r30, lo8(_eronly)
	ldi		r31, hi8(_eronly)
	rjmp	.Lcopystart

.Lcopyloop:
	lpm		r0, Z+
	st		X+, r0

.Lcopystart:
	cpi		r26, lo8(_edata)
	cpc		r27, r17
	brne	.Lcopyloop
#endif

	/* Clear uninitialized data */

	.global	__do_clear_bss;			/* Required to suppress dragging in logic from libgcc */
__do_clear_bss:

	ldi		r17, hi8(_ebss)
	ldi		r26, lo8(_sbss)
	ldi		r27, hi8(_sbss)
	rjmp	.Lclearstart

.Lclearloop:
	st		X+, r1

.Lclearstart:
	cpi		r26, lo8(_ebss)
	cpc		r27, r17
	brne	.Lclearloop

	/* Perform any low-level initialization */

	call	avr_lowinit

	/* Now start NuttX */

	call	nx_start
	jmp		exit
	.endfunc

/****************************************************************************
 * Heap Base
 ****************************************************************************/

	/* This global variable is unsigned long g_idle_topstack and is exported from
	 * here only because of its coupling to other uses of _enoinit in this file
	 */

	.data
	.globl	g_idle_topstack
	.type	g_idle_topstack, object
g_idle_topstack:
	.word	_enoinit+CONFIG_IDLETHREAD_STACKSIZE
	.size	g_idle_topstack, .-g_idle_topstack
	.end
