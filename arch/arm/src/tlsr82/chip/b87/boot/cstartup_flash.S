/****************************************************************************
 * arch/arm/src/tlsr82/chip/b87/boot/cstartup_flash.S
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

#include <nuttx/config.h>
#include <arch/chip/irq.h>

#define SRAM_SIZE_32K_EN				0 /* defaut sram size=64k */

#define FLL_STK_EN					1
#define ZERO_IC_TAG_EN					1
#define IC_TAG_CACEH_ADDR_EQU_EN			0
#define FLASH_WAKEUP_EN					1
#define COPY_DATA_EN					1
#define MULTI_ADDRESS_START				1
#define MOVE_BIN_CODE_EN				0

#ifndef __LOAD_RAM_SIZE__
#define __LOAD_RAM_SIZE__				0xc
#endif

#ifndef __IRQ_STK_SIZE__
#define __IRQ_STK_SIZE__				CONFIG_ARCH_INTERRUPTSTACK
#if CONFIG_ARCH_INTERRUPTSTACK <= 128
#  error "Interrupt Stack must be larger than 128"
#endif
#endif
	.code	16
@********************************************************************************************************
@                                           MACROS AND DEFINIITIONS
@********************************************************************************************************

	@ Mode, correspords to bits 0-5 in CPSR

	.equ	MODE_BITS, 0x1F		@ Bit mask for mode bits in CPSR
	.equ	IRQ_MODE, 0x12		@ Interrupt Request mode
	.equ	SVC_MODE, 0x13		@ Supervisor mode
	.equ	IRQ_STK_SIZE, __IRQ_STK_SIZE__
	.equ	__LOAD_FLASH, 1
	.equ	__LOAD_DUT, 0
	.equ	__LOAD_RAM, 0

@********************************************************************************************************
@                                            TC32 EXCEPTION VECTORS
@********************************************************************************************************

	.section	.vectors, "ax"
	.global	__reset
	.global	__irq
	.global	__start
	.global	__LOAD_RAM
	.global	__LOAD_DUT
	.global	__LOAD_FLASH

__start:					@ MUST,  referenced by boot.link

	.extern	tc32_exception

	.extern	firmwareVersion

	.extern	_ramcode_size_div_16_
	.extern	_ramcode_size_div_256_
	.extern	_ramcode_size_div_16_align_256_
	.extern	_ramcode_size_align_256_
	.extern	_ictag_start_
	.extern	_ictag_end_
	.extern	tl_multi_addr_L
	.extern	tl_multi_addr_H
	.org	0x0
	tj		__reset
@	.word	(firmwareVersion)
	.org	0x8
	.word	(0x544c4e4b)
#if MOVE_BIN_CODE_EN
	.word	(0x00880000 + _bin_size_div_16)
#else
	.word	(0x00880000 + _ramcode_size_div_16_align_256_)
#endif

	.org	0x10
	tj		__irq
	.org	0x18
	.word	(_bin_size_)
	.extern	__tc32_start

	.org	0x20

__reset:

#if 0
	@ add debug, PB4 output 1
	tloadr		r1, DEBUG_GPIO		@0x80058a  PB oen
	tmov		r0, #139		@0b 11101111
	tstorerb	r0, [r1, #0]

	tmov		r0, #16			@0b 00010000
	tstorerb	r0, [r1, #1]		@0x800583  PB output
#endif

#if FLASH_WAKEUP_EN

FLASH_WAKEUP_BEGIN:
	tloadr		r0,DATA_I+44
	tmov		r1,#0
	tstorerb	r1,[r0,#1]
	tmov		r1,#171			@Flash deep cmd: 0xAB
	tstorerb	r1,[r0,#0]
	tmov		r2,#0
	tmov		r3,#6
TNOP:
	tadd		r2,#1
	tcmp		r2,r3
	tjle		TNOP
	tmov		r1,#1
	tstorerb	r1,[r0,#1]
FLASH_WAKEUP_END:

#endif

	@ efuse load need delay about 18us
	tmov		r2,#0
	tmov		r3,#110
EFUSE_DELAY:
	tadd		r2,#1
	tcmp		r2,r3
	tjle		EFUSE_DELAY
EFUSE_DELAY_END:

@********************************************************************************************************
@                                    FILL .DATA AND .BSS WITH 0xFF
@********************************************************************************************************
#if FLL_STK_EN

	@tloadr	r0, FLL_D				@r0 = 0xffffffff
	tmov	r0, #0					@r0 = 0xffffffff
	tloadr	r1, FLL_D+4				@r1 = _start_data_
	tloadr	r2, FLL_D+8				@r2 = 0x850000

FLL_STK:
	tcmp	r1, r2					@
	tjge	FLL_STK_END				@r1>=r2 jump to FLL_STK_END
	tstorer	r0, [r1, #0]				@*(unsigned int*)(_start_data_)=0xffffffff
	tadd	r1, #4					@r1 = r1+4
	tj		FLL_STK				@jump to FLL_STK
FLL_STK_END:

#endif
@********************************************************************************************************
@                                    UPDATE SP UNDER IRQ/SVC MODE
@********************************************************************************************************
	tloadr	r0, DAT0				@r0 = 0x12 IRQ
	tmcsr	r0					@CPSR=r0
	tloadr	r0, DAT0 + 8				@r0 = g_intstacktop
	tmov	r13, r0					@r13/SP= r0 update SP under IRQ mode

	tloadr	r0, DAT0 + 4				@r0 = 0x13 SVC
	tmcsr	r0					@CPSR=r0
	tloadr	r0, IDLE_STACK				@r0 = _ebss + CONFIG_IDLETHREAD_STACKSIZE
	tmov	r13, r0					@r13= r0 update SP under SVC mode
@********************************************************************************************************
@                                    .BSS INITIALIZATION FOR 0
@********************************************************************************************************
	tmov	r0, #0					@r0 = 0
	tloadr	r1, DAT0 + 16				@r1 = _sbss
	tloadr	r2, DAT0 + 20				@r2 = _ebss

ZERO_BSS_BEGIN:
	tcmp	r1, r2
	tjge	ZERO_BSS_END				@r1>=r2 jump to ZERO_BSS_END
	tstorer	r0, [r1, #0]				@*(unsigned int*)(_sbss)=r0=0
	tadd	r1, #4					@r1 = r1 + 4
	tj		ZERO_BSS_BEGIN			@jump to ZERO_BSS_BEGIN
ZERO_BSS_END:
@********************************************************************************************************
@                                    IC TAG INITIALIZATION
@********************************************************************************************************
#if ZERO_IC_TAG_EN
ZERO_TAG:
	tmov	r0, #0
	tloadr	r1, DAT0 + 28				@r1 = _ictag_start_
	tloadr	r2, DAT0 + 32				@r2 = _ictag_end_
ZERO_TAG_BEGIN:
	tcmp	r1, r2
	tjge	ZERO_TAG_END				@r1>=r2 jump to ZERO_TAG_END
	tstorer	r0, [r1, #0]				@*(unsigned int*)(_ictag_start_)=r0=0
	tadd	r1, #4					@r1 = r1 + 4
	tj		ZERO_TAG_BEGIN			@jump to ZERO_TAG_BEGIN
ZERO_TAG_END:
#endif
@********************************************************************************************************
@                                    IC CACHE INITIALIZATION
@********************************************************************************************************

SET_IC:
	tloadr		r1, DAT0 + 24			@ r1 = 0x80060c
	tloadr		r0, DAT0 + 36			@ r0 = _ramcode_size_align_256_
	tshftr		r0, r0,#8			@ r0 = _ramcode_size_align_256_/256
	tstorerb	r0, [r1, #0]			@ *(unsigned int*)(0x80060c) = r0
#if IC_TAG_CACHE_ADDR_EQU_EN

#else
	tadd		r0, #1				@ r0 = r0 + 1
#endif
	tstorerb	r0, [r1, #1]			@ *(unsigned int*)(0x80060d) = r0
SET_IC_END:
@********************************************************************************************************
@                                    DCDC INITIALIZATION FOR FLASH
@********************************************************************************************************
	/*system on*/
	tloadr		r0,DATA_I+32			@0x00800060
	tloadr		r1,DATA_I+36			@0xff000000
	tstorer		r1,[r0,#0]			@*(unsigned int*)0x800060=0xff000000
	tshftr		r1,r1,#24			@0x000000ff
	tstorerb	r1,[r0,#4]			@dig_0x64=0xff
	tloadr		r1,DATA_I+40			@dig_0x65=0xf7
	tstorerb	r1,[r0,#5]

	/*check ana_reg_0x7e*/

	tloadr		r0,DATA_I+24			@0x7e

RANA_REG_BEGIN:
	tloadr		r1,DATA_I+28			@0x008000b8
	tmov		r2,r0				@ana_reg_adr:0x7e
	tstorerb	r2,[r1,#0]			@*(unsigned int*)0x8000b8=0x7e;
	tmov		r2,#64				@ana_reg_dat:0x40=64
	tstorerb	r2,[r1,#2]			@*(unsigned int*)0x8000ba=0x40;

RWAIT_REG_FINISH:
	tloadrb		r2,[r1,#2]
	tshftl		r2,r2,#31
	tshftr		r2,r2,#31
	tcmp		r2,#1
	tjeq		RWAIT_REG_FINISH
	tloadrb		r2,[r1,#1]
RANA_REG_END:

	tshftl		r2,r2,#31
	tshftr		r2,r2,#31
	tcmp		r2,#0

	/*if ana_reg_0x7e==0x00 retention data initiate after waking up from deep*/

#if MULTI_ADDRESS_START
	tjeq		MULTI_ADDRESS_END
MULTI_ADDRESS_BEGIN:
	tloadr		r0, MULTI_ADDRESS_DATA
	tloadr		r1, MULTI_ADDRESS_DATA+4
	tloadr		r3, MULTI_ADDRESS_DATA+8
	tloadrb		r2, [r1,#0]
	tstorerb	r2, [r0,#0]
	tloadrb		r2, [r3,#0]
	tstorerb	r2, [r0,#1]
#if COPY_DATA_EN
	tj			COPY_DATA		@deep wakeup with retention
#else
	tj			COPY_DATA_END
#endif
MULTI_ADDRESS_END:
#endif


@********************************************************************************************************
@                                    RETENTION DATA INITIALIZATION
@********************************************************************************************************
COPY_RET_DATA:
	tloadr		r1, DATA_I			@ r1 = _rstored_
	tloadr		r2, DATA_I+4			@ r2 = _retention_data_start_
	tloadr		r3, DATA_I+8			@ r3 = _retention_data_end_
COPY_RET_DATA_BEGIN:
	tcmp		r2, r3				@
	tjge		COPY_RET_DATA_END		@ r2>=r3 jump to COPY_RET_DATA_END
	tloadr		r0, [r1, #0]			@ r0 = *(unsigned int*)(_rstored_)
	tstorer		r0, [r2, #0]			@ *(unsigned int*)(_retention_data_start_) = r0
	tadd		r1, #4				@ r1 = r1 + 4
	tadd		r2, #4				@ r2 = r2 + 4
	tj		COPY_RET_DATA_BEGIN		@ jump to COPY_RET_DATA_BEGIN
COPY_RET_DATA_END:


@********************************************************************************************************
@                                    DATA INITIALIZATION
@********************************************************************************************************
COPY_DATA:
	tloadr		r1, DATA_I+12			@ r1 = _dstored_
	tloadr		r2, DATA_I+16			@ r2 = _start_data_
	tloadr		r3, DATA_I+20			@ r3 = _end_data_
COPY_DATA_BEGIN:
	tcmp		r2, r3				@
	tjge		COPY_DATA_END			@ r2>=r3 jump to COPY_DATA_END
	tloadr		r0, [r1, #0]			@ r0 = *(unsigned int*)(_dstored_)
	tstorer		r0, [r2, #0]			@ *(unsigned int*)(_start_data_) = r0
	tadd		r1, #4				@ r1 = r1 + 4
	tadd		r2, #4				@ r2 = r2 + 4
	tj		COPY_DATA_BEGIN			@ jump to COPY_DATA_BEGIN
COPY_DATA_END:



#if 0
SETSPISPEED:
	tloadr		r1, DAT0 + 36
	tmov		r0, #0xbb			@0x0b for fast read; 0xbb for dual dat/adr
	tstorerb	r0, [r1, #0]
	tmov		r0, #3				@3 for dual dat/adr
	tstorerb	r0, [r1, #1]
#endif

ENTER_MAIN:
	tjl			__tc32_start
END:	tj	END

	.balign	4
DAT0:
	.word	0x12					@0 IRQ
	.word	0x13					@4 SVC
	.word	g_intstacktop				@8 interrupt stack top
#if  SRAM_SIZE_32K_EN
	.word	(0x848000)				@12 stack end
#else
	.word	(0x850000)				@12 stack end
#endif
	.word	(_sbss)					@16
	.word	(_ebss)					@20
	.word	(0x80060c)				@24
	.word	_ictag_start_				@28 IC tag start
	.word	_ictag_end_				@32 IC tag end
#if IC_TAG_CACHE_ADDR_EQU_EN
	.word	(0x00f000)@36
#endif
	.word	_ramcode_size_align_256_		@40

DATA_I:
	.word	(_rstored_)				@0
	.word	(_retention_data_start_)		@4
	.word	(_retention_data_end_)			@8
	.word	_dstored_				@12
	.word	_start_data_				@16
	.word	_end_data_				@20
	.word	(0x0000007e)				@24
	.word	(0x008000b8)				@28
	.word	(0x00800060)				@32
	.word	(0xff080000)				@36
	.word	(0x000000f7)				@40
	.word	(0x0080000c)				@44

DEBUG_GPIO:
	.word	(0x80058a)				@ PBx oen

FLL_D:
	.word	0xffffffff				@0
	.word	(_start_data_)				@4
#if  SRAM_SIZE_32K_EN
	.word	(0x848000)				@8
#else
	.word	(0x850000)				@8
#endif
	.word	_ramcode_size_div_256_			@12

MULTI_ADDRESS_DATA:
	.word	(0x0080063e)				@0
	.word	tl_multi_addr_L				@4
	.word	tl_multi_addr_H				@8

IDLE_STACK:
	.word	(_ebss + CONFIG_IDLETHREAD_STACKSIZE)

	.align 4

	@ src code for the irq proc part .

__irq:
	/* Save R0 ~ R3, these registers will be used before saved */

	tpush		{r0}
	tpush		{r1}
	tpush		{r2}
	tpush		{r3}

	/* Save LR, LR will change after tjl instruction */

	tpush		{r14}

	/* Call tc32_exception */

	tjl		tc32_exception

ASMEND:
	.section	.bss
	.global	g_intstackalloc
	.global	g_intstacktop
	.align	4
g_intstackalloc:
	.skip	(CONFIG_ARCH_INTERRUPTSTACK & ~3)
g_intstacktop:
	.size	g_intstackalloc, .-g_intstackalloc
	.end
