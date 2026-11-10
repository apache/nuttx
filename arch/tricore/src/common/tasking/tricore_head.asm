/****************************************************************************
 * arch/tricore/src/common/tasking/tricore_head.asm
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
#include <arch/csfr.h>
#ifdef CONFIG_ARCH_CHIP_FAMILY_TC4X
#  include "hardware/tc4xx_vmt.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TRICORE_PSW_RESET  0x980   /* Supervisor, call depth counting on */
#define TRICORE_BIV_VECTOR 0x1fe0  /* Single interrupt vector at INTTAB + 0x1fe0 */

#ifdef CONFIG_ARCH_CHIP_FAMILY_TC3X
#  define TC3XX_WDTCPU0_CON0 0xf003624c  /* CPU0 watchdog CON0, 0xc apart per CPU */
#endif
#ifdef CONFIG_ARCH_CHIP_FAMILY_TC4X
#  define TC4XX_PSW_VIRT     0xb00       /* PSW for disabling the virtualization */
#  define TC4XX_UCB0_RAMINIT 0xae408814  /* SRAMs that the boot firmware initializes */
#endif

/* Make the CSA link word of the CSA address in addr: the segment in bits
 * 19:16 and the offset in bits 15:0.
 */

tricore_csa_link .macro addr, tmp1, tmp2
	extr.u	tmp1, addr, #6, #16
	extr.u	tmp2, addr, #12, #20
	insert	addr, tmp2, tmp1, #0, #16
	.endm

	.extern	tricore_wdt_disable
	.extern	_data_flash
	.extern	_sdata
	.extern	_edata
	.extern	_sbss
	.extern	_ebss
	.extern	tricore_clockconfig
	.extern	tricore_start
	.extern	__ustack
	.extern	__istack
	.extern	__csa_begin
	.extern	__csa_end
	.extern	__a0_mem
	.extern	__a1_mem
	.extern	__a8_mem
	.extern	__a9_mem
	.extern	__TRAPTAB_CPU0
	.extern	__INTTAB_CPU0
	.extern	__psram_ustack
#ifdef CONFIG_ARCH_CHIP_FAMILY_TC4X
	.extern	__psram_boot_csa
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tricore_head
 *
 * Description:
 *   Set up the core registers and the CSA list, which the code has to do
 *   before it can call anything: the trap and interrupt vectors, the
 *   interrupt stack, the caches and the free CSA list with LCX at the
 *   third last entry.  Then set the stack, the small data base registers
 *   and the PSW, disable the watchdogs, copy .data and clear .bss, set
 *   up the clocks (see tricore_internal.h) and jump to tricore_start.
 *
 *   On TC3xx BTV, BIV, ISP, PCON0 and DCON0 are CPU ENDINIT protected,
 *   so the ENDINIT of the CPU watchdog is cleared around them.
 *
 *   TC4xx first disables the virtualization with a call to
 *   tricore_virt_off, on the boot stack and the single boot CSA in the
 *   PSPR that the linker script gives, as the iLLD startup code does.
 *   Then, before anything is stored there, it initializes the DSPR (CSA
 *   list, stacks) and the DLMU (.data, .bss) of CPU0 if the boot firmware
 *   does not, and before .data and .bss the LMU and MCAN RAMs, which the
 *   boot firmware never initializes.
 *
 ****************************************************************************/

	.sdecl	".text.tricore_head",code
	.sect	".text.tricore_head"
tricore_head:
#ifdef CONFIG_ARCH_CHIP_FAMILY_TC3X
	jl	tc3xx_endinit_clear

#endif
	movh.a	a15, #@HIS(__TRAPTAB_CPU0)
	lea	a15, [a15]@LOS(__TRAPTAB_CPU0)
	mov.d	d15, a15
	mtcr	#TRICORE_CPU_BTV, d15
	isync

#ifdef CONFIG_ARCH_CHIP_FAMILY_TC4X
	movh.a	a15, #@HIS(__psram_boot_csa)
	lea	a15, [a15]@LOS(__psram_boot_csa)
	mov.d	d15, a15
	tricore_csa_link d15, d14, d13
	mtcr	#TRICORE_CPU_FCX, d15
	isync

	call	tricore_virt_off

	movh.a	a15, #@HIS(__TRAPTAB_CPU0)
	lea	a15, [a15]@LOS(__TRAPTAB_CPU0)
	mov.d	d15, a15
	mtcr	#TRICORE_CPU_BTV, d15
	isync

	; The DSPR and the DLMU of CPU0, unless UCB0_RAMINIT has the boot
	; firmware initialize them after every power-on reset: RAMIN 0 and the
	; CPU0 bits of RAMINSEL and LMUINSEL 0

	movh.a	a15, #@HIS(TC4XX_UCB0_RAMINIT)
	ld.w	d15, [a15]@LOS(TC4XX_UCB0_RAMINIT)
	mov	d5, #0x11
	extr.u	d0, d15, #0, #2
	jnz	d0, 1n
	extr.u	d0, d15, #16, #1
	extr.u	d1, d15, #24, #1
	sh	d1, d1, #4
	or	d5, d0, d1
1:
	mov	d4, #0
	jl	tc4xx_sram_clear

#endif
	; Caches on, interrupt vector, interrupt stack

	mov	d15, #0
	mtcr	#TRICORE_CPU_PCON0, d15
	isync
	mtcr	#TRICORE_CPU_DCON0, d15
	isync

	movh.a	a15, #@HIS(__INTTAB_CPU0)
	lea	a15, [a15]@LOS(__INTTAB_CPU0)
	lea	a15, [a15]TRICORE_BIV_VECTOR
	mov.d	d15, a15
	mtcr	#TRICORE_CPU_BIV, d15
	isync

	movh.a	a15, #@HIS(__istack)
	lea	a15, [a15]@LOS(__istack)
	mov.d	d15, a15
	mtcr	#TRICORE_CPU_ISP, d15
	isync

	; Free CSA list from __csa_begin to __csa_end, LCX at the third last

	movh.a	a15, #@HIS(__csa_begin)
	lea	a15, [a15]@LOS(__csa_begin)
	movh.a	a14, #@HIS(__csa_end)
	lea	a14, [a14]@LOS(__csa_end)
	mov.d	d15, a15
	tricore_csa_link d15, d14, d13
	mtcr	#TRICORE_CPU_FCX, d15
	isync
1:
	addi	d15, d15, #1
	st.w	[a15], d15
	lea	a15, [a15]64
	jne.a	a15, a14, 1p

	mov	d14, #0
	st.w	[a15]-64, d14
	addi	d15, d15, #-3
	mtcr	#TRICORE_CPU_LCX, d15
	isync

#ifdef CONFIG_ARCH_CHIP_FAMILY_TC3X
	jl	tc3xx_endinit_set

#endif
	movh.a	a10, #@HIS(__ustack)
	lea	a10, [a10]@LOS(__ustack)
	dsync

	movh.a	a1, #@HIS(__a1_mem)
	lea	a1, [a1]@LOS(__a1_mem)
	movh.a	a0, #@HIS(__a0_mem)
	lea	a0, [a0]@LOS(__a0_mem)
	movh.a	a8, #@HIS(__a8_mem)
	lea	a8, [a8]@LOS(__a8_mem)
	movh.a	a9, #@HIS(__a9_mem)
	lea	a9, [a9]@LOS(__a9_mem)

	mov	d0, #TRICORE_PSW_RESET
	mtcr	#TRICORE_CPU_PSW, d0
	isync

	call	tricore_wdt_disable

#ifdef CONFIG_ARCH_CHIP_FAMILY_TC4X
	; The LMU and MCAN RAMs, which the boot firmware never initializes:
	; LMU0, 1 and 4 to 6, LMU2, 3 and 7 to 9, and MCAN0 to 4

	mov	d4, #0
	movh	d5, #0x0f80
	jl	tc4xx_sram_clear
	mov	d4, #1
	mov	d5, #0x1f
	jl	tc4xx_sram_clear
	mov	d4, #5
	mov	d5, #0x7c
	jl	tc4xx_sram_clear

#endif
	; Copy .data from its image in flash and clear .bss, a word at a time
	; up to the word aligned ends that the linker script gives

	movh.a	a2, #@HIS(_data_flash)
	lea	a2, [a2]@LOS(_data_flash)
	movh.a	a3, #@HIS(_sdata)
	lea	a3, [a3]@LOS(_sdata)
	movh.a	a4, #@HIS(_edata)
	lea	a4, [a4]@LOS(_edata)
	j	7n
6:
	ld.w	d15, [a2+]4
	st.w	[a3+]4, d15
7:
	lt.a	d15, a3, a4
	jnz	d15, 6p

	mov	d14, #0
	movh.a	a3, #@HIS(_sbss)
	lea	a3, [a3]@LOS(_sbss)
	movh.a	a4, #@HIS(_ebss)
	lea	a4, [a4]@LOS(_ebss)
	j	9n
8:
	st.w	[a3+]4, d14
9:
	lt.a	d15, a3, a4
	jnz	d15, 8p

	call	tricore_clockconfig
	jz	d2, 5n
	debug
5:
	movh.a	a15, #@HIS(tricore_start)
	lea	a15, [a15]@LOS(tricore_start)
	ji	a15

#ifdef CONFIG_ARCH_CHIP_FAMILY_TC3X
/****************************************************************************
 * Name: tc3xx_endinit_clear, tc3xx_endinit_set
 *
 * Description:
 *   Clear (tc3xx_endinit_clear) or set (tc3xx_endinit_set) the ENDINIT
 *   of the watchdog of the calling CPU with the password and the lock
 *   sequence, and wait until it has changed.  Called with jl, before
 *   any CSA exists.  CPU5 has the core ID 6 on TC39x.
 *
 ****************************************************************************/

tc3xx_endinit_clear:
	mov	d5, #0
	j	2n

tc3xx_endinit_set:
	mov	d5, #1
2:
	mfcr	d4, #TRICORE_CPU_CORE_ID
	extr.u	d4, d4, #0, #3
	jne	d4, #6, 3n
	mov	d4, #5
3:
	sh	d15, d4, #1
	add	d4, d15, d4
	movh.a	a15, #@HIS(TC3XX_WDTCPU0_CON0)
	lea	a15, [a15]@LOS(TC3XX_WDTCPU0_CON0)
	addsc.a	a15, a15, d4, #2

	; Password access with the inverted password bits 7:2, then the
	; modify access with LCK set and the new ENDINIT

	ld.w	d15, [a15]
	xor	d15, d15, #0xfc
	insert	d15, d15, #1, #0, #2
	st.w	[a15], d15
	or	d4, d5, #2
	insert	d15, d15, d4, #0, #2
	st.w	[a15], d15
4:
	ld.w	d15, [a15]
	extr.u	d15, d15, #0, #1
	jne	d15, d5, 4p
	ji	a11
#endif

#ifdef CONFIG_ARCH_CHIP_FAMILY_TC4X
/****************************************************************************
 * Name: tricore_virt_off
 *
 * Description:
 *   Disable the virtualization and return with rfh.
 *
 ****************************************************************************/

tricore_virt_off:
	mov	d0, #0
	mtcr	#TRICORE_CPU_VCON0, d0
	isync
	mov	d0, #TC4XX_PSW_VIRT
	mtcr	#TRICORE_CPU_PSW, d0
	isync
	rfh

/****************************************************************************
 * Name: tc4xx_sram_clear
 *
 * Description:
 *   Fill the SRAMs of the SSHs in the mask d5 of the VMT group d4, the SSH
 *   selectors 32 * d4 to 32 * d4 + 31, one after the other with ECC
 *   correct zeros through MCONTROL.SRAM_CLR, as IfxVmt_clearSram does.
 *   Called with jl, as it runs before the CSA list exists; it uses d0 to
 *   d3, d5, d15, a14 and a15.
 *
 ****************************************************************************/

tc4xx_sram_clear:
	movh	d15, #@HIS(TC4XX_VMT_BASE)
	sh	d0, d4, #17
	add	d15, d15, d0
	mov.a	a15, d15
	ld.w	d15, [a15]TC4XX_VMT_CLC
	jz.t	d15, #1, 1n
	mov	d15, #0
	st.w	[a15]TC4XX_VMT_CLC, d15
1:
	mov	d1, #0
2:
	jeq	d5, #0, 8n
	jz.t	d5, #0, 7n
	mov	d2, #1
	sh	d2, d2, d1
	sh	d3, d1, #9
	mov.a	a14, d3
	add.a	a14, a14, a15
	lea	a14, [a14]TC4XX_VMT_MC(0)

	; Enable the MBIST shell and start the clear

	ld.w	d15, [a14]TC4XX_VMT_MC_ALMSRCS
	andn	d15, d15, #TC4XX_ALMSRCS_ALARMS
	st.w	[a14]TC4XX_VMT_MC_ALMSRCS, d15
	ld.w	d15, [a15]TC4XX_VMT_MEMTEST
	or	d15, d15, d2
	st.w	[a15]TC4XX_VMT_MEMTEST, d15
3:
	ld.w	d15, [a15]TC4XX_VMT_MEMSTAT
	and	d15, d15, d2
	jnz	d15, 3p

	mov.u	d15, #0x8001
	st.w	[a14]TC4XX_VMT_MC_MCONTROL, d15
	ld.w	d15, [a14]TC4XX_VMT_MC_MCONTROL
	andn	d15, d15, #1
	st.w	[a14]TC4XX_VMT_MC_MCONTROL, d15
4:
	ld.w	d15, [a15]TC4XX_VMT_MEMDONE
	and	d15, d15, d2
	jz	d15, 4p

	; Read address 0 once, then disable the MBIST shell again

	mov	d15, #0x8
	st.w	[a14]TC4XX_VMT_MC_MCONTROL, d15
	mov	d15, #0x1001
	st.w	[a14]TC4XX_VMT_MC_CONFIG0, d15
	mov	d15, #0
	st.w	[a14]TC4XX_VMT_MC_CONFIG1, d15
	st.w	[a14]TC4XX_VMT_MC_RANGE, d15
	mov	d15, #0x9
	st.w	[a14]TC4XX_VMT_MC_MCONTROL, d15
	mov	d15, #0x8
	st.w	[a14]TC4XX_VMT_MC_MCONTROL, d15
5:
	ld.w	d15, [a15]TC4XX_VMT_MEMDONE
	and	d15, d15, d2
	jz	d15, 5p

	ld.w	d15, [a15]TC4XX_VMT_MEMTEST
	andn	d15, d15, d2
	st.w	[a15]TC4XX_VMT_MEMTEST, d15
6:
	ld.w	d15, [a15]TC4XX_VMT_MEMSTAT
	and	d15, d15, d2
	jnz	d15, 6p

	ld.w	d15, [a14]TC4XX_VMT_MC_ALMSRCS
	or	d15, d15, #TC4XX_ALMSRCS_ALARMS
	st.w	[a14]TC4XX_VMT_MC_ALMSRCS, d15
7:
	sh	d5, d5, #-1
	add	d1, d1, #1
	j	2p
8:
	ji	a11
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: __start
 *
 * Description:
 *   Reset entry of CPU0.  It only sets the boot stack in the PSPR and
 *   jumps to tricore_head, as the linker script keeps .start to 0x20
 *   bytes ahead of the interface constants.
 *
 *   This is the TASKING syntax of gnu/tricore_head.S.
 *
 ****************************************************************************/

	.sdecl	".text.start",code,protect
	.sect	".text.start"
	.global	__start
__start:
	movh.a	a10, #@HIS(__psram_ustack)
	lea	a10, [a10]@LOS(__psram_ustack)
	dsync
	movh.a	a15, #@HIS(tricore_head)
	lea	a15, [a15]@LOS(tricore_head)
	ji	a15

	.end
