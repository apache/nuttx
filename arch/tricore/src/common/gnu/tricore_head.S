/****************************************************************************
 * arch/tricore/src/common/gnu/tricore_head.S
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

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TRICORE_PSW_RESET  0x980   /* Supervisor, call depth counting on */
#define TRICORE_BIV_VECTOR 0x1fe0  /* Single interrupt vector at INTTAB + 0x1fe0 */

#ifdef CONFIG_ARCH_CHIP_TC3XX
#  define TC3XX_WDTCPU0_CON0 0xf003624c  /* CPU0 watchdog CON0, 0xc apart per CPU */
#endif
#ifdef CONFIG_ARCH_CHIP_TC4XX
#  define TC4XX_PSW_VIRT   0xb00       /* PSW for disabling the virtualization */
#endif

/* Make the CSA link word of the CSA address in addr: the segment in bits
 * 19:16 and the offset in bits 15:0.
 */

	.macro	tricore_csa_link addr, tmp1, tmp2
	extr.u	\tmp1, \addr, 6, 16
	extr.u	\tmp2, \addr, 12, 20
	insert	\addr, \tmp2, \tmp1, 0, 16
	.endm

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tricore_head
 *
 * Description:
 *   Set up the core registers and the CSA list, which the code has to do
 *   before it can call anything: the small data base registers, the trap
 *   and interrupt vectors, the interrupt stack, the caches and the free
 *   CSA list with LCX at the third last entry.  Then set the stack and the
 *   PSW, disable the watchdogs, copy .data and clear .bss, set up the
 *   clocks (see tricore_internal.h) and jump to tricore_start.
 *
 *   On TC3xx BTV, BIV, ISP, PCON0 and DCON0 are CPU ENDINIT protected,
 *   so the ENDINIT of the CPU watchdog is cleared around them.
 *
 *   TC4xx first disables the virtualization with a call to
 *   tricore_virt_off, on the boot stack and the single boot CSA in the
 *   PSPR that the linker script gives, as the iLLD startup code does.
 *   Then it links the boot CSAs in the PSPR and, before the DSPR that
 *   holds the CSA list and the DLMU that holds .data and .bss are
 *   initialized, disables the watchdogs and initializes the RAMs in C
 *   (tricore_wdt_disable, tricore_ram_init).
 *
 ****************************************************************************/

	.text
	.type	tricore_head, %function
tricore_head:
	movh.a	%a1, hi:__a1_mem
	lea	%a1, [%a1]lo:__a1_mem
	movh.a	%a0, hi:__a0_mem
	lea	%a0, [%a0]lo:__a0_mem
	movh.a	%a8, hi:__a8_mem
	lea	%a8, [%a8]lo:__a8_mem
	movh.a	%a9, hi:__a9_mem
	lea	%a9, [%a9]lo:__a9_mem

#ifdef CONFIG_ARCH_CHIP_TC3XX
	jl	tc3xx_endinit_clear

#endif
	movh.a	%a15, hi:__TRAPTAB_CPU0
	lea	%a15, [%a15]lo:__TRAPTAB_CPU0
	mov.d	%d15, %a15
	mtcr	TRICORE_CPU_BTV, %d15
	isync

#ifdef CONFIG_ARCH_CHIP_TC4XX
	movh.a	%a15, hi:__psram_boot_csa
	lea	%a15, [%a15]lo:__psram_boot_csa
	mov.d	%d15, %a15
	tricore_csa_link %d15, %d14, %d13
	mtcr	TRICORE_CPU_FCX, %d15
	isync

	call	tricore_virt_off

	movh.a	%a15, hi:__TRAPTAB_CPU0
	lea	%a15, [%a15]lo:__TRAPTAB_CPU0
	mov.d	%d15, %a15
	mtcr	TRICORE_CPU_BTV, %d15
	isync

	/* Free CSA list in the PSPR from __psram_boot_csa to
	 * __psram_boot_csa_end, LCX at the third last, for the C steps that
	 * run before the DSPR and the DLMU are initialized
	 */

	movh.a	%a15, hi:__psram_boot_csa
	lea	%a15, [%a15]lo:__psram_boot_csa
	movh.a	%a14, hi:__psram_boot_csa_end
	lea	%a14, [%a14]lo:__psram_boot_csa_end
	mov.d	%d15, %a15
	tricore_csa_link %d15, %d14, %d13
	mtcr	TRICORE_CPU_FCX, %d15
	isync
1:
	addi	%d15, %d15, 1
	st.w	[%a15], %d15
	lea	%a15, [%a15]64
	jne.a	%a15, %a14, 1b

	mov	%d14, 0
	st.w	[%a15]-64, %d14
	addi	%d15, %d15, -3
	mtcr	TRICORE_CPU_LCX, %d15
	isync

	call	tricore_wdt_disable
	call	tricore_ram_init

#endif
	/* Caches on, interrupt vector, interrupt stack */

	mov	%d15, 0
	mtcr	TRICORE_CPU_PCON0, %d15
	isync
	mtcr	TRICORE_CPU_DCON0, %d15
	isync

	movh.a	%a15, hi:__INTTAB_CPU0
	lea	%a15, [%a15]lo:__INTTAB_CPU0
	lea	%a15, [%a15]TRICORE_BIV_VECTOR
	mov.d	%d15, %a15
	mtcr	TRICORE_CPU_BIV, %d15
	isync

	movh.a	%a15, hi:__istack
	lea	%a15, [%a15]lo:__istack
	mov.d	%d15, %a15
	mtcr	TRICORE_CPU_ISP, %d15
	isync

	/* Free CSA list from __csa_begin to __csa_end, LCX at the third last */

	movh.a	%a15, hi:__csa_begin
	lea	%a15, [%a15]lo:__csa_begin
	movh.a	%a14, hi:__csa_end
	lea	%a14, [%a14]lo:__csa_end
	mov.d	%d15, %a15
	tricore_csa_link %d15, %d14, %d13
	mtcr	TRICORE_CPU_FCX, %d15
	isync
1:
	addi	%d15, %d15, 1
	st.w	[%a15], %d15
	lea	%a15, [%a15]64
	jne.a	%a15, %a14, 1b

	mov	%d14, 0
	st.w	[%a15]-64, %d14
	addi	%d15, %d15, -3
	mtcr	TRICORE_CPU_LCX, %d15
	isync

#ifdef CONFIG_ARCH_CHIP_TC3XX
	jl	tc3xx_endinit_set

#endif
	movh.a	%sp, hi:__ustack
	lea	%sp, [%sp]lo:__ustack
	dsync

	mov	%d0, TRICORE_PSW_RESET
	mtcr	TRICORE_CPU_PSW, %d0
	isync

#ifdef CONFIG_ARCH_CHIP_TC3XX
	call	tricore_wdt_disable

#endif
	/* Copy .data from its image in flash and clear .bss, a word at a time
	 * up to the word aligned ends that the linker script gives
	 */

	movh.a	%a2, hi:_data_flash
	lea	%a2, [%a2]lo:_data_flash
	movh.a	%a3, hi:_sdata
	lea	%a3, [%a3]lo:_sdata
	movh.a	%a4, hi:_edata
	lea	%a4, [%a4]lo:_edata
	j	7f
6:
	ld.w	%d15, [%a2+]4
	st.w	[%a3+]4, %d15
7:
	lt.a	%d15, %a3, %a4
	jnz	%d15, 6b

	mov	%d14, 0
	movh.a	%a3, hi:_sbss
	lea	%a3, [%a3]lo:_sbss
	movh.a	%a4, hi:_ebss
	lea	%a4, [%a4]lo:_ebss
	j	9f
8:
	st.w	[%a3+]4, %d14
9:
	lt.a	%d15, %a3, %a4
	jnz	%d15, 8b

	call	tricore_clockconfig
	jz	%d2, 5f
	debug
5:
	movh.a	%a15, hi:tricore_start
	lea	%a15, [%a15]lo:tricore_start
	ji	%a15

#ifdef CONFIG_ARCH_CHIP_TC3XX
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

	.type	tc3xx_endinit_clear, %function
tc3xx_endinit_clear:
	mov	%d5, 0
	j	2f

	.type	tc3xx_endinit_set, %function
tc3xx_endinit_set:
	mov	%d5, 1
2:
	mfcr	%d4, TRICORE_CPU_CORE_ID
	extr.u	%d4, %d4, 0, 3
	jne	%d4, 6, 3f
	mov	%d4, 5
3:
	sh	%d15, %d4, 1
	add	%d4, %d15, %d4
	movh.a	%a15, hi:TC3XX_WDTCPU0_CON0
	lea	%a15, [%a15]lo:TC3XX_WDTCPU0_CON0
	addsc.a	%a15, %a15, %d4, 2

	/* Password access with the inverted password bits 7:2, then the
	 * modify access with LCK set and the new ENDINIT
	 */

	ld.w	%d15, [%a15]
	xor	%d15, %d15, 0xfc
	insert	%d15, %d15, 1, 0, 2
	st.w	[%a15], %d15
	or	%d4, %d5, 2
	insert	%d15, %d15, %d4, 0, 2
	st.w	[%a15], %d15
4:
	ld.w	%d15, [%a15]
	extr.u	%d15, %d15, 0, 1
	jne	%d15, %d5, 4b
	ji	%a11
#endif

#ifdef CONFIG_ARCH_CHIP_TC4XX
/****************************************************************************
 * Name: tricore_virt_off
 *
 * Description:
 *   Disable the virtualization and return with rfh.
 *
 ****************************************************************************/

	.type	tricore_virt_off, %function
tricore_virt_off:
	mov	%d0, 0
	mtcr	TRICORE_CPU_VCON0, %d0
	isync
	mov	%d0, TC4XX_PSW_VIRT
	mtcr	TRICORE_CPU_PSW, %d0
	isync
	rfh
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
 *   tasking/tricore_head.asm is the same code in the TASKING syntax.
 *
 ****************************************************************************/

	.section .start, "ax", %progbits
	.global	__start
	.type	__start, %function
__start:
	movh.a	%sp, hi:__psram_ustack
	lea	%sp, [%sp]lo:__psram_ustack
	dsync
	movh.a	%a15, hi:tricore_head
	lea	%a15, [%a15]lo:tricore_head
	ji	%a15
