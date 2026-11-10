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

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TRICORE_PSW_RESET  0x980   /* Supervisor, call depth counting on */

#ifdef CONFIG_ARCH_CHIP_FAMILY_TC4X
#  define TC4XX_BOOT_SP    0x7010eb00
#  define TC4XX_BOOT_FCX   0x000743b0  /* Boot CSA 0x7010ec00 */
#  define TC4XX_PSW_VIRT   0xb00       /* PSW for disabling the virtualization */
#endif

	.extern	tricore_start
	.extern	__USTACK0
	.extern	__A0_MEM
	.extern	__A1_MEM
	.extern	__A8_MEM
	.extern	__A9_MEM
	.extern	__TRAPTAB_CPU0

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tricore_head
 *
 * Description:
 *   Set the small data base registers and the PSW and jump to
 *   tricore_start, which links the CSAs before it calls anything.
 *
 *   TC4xx first disables the virtualization with a call to
 *   tricore_virt_off, using a boot stack and a single boot CSA in
 *   the PSPR, as the iLLD startup code does.
 *
 ****************************************************************************/

	.sdecl	".text.tricore_head",code
	.sect	".text.tricore_head"
tricore_head:
#ifdef CONFIG_ARCH_CHIP_FAMILY_TC4X
	movh.a	a15, #@HIS(__TRAPTAB_CPU0)
	lea	a15, [a15]@LOS(__TRAPTAB_CPU0)
	mov.d	d0, a15
	mtcr	#TRICORE_CPU_BTV, d0
	isync

	movh	d0, #@HIS(TC4XX_BOOT_FCX)
	addi	d0, d0, #@LOS(TC4XX_BOOT_FCX)
	mtcr	#TRICORE_CPU_FCX, d0
	isync

	call	tricore_virt_off

	movh.a	a15, #@HIS(__TRAPTAB_CPU0)
	lea	a15, [a15]@LOS(__TRAPTAB_CPU0)
	mov.d	d0, a15
	mtcr	#TRICORE_CPU_BTV, d0
	isync

	movh.a	a10, #@HIS(__USTACK0)
	lea	a10, [a10]@LOS(__USTACK0)
	dsync
#endif

	movh.a	a1, #@HIS(__A1_MEM)
	lea	a1, [a1]@LOS(__A1_MEM)
	movh.a	a0, #@HIS(__A0_MEM)
	lea	a0, [a0]@LOS(__A0_MEM)
	movh.a	a8, #@HIS(__A8_MEM)
	lea	a8, [a8]@LOS(__A8_MEM)
	movh.a	a9, #@HIS(__A9_MEM)
	lea	a9, [a9]@LOS(__A9_MEM)

	mov	d0, #TRICORE_PSW_RESET
	mtcr	#TRICORE_CPU_PSW, d0
	isync

	movh.a	a15, #@HIS(tricore_start)
	lea	a15, [a15]@LOS(tricore_start)
	ji	a15

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
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: __start
 *
 * Description:
 *   Reset entry of CPU0.  It only sets the stack and jumps to
 *   tricore_head, as the linker script keeps .start to 0x20 bytes
 *   ahead of the interface constants.
 *
 *   This is the TASKING syntax of gnu/tricore_head.S.
 *
 ****************************************************************************/

	.sdecl	".text.start",code,protect
	.sect	".text.start"
	.global	__start
__start:
#ifdef CONFIG_ARCH_CHIP_FAMILY_TC4X
	movh.a	a10, #@HIS(TC4XX_BOOT_SP)
	lea	a10, [a10]@LOS(TC4XX_BOOT_SP)
#else
	movh.a	a10, #@HIS(__USTACK0)
	lea	a10, [a10]@LOS(__USTACK0)
#endif
	dsync
	movh.a	a15, #@HIS(tricore_head)
	lea	a15, [a15]@LOS(tricore_head)
	ji	a15

	.end
