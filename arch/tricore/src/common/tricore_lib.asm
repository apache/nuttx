/****************************************************************************
 * arch/tricore/src/common/tricore_lib.asm
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

/* TASKING astc copy of tricore_lib.S, keep both in sync */

#include <arch/arch.h>

	.global tricore_endinit_disable
	.global tricore_endinit_enable
	.global tricore_safety_endinit_enable

	.sdecl	".text", CODE
	.sect	".text"

#if defined(CONFIG_TRICORE_TC18)
#define CPU_PROTSFR	0xF880E080

tricore_endinit_disable:
	mfcr	d4, #TRICORE_CPU_CORE_ID
	extr.u	d4, d4, #0, #3
	sh	d4, d4, #18
	movh.a	a15, #@HIS(CPU_PROTSFR)
	lea	a15, [a15]@LOS(CPU_PROTSFR)
	addsc.a	a15, a15, d4, #0

	ld.w	d15, [a15]
	extr.u	d15, d15, #0, #3
	jlt.u	d15, #3, already_config

	/* Transition: Run -> Config (unlock) */
	insert	d15, d15, #9, #0, #16
	st.w	[a15], d15
	dsync

already_config:
	ji	a11

tricore_endinit_enable:
	mfcr	d4, #TRICORE_CPU_CORE_ID
	extr.u	d4, d4, #0, #3
	sh	d4, d4, #18
	movh.a	a15, #@HIS(CPU_PROTSFR)
	lea	a15, [a15]@LOS(CPU_PROTSFR)
	addsc.a	a15, a15, d4, #0

	ld.w	d15, [a15]
	extr.u	d15, d15, #0, #3
	jge.u	d15, #2, already_locked

	/* Transition: Config -> Run (lock) */
	insert	d15, d15, #12, #0, #16
	st.w	[a15], d15
	dsync

already_locked:
	ji	a11

tricore_safety_endinit_enable:
	ji	a11

	/* Virtualization disable */

	.global tricore_virt_disable
tricore_virt_disable:
	mov	d15, #0
	mtcr	#TRICORE_CPU_VCON0, d15

	mov	d15, #0xB00
	mtcr	#TRICORE_CPU_PSW, d15
	isync

	rfh

#else /* TC1.6 (AURIX TC3x) */

/* TC1.6 (AURIX TC3x) -- WDT-based endinit */

#define SCU_BASE	0xF0036000
#define WDTCPU0CON0	(SCU_BASE + 0x024C)
#define WDTSCON0	(SCU_BASE + 0x02A8)

tricore_endinit_disable:
	mfcr	d4, #TRICORE_CPU_CORE_ID
	extr.u	d4, d4, #0, #3
	jne	d4, #6, 1n
	mov	d4, #5
1:
	sh	d5, d4, #1
	add	d4, d5, d4
	movh.a	a15, #@HIS(WDTCPU0CON0)
	lea	a15, [a15]@LOS(WDTCPU0CON0)
	addsc.a	a15, a15, d4, #2

	/* Read current WDT register, invert password bits */
	ld.w	d15, [a15]
	xor	d15, d15, #0xFC

	insert	d15, d15, #1, #0, #2
	st.w	[a15], d15

	insert	d15, d15, #2, #0, #2
	st.w	[a15], d15
	dsync

	ji	a11

tricore_endinit_enable:
	mfcr	d4, #TRICORE_CPU_CORE_ID
	extr.u	d4, d4, #0, #3
	jne	d4, #6, 1n
	mov	d4, #5
1:
	sh	d5, d4, #1
	add	d4, d5, d4
	movh.a	a15, #@HIS(WDTCPU0CON0)
	lea	a15, [a15]@LOS(WDTCPU0CON0)
	addsc.a	a15, a15, d4, #2

	/* Read current WDT register, invert password bits */
	ld.w	d15, [a15]
	xor	d15, d15, #0xFC

	insert	d15, d15, #1, #0, #2
	st.w	[a15], d15

	insert	d15, d15, #3, #0, #2
	st.w	[a15], d15
	dsync

	ji	a11

tricore_safety_endinit_enable:
	mfcr	d4, #TRICORE_CPU_CORE_ID
	extr.u	d4, d4, #0, #3
	jne	d4, #0, _safety_endinit_skip

	movh.a	a15, #@HIS(WDTSCON0)
	lea	a15, [a15]@LOS(WDTSCON0)

	ld.w	d15, [a15]
	xor	d15, d15, #0xFC

	insert	d15, d15, #1, #0, #2
	st.w	[a15], d15

	insert	d15, d15, #3, #0, #2
	st.w	[a15], d15
	dsync

_safety_endinit_skip:
	ji	a11

#endif /* CONFIG_TRICORE_TC18 */

	.end
