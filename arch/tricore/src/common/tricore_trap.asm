/****************************************************************************
 * arch/tricore/src/common/tricore_trap.asm
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

/* TASKING astc copy of tricore_trap.S, keep both in sync */

#include <arch/arch.h>

	.sdecl	".traptab", CODE
	.sect	".traptab"
	.global btv_vector
	.align	256

TRAP_SLOT .macro num, handler
	mov	d4, #num
	mov	d5, d15
	movh.a	a2, #@HIS(handler)
	lea	a2, [a2]@LOS(handler)
	ji	a2
	.align	32
	.endm

btv_vector:
	TRAP_SLOT 0, tricore_trap_dispatch
	TRAP_SLOT 1, tricore_trap_dispatch
	TRAP_SLOT 2, tricore_trap_dispatch
	TRAP_SLOT 3, tricore_trap_dispatch
	TRAP_SLOT 4, tricore_trap_dispatch
	TRAP_SLOT 5, tricore_trap_dispatch
	TRAP_SLOT 6, tricore_svcall_local
	TRAP_SLOT 7, tricore_trap_dispatch

	.sdecl	".text", CODE
	.sect	".text"
	.align	4

	.global tricore_trap_dispatch
tricore_trap_dispatch:
	svlcx
	call	tricore_trap_handler
	rslcx
	rfe

	.global tricore_svcall_local
tricore_svcall_local:
	svlcx
	call	tricore_svcall
	rslcx
	rfe

	.end
