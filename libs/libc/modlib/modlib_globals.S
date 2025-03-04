/****************************************************************************
 * libs/libc/modlib/modlib_globals.S
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

#ifdef __CYGWIN__
#  define SYMBOL(s) _##s
#  define GLOBAL .global
#  define SECTION .data
	.macro GLOBAL ep
	.global	SYMBOL(\ep)
	.type	SYMBOL(\ep), "object"
	.endm
	.macro SIZE ep
	.endm
#elif defined(__ELF__)
#  define SYMBOL(s) s
#  define SECTION .data
	.macro GLOBAL ep
	.global	SYMBOL(\ep)
	.type	SYMBOL(\ep), "object"
	.endm
	.macro SIZE ep
#  if defined(__ghs__)
	.size	SYMBOL(ep), . - SYMBOL(ep)
#  else
	.size	SYMBOL(\ep), . - SYMBOL(\ep)
#  endif
	.endm
#else
#  define SYMBOL(s) _##s
#  define SECTION .section __DATA,__data
	.macro GLOBAL ep
	.private_extern SYMBOL(\ep)
	.globl 	SYMBOL(\ep)
	.endm
	.macro SIZE ep
	.endm
#endif

#if defined(__SIZEOF_POINTER__ ) && __SIZEOF_POINTER__ == 8
	.macro globalEntry index, ep
	.quad	.l\index
	.quad	\ep
	.endm
# define ALIGN 8
#else
	.macro globalEntry index, ep
	.long	.l\index
	.long	\ep
	.endm
# define ALIGN 4
#endif
#ifdef __ARM_ARCH_ISA_THUMB2
# ifdef __ARM_ARCH_7M__
	.arch armv7e-m
# elif defined ___ARM_ARCH 8
	.arch armv8-m.base
#endif
#ifdef __ARM_ASM_SYNTAX_UNIFIED__
	.syntax	unified
#endif
	.thumb
#endif
	.data
	.align	ALIGN
	GLOBAL	globalNames 

SYMBOL(globalNames):
	SIZE	globalNames 

	.align	ALIGN
	GLOBAL	nglobals
SYMBOL(nglobals):	
	.word	0
	SIZE	nglobals

	.align	ALIGN
	GLOBAL	global_table
SYMBOL(global_table):
	SIZE	global_table
